
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
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_VTOL_RESERVED5);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_STANDBY);
            assert(pack.custom_mode_GET() == 847068421L);
            assert(pack.mavlink_version_GET() == (char)89);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_PPZ);
            assert(pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED);
        });
        DemoDevice.HEARTBEAT p0 = LoopBackDemoChannel.new_HEARTBEAT();
        PH.setPack(p0);
        p0.system_status_SET(MAV_STATE.MAV_STATE_STANDBY) ;
        p0.mavlink_version_SET((char)89) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_PPZ) ;
        p0.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED) ;
        p0.custom_mode_SET(847068421L) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_VTOL_RESERVED5) ;
        LoopBackDemoChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.drop_rate_comm_GET() == (char)8582);
            assert(pack.current_battery_GET() == (short)15128);
            assert(pack.voltage_battery_GET() == (char)5086);
            assert(pack.errors_count2_GET() == (char)35587);
            assert(pack.battery_remaining_GET() == (byte) - 83);
            assert(pack.errors_count3_GET() == (char)15281);
            assert(pack.onboard_control_sensors_present_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL);
            assert(pack.onboard_control_sensors_enabled_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION);
            assert(pack.errors_count1_GET() == (char)54876);
            assert(pack.load_GET() == (char)3751);
            assert(pack.errors_comm_GET() == (char)42960);
            assert(pack.onboard_control_sensors_health_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW);
            assert(pack.errors_count4_GET() == (char)12579);
        });
        DemoDevice.SYS_STATUS p1 = LoopBackDemoChannel.new_SYS_STATUS();
        PH.setPack(p1);
        p1.onboard_control_sensors_enabled_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION) ;
        p1.errors_count2_SET((char)35587) ;
        p1.errors_comm_SET((char)42960) ;
        p1.drop_rate_comm_SET((char)8582) ;
        p1.voltage_battery_SET((char)5086) ;
        p1.load_SET((char)3751) ;
        p1.onboard_control_sensors_health_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW) ;
        p1.battery_remaining_SET((byte) - 83) ;
        p1.current_battery_SET((short)15128) ;
        p1.errors_count1_SET((char)54876) ;
        p1.errors_count3_SET((char)15281) ;
        p1.errors_count4_SET((char)12579) ;
        p1.onboard_control_sensors_present_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL) ;
        LoopBackDemoChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_unix_usec_GET() == 9018745968549938027L);
            assert(pack.time_boot_ms_GET() == 2911010513L);
        });
        DemoDevice.SYSTEM_TIME p2 = LoopBackDemoChannel.new_SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_boot_ms_SET(2911010513L) ;
        p2.time_unix_usec_SET(9018745968549938027L) ;
        LoopBackDemoChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -7.644953E37F);
            assert(pack.vz_GET() == -5.8802625E37F);
            assert(pack.afx_GET() == -1.9346267E38F);
            assert(pack.type_mask_GET() == (char)62836);
            assert(pack.vx_GET() == 1.3567305E38F);
            assert(pack.yaw_rate_GET() == 2.6337906E38F);
            assert(pack.afy_GET() == -1.2014878E38F);
            assert(pack.x_GET() == -5.415351E36F);
            assert(pack.vy_GET() == -1.1188911E38F);
            assert(pack.z_GET() == 3.2483522E38F);
            assert(pack.y_GET() == 1.972372E38F);
            assert(pack.time_boot_ms_GET() == 3280727605L);
            assert(pack.afz_GET() == 3.1838633E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
        });
        DemoDevice.POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.vx_SET(1.3567305E38F) ;
        p3.vz_SET(-5.8802625E37F) ;
        p3.x_SET(-5.415351E36F) ;
        p3.y_SET(1.972372E38F) ;
        p3.z_SET(3.2483522E38F) ;
        p3.yaw_SET(-7.644953E37F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p3.yaw_rate_SET(2.6337906E38F) ;
        p3.vy_SET(-1.1188911E38F) ;
        p3.time_boot_ms_SET(3280727605L) ;
        p3.type_mask_SET((char)62836) ;
        p3.afx_SET(-1.9346267E38F) ;
        p3.afz_SET(3.1838633E38F) ;
        p3.afy_SET(-1.2014878E38F) ;
        LoopBackDemoChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)113);
            assert(pack.target_system_GET() == (char)131);
            assert(pack.seq_GET() == 1346041612L);
            assert(pack.time_usec_GET() == 4421754595428583796L);
        });
        DemoDevice.PING p4 = LoopBackDemoChannel.new_PING();
        PH.setPack(p4);
        p4.target_component_SET((char)113) ;
        p4.time_usec_SET(4421754595428583796L) ;
        p4.seq_SET(1346041612L) ;
        p4.target_system_SET((char)131) ;
        LoopBackDemoChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)107);
            assert(pack.control_request_GET() == (char)80);
            assert(pack.version_GET() == (char)147);
            assert(pack.passkey_LEN(ph) == 8);
            assert(pack.passkey_TRY(ph).equals("zimkffny"));
        });
        DemoDevice.CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.target_system_SET((char)107) ;
        p5.control_request_SET((char)80) ;
        p5.version_SET((char)147) ;
        p5.passkey_SET("zimkffny", PH) ;
        LoopBackDemoChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.gcs_system_id_GET() == (char)203);
            assert(pack.ack_GET() == (char)252);
            assert(pack.control_request_GET() == (char)248);
        });
        DemoDevice.CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.gcs_system_id_SET((char)203) ;
        p6.control_request_SET((char)248) ;
        p6.ack_SET((char)252) ;
        LoopBackDemoChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 12);
            assert(pack.key_TRY(ph).equals("kjfpoQmtaksW"));
        });
        DemoDevice.AUTH_KEY p7 = LoopBackDemoChannel.new_AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("kjfpoQmtaksW", PH) ;
        LoopBackDemoChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_TEST_DISARMED);
            assert(pack.custom_mode_GET() == 4154068440L);
            assert(pack.target_system_GET() == (char)78);
        });
        DemoDevice.SET_MODE p11 = LoopBackDemoChannel.new_SET_MODE();
        PH.setPack(p11);
        p11.custom_mode_SET(4154068440L) ;
        p11.target_system_SET((char)78) ;
        p11.base_mode_SET(MAV_MODE.MAV_MODE_TEST_DISARMED) ;
        LoopBackDemoChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)238);
            assert(pack.param_index_GET() == (short)28216);
            assert(pack.param_id_LEN(ph) == 12);
            assert(pack.param_id_TRY(ph).equals("reyuhCzmmqge"));
            assert(pack.target_system_GET() == (char)112);
        });
        DemoDevice.PARAM_REQUEST_READ p20 = LoopBackDemoChannel.new_PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_system_SET((char)112) ;
        p20.param_index_SET((short)28216) ;
        p20.param_id_SET("reyuhCzmmqge", PH) ;
        p20.target_component_SET((char)238) ;
        LoopBackDemoChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)45);
            assert(pack.target_component_GET() == (char)174);
        });
        DemoDevice.PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.new_PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_component_SET((char)174) ;
        p21.target_system_SET((char)45) ;
        LoopBackDemoChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 5);
            assert(pack.param_id_TRY(ph).equals("KyijE"));
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8);
            assert(pack.param_count_GET() == (char)29871);
            assert(pack.param_index_GET() == (char)43758);
            assert(pack.param_value_GET() == 4.6530413E37F);
        });
        DemoDevice.PARAM_VALUE p22 = LoopBackDemoChannel.new_PARAM_VALUE();
        PH.setPack(p22);
        p22.param_index_SET((char)43758) ;
        p22.param_count_SET((char)29871) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8) ;
        p22.param_value_SET(4.6530413E37F) ;
        p22.param_id_SET("KyijE", PH) ;
        LoopBackDemoChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32);
            assert(pack.param_value_GET() == 2.7616894E38F);
            assert(pack.target_component_GET() == (char)174);
            assert(pack.param_id_LEN(ph) == 2);
            assert(pack.param_id_TRY(ph).equals("yb"));
            assert(pack.target_system_GET() == (char)23);
        });
        DemoDevice.PARAM_SET p23 = LoopBackDemoChannel.new_PARAM_SET();
        PH.setPack(p23);
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32) ;
        p23.param_value_SET(2.7616894E38F) ;
        p23.target_component_SET((char)174) ;
        p23.target_system_SET((char)23) ;
        p23.param_id_SET("yb", PH) ;
        LoopBackDemoChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.vel_GET() == (char)19184);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
            assert(pack.lon_GET() == 300894456);
            assert(pack.satellites_visible_GET() == (char)174);
            assert(pack.v_acc_TRY(ph) == 1306994036L);
            assert(pack.hdg_acc_TRY(ph) == 2181182185L);
            assert(pack.alt_GET() == 306652871);
            assert(pack.vel_acc_TRY(ph) == 3149570617L);
            assert(pack.time_usec_GET() == 1434309510285022159L);
            assert(pack.epv_GET() == (char)1098);
            assert(pack.h_acc_TRY(ph) == 2135584195L);
            assert(pack.cog_GET() == (char)48153);
            assert(pack.eph_GET() == (char)60048);
            assert(pack.alt_ellipsoid_TRY(ph) == 691136539);
            assert(pack.lat_GET() == 2016434768);
        });
        DemoDevice.GPS_RAW_INT p24 = LoopBackDemoChannel.new_GPS_RAW_INT();
        PH.setPack(p24);
        p24.vel_acc_SET(3149570617L, PH) ;
        p24.vel_SET((char)19184) ;
        p24.v_acc_SET(1306994036L, PH) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX) ;
        p24.hdg_acc_SET(2181182185L, PH) ;
        p24.eph_SET((char)60048) ;
        p24.lat_SET(2016434768) ;
        p24.lon_SET(300894456) ;
        p24.satellites_visible_SET((char)174) ;
        p24.h_acc_SET(2135584195L, PH) ;
        p24.cog_SET((char)48153) ;
        p24.alt_SET(306652871) ;
        p24.time_usec_SET(1434309510285022159L) ;
        p24.epv_SET((char)1098) ;
        p24.alt_ellipsoid_SET(691136539, PH) ;
        LoopBackDemoChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)78, (char)174, (char)85, (char)247, (char)0, (char)160, (char)165, (char)211, (char)203, (char)97, (char)254, (char)162, (char)54, (char)245, (char)43, (char)227, (char)126, (char)110, (char)44, (char)135}));
            assert(pack.satellites_visible_GET() == (char)205);
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)45, (char)95, (char)93, (char)151, (char)50, (char)1, (char)235, (char)59, (char)227, (char)24, (char)85, (char)158, (char)110, (char)191, (char)103, (char)59, (char)150, (char)14, (char)188, (char)42}));
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)18, (char)200, (char)114, (char)159, (char)35, (char)157, (char)223, (char)234, (char)234, (char)233, (char)243, (char)26, (char)64, (char)192, (char)215, (char)158, (char)25, (char)62, (char)0, (char)96}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)150, (char)106, (char)254, (char)43, (char)186, (char)220, (char)94, (char)90, (char)53, (char)189, (char)43, (char)21, (char)158, (char)124, (char)108, (char)19, (char)227, (char)87, (char)13, (char)219}));
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)95, (char)112, (char)29, (char)148, (char)155, (char)33, (char)45, (char)179, (char)15, (char)122, (char)166, (char)79, (char)144, (char)144, (char)218, (char)95, (char)209, (char)254, (char)27, (char)24}));
        });
        DemoDevice.GPS_STATUS p25 = LoopBackDemoChannel.new_GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_azimuth_SET(new char[] {(char)150, (char)106, (char)254, (char)43, (char)186, (char)220, (char)94, (char)90, (char)53, (char)189, (char)43, (char)21, (char)158, (char)124, (char)108, (char)19, (char)227, (char)87, (char)13, (char)219}, 0) ;
        p25.satellite_elevation_SET(new char[] {(char)45, (char)95, (char)93, (char)151, (char)50, (char)1, (char)235, (char)59, (char)227, (char)24, (char)85, (char)158, (char)110, (char)191, (char)103, (char)59, (char)150, (char)14, (char)188, (char)42}, 0) ;
        p25.satellite_prn_SET(new char[] {(char)95, (char)112, (char)29, (char)148, (char)155, (char)33, (char)45, (char)179, (char)15, (char)122, (char)166, (char)79, (char)144, (char)144, (char)218, (char)95, (char)209, (char)254, (char)27, (char)24}, 0) ;
        p25.satellite_snr_SET(new char[] {(char)78, (char)174, (char)85, (char)247, (char)0, (char)160, (char)165, (char)211, (char)203, (char)97, (char)254, (char)162, (char)54, (char)245, (char)43, (char)227, (char)126, (char)110, (char)44, (char)135}, 0) ;
        p25.satellite_used_SET(new char[] {(char)18, (char)200, (char)114, (char)159, (char)35, (char)157, (char)223, (char)234, (char)234, (char)233, (char)243, (char)26, (char)64, (char)192, (char)215, (char)158, (char)25, (char)62, (char)0, (char)96}, 0) ;
        p25.satellites_visible_SET((char)205) ;
        LoopBackDemoChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == (short) -9779);
            assert(pack.xgyro_GET() == (short)14453);
            assert(pack.zacc_GET() == (short)20264);
            assert(pack.xmag_GET() == (short) -26656);
            assert(pack.zgyro_GET() == (short) -631);
            assert(pack.ygyro_GET() == (short) -8380);
            assert(pack.ymag_GET() == (short)10566);
            assert(pack.zmag_GET() == (short) -22487);
            assert(pack.xacc_GET() == (short) -8603);
            assert(pack.time_boot_ms_GET() == 2948511994L);
        });
        DemoDevice.SCALED_IMU p26 = LoopBackDemoChannel.new_SCALED_IMU();
        PH.setPack(p26);
        p26.zgyro_SET((short) -631) ;
        p26.ygyro_SET((short) -8380) ;
        p26.xmag_SET((short) -26656) ;
        p26.zacc_SET((short)20264) ;
        p26.yacc_SET((short) -9779) ;
        p26.xacc_SET((short) -8603) ;
        p26.ymag_SET((short)10566) ;
        p26.time_boot_ms_SET(2948511994L) ;
        p26.zmag_SET((short) -22487) ;
        p26.xgyro_SET((short)14453) ;
        LoopBackDemoChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == (short)11882);
            assert(pack.xgyro_GET() == (short) -9683);
            assert(pack.xmag_GET() == (short)1611);
            assert(pack.ymag_GET() == (short)27056);
            assert(pack.xacc_GET() == (short)21787);
            assert(pack.zmag_GET() == (short) -4376);
            assert(pack.time_usec_GET() == 5568464465947650714L);
            assert(pack.zacc_GET() == (short) -11666);
            assert(pack.yacc_GET() == (short)25731);
            assert(pack.zgyro_GET() == (short) -24322);
        });
        DemoDevice.RAW_IMU p27 = LoopBackDemoChannel.new_RAW_IMU();
        PH.setPack(p27);
        p27.xacc_SET((short)21787) ;
        p27.ygyro_SET((short)11882) ;
        p27.zgyro_SET((short) -24322) ;
        p27.xmag_SET((short)1611) ;
        p27.time_usec_SET(5568464465947650714L) ;
        p27.xgyro_SET((short) -9683) ;
        p27.zacc_SET((short) -11666) ;
        p27.zmag_SET((short) -4376) ;
        p27.ymag_SET((short)27056) ;
        p27.yacc_SET((short)25731) ;
        LoopBackDemoChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_diff2_GET() == (short) -11327);
            assert(pack.temperature_GET() == (short)26903);
            assert(pack.press_diff1_GET() == (short)11017);
            assert(pack.time_usec_GET() == 4854755454346547482L);
            assert(pack.press_abs_GET() == (short) -19745);
        });
        DemoDevice.RAW_PRESSURE p28 = LoopBackDemoChannel.new_RAW_PRESSURE();
        PH.setPack(p28);
        p28.time_usec_SET(4854755454346547482L) ;
        p28.press_abs_SET((short) -19745) ;
        p28.temperature_SET((short)26903) ;
        p28.press_diff2_SET((short) -11327) ;
        p28.press_diff1_SET((short)11017) ;
        LoopBackDemoChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_diff_GET() == 2.0024657E38F);
            assert(pack.press_abs_GET() == -3.0206351E38F);
            assert(pack.time_boot_ms_GET() == 4228568212L);
            assert(pack.temperature_GET() == (short) -11390);
        });
        DemoDevice.SCALED_PRESSURE p29 = LoopBackDemoChannel.new_SCALED_PRESSURE();
        PH.setPack(p29);
        p29.time_boot_ms_SET(4228568212L) ;
        p29.press_abs_SET(-3.0206351E38F) ;
        p29.press_diff_SET(2.0024657E38F) ;
        p29.temperature_SET((short) -11390) ;
        LoopBackDemoChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.pitchspeed_GET() == 2.6425672E38F);
            assert(pack.roll_GET() == 1.3139229E38F);
            assert(pack.rollspeed_GET() == 2.3706697E38F);
            assert(pack.yawspeed_GET() == 2.6245683E38F);
            assert(pack.pitch_GET() == 2.0672485E38F);
            assert(pack.time_boot_ms_GET() == 4109649660L);
            assert(pack.yaw_GET() == 1.9872748E37F);
        });
        DemoDevice.ATTITUDE p30 = LoopBackDemoChannel.new_ATTITUDE();
        PH.setPack(p30);
        p30.yaw_SET(1.9872748E37F) ;
        p30.pitch_SET(2.0672485E38F) ;
        p30.time_boot_ms_SET(4109649660L) ;
        p30.rollspeed_SET(2.3706697E38F) ;
        p30.yawspeed_SET(2.6245683E38F) ;
        p30.pitchspeed_SET(2.6425672E38F) ;
        p30.roll_SET(1.3139229E38F) ;
        LoopBackDemoChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.q3_GET() == -9.197792E37F);
            assert(pack.time_boot_ms_GET() == 993046313L);
            assert(pack.q2_GET() == -2.9585593E37F);
            assert(pack.rollspeed_GET() == -1.2526071E38F);
            assert(pack.q4_GET() == 3.3191849E38F);
            assert(pack.q1_GET() == 5.0789527E37F);
            assert(pack.yawspeed_GET() == 1.339529E37F);
            assert(pack.pitchspeed_GET() == -1.4373183E37F);
        });
        DemoDevice.ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.q1_SET(5.0789527E37F) ;
        p31.pitchspeed_SET(-1.4373183E37F) ;
        p31.q4_SET(3.3191849E38F) ;
        p31.rollspeed_SET(-1.2526071E38F) ;
        p31.yawspeed_SET(1.339529E37F) ;
        p31.q3_SET(-9.197792E37F) ;
        p31.time_boot_ms_SET(993046313L) ;
        p31.q2_SET(-2.9585593E37F) ;
        LoopBackDemoChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 3.700275E36F);
            assert(pack.time_boot_ms_GET() == 3767900575L);
            assert(pack.vx_GET() == 4.2249732E37F);
            assert(pack.x_GET() == -4.685135E37F);
            assert(pack.y_GET() == 2.6423386E38F);
            assert(pack.vz_GET() == 2.253257E38F);
            assert(pack.vy_GET() == 1.7908214E38F);
        });
        DemoDevice.LOCAL_POSITION_NED p32 = LoopBackDemoChannel.new_LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.time_boot_ms_SET(3767900575L) ;
        p32.z_SET(3.700275E36F) ;
        p32.y_SET(2.6423386E38F) ;
        p32.x_SET(-4.685135E37F) ;
        p32.vz_SET(2.253257E38F) ;
        p32.vx_SET(4.2249732E37F) ;
        p32.vy_SET(1.7908214E38F) ;
        LoopBackDemoChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.relative_alt_GET() == 243498351);
            assert(pack.alt_GET() == -1276977883);
            assert(pack.vz_GET() == (short) -16863);
            assert(pack.time_boot_ms_GET() == 2423747167L);
            assert(pack.vx_GET() == (short)4444);
            assert(pack.lon_GET() == -770498266);
            assert(pack.hdg_GET() == (char)25159);
            assert(pack.lat_GET() == -1565102771);
            assert(pack.vy_GET() == (short)8434);
        });
        DemoDevice.GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.vy_SET((short)8434) ;
        p33.lat_SET(-1565102771) ;
        p33.alt_SET(-1276977883) ;
        p33.vx_SET((short)4444) ;
        p33.time_boot_ms_SET(2423747167L) ;
        p33.hdg_SET((char)25159) ;
        p33.relative_alt_SET(243498351) ;
        p33.vz_SET((short) -16863) ;
        p33.lon_SET(-770498266) ;
        LoopBackDemoChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.port_GET() == (char)176);
            assert(pack.chan5_scaled_GET() == (short)11306);
            assert(pack.chan4_scaled_GET() == (short) -6629);
            assert(pack.chan6_scaled_GET() == (short) -26746);
            assert(pack.chan1_scaled_GET() == (short)17209);
            assert(pack.chan2_scaled_GET() == (short)26126);
            assert(pack.time_boot_ms_GET() == 2461679325L);
            assert(pack.chan8_scaled_GET() == (short)7054);
            assert(pack.rssi_GET() == (char)48);
            assert(pack.chan3_scaled_GET() == (short)1078);
            assert(pack.chan7_scaled_GET() == (short)8206);
        });
        DemoDevice.RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.new_RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.port_SET((char)176) ;
        p34.chan7_scaled_SET((short)8206) ;
        p34.chan2_scaled_SET((short)26126) ;
        p34.rssi_SET((char)48) ;
        p34.chan5_scaled_SET((short)11306) ;
        p34.chan6_scaled_SET((short) -26746) ;
        p34.chan3_scaled_SET((short)1078) ;
        p34.chan8_scaled_SET((short)7054) ;
        p34.time_boot_ms_SET(2461679325L) ;
        p34.chan1_scaled_SET((short)17209) ;
        p34.chan4_scaled_SET((short) -6629) ;
        LoopBackDemoChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan6_raw_GET() == (char)41066);
            assert(pack.chan3_raw_GET() == (char)31186);
            assert(pack.time_boot_ms_GET() == 3788949471L);
            assert(pack.chan4_raw_GET() == (char)4941);
            assert(pack.chan5_raw_GET() == (char)28692);
            assert(pack.chan2_raw_GET() == (char)8580);
            assert(pack.port_GET() == (char)135);
            assert(pack.chan1_raw_GET() == (char)16902);
            assert(pack.rssi_GET() == (char)73);
            assert(pack.chan7_raw_GET() == (char)31073);
            assert(pack.chan8_raw_GET() == (char)23004);
        });
        DemoDevice.RC_CHANNELS_RAW p35 = LoopBackDemoChannel.new_RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.time_boot_ms_SET(3788949471L) ;
        p35.chan8_raw_SET((char)23004) ;
        p35.chan7_raw_SET((char)31073) ;
        p35.chan6_raw_SET((char)41066) ;
        p35.chan5_raw_SET((char)28692) ;
        p35.chan1_raw_SET((char)16902) ;
        p35.port_SET((char)135) ;
        p35.chan4_raw_SET((char)4941) ;
        p35.chan2_raw_SET((char)8580) ;
        p35.rssi_SET((char)73) ;
        p35.chan3_raw_SET((char)31186) ;
        LoopBackDemoChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo7_raw_GET() == (char)31773);
            assert(pack.servo3_raw_GET() == (char)64479);
            assert(pack.servo13_raw_TRY(ph) == (char)13633);
            assert(pack.servo11_raw_TRY(ph) == (char)4955);
            assert(pack.servo1_raw_GET() == (char)42384);
            assert(pack.time_usec_GET() == 3068734660L);
            assert(pack.port_GET() == (char)243);
            assert(pack.servo15_raw_TRY(ph) == (char)35630);
            assert(pack.servo8_raw_GET() == (char)48958);
            assert(pack.servo16_raw_TRY(ph) == (char)39421);
            assert(pack.servo14_raw_TRY(ph) == (char)17761);
            assert(pack.servo9_raw_TRY(ph) == (char)21663);
            assert(pack.servo6_raw_GET() == (char)29203);
            assert(pack.servo2_raw_GET() == (char)52599);
            assert(pack.servo5_raw_GET() == (char)59042);
            assert(pack.servo12_raw_TRY(ph) == (char)19408);
            assert(pack.servo10_raw_TRY(ph) == (char)10116);
            assert(pack.servo4_raw_GET() == (char)56827);
        });
        DemoDevice.SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.new_SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo12_raw_SET((char)19408, PH) ;
        p36.servo8_raw_SET((char)48958) ;
        p36.servo7_raw_SET((char)31773) ;
        p36.servo5_raw_SET((char)59042) ;
        p36.servo2_raw_SET((char)52599) ;
        p36.servo14_raw_SET((char)17761, PH) ;
        p36.servo10_raw_SET((char)10116, PH) ;
        p36.servo1_raw_SET((char)42384) ;
        p36.servo11_raw_SET((char)4955, PH) ;
        p36.servo4_raw_SET((char)56827) ;
        p36.servo15_raw_SET((char)35630, PH) ;
        p36.servo6_raw_SET((char)29203) ;
        p36.time_usec_SET(3068734660L) ;
        p36.servo9_raw_SET((char)21663, PH) ;
        p36.servo16_raw_SET((char)39421, PH) ;
        p36.port_SET((char)243) ;
        p36.servo3_raw_SET((char)64479) ;
        p36.servo13_raw_SET((char)13633, PH) ;
        LoopBackDemoChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.start_index_GET() == (short)29495);
            assert(pack.target_component_GET() == (char)220);
            assert(pack.end_index_GET() == (short)2013);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_system_GET() == (char)159);
        });
        DemoDevice.MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.new_MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.start_index_SET((short)29495) ;
        p37.end_index_SET((short)2013) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p37.target_system_SET((char)159) ;
        p37.target_component_SET((char)220) ;
        LoopBackDemoChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.end_index_GET() == (short)10605);
            assert(pack.target_system_GET() == (char)5);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.start_index_GET() == (short)4328);
            assert(pack.target_component_GET() == (char)213);
        });
        DemoDevice.MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.new_MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.target_system_SET((char)5) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p38.target_component_SET((char)213) ;
        p38.end_index_SET((short)10605) ;
        p38.start_index_SET((short)4328) ;
        LoopBackDemoChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -6.7183727E37F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_MOTOR_TEST);
            assert(pack.param1_GET() == 8.82328E37F);
            assert(pack.param3_GET() == -3.3016153E38F);
            assert(pack.x_GET() == 2.407712E38F);
            assert(pack.seq_GET() == (char)57943);
            assert(pack.param4_GET() == 2.2093086E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.y_GET() == 5.706844E37F);
            assert(pack.current_GET() == (char)188);
            assert(pack.target_component_GET() == (char)22);
            assert(pack.autocontinue_GET() == (char)176);
            assert(pack.param2_GET() == 1.1385379E38F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_system_GET() == (char)14);
        });
        DemoDevice.MISSION_ITEM p39 = LoopBackDemoChannel.new_MISSION_ITEM();
        PH.setPack(p39);
        p39.seq_SET((char)57943) ;
        p39.autocontinue_SET((char)176) ;
        p39.x_SET(2.407712E38F) ;
        p39.command_SET(MAV_CMD.MAV_CMD_DO_MOTOR_TEST) ;
        p39.current_SET((char)188) ;
        p39.y_SET(5.706844E37F) ;
        p39.z_SET(-6.7183727E37F) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p39.target_component_SET((char)22) ;
        p39.param3_SET(-3.3016153E38F) ;
        p39.param4_SET(2.2093086E37F) ;
        p39.param2_SET(1.1385379E38F) ;
        p39.param1_SET(8.82328E37F) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p39.target_system_SET((char)14) ;
        LoopBackDemoChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.seq_GET() == (char)11880);
            assert(pack.target_system_GET() == (char)248);
            assert(pack.target_component_GET() == (char)48);
        });
        DemoDevice.MISSION_REQUEST p40 = LoopBackDemoChannel.new_MISSION_REQUEST();
        PH.setPack(p40);
        p40.target_component_SET((char)48) ;
        p40.seq_SET((char)11880) ;
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p40.target_system_SET((char)248) ;
        LoopBackDemoChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)159);
            assert(pack.seq_GET() == (char)48074);
            assert(pack.target_component_GET() == (char)154);
        });
        DemoDevice.MISSION_SET_CURRENT p41 = LoopBackDemoChannel.new_MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_system_SET((char)159) ;
        p41.target_component_SET((char)154) ;
        p41.seq_SET((char)48074) ;
        LoopBackDemoChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)30305);
        });
        DemoDevice.MISSION_CURRENT p42 = LoopBackDemoChannel.new_MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)30305) ;
        LoopBackDemoChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_system_GET() == (char)162);
            assert(pack.target_component_GET() == (char)206);
        });
        DemoDevice.MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.new_MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_system_SET((char)162) ;
        p43.target_component_SET((char)206) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        LoopBackDemoChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)58);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_system_GET() == (char)6);
            assert(pack.count_GET() == (char)56523);
        });
        DemoDevice.MISSION_COUNT p44 = LoopBackDemoChannel.new_MISSION_COUNT();
        PH.setPack(p44);
        p44.target_component_SET((char)58) ;
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p44.target_system_SET((char)6) ;
        p44.count_SET((char)56523) ;
        LoopBackDemoChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)167);
            assert(pack.target_system_GET() == (char)130);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        });
        DemoDevice.MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.new_MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_system_SET((char)130) ;
        p45.target_component_SET((char)167) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        LoopBackDemoChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)25587);
        });
        DemoDevice.MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.new_MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)25587) ;
        LoopBackDemoChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_INVALID_SEQUENCE);
            assert(pack.target_component_GET() == (char)18);
            assert(pack.target_system_GET() == (char)14);
        });
        DemoDevice.MISSION_ACK p47 = LoopBackDemoChannel.new_MISSION_ACK();
        PH.setPack(p47);
        p47.target_system_SET((char)14) ;
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_INVALID_SEQUENCE) ;
        p47.target_component_SET((char)18) ;
        LoopBackDemoChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.altitude_GET() == -521783269);
            assert(pack.longitude_GET() == 771797581);
            assert(pack.latitude_GET() == -1295744397);
            assert(pack.target_system_GET() == (char)121);
            assert(pack.time_usec_TRY(ph) == 4864698788788643617L);
        });
        DemoDevice.SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.new_SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.time_usec_SET(4864698788788643617L, PH) ;
        p48.altitude_SET(-521783269) ;
        p48.target_system_SET((char)121) ;
        p48.latitude_SET(-1295744397) ;
        p48.longitude_SET(771797581) ;
        LoopBackDemoChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == 954842047);
            assert(pack.latitude_GET() == -386879205);
            assert(pack.time_usec_TRY(ph) == 1453204992025311892L);
            assert(pack.altitude_GET() == 1895469226);
        });
        DemoDevice.GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.new_GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.longitude_SET(954842047) ;
        p49.time_usec_SET(1453204992025311892L, PH) ;
        p49.altitude_SET(1895469226) ;
        p49.latitude_SET(-386879205) ;
        LoopBackDemoChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.scale_GET() == 9.184089E37F);
            assert(pack.parameter_rc_channel_index_GET() == (char)170);
            assert(pack.param_id_LEN(ph) == 3);
            assert(pack.param_id_TRY(ph).equals("ank"));
            assert(pack.param_value_min_GET() == 2.3817567E38F);
            assert(pack.param_index_GET() == (short) -19583);
            assert(pack.param_value0_GET() == 1.9428512E38F);
            assert(pack.target_system_GET() == (char)175);
            assert(pack.param_value_max_GET() == -2.392089E38F);
            assert(pack.target_component_GET() == (char)119);
        });
        DemoDevice.PARAM_MAP_RC p50 = LoopBackDemoChannel.new_PARAM_MAP_RC();
        PH.setPack(p50);
        p50.parameter_rc_channel_index_SET((char)170) ;
        p50.param_value0_SET(1.9428512E38F) ;
        p50.param_value_max_SET(-2.392089E38F) ;
        p50.target_system_SET((char)175) ;
        p50.param_id_SET("ank", PH) ;
        p50.target_component_SET((char)119) ;
        p50.param_value_min_SET(2.3817567E38F) ;
        p50.param_index_SET((short) -19583) ;
        p50.scale_SET(9.184089E37F) ;
        LoopBackDemoChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_system_GET() == (char)216);
            assert(pack.target_component_GET() == (char)149);
            assert(pack.seq_GET() == (char)30633);
        });
        DemoDevice.MISSION_REQUEST_INT p51 = LoopBackDemoChannel.new_MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p51.target_component_SET((char)149) ;
        p51.target_system_SET((char)216) ;
        p51.seq_SET((char)30633) ;
        LoopBackDemoChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p1x_GET() == -2.2969131E38F);
            assert(pack.p1z_GET() == 1.2497938E38F);
            assert(pack.p1y_GET() == 6.817665E37F);
            assert(pack.target_system_GET() == (char)208);
            assert(pack.target_component_GET() == (char)244);
            assert(pack.p2y_GET() == 4.0409606E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.p2x_GET() == -7.901109E37F);
            assert(pack.p2z_GET() == 1.5501774E38F);
        });
        DemoDevice.SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.new_SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p54.p1z_SET(1.2497938E38F) ;
        p54.target_system_SET((char)208) ;
        p54.p2y_SET(4.0409606E37F) ;
        p54.p1y_SET(6.817665E37F) ;
        p54.target_component_SET((char)244) ;
        p54.p1x_SET(-2.2969131E38F) ;
        p54.p2x_SET(-7.901109E37F) ;
        p54.p2z_SET(1.5501774E38F) ;
        LoopBackDemoChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p1y_GET() == 2.9035352E37F);
            assert(pack.p2z_GET() == -3.4905746E37F);
            assert(pack.p2x_GET() == -1.249726E38F);
            assert(pack.p2y_GET() == -5.0253787E37F);
            assert(pack.p1x_GET() == 4.2052653E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.p1z_GET() == -2.8293045E38F);
        });
        DemoDevice.SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.new_SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p2z_SET(-3.4905746E37F) ;
        p55.p1z_SET(-2.8293045E38F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p55.p2x_SET(-1.249726E38F) ;
        p55.p2y_SET(-5.0253787E37F) ;
        p55.p1x_SET(4.2052653E37F) ;
        p55.p1y_SET(2.9035352E37F) ;
        LoopBackDemoChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.rollspeed_GET() == 2.8912104E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-1.9728207E38F, -4.4519287E36F, 2.1160747E38F, 2.5654944E37F, -1.8038307E38F, -1.1260765E38F, 2.4285835E37F, -2.7778212E38F, 3.398049E38F}));
            assert(pack.yawspeed_GET() == 3.1937096E38F);
            assert(pack.pitchspeed_GET() == -5.959213E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.009326E38F, -2.691321E38F, -2.2592613E38F, -3.380157E38F}));
            assert(pack.time_usec_GET() == 2966830606060381511L);
        });
        DemoDevice.ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.pitchspeed_SET(-5.959213E37F) ;
        p61.yawspeed_SET(3.1937096E38F) ;
        p61.time_usec_SET(2966830606060381511L) ;
        p61.rollspeed_SET(2.8912104E38F) ;
        p61.covariance_SET(new float[] {-1.9728207E38F, -4.4519287E36F, 2.1160747E38F, 2.5654944E37F, -1.8038307E38F, -1.1260765E38F, 2.4285835E37F, -2.7778212E38F, 3.398049E38F}, 0) ;
        p61.q_SET(new float[] {1.009326E38F, -2.691321E38F, -2.2592613E38F, -3.380157E38F}, 0) ;
        LoopBackDemoChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.nav_bearing_GET() == (short) -18563);
            assert(pack.aspd_error_GET() == 3.643869E36F);
            assert(pack.xtrack_error_GET() == 5.690533E37F);
            assert(pack.nav_roll_GET() == -1.7465098E38F);
            assert(pack.alt_error_GET() == 1.915104E38F);
            assert(pack.wp_dist_GET() == (char)46663);
            assert(pack.nav_pitch_GET() == -2.6696767E37F);
            assert(pack.target_bearing_GET() == (short) -1998);
        });
        DemoDevice.NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.new_NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.wp_dist_SET((char)46663) ;
        p62.nav_bearing_SET((short) -18563) ;
        p62.target_bearing_SET((short) -1998) ;
        p62.aspd_error_SET(3.643869E36F) ;
        p62.alt_error_SET(1.915104E38F) ;
        p62.nav_pitch_SET(-2.6696767E37F) ;
        p62.xtrack_error_SET(5.690533E37F) ;
        p62.nav_roll_SET(-1.7465098E38F) ;
        LoopBackDemoChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 3398190574979062884L);
            assert(pack.lat_GET() == 1001151183);
            assert(pack.relative_alt_GET() == 2073300002);
            assert(pack.vz_GET() == 2.3943072E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-2.9597864E36F, 1.9588202E38F, -1.7541808E38F, -1.9672978E38F, 3.264891E38F, 1.6762156E38F, 1.3944294E38F, -8.778361E37F, 1.5919102E38F, 2.4930984E38F, -2.1144057E38F, -1.831767E38F, -1.3728055E38F, -1.7980985E38F, -1.873304E38F, 2.7363964E38F, 2.393178E37F, 2.86208E38F, -4.115663E37F, -2.695799E38F, -2.8696529E38F, -2.7549135E38F, 5.0250075E37F, -2.17425E38F, -7.633149E37F, -8.659269E37F, -3.140689E38F, -2.6233788E38F, -3.3753866E38F, -7.1653646E36F, 1.235787E38F, 1.7272427E38F, 2.3291628E38F, -1.5290546E38F, 3.1887071E37F, 2.7545979E38F}));
            assert(pack.alt_GET() == 339545305);
            assert(pack.vx_GET() == 1.1397266E38F);
            assert(pack.lon_GET() == -1821563058);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
            assert(pack.vy_GET() == 3.2930257E38F);
        });
        DemoDevice.GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.lat_SET(1001151183) ;
        p63.vx_SET(1.1397266E38F) ;
        p63.lon_SET(-1821563058) ;
        p63.time_usec_SET(3398190574979062884L) ;
        p63.vz_SET(2.3943072E38F) ;
        p63.alt_SET(339545305) ;
        p63.relative_alt_SET(2073300002) ;
        p63.vy_SET(3.2930257E38F) ;
        p63.covariance_SET(new float[] {-2.9597864E36F, 1.9588202E38F, -1.7541808E38F, -1.9672978E38F, 3.264891E38F, 1.6762156E38F, 1.3944294E38F, -8.778361E37F, 1.5919102E38F, 2.4930984E38F, -2.1144057E38F, -1.831767E38F, -1.3728055E38F, -1.7980985E38F, -1.873304E38F, 2.7363964E38F, 2.393178E37F, 2.86208E38F, -4.115663E37F, -2.695799E38F, -2.8696529E38F, -2.7549135E38F, 5.0250075E37F, -2.17425E38F, -7.633149E37F, -8.659269E37F, -3.140689E38F, -2.6233788E38F, -3.3753866E38F, -7.1653646E36F, 1.235787E38F, 1.7272427E38F, 2.3291628E38F, -1.5290546E38F, 3.1887071E37F, 2.7545979E38F}, 0) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS) ;
        LoopBackDemoChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 5.3329883E37F);
            assert(pack.y_GET() == 1.5053172E38F);
            assert(pack.az_GET() == -5.689659E37F);
            assert(pack.vz_GET() == -2.683299E38F);
            assert(pack.vy_GET() == -2.790002E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {2.4119424E38F, -1.9907073E38F, -1.2743609E38F, -1.753276E38F, 2.7232976E38F, 2.8318749E38F, -2.0618866E38F, -2.577602E38F, -9.255174E37F, -1.9577464E38F, -2.1083686E38F, 2.1215204E38F, -1.4819825E38F, 1.4811685E38F, -5.3678994E37F, -5.157518E37F, 2.7644506E38F, 2.1900331E38F, -1.5994918E38F, -4.6191555E37F, 1.9366089E38F, -1.4753655E38F, 3.166061E38F, 2.0311276E38F, 2.7836106E38F, 1.4261742E38F, -1.4324717E38F, -1.9494845E37F, 3.0410528E38F, 2.9543325E37F, -1.2800404E38F, -2.9262339E38F, -2.994354E38F, -8.051261E37F, -3.1046449E38F, -1.6965289E38F, -5.6194763E37F, -4.4054243E36F, 1.0963552E38F, 1.0156842E38F, 3.1984508E38F, -2.0811551E38F, -2.2548385E38F, -4.948332E37F, 3.338598E37F}));
            assert(pack.time_usec_GET() == 8153926662231606815L);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE);
            assert(pack.ax_GET() == 2.7494035E38F);
            assert(pack.vx_GET() == 8.3684396E37F);
            assert(pack.z_GET() == -2.0032816E38F);
            assert(pack.ay_GET() == -2.7167373E38F);
        });
        DemoDevice.LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.covariance_SET(new float[] {2.4119424E38F, -1.9907073E38F, -1.2743609E38F, -1.753276E38F, 2.7232976E38F, 2.8318749E38F, -2.0618866E38F, -2.577602E38F, -9.255174E37F, -1.9577464E38F, -2.1083686E38F, 2.1215204E38F, -1.4819825E38F, 1.4811685E38F, -5.3678994E37F, -5.157518E37F, 2.7644506E38F, 2.1900331E38F, -1.5994918E38F, -4.6191555E37F, 1.9366089E38F, -1.4753655E38F, 3.166061E38F, 2.0311276E38F, 2.7836106E38F, 1.4261742E38F, -1.4324717E38F, -1.9494845E37F, 3.0410528E38F, 2.9543325E37F, -1.2800404E38F, -2.9262339E38F, -2.994354E38F, -8.051261E37F, -3.1046449E38F, -1.6965289E38F, -5.6194763E37F, -4.4054243E36F, 1.0963552E38F, 1.0156842E38F, 3.1984508E38F, -2.0811551E38F, -2.2548385E38F, -4.948332E37F, 3.338598E37F}, 0) ;
        p64.time_usec_SET(8153926662231606815L) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE) ;
        p64.ay_SET(-2.7167373E38F) ;
        p64.vy_SET(-2.790002E38F) ;
        p64.y_SET(1.5053172E38F) ;
        p64.vx_SET(8.3684396E37F) ;
        p64.ax_SET(2.7494035E38F) ;
        p64.az_SET(-5.689659E37F) ;
        p64.vz_SET(-2.683299E38F) ;
        p64.x_SET(5.3329883E37F) ;
        p64.z_SET(-2.0032816E38F) ;
        LoopBackDemoChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chancount_GET() == (char)173);
            assert(pack.chan15_raw_GET() == (char)51292);
            assert(pack.chan13_raw_GET() == (char)11751);
            assert(pack.chan9_raw_GET() == (char)64875);
            assert(pack.chan6_raw_GET() == (char)55964);
            assert(pack.chan8_raw_GET() == (char)14648);
            assert(pack.chan12_raw_GET() == (char)62450);
            assert(pack.chan11_raw_GET() == (char)24390);
            assert(pack.chan10_raw_GET() == (char)36897);
            assert(pack.chan17_raw_GET() == (char)56712);
            assert(pack.chan18_raw_GET() == (char)32299);
            assert(pack.chan16_raw_GET() == (char)44324);
            assert(pack.chan2_raw_GET() == (char)3012);
            assert(pack.chan1_raw_GET() == (char)21914);
            assert(pack.chan5_raw_GET() == (char)15814);
            assert(pack.chan7_raw_GET() == (char)895);
            assert(pack.rssi_GET() == (char)147);
            assert(pack.chan3_raw_GET() == (char)1943);
            assert(pack.chan14_raw_GET() == (char)13765);
            assert(pack.chan4_raw_GET() == (char)51720);
            assert(pack.time_boot_ms_GET() == 344435505L);
        });
        DemoDevice.RC_CHANNELS p65 = LoopBackDemoChannel.new_RC_CHANNELS();
        PH.setPack(p65);
        p65.rssi_SET((char)147) ;
        p65.chan7_raw_SET((char)895) ;
        p65.chan15_raw_SET((char)51292) ;
        p65.chan18_raw_SET((char)32299) ;
        p65.chan8_raw_SET((char)14648) ;
        p65.chan6_raw_SET((char)55964) ;
        p65.time_boot_ms_SET(344435505L) ;
        p65.chancount_SET((char)173) ;
        p65.chan11_raw_SET((char)24390) ;
        p65.chan2_raw_SET((char)3012) ;
        p65.chan5_raw_SET((char)15814) ;
        p65.chan10_raw_SET((char)36897) ;
        p65.chan17_raw_SET((char)56712) ;
        p65.chan3_raw_SET((char)1943) ;
        p65.chan9_raw_SET((char)64875) ;
        p65.chan4_raw_SET((char)51720) ;
        p65.chan14_raw_SET((char)13765) ;
        p65.chan16_raw_SET((char)44324) ;
        p65.chan13_raw_SET((char)11751) ;
        p65.chan12_raw_SET((char)62450) ;
        p65.chan1_raw_SET((char)21914) ;
        LoopBackDemoChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)32);
            assert(pack.target_component_GET() == (char)36);
            assert(pack.req_message_rate_GET() == (char)52231);
            assert(pack.start_stop_GET() == (char)158);
            assert(pack.req_stream_id_GET() == (char)75);
        });
        DemoDevice.REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.new_REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.req_stream_id_SET((char)75) ;
        p66.target_component_SET((char)36) ;
        p66.target_system_SET((char)32) ;
        p66.start_stop_SET((char)158) ;
        p66.req_message_rate_SET((char)52231) ;
        LoopBackDemoChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.on_off_GET() == (char)109);
            assert(pack.message_rate_GET() == (char)50306);
            assert(pack.stream_id_GET() == (char)156);
        });
        DemoDevice.DATA_STREAM p67 = LoopBackDemoChannel.new_DATA_STREAM();
        PH.setPack(p67);
        p67.stream_id_SET((char)156) ;
        p67.on_off_SET((char)109) ;
        p67.message_rate_SET((char)50306) ;
        LoopBackDemoChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == (short) -7230);
            assert(pack.y_GET() == (short) -13957);
            assert(pack.r_GET() == (short) -9161);
            assert(pack.target_GET() == (char)188);
            assert(pack.x_GET() == (short) -16909);
            assert(pack.buttons_GET() == (char)15042);
        });
        DemoDevice.MANUAL_CONTROL p69 = LoopBackDemoChannel.new_MANUAL_CONTROL();
        PH.setPack(p69);
        p69.y_SET((short) -13957) ;
        p69.r_SET((short) -9161) ;
        p69.z_SET((short) -7230) ;
        p69.buttons_SET((char)15042) ;
        p69.x_SET((short) -16909) ;
        p69.target_SET((char)188) ;
        LoopBackDemoChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan4_raw_GET() == (char)2067);
            assert(pack.chan2_raw_GET() == (char)7480);
            assert(pack.target_system_GET() == (char)12);
            assert(pack.chan8_raw_GET() == (char)600);
            assert(pack.chan7_raw_GET() == (char)40601);
            assert(pack.chan1_raw_GET() == (char)1754);
            assert(pack.target_component_GET() == (char)137);
            assert(pack.chan5_raw_GET() == (char)42456);
            assert(pack.chan3_raw_GET() == (char)11863);
            assert(pack.chan6_raw_GET() == (char)55051);
        });
        DemoDevice.RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.new_RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan1_raw_SET((char)1754) ;
        p70.target_system_SET((char)12) ;
        p70.chan3_raw_SET((char)11863) ;
        p70.chan8_raw_SET((char)600) ;
        p70.chan7_raw_SET((char)40601) ;
        p70.chan2_raw_SET((char)7480) ;
        p70.chan6_raw_SET((char)55051) ;
        p70.chan5_raw_SET((char)42456) ;
        p70.chan4_raw_SET((char)2067) ;
        p70.target_component_SET((char)137) ;
        LoopBackDemoChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.param4_GET() == 1.0513946E38F);
            assert(pack.y_GET() == -1191212641);
            assert(pack.param2_GET() == -2.3992074E38F);
            assert(pack.x_GET() == -1330667441);
            assert(pack.target_component_GET() == (char)254);
            assert(pack.param1_GET() == -3.1740647E38F);
            assert(pack.autocontinue_GET() == (char)205);
            assert(pack.param3_GET() == -3.156343E37F);
            assert(pack.target_system_GET() == (char)229);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            assert(pack.z_GET() == -6.4780683E37F);
            assert(pack.seq_GET() == (char)24484);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.current_GET() == (char)105);
        });
        DemoDevice.MISSION_ITEM_INT p73 = LoopBackDemoChannel.new_MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p73.target_system_SET((char)229) ;
        p73.param2_SET(-2.3992074E38F) ;
        p73.x_SET(-1330667441) ;
        p73.y_SET(-1191212641) ;
        p73.z_SET(-6.4780683E37F) ;
        p73.param3_SET(-3.156343E37F) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p73.seq_SET((char)24484) ;
        p73.current_SET((char)105) ;
        p73.param1_SET(-3.1740647E38F) ;
        p73.command_SET(MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE) ;
        p73.autocontinue_SET((char)205) ;
        p73.target_component_SET((char)254) ;
        p73.param4_SET(1.0513946E38F) ;
        LoopBackDemoChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.climb_GET() == 6.2100885E36F);
            assert(pack.alt_GET() == -2.4592101E38F);
            assert(pack.airspeed_GET() == -1.0157088E38F);
            assert(pack.throttle_GET() == (char)41409);
            assert(pack.heading_GET() == (short) -11912);
            assert(pack.groundspeed_GET() == -2.3073888E37F);
        });
        DemoDevice.VFR_HUD p74 = LoopBackDemoChannel.new_VFR_HUD();
        PH.setPack(p74);
        p74.throttle_SET((char)41409) ;
        p74.heading_SET((short) -11912) ;
        p74.groundspeed_SET(-2.3073888E37F) ;
        p74.climb_SET(6.2100885E36F) ;
        p74.alt_SET(-2.4592101E38F) ;
        p74.airspeed_SET(-1.0157088E38F) ;
        LoopBackDemoChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)112);
            assert(pack.param1_GET() == 3.0815012E38F);
            assert(pack.current_GET() == (char)159);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.x_GET() == 1604362362);
            assert(pack.param2_GET() == -3.3357307E38F);
            assert(pack.autocontinue_GET() == (char)77);
            assert(pack.z_GET() == 2.0032354E38F);
            assert(pack.param4_GET() == 2.7943863E38F);
            assert(pack.param3_GET() == -2.550603E38F);
            assert(pack.y_GET() == 294948297);
            assert(pack.target_system_GET() == (char)13);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_USER_4);
        });
        DemoDevice.COMMAND_INT p75 = LoopBackDemoChannel.new_COMMAND_INT();
        PH.setPack(p75);
        p75.current_SET((char)159) ;
        p75.target_component_SET((char)112) ;
        p75.x_SET(1604362362) ;
        p75.autocontinue_SET((char)77) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p75.param4_SET(2.7943863E38F) ;
        p75.command_SET(MAV_CMD.MAV_CMD_USER_4) ;
        p75.z_SET(2.0032354E38F) ;
        p75.param3_SET(-2.550603E38F) ;
        p75.param1_SET(3.0815012E38F) ;
        p75.y_SET(294948297) ;
        p75.target_system_SET((char)13) ;
        p75.param2_SET(-3.3357307E38F) ;
        LoopBackDemoChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)96);
            assert(pack.confirmation_GET() == (char)173);
            assert(pack.target_system_GET() == (char)114);
            assert(pack.param5_GET() == 3.6195915E37F);
            assert(pack.param6_GET() == 2.563661E38F);
            assert(pack.param2_GET() == -1.6572118E38F);
            assert(pack.param1_GET() == 2.303612E38F);
            assert(pack.param7_GET() == 1.7550836E38F);
            assert(pack.param3_GET() == 1.6069659E38F);
            assert(pack.param4_GET() == 2.7806871E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_LAND_START);
        });
        DemoDevice.COMMAND_LONG p76 = LoopBackDemoChannel.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.target_system_SET((char)114) ;
        p76.confirmation_SET((char)173) ;
        p76.param5_SET(3.6195915E37F) ;
        p76.target_component_SET((char)96) ;
        p76.param4_SET(2.7806871E38F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_DO_LAND_START) ;
        p76.param7_SET(1.7550836E38F) ;
        p76.param2_SET(-1.6572118E38F) ;
        p76.param3_SET(1.6069659E38F) ;
        p76.param1_SET(2.303612E38F) ;
        p76.param6_SET(2.563661E38F) ;
        LoopBackDemoChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.result_param2_TRY(ph) == 282647117);
            assert(pack.target_component_TRY(ph) == (char)95);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_UNSUPPORTED);
            assert(pack.target_system_TRY(ph) == (char)150);
            assert(pack.progress_TRY(ph) == (char)108);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_LAND);
        });
        DemoDevice.COMMAND_ACK p77 = LoopBackDemoChannel.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.target_component_SET((char)95, PH) ;
        p77.progress_SET((char)108, PH) ;
        p77.target_system_SET((char)150, PH) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_UNSUPPORTED) ;
        p77.command_SET(MAV_CMD.MAV_CMD_NAV_LAND) ;
        p77.result_param2_SET(282647117, PH) ;
        LoopBackDemoChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 383984885L);
            assert(pack.mode_switch_GET() == (char)187);
            assert(pack.yaw_GET() == 1.9922362E38F);
            assert(pack.pitch_GET() == -2.162921E38F);
            assert(pack.manual_override_switch_GET() == (char)111);
            assert(pack.roll_GET() == -5.695899E36F);
            assert(pack.thrust_GET() == -7.14565E37F);
        });
        DemoDevice.MANUAL_SETPOINT p81 = LoopBackDemoChannel.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.time_boot_ms_SET(383984885L) ;
        p81.mode_switch_SET((char)187) ;
        p81.yaw_SET(1.9922362E38F) ;
        p81.roll_SET(-5.695899E36F) ;
        p81.pitch_SET(-2.162921E38F) ;
        p81.thrust_SET(-7.14565E37F) ;
        p81.manual_override_switch_SET((char)111) ;
        LoopBackDemoChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_yaw_rate_GET() == -1.8632523E38F);
            assert(pack.time_boot_ms_GET() == 100357147L);
            assert(pack.thrust_GET() == -1.98548E38F);
            assert(pack.target_component_GET() == (char)28);
            assert(pack.type_mask_GET() == (char)68);
            assert(pack.body_pitch_rate_GET() == -5.223044E37F);
            assert(pack.target_system_GET() == (char)105);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.5147351E37F, 1.0530642E38F, 7.5169647E37F, 3.1758556E38F}));
            assert(pack.body_roll_rate_GET() == -9.635349E37F);
        });
        DemoDevice.SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.type_mask_SET((char)68) ;
        p82.body_pitch_rate_SET(-5.223044E37F) ;
        p82.q_SET(new float[] {1.5147351E37F, 1.0530642E38F, 7.5169647E37F, 3.1758556E38F}, 0) ;
        p82.time_boot_ms_SET(100357147L) ;
        p82.target_system_SET((char)105) ;
        p82.body_yaw_rate_SET(-1.8632523E38F) ;
        p82.thrust_SET(-1.98548E38F) ;
        p82.body_roll_rate_SET(-9.635349E37F) ;
        p82.target_component_SET((char)28) ;
        LoopBackDemoChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.type_mask_GET() == (char)187);
            assert(pack.body_yaw_rate_GET() == 3.3190378E38F);
            assert(pack.thrust_GET() == -1.7578533E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.0188857E38F, -1.2359562E38F, -2.77766E38F, 1.0096381E38F}));
            assert(pack.time_boot_ms_GET() == 1314123523L);
            assert(pack.body_roll_rate_GET() == 7.2476E37F);
            assert(pack.body_pitch_rate_GET() == -2.0033938E38F);
        });
        DemoDevice.ATTITUDE_TARGET p83 = LoopBackDemoChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.body_roll_rate_SET(7.2476E37F) ;
        p83.thrust_SET(-1.7578533E38F) ;
        p83.q_SET(new float[] {-2.0188857E38F, -1.2359562E38F, -2.77766E38F, 1.0096381E38F}, 0) ;
        p83.time_boot_ms_SET(1314123523L) ;
        p83.type_mask_SET((char)187) ;
        p83.body_pitch_rate_SET(-2.0033938E38F) ;
        p83.body_yaw_rate_SET(3.3190378E38F) ;
        LoopBackDemoChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -2.1506709E38F);
            assert(pack.yaw_GET() == 1.1161547E38F);
            assert(pack.time_boot_ms_GET() == 3744603336L);
            assert(pack.target_system_GET() == (char)179);
            assert(pack.vy_GET() == -2.5988768E38F);
            assert(pack.vx_GET() == -1.1759666E38F);
            assert(pack.afz_GET() == -1.737658E38F);
            assert(pack.z_GET() == 2.1734413E38F);
            assert(pack.target_component_GET() == (char)120);
            assert(pack.yaw_rate_GET() == 1.8315355E38F);
            assert(pack.x_GET() == -9.849459E37F);
            assert(pack.afy_GET() == 8.551871E37F);
            assert(pack.vz_GET() == -2.1519925E38F);
            assert(pack.type_mask_GET() == (char)41092);
            assert(pack.afx_GET() == 7.4106E36F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
        });
        DemoDevice.SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.target_system_SET((char)179) ;
        p84.yaw_SET(1.1161547E38F) ;
        p84.y_SET(-2.1506709E38F) ;
        p84.z_SET(2.1734413E38F) ;
        p84.time_boot_ms_SET(3744603336L) ;
        p84.vz_SET(-2.1519925E38F) ;
        p84.afx_SET(7.4106E36F) ;
        p84.target_component_SET((char)120) ;
        p84.vy_SET(-2.5988768E38F) ;
        p84.type_mask_SET((char)41092) ;
        p84.yaw_rate_SET(1.8315355E38F) ;
        p84.afz_SET(-1.737658E38F) ;
        p84.afy_SET(8.551871E37F) ;
        p84.x_SET(-9.849459E37F) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p84.vx_SET(-1.1759666E38F) ;
        LoopBackDemoChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == -2.6509862E38F);
            assert(pack.vz_GET() == 2.1068978E38F);
            assert(pack.afy_GET() == -7.7411305E37F);
            assert(pack.target_system_GET() == (char)146);
            assert(pack.afz_GET() == 3.0166575E38F);
            assert(pack.lat_int_GET() == -2074784703);
            assert(pack.time_boot_ms_GET() == 3548943688L);
            assert(pack.vx_GET() == -2.422327E38F);
            assert(pack.lon_int_GET() == -599189808);
            assert(pack.afx_GET() == -6.644788E37F);
            assert(pack.alt_GET() == -1.5927253E38F);
            assert(pack.target_component_GET() == (char)152);
            assert(pack.yaw_GET() == -1.6169227E38F);
            assert(pack.type_mask_GET() == (char)3242);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.yaw_rate_GET() == 1.5715297E38F);
        });
        DemoDevice.SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.afz_SET(3.0166575E38F) ;
        p86.afx_SET(-6.644788E37F) ;
        p86.type_mask_SET((char)3242) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p86.lon_int_SET(-599189808) ;
        p86.vy_SET(-2.6509862E38F) ;
        p86.yaw_SET(-1.6169227E38F) ;
        p86.target_system_SET((char)146) ;
        p86.vz_SET(2.1068978E38F) ;
        p86.time_boot_ms_SET(3548943688L) ;
        p86.lat_int_SET(-2074784703) ;
        p86.yaw_rate_SET(1.5715297E38F) ;
        p86.afy_SET(-7.7411305E37F) ;
        p86.vx_SET(-2.422327E38F) ;
        p86.alt_SET(-1.5927253E38F) ;
        p86.target_component_SET((char)152) ;
        LoopBackDemoChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.lon_int_GET() == 1863056384);
            assert(pack.type_mask_GET() == (char)22928);
            assert(pack.vz_GET() == 3.0871555E38F);
            assert(pack.alt_GET() == 2.5598146E36F);
            assert(pack.vx_GET() == 1.9022554E38F);
            assert(pack.yaw_rate_GET() == 1.716158E38F);
            assert(pack.lat_int_GET() == -284446766);
            assert(pack.afz_GET() == 2.496174E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.time_boot_ms_GET() == 3319449302L);
            assert(pack.vy_GET() == -1.6376719E38F);
            assert(pack.afx_GET() == -1.2410062E38F);
            assert(pack.afy_GET() == -1.544844E38F);
            assert(pack.yaw_GET() == 1.7227341E38F);
        });
        DemoDevice.POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.afx_SET(-1.2410062E38F) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p87.lat_int_SET(-284446766) ;
        p87.yaw_rate_SET(1.716158E38F) ;
        p87.vy_SET(-1.6376719E38F) ;
        p87.type_mask_SET((char)22928) ;
        p87.time_boot_ms_SET(3319449302L) ;
        p87.afy_SET(-1.544844E38F) ;
        p87.afz_SET(2.496174E38F) ;
        p87.yaw_SET(1.7227341E38F) ;
        p87.lon_int_SET(1863056384) ;
        p87.alt_SET(2.5598146E36F) ;
        p87.vz_SET(3.0871555E38F) ;
        p87.vx_SET(1.9022554E38F) ;
        LoopBackDemoChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2099609995L);
            assert(pack.roll_GET() == 2.0154825E38F);
            assert(pack.z_GET() == -3.1902762E38F);
            assert(pack.pitch_GET() == -2.3760265E38F);
            assert(pack.yaw_GET() == 2.727604E38F);
            assert(pack.x_GET() == -2.8689278E38F);
            assert(pack.y_GET() == 1.6679301E38F);
        });
        DemoDevice.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.z_SET(-3.1902762E38F) ;
        p89.yaw_SET(2.727604E38F) ;
        p89.x_SET(-2.8689278E38F) ;
        p89.y_SET(1.6679301E38F) ;
        p89.pitch_SET(-2.3760265E38F) ;
        p89.time_boot_ms_SET(2099609995L) ;
        p89.roll_SET(2.0154825E38F) ;
        LoopBackDemoChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == (short)11608);
            assert(pack.rollspeed_GET() == 3.1202688E38F);
            assert(pack.vz_GET() == (short) -18062);
            assert(pack.alt_GET() == 1443971537);
            assert(pack.lon_GET() == 62963504);
            assert(pack.lat_GET() == 2144972823);
            assert(pack.time_usec_GET() == 7391921784851965473L);
            assert(pack.pitchspeed_GET() == -8.857238E37F);
            assert(pack.vy_GET() == (short) -9715);
            assert(pack.zacc_GET() == (short) -8275);
            assert(pack.yaw_GET() == 8.429323E37F);
            assert(pack.xacc_GET() == (short) -10908);
            assert(pack.vx_GET() == (short)25311);
            assert(pack.yawspeed_GET() == 9.254704E37F);
            assert(pack.pitch_GET() == 2.7592847E38F);
            assert(pack.roll_GET() == 2.7581087E38F);
        });
        DemoDevice.HIL_STATE p90 = LoopBackDemoChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.lat_SET(2144972823) ;
        p90.rollspeed_SET(3.1202688E38F) ;
        p90.xacc_SET((short) -10908) ;
        p90.zacc_SET((short) -8275) ;
        p90.lon_SET(62963504) ;
        p90.pitchspeed_SET(-8.857238E37F) ;
        p90.vx_SET((short)25311) ;
        p90.yaw_SET(8.429323E37F) ;
        p90.vz_SET((short) -18062) ;
        p90.yacc_SET((short)11608) ;
        p90.yawspeed_SET(9.254704E37F) ;
        p90.vy_SET((short) -9715) ;
        p90.time_usec_SET(7391921784851965473L) ;
        p90.alt_SET(1443971537) ;
        p90.roll_SET(2.7581087E38F) ;
        p90.pitch_SET(2.7592847E38F) ;
        LoopBackDemoChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.throttle_GET() == 2.930152E38F);
            assert(pack.time_usec_GET() == 3379373253764794242L);
            assert(pack.aux1_GET() == 2.5214118E38F);
            assert(pack.yaw_rudder_GET() == -1.6722115E38F);
            assert(pack.pitch_elevator_GET() == -5.6705033E37F);
            assert(pack.roll_ailerons_GET() == -1.2006314E38F);
            assert(pack.aux2_GET() == -1.855376E38F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_GUIDED_DISARMED);
            assert(pack.nav_mode_GET() == (char)242);
            assert(pack.aux4_GET() == -2.6085984E38F);
            assert(pack.aux3_GET() == -2.1038617E38F);
        });
        DemoDevice.HIL_CONTROLS p91 = LoopBackDemoChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.throttle_SET(2.930152E38F) ;
        p91.time_usec_SET(3379373253764794242L) ;
        p91.pitch_elevator_SET(-5.6705033E37F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_GUIDED_DISARMED) ;
        p91.roll_ailerons_SET(-1.2006314E38F) ;
        p91.aux2_SET(-1.855376E38F) ;
        p91.yaw_rudder_SET(-1.6722115E38F) ;
        p91.aux1_SET(2.5214118E38F) ;
        p91.aux4_SET(-2.6085984E38F) ;
        p91.nav_mode_SET((char)242) ;
        p91.aux3_SET(-2.1038617E38F) ;
        LoopBackDemoChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan11_raw_GET() == (char)63281);
            assert(pack.chan12_raw_GET() == (char)9196);
            assert(pack.chan3_raw_GET() == (char)59365);
            assert(pack.time_usec_GET() == 7333281174624028448L);
            assert(pack.chan1_raw_GET() == (char)41545);
            assert(pack.chan7_raw_GET() == (char)63161);
            assert(pack.chan2_raw_GET() == (char)28924);
            assert(pack.chan4_raw_GET() == (char)55709);
            assert(pack.chan8_raw_GET() == (char)55400);
            assert(pack.chan5_raw_GET() == (char)7638);
            assert(pack.chan10_raw_GET() == (char)56786);
            assert(pack.rssi_GET() == (char)235);
            assert(pack.chan6_raw_GET() == (char)6533);
            assert(pack.chan9_raw_GET() == (char)54415);
        });
        DemoDevice.HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan1_raw_SET((char)41545) ;
        p92.time_usec_SET(7333281174624028448L) ;
        p92.chan3_raw_SET((char)59365) ;
        p92.chan11_raw_SET((char)63281) ;
        p92.chan9_raw_SET((char)54415) ;
        p92.rssi_SET((char)235) ;
        p92.chan2_raw_SET((char)28924) ;
        p92.chan4_raw_SET((char)55709) ;
        p92.chan5_raw_SET((char)7638) ;
        p92.chan6_raw_SET((char)6533) ;
        p92.chan10_raw_SET((char)56786) ;
        p92.chan12_raw_SET((char)9196) ;
        p92.chan7_raw_SET((char)63161) ;
        p92.chan8_raw_SET((char)55400) ;
        LoopBackDemoChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == 4451631455888781L);
            assert(pack.time_usec_GET() == 4733986153206004091L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-1.7962627E38F, -3.037487E38F, 2.4348968E38F, -1.080599E38F, 2.6327063E38F, 1.994092E38F, 9.190415E35F, 3.2915193E38F, -7.709768E37F, 2.6637144E38F, 3.9829473E37F, -9.496322E37F, 3.7149794E37F, -8.627708E37F, 2.5987748E38F, -2.876478E38F}));
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_GUIDED_ARMED);
        });
        DemoDevice.HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.controls_SET(new float[] {-1.7962627E38F, -3.037487E38F, 2.4348968E38F, -1.080599E38F, 2.6327063E38F, 1.994092E38F, 9.190415E35F, 3.2915193E38F, -7.709768E37F, 2.6637144E38F, 3.9829473E37F, -9.496322E37F, 3.7149794E37F, -8.627708E37F, 2.5987748E38F, -2.876478E38F}, 0) ;
        p93.time_usec_SET(4733986153206004091L) ;
        p93.flags_SET(4451631455888781L) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_GUIDED_ARMED) ;
        LoopBackDemoChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.flow_x_GET() == (short)15288);
            assert(pack.ground_distance_GET() == 1.6204403E38F);
            assert(pack.flow_rate_y_TRY(ph) == 1.1168547E38F);
            assert(pack.sensor_id_GET() == (char)253);
            assert(pack.flow_comp_m_x_GET() == 3.3082287E38F);
            assert(pack.flow_y_GET() == (short)26023);
            assert(pack.flow_rate_x_TRY(ph) == 1.6399329E38F);
            assert(pack.flow_comp_m_y_GET() == -6.842762E36F);
            assert(pack.time_usec_GET() == 8250580598260150743L);
            assert(pack.quality_GET() == (char)208);
        });
        DemoDevice.OPTICAL_FLOW p100 = LoopBackDemoChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.flow_comp_m_x_SET(3.3082287E38F) ;
        p100.flow_rate_x_SET(1.6399329E38F, PH) ;
        p100.quality_SET((char)208) ;
        p100.ground_distance_SET(1.6204403E38F) ;
        p100.flow_rate_y_SET(1.1168547E38F, PH) ;
        p100.flow_x_SET((short)15288) ;
        p100.sensor_id_SET((char)253) ;
        p100.time_usec_SET(8250580598260150743L) ;
        p100.flow_comp_m_y_SET(-6.842762E36F) ;
        p100.flow_y_SET((short)26023) ;
        LoopBackDemoChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 2.4497693E38F);
            assert(pack.roll_GET() == -3.2966284E38F);
            assert(pack.pitch_GET() == 2.8535561E38F);
            assert(pack.yaw_GET() == -3.162006E37F);
            assert(pack.y_GET() == 3.118164E38F);
            assert(pack.x_GET() == -2.1709845E37F);
            assert(pack.usec_GET() == 5911607799823774975L);
        });
        DemoDevice.GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.roll_SET(-3.2966284E38F) ;
        p101.pitch_SET(2.8535561E38F) ;
        p101.yaw_SET(-3.162006E37F) ;
        p101.x_SET(-2.1709845E37F) ;
        p101.y_SET(3.118164E38F) ;
        p101.usec_SET(5911607799823774975L) ;
        p101.z_SET(2.4497693E38F) ;
        LoopBackDemoChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 7.8986523E37F);
            assert(pack.pitch_GET() == -1.2426214E38F);
            assert(pack.usec_GET() == 2497163179037513615L);
            assert(pack.roll_GET() == -1.4961044E38F);
            assert(pack.z_GET() == -5.0278613E37F);
            assert(pack.y_GET() == -5.2103416E37F);
            assert(pack.yaw_GET() == -1.2514885E38F);
        });
        DemoDevice.VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.usec_SET(2497163179037513615L) ;
        p102.roll_SET(-1.4961044E38F) ;
        p102.x_SET(7.8986523E37F) ;
        p102.yaw_SET(-1.2514885E38F) ;
        p102.y_SET(-5.2103416E37F) ;
        p102.pitch_SET(-1.2426214E38F) ;
        p102.z_SET(-5.0278613E37F) ;
        LoopBackDemoChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 2809275460453344891L);
            assert(pack.x_GET() == 3.8567238E37F);
            assert(pack.z_GET() == -2.2025583E38F);
            assert(pack.y_GET() == 2.2510944E37F);
        });
        DemoDevice.VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.z_SET(-2.2025583E38F) ;
        p103.y_SET(2.2510944E37F) ;
        p103.usec_SET(2809275460453344891L) ;
        p103.x_SET(3.8567238E37F) ;
        LoopBackDemoChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == -5.9374057E37F);
            assert(pack.yaw_GET() == 4.7779404E37F);
            assert(pack.usec_GET() == 3174937446771403025L);
            assert(pack.x_GET() == 6.8566506E37F);
            assert(pack.pitch_GET() == 7.341056E37F);
            assert(pack.z_GET() == 1.5838999E38F);
            assert(pack.y_GET() == 3.2485496E38F);
        });
        DemoDevice.VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.usec_SET(3174937446771403025L) ;
        p104.roll_SET(-5.9374057E37F) ;
        p104.yaw_SET(4.7779404E37F) ;
        p104.y_SET(3.2485496E38F) ;
        p104.x_SET(6.8566506E37F) ;
        p104.z_SET(1.5838999E38F) ;
        p104.pitch_SET(7.341056E37F) ;
        LoopBackDemoChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == 1.57801E38F);
            assert(pack.xmag_GET() == 2.0745297E38F);
            assert(pack.diff_pressure_GET() == 3.0802954E38F);
            assert(pack.abs_pressure_GET() == -3.3655633E38F);
            assert(pack.fields_updated_GET() == (char)61101);
            assert(pack.yacc_GET() == 1.3550168E38F);
            assert(pack.zmag_GET() == -4.7240693E37F);
            assert(pack.ygyro_GET() == 3.3089559E38F);
            assert(pack.zacc_GET() == -9.987387E37F);
            assert(pack.ymag_GET() == 8.0766644E37F);
            assert(pack.xgyro_GET() == -2.6228952E38F);
            assert(pack.temperature_GET() == 1.2975188E38F);
            assert(pack.pressure_alt_GET() == -9.444385E37F);
            assert(pack.xacc_GET() == 4.9491305E37F);
            assert(pack.time_usec_GET() == 1126195895902662896L);
        });
        DemoDevice.HIGHRES_IMU p105 = LoopBackDemoChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.pressure_alt_SET(-9.444385E37F) ;
        p105.ymag_SET(8.0766644E37F) ;
        p105.fields_updated_SET((char)61101) ;
        p105.zmag_SET(-4.7240693E37F) ;
        p105.yacc_SET(1.3550168E38F) ;
        p105.abs_pressure_SET(-3.3655633E38F) ;
        p105.diff_pressure_SET(3.0802954E38F) ;
        p105.ygyro_SET(3.3089559E38F) ;
        p105.xacc_SET(4.9491305E37F) ;
        p105.temperature_SET(1.2975188E38F) ;
        p105.zacc_SET(-9.987387E37F) ;
        p105.zgyro_SET(1.57801E38F) ;
        p105.time_usec_SET(1126195895902662896L) ;
        p105.xgyro_SET(-2.6228952E38F) ;
        p105.xmag_SET(2.0745297E38F) ;
        LoopBackDemoChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.integrated_ygyro_GET() == -3.0850344E38F);
            assert(pack.integrated_y_GET() == 1.4126376E38F);
            assert(pack.time_delta_distance_us_GET() == 511315125L);
            assert(pack.integrated_zgyro_GET() == 3.3356368E38F);
            assert(pack.integration_time_us_GET() == 2042814300L);
            assert(pack.sensor_id_GET() == (char)25);
            assert(pack.integrated_x_GET() == 4.974038E37F);
            assert(pack.temperature_GET() == (short) -16455);
            assert(pack.distance_GET() == -2.08507E38F);
            assert(pack.integrated_xgyro_GET() == -1.8289766E38F);
            assert(pack.time_usec_GET() == 4804024153419969155L);
            assert(pack.quality_GET() == (char)174);
        });
        DemoDevice.OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.time_delta_distance_us_SET(511315125L) ;
        p106.quality_SET((char)174) ;
        p106.integrated_y_SET(1.4126376E38F) ;
        p106.integrated_xgyro_SET(-1.8289766E38F) ;
        p106.integration_time_us_SET(2042814300L) ;
        p106.integrated_zgyro_SET(3.3356368E38F) ;
        p106.integrated_ygyro_SET(-3.0850344E38F) ;
        p106.distance_SET(-2.08507E38F) ;
        p106.temperature_SET((short) -16455) ;
        p106.sensor_id_SET((char)25) ;
        p106.time_usec_SET(4804024153419969155L) ;
        p106.integrated_x_SET(4.974038E37F) ;
        LoopBackDemoChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.fields_updated_GET() == 2539395534L);
            assert(pack.xgyro_GET() == 2.2061402E38F);
            assert(pack.time_usec_GET() == 5916705571344669869L);
            assert(pack.zacc_GET() == 1.972135E38F);
            assert(pack.yacc_GET() == -8.2737666E36F);
            assert(pack.xmag_GET() == -2.1236953E38F);
            assert(pack.ymag_GET() == -2.8405646E37F);
            assert(pack.abs_pressure_GET() == 2.0928303E38F);
            assert(pack.diff_pressure_GET() == 7.8687E37F);
            assert(pack.ygyro_GET() == 1.9838809E38F);
            assert(pack.pressure_alt_GET() == 3.202006E38F);
            assert(pack.xacc_GET() == -2.2969985E38F);
            assert(pack.temperature_GET() == 2.207778E38F);
            assert(pack.zmag_GET() == 3.0500972E38F);
            assert(pack.zgyro_GET() == -3.044874E38F);
        });
        DemoDevice.HIL_SENSOR p107 = LoopBackDemoChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.time_usec_SET(5916705571344669869L) ;
        p107.zgyro_SET(-3.044874E38F) ;
        p107.yacc_SET(-8.2737666E36F) ;
        p107.temperature_SET(2.207778E38F) ;
        p107.zacc_SET(1.972135E38F) ;
        p107.ygyro_SET(1.9838809E38F) ;
        p107.diff_pressure_SET(7.8687E37F) ;
        p107.fields_updated_SET(2539395534L) ;
        p107.xmag_SET(-2.1236953E38F) ;
        p107.xacc_SET(-2.2969985E38F) ;
        p107.abs_pressure_SET(2.0928303E38F) ;
        p107.ymag_SET(-2.8405646E37F) ;
        p107.pressure_alt_SET(3.202006E38F) ;
        p107.zmag_SET(3.0500972E38F) ;
        p107.xgyro_SET(2.2061402E38F) ;
        LoopBackDemoChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == 1.1557427E38F);
            assert(pack.vn_GET() == 2.0300867E38F);
            assert(pack.std_dev_vert_GET() == -2.2254128E38F);
            assert(pack.lat_GET() == -1.7045804E38F);
            assert(pack.std_dev_horz_GET() == -2.4865356E38F);
            assert(pack.roll_GET() == -1.8725372E38F);
            assert(pack.q3_GET() == -1.8786833E38F);
            assert(pack.xgyro_GET() == -6.5317335E37F);
            assert(pack.xacc_GET() == 2.5480674E38F);
            assert(pack.zgyro_GET() == 7.519021E37F);
            assert(pack.ygyro_GET() == -2.6218249E36F);
            assert(pack.alt_GET() == 2.2088386E37F);
            assert(pack.zacc_GET() == -2.4772716E38F);
            assert(pack.pitch_GET() == -3.175626E38F);
            assert(pack.q4_GET() == 6.385354E37F);
            assert(pack.ve_GET() == 6.39029E37F);
            assert(pack.vd_GET() == -6.399807E37F);
            assert(pack.q1_GET() == -4.1310157E37F);
            assert(pack.q2_GET() == -3.802543E37F);
            assert(pack.yaw_GET() == 7.531747E37F);
            assert(pack.lon_GET() == -2.7257354E38F);
        });
        DemoDevice.SIM_STATE p108 = LoopBackDemoChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.lon_SET(-2.7257354E38F) ;
        p108.std_dev_horz_SET(-2.4865356E38F) ;
        p108.ygyro_SET(-2.6218249E36F) ;
        p108.alt_SET(2.2088386E37F) ;
        p108.pitch_SET(-3.175626E38F) ;
        p108.yacc_SET(1.1557427E38F) ;
        p108.vn_SET(2.0300867E38F) ;
        p108.lat_SET(-1.7045804E38F) ;
        p108.q3_SET(-1.8786833E38F) ;
        p108.q2_SET(-3.802543E37F) ;
        p108.q1_SET(-4.1310157E37F) ;
        p108.roll_SET(-1.8725372E38F) ;
        p108.ve_SET(6.39029E37F) ;
        p108.xgyro_SET(-6.5317335E37F) ;
        p108.vd_SET(-6.399807E37F) ;
        p108.zgyro_SET(7.519021E37F) ;
        p108.zacc_SET(-2.4772716E38F) ;
        p108.yaw_SET(7.531747E37F) ;
        p108.q4_SET(6.385354E37F) ;
        p108.std_dev_vert_SET(-2.2254128E38F) ;
        p108.xacc_SET(2.5480674E38F) ;
        LoopBackDemoChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.rssi_GET() == (char)243);
            assert(pack.fixed__GET() == (char)32964);
            assert(pack.txbuf_GET() == (char)249);
            assert(pack.remnoise_GET() == (char)156);
            assert(pack.noise_GET() == (char)11);
            assert(pack.remrssi_GET() == (char)187);
            assert(pack.rxerrors_GET() == (char)63957);
        });
        DemoDevice.RADIO_STATUS p109 = LoopBackDemoChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.noise_SET((char)11) ;
        p109.remnoise_SET((char)156) ;
        p109.remrssi_SET((char)187) ;
        p109.txbuf_SET((char)249) ;
        p109.rxerrors_SET((char)63957) ;
        p109.rssi_SET((char)243) ;
        p109.fixed__SET((char)32964) ;
        LoopBackDemoChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)30, (char)200, (char)16, (char)28, (char)91, (char)66, (char)112, (char)56, (char)90, (char)164, (char)10, (char)48, (char)221, (char)69, (char)201, (char)197, (char)191, (char)128, (char)206, (char)235, (char)30, (char)201, (char)126, (char)110, (char)161, (char)184, (char)0, (char)21, (char)140, (char)94, (char)159, (char)210, (char)141, (char)72, (char)13, (char)159, (char)233, (char)193, (char)24, (char)208, (char)88, (char)233, (char)134, (char)194, (char)114, (char)251, (char)46, (char)56, (char)165, (char)203, (char)24, (char)179, (char)210, (char)143, (char)164, (char)4, (char)228, (char)134, (char)87, (char)73, (char)163, (char)31, (char)104, (char)92, (char)221, (char)40, (char)93, (char)150, (char)68, (char)204, (char)0, (char)147, (char)102, (char)12, (char)101, (char)16, (char)79, (char)22, (char)95, (char)39, (char)53, (char)119, (char)92, (char)113, (char)180, (char)39, (char)113, (char)121, (char)63, (char)44, (char)36, (char)172, (char)131, (char)149, (char)233, (char)125, (char)181, (char)206, (char)68, (char)107, (char)181, (char)86, (char)95, (char)192, (char)153, (char)13, (char)120, (char)127, (char)115, (char)213, (char)98, (char)7, (char)30, (char)245, (char)99, (char)37, (char)255, (char)164, (char)51, (char)82, (char)23, (char)105, (char)161, (char)86, (char)178, (char)135, (char)12, (char)42, (char)159, (char)69, (char)117, (char)200, (char)139, (char)144, (char)67, (char)171, (char)119, (char)165, (char)31, (char)99, (char)31, (char)233, (char)228, (char)2, (char)135, (char)181, (char)71, (char)0, (char)134, (char)127, (char)252, (char)37, (char)42, (char)166, (char)7, (char)110, (char)191, (char)75, (char)157, (char)67, (char)220, (char)208, (char)165, (char)3, (char)251, (char)188, (char)167, (char)134, (char)159, (char)147, (char)75, (char)255, (char)176, (char)103, (char)122, (char)183, (char)30, (char)28, (char)40, (char)62, (char)222, (char)65, (char)48, (char)203, (char)212, (char)171, (char)181, (char)119, (char)193, (char)0, (char)43, (char)221, (char)144, (char)96, (char)234, (char)136, (char)157, (char)222, (char)218, (char)33, (char)213, (char)102, (char)112, (char)13, (char)180, (char)244, (char)200, (char)188, (char)134, (char)142, (char)117, (char)97, (char)252, (char)126, (char)157, (char)1, (char)37, (char)62, (char)99, (char)225, (char)124, (char)122, (char)167, (char)59, (char)46, (char)125, (char)243, (char)81, (char)175, (char)231, (char)132, (char)26, (char)32, (char)217, (char)196, (char)41, (char)21, (char)110, (char)175, (char)220, (char)54, (char)77, (char)31, (char)115, (char)128, (char)119, (char)147, (char)92, (char)53, (char)121, (char)159}));
            assert(pack.target_component_GET() == (char)222);
            assert(pack.target_network_GET() == (char)194);
            assert(pack.target_system_GET() == (char)133);
        });
        DemoDevice.FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.payload_SET(new char[] {(char)30, (char)200, (char)16, (char)28, (char)91, (char)66, (char)112, (char)56, (char)90, (char)164, (char)10, (char)48, (char)221, (char)69, (char)201, (char)197, (char)191, (char)128, (char)206, (char)235, (char)30, (char)201, (char)126, (char)110, (char)161, (char)184, (char)0, (char)21, (char)140, (char)94, (char)159, (char)210, (char)141, (char)72, (char)13, (char)159, (char)233, (char)193, (char)24, (char)208, (char)88, (char)233, (char)134, (char)194, (char)114, (char)251, (char)46, (char)56, (char)165, (char)203, (char)24, (char)179, (char)210, (char)143, (char)164, (char)4, (char)228, (char)134, (char)87, (char)73, (char)163, (char)31, (char)104, (char)92, (char)221, (char)40, (char)93, (char)150, (char)68, (char)204, (char)0, (char)147, (char)102, (char)12, (char)101, (char)16, (char)79, (char)22, (char)95, (char)39, (char)53, (char)119, (char)92, (char)113, (char)180, (char)39, (char)113, (char)121, (char)63, (char)44, (char)36, (char)172, (char)131, (char)149, (char)233, (char)125, (char)181, (char)206, (char)68, (char)107, (char)181, (char)86, (char)95, (char)192, (char)153, (char)13, (char)120, (char)127, (char)115, (char)213, (char)98, (char)7, (char)30, (char)245, (char)99, (char)37, (char)255, (char)164, (char)51, (char)82, (char)23, (char)105, (char)161, (char)86, (char)178, (char)135, (char)12, (char)42, (char)159, (char)69, (char)117, (char)200, (char)139, (char)144, (char)67, (char)171, (char)119, (char)165, (char)31, (char)99, (char)31, (char)233, (char)228, (char)2, (char)135, (char)181, (char)71, (char)0, (char)134, (char)127, (char)252, (char)37, (char)42, (char)166, (char)7, (char)110, (char)191, (char)75, (char)157, (char)67, (char)220, (char)208, (char)165, (char)3, (char)251, (char)188, (char)167, (char)134, (char)159, (char)147, (char)75, (char)255, (char)176, (char)103, (char)122, (char)183, (char)30, (char)28, (char)40, (char)62, (char)222, (char)65, (char)48, (char)203, (char)212, (char)171, (char)181, (char)119, (char)193, (char)0, (char)43, (char)221, (char)144, (char)96, (char)234, (char)136, (char)157, (char)222, (char)218, (char)33, (char)213, (char)102, (char)112, (char)13, (char)180, (char)244, (char)200, (char)188, (char)134, (char)142, (char)117, (char)97, (char)252, (char)126, (char)157, (char)1, (char)37, (char)62, (char)99, (char)225, (char)124, (char)122, (char)167, (char)59, (char)46, (char)125, (char)243, (char)81, (char)175, (char)231, (char)132, (char)26, (char)32, (char)217, (char)196, (char)41, (char)21, (char)110, (char)175, (char)220, (char)54, (char)77, (char)31, (char)115, (char)128, (char)119, (char)147, (char)92, (char)53, (char)121, (char)159}, 0) ;
        p110.target_component_SET((char)222) ;
        p110.target_network_SET((char)194) ;
        p110.target_system_SET((char)133) ;
        LoopBackDemoChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.tc1_GET() == -2799416174065396830L);
            assert(pack.ts1_GET() == 9033883618158964826L);
        });
        DemoDevice.TIMESYNC p111 = LoopBackDemoChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.ts1_SET(9033883618158964826L) ;
        p111.tc1_SET(-2799416174065396830L) ;
        LoopBackDemoChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 6783195509365938274L);
            assert(pack.seq_GET() == 4201846934L);
        });
        DemoDevice.CAMERA_TRIGGER p112 = LoopBackDemoChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(6783195509365938274L) ;
        p112.seq_SET(4201846934L) ;
        LoopBackDemoChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.vel_GET() == (char)22631);
            assert(pack.cog_GET() == (char)27266);
            assert(pack.vn_GET() == (short)30348);
            assert(pack.time_usec_GET() == 1697549061575527128L);
            assert(pack.satellites_visible_GET() == (char)112);
            assert(pack.vd_GET() == (short)18897);
            assert(pack.eph_GET() == (char)27488);
            assert(pack.lon_GET() == 1651632940);
            assert(pack.fix_type_GET() == (char)217);
            assert(pack.ve_GET() == (short) -14552);
            assert(pack.epv_GET() == (char)64852);
            assert(pack.alt_GET() == -1367375760);
            assert(pack.lat_GET() == 1830326812);
        });
        DemoDevice.HIL_GPS p113 = LoopBackDemoChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.eph_SET((char)27488) ;
        p113.lat_SET(1830326812) ;
        p113.alt_SET(-1367375760) ;
        p113.time_usec_SET(1697549061575527128L) ;
        p113.vn_SET((short)30348) ;
        p113.ve_SET((short) -14552) ;
        p113.satellites_visible_SET((char)112) ;
        p113.vel_SET((char)22631) ;
        p113.vd_SET((short)18897) ;
        p113.cog_SET((char)27266) ;
        p113.lon_SET(1651632940) ;
        p113.fix_type_SET((char)217) ;
        p113.epv_SET((char)64852) ;
        LoopBackDemoChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.quality_GET() == (char)51);
            assert(pack.integrated_y_GET() == 3.1111026E38F);
            assert(pack.time_usec_GET() == 2264934184107994887L);
            assert(pack.temperature_GET() == (short) -11840);
            assert(pack.integrated_x_GET() == 9.878175E36F);
            assert(pack.integrated_ygyro_GET() == -3.2322588E37F);
            assert(pack.integrated_zgyro_GET() == -5.4549454E37F);
            assert(pack.integration_time_us_GET() == 3819699006L);
            assert(pack.time_delta_distance_us_GET() == 4011515971L);
            assert(pack.integrated_xgyro_GET() == -1.0331718E38F);
            assert(pack.sensor_id_GET() == (char)211);
            assert(pack.distance_GET() == -2.1388845E38F);
        });
        DemoDevice.HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.integrated_xgyro_SET(-1.0331718E38F) ;
        p114.sensor_id_SET((char)211) ;
        p114.time_usec_SET(2264934184107994887L) ;
        p114.integration_time_us_SET(3819699006L) ;
        p114.integrated_zgyro_SET(-5.4549454E37F) ;
        p114.integrated_ygyro_SET(-3.2322588E37F) ;
        p114.integrated_x_SET(9.878175E36F) ;
        p114.time_delta_distance_us_SET(4011515971L) ;
        p114.temperature_SET((short) -11840) ;
        p114.integrated_y_SET(3.1111026E38F) ;
        p114.distance_SET(-2.1388845E38F) ;
        p114.quality_SET((char)51) ;
        LoopBackDemoChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.ind_airspeed_GET() == (char)5457);
            assert(pack.vz_GET() == (short) -2374);
            assert(pack.rollspeed_GET() == -2.767228E38F);
            assert(pack.vy_GET() == (short) -7148);
            assert(pack.vx_GET() == (short) -17669);
            assert(pack.yacc_GET() == (short)8754);
            assert(pack.alt_GET() == 1361570234);
            assert(pack.yawspeed_GET() == -1.8966501E38F);
            assert(pack.zacc_GET() == (short) -25727);
            assert(pack.lon_GET() == -1094790508);
            assert(pack.time_usec_GET() == 114694072284428466L);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {4.8974276E37F, -3.0230506E38F, -1.5996007E38F, 5.3971284E37F}));
            assert(pack.xacc_GET() == (short) -17206);
            assert(pack.true_airspeed_GET() == (char)44504);
            assert(pack.pitchspeed_GET() == -4.4111006E37F);
            assert(pack.lat_GET() == 1400188003);
        });
        DemoDevice.HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.alt_SET(1361570234) ;
        p115.pitchspeed_SET(-4.4111006E37F) ;
        p115.vz_SET((short) -2374) ;
        p115.ind_airspeed_SET((char)5457) ;
        p115.true_airspeed_SET((char)44504) ;
        p115.time_usec_SET(114694072284428466L) ;
        p115.yawspeed_SET(-1.8966501E38F) ;
        p115.yacc_SET((short)8754) ;
        p115.vy_SET((short) -7148) ;
        p115.vx_SET((short) -17669) ;
        p115.xacc_SET((short) -17206) ;
        p115.rollspeed_SET(-2.767228E38F) ;
        p115.lat_SET(1400188003) ;
        p115.attitude_quaternion_SET(new float[] {4.8974276E37F, -3.0230506E38F, -1.5996007E38F, 5.3971284E37F}, 0) ;
        p115.lon_SET(-1094790508) ;
        p115.zacc_SET((short) -25727) ;
        LoopBackDemoChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.zmag_GET() == (short)15118);
            assert(pack.xacc_GET() == (short)28620);
            assert(pack.ygyro_GET() == (short)17649);
            assert(pack.xmag_GET() == (short) -8132);
            assert(pack.xgyro_GET() == (short)28670);
            assert(pack.ymag_GET() == (short) -9483);
            assert(pack.zgyro_GET() == (short)32274);
            assert(pack.time_boot_ms_GET() == 1744826015L);
            assert(pack.yacc_GET() == (short)16838);
            assert(pack.zacc_GET() == (short) -20584);
        });
        DemoDevice.SCALED_IMU2 p116 = LoopBackDemoChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.time_boot_ms_SET(1744826015L) ;
        p116.zmag_SET((short)15118) ;
        p116.xgyro_SET((short)28670) ;
        p116.xmag_SET((short) -8132) ;
        p116.zgyro_SET((short)32274) ;
        p116.yacc_SET((short)16838) ;
        p116.zacc_SET((short) -20584) ;
        p116.ymag_SET((short) -9483) ;
        p116.xacc_SET((short)28620) ;
        p116.ygyro_SET((short)17649) ;
        LoopBackDemoChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)98);
            assert(pack.end_GET() == (char)57847);
            assert(pack.target_system_GET() == (char)183);
            assert(pack.start_GET() == (char)31921);
        });
        DemoDevice.LOG_REQUEST_LIST p117 = LoopBackDemoChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.start_SET((char)31921) ;
        p117.end_SET((char)57847) ;
        p117.target_system_SET((char)183) ;
        p117.target_component_SET((char)98) ;
        LoopBackDemoChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)59038);
            assert(pack.num_logs_GET() == (char)59035);
            assert(pack.last_log_num_GET() == (char)48613);
            assert(pack.size_GET() == 60943422L);
            assert(pack.time_utc_GET() == 1362919531L);
        });
        DemoDevice.LOG_ENTRY p118 = LoopBackDemoChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.last_log_num_SET((char)48613) ;
        p118.size_SET(60943422L) ;
        p118.time_utc_SET(1362919531L) ;
        p118.id_SET((char)59038) ;
        p118.num_logs_SET((char)59035) ;
        LoopBackDemoChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)33936);
            assert(pack.ofs_GET() == 2876322540L);
            assert(pack.count_GET() == 2222527848L);
            assert(pack.target_system_GET() == (char)147);
            assert(pack.target_component_GET() == (char)142);
        });
        DemoDevice.LOG_REQUEST_DATA p119 = LoopBackDemoChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_system_SET((char)147) ;
        p119.count_SET(2222527848L) ;
        p119.id_SET((char)33936) ;
        p119.target_component_SET((char)142) ;
        p119.ofs_SET(2876322540L) ;
        LoopBackDemoChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)41388);
            assert(pack.ofs_GET() == 2229973662L);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)151, (char)16, (char)100, (char)54, (char)100, (char)150, (char)225, (char)143, (char)84, (char)252, (char)56, (char)172, (char)208, (char)144, (char)99, (char)242, (char)82, (char)68, (char)154, (char)146, (char)43, (char)62, (char)39, (char)28, (char)73, (char)120, (char)110, (char)247, (char)244, (char)0, (char)59, (char)132, (char)3, (char)108, (char)150, (char)101, (char)84, (char)10, (char)204, (char)29, (char)119, (char)41, (char)126, (char)153, (char)2, (char)74, (char)164, (char)138, (char)90, (char)199, (char)8, (char)105, (char)254, (char)163, (char)174, (char)108, (char)20, (char)195, (char)89, (char)175, (char)112, (char)40, (char)216, (char)75, (char)162, (char)18, (char)158, (char)80, (char)32, (char)13, (char)202, (char)9, (char)213, (char)149, (char)195, (char)211, (char)30, (char)224, (char)97, (char)205, (char)128, (char)223, (char)162, (char)102, (char)37, (char)29, (char)231, (char)104, (char)61, (char)5}));
            assert(pack.count_GET() == (char)108);
        });
        DemoDevice.LOG_DATA p120 = LoopBackDemoChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.ofs_SET(2229973662L) ;
        p120.id_SET((char)41388) ;
        p120.count_SET((char)108) ;
        p120.data__SET(new char[] {(char)151, (char)16, (char)100, (char)54, (char)100, (char)150, (char)225, (char)143, (char)84, (char)252, (char)56, (char)172, (char)208, (char)144, (char)99, (char)242, (char)82, (char)68, (char)154, (char)146, (char)43, (char)62, (char)39, (char)28, (char)73, (char)120, (char)110, (char)247, (char)244, (char)0, (char)59, (char)132, (char)3, (char)108, (char)150, (char)101, (char)84, (char)10, (char)204, (char)29, (char)119, (char)41, (char)126, (char)153, (char)2, (char)74, (char)164, (char)138, (char)90, (char)199, (char)8, (char)105, (char)254, (char)163, (char)174, (char)108, (char)20, (char)195, (char)89, (char)175, (char)112, (char)40, (char)216, (char)75, (char)162, (char)18, (char)158, (char)80, (char)32, (char)13, (char)202, (char)9, (char)213, (char)149, (char)195, (char)211, (char)30, (char)224, (char)97, (char)205, (char)128, (char)223, (char)162, (char)102, (char)37, (char)29, (char)231, (char)104, (char)61, (char)5}, 0) ;
        LoopBackDemoChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)36);
            assert(pack.target_system_GET() == (char)169);
        });
        DemoDevice.LOG_ERASE p121 = LoopBackDemoChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)169) ;
        p121.target_component_SET((char)36) ;
        LoopBackDemoChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)134);
            assert(pack.target_component_GET() == (char)59);
        });
        DemoDevice.LOG_REQUEST_END p122 = LoopBackDemoChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)134) ;
        p122.target_component_SET((char)59) ;
        LoopBackDemoChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)254);
            assert(pack.target_component_GET() == (char)130);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)64, (char)40, (char)92, (char)224, (char)26, (char)226, (char)34, (char)230, (char)97, (char)97, (char)61, (char)225, (char)87, (char)71, (char)200, (char)154, (char)244, (char)4, (char)93, (char)195, (char)186, (char)200, (char)70, (char)223, (char)17, (char)127, (char)101, (char)132, (char)222, (char)203, (char)119, (char)122, (char)38, (char)120, (char)223, (char)135, (char)122, (char)37, (char)156, (char)35, (char)115, (char)65, (char)184, (char)118, (char)223, (char)2, (char)77, (char)41, (char)138, (char)147, (char)19, (char)126, (char)229, (char)126, (char)110, (char)182, (char)249, (char)202, (char)248, (char)35, (char)213, (char)161, (char)34, (char)98, (char)126, (char)185, (char)73, (char)16, (char)152, (char)170, (char)48, (char)31, (char)206, (char)245, (char)236, (char)143, (char)222, (char)115, (char)94, (char)67, (char)18, (char)133, (char)218, (char)255, (char)88, (char)37, (char)170, (char)153, (char)96, (char)61, (char)107, (char)94, (char)252, (char)8, (char)204, (char)215, (char)9, (char)163, (char)89, (char)13, (char)178, (char)65, (char)47, (char)170, (char)57, (char)225, (char)39, (char)254, (char)165, (char)230}));
            assert(pack.len_GET() == (char)46);
        });
        DemoDevice.GPS_INJECT_DATA p123 = LoopBackDemoChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.data__SET(new char[] {(char)64, (char)40, (char)92, (char)224, (char)26, (char)226, (char)34, (char)230, (char)97, (char)97, (char)61, (char)225, (char)87, (char)71, (char)200, (char)154, (char)244, (char)4, (char)93, (char)195, (char)186, (char)200, (char)70, (char)223, (char)17, (char)127, (char)101, (char)132, (char)222, (char)203, (char)119, (char)122, (char)38, (char)120, (char)223, (char)135, (char)122, (char)37, (char)156, (char)35, (char)115, (char)65, (char)184, (char)118, (char)223, (char)2, (char)77, (char)41, (char)138, (char)147, (char)19, (char)126, (char)229, (char)126, (char)110, (char)182, (char)249, (char)202, (char)248, (char)35, (char)213, (char)161, (char)34, (char)98, (char)126, (char)185, (char)73, (char)16, (char)152, (char)170, (char)48, (char)31, (char)206, (char)245, (char)236, (char)143, (char)222, (char)115, (char)94, (char)67, (char)18, (char)133, (char)218, (char)255, (char)88, (char)37, (char)170, (char)153, (char)96, (char)61, (char)107, (char)94, (char)252, (char)8, (char)204, (char)215, (char)9, (char)163, (char)89, (char)13, (char)178, (char)65, (char)47, (char)170, (char)57, (char)225, (char)39, (char)254, (char)165, (char)230}, 0) ;
        p123.len_SET((char)46) ;
        p123.target_system_SET((char)254) ;
        p123.target_component_SET((char)130) ;
        LoopBackDemoChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == -769521236);
            assert(pack.lat_GET() == -1739859502);
            assert(pack.eph_GET() == (char)32883);
            assert(pack.dgps_age_GET() == 2621578392L);
            assert(pack.dgps_numch_GET() == (char)110);
            assert(pack.vel_GET() == (char)39968);
            assert(pack.satellites_visible_GET() == (char)61);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED);
            assert(pack.epv_GET() == (char)44410);
            assert(pack.cog_GET() == (char)49011);
            assert(pack.lon_GET() == 1701266767);
            assert(pack.time_usec_GET() == 8979593971471257689L);
        });
        DemoDevice.GPS2_RAW p124 = LoopBackDemoChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.satellites_visible_SET((char)61) ;
        p124.epv_SET((char)44410) ;
        p124.alt_SET(-769521236) ;
        p124.dgps_age_SET(2621578392L) ;
        p124.dgps_numch_SET((char)110) ;
        p124.cog_SET((char)49011) ;
        p124.lat_SET(-1739859502) ;
        p124.vel_SET((char)39968) ;
        p124.lon_SET(1701266767) ;
        p124.time_usec_SET(8979593971471257689L) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED) ;
        p124.eph_SET((char)32883) ;
        LoopBackDemoChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED);
            assert(pack.Vservo_GET() == (char)37732);
            assert(pack.Vcc_GET() == (char)27170);
        });
        DemoDevice.POWER_STATUS p125 = LoopBackDemoChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vservo_SET((char)37732) ;
        p125.Vcc_SET((char)27170) ;
        p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED) ;
        LoopBackDemoChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)26);
            assert(pack.timeout_GET() == (char)48370);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)145, (char)161, (char)177, (char)146, (char)71, (char)232, (char)125, (char)173, (char)114, (char)144, (char)235, (char)144, (char)185, (char)95, (char)212, (char)27, (char)211, (char)153, (char)169, (char)80, (char)190, (char)89, (char)50, (char)196, (char)16, (char)133, (char)223, (char)231, (char)115, (char)98, (char)194, (char)115, (char)153, (char)237, (char)60, (char)219, (char)117, (char)75, (char)42, (char)42, (char)97, (char)116, (char)133, (char)42, (char)21, (char)24, (char)15, (char)80, (char)11, (char)20, (char)217, (char)4, (char)44, (char)91, (char)10, (char)40, (char)235, (char)154, (char)169, (char)188, (char)153, (char)132, (char)177, (char)218, (char)2, (char)190, (char)245, (char)218, (char)135, (char)50}));
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2);
            assert(pack.baudrate_GET() == 3318399838L);
            assert(pack.flags_GET() == SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND);
        });
        DemoDevice.SERIAL_CONTROL p126 = LoopBackDemoChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.baudrate_SET(3318399838L) ;
        p126.data__SET(new char[] {(char)145, (char)161, (char)177, (char)146, (char)71, (char)232, (char)125, (char)173, (char)114, (char)144, (char)235, (char)144, (char)185, (char)95, (char)212, (char)27, (char)211, (char)153, (char)169, (char)80, (char)190, (char)89, (char)50, (char)196, (char)16, (char)133, (char)223, (char)231, (char)115, (char)98, (char)194, (char)115, (char)153, (char)237, (char)60, (char)219, (char)117, (char)75, (char)42, (char)42, (char)97, (char)116, (char)133, (char)42, (char)21, (char)24, (char)15, (char)80, (char)11, (char)20, (char)217, (char)4, (char)44, (char)91, (char)10, (char)40, (char)235, (char)154, (char)169, (char)188, (char)153, (char)132, (char)177, (char)218, (char)2, (char)190, (char)245, (char)218, (char)135, (char)50}, 0) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2) ;
        p126.count_SET((char)26) ;
        p126.timeout_SET((char)48370) ;
        p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND) ;
        LoopBackDemoChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.accuracy_GET() == 2789431958L);
            assert(pack.wn_GET() == (char)6121);
            assert(pack.nsats_GET() == (char)54);
            assert(pack.rtk_health_GET() == (char)253);
            assert(pack.baseline_a_mm_GET() == -1567398511);
            assert(pack.baseline_b_mm_GET() == -561935272);
            assert(pack.time_last_baseline_ms_GET() == 536224390L);
            assert(pack.baseline_coords_type_GET() == (char)235);
            assert(pack.iar_num_hypotheses_GET() == -992414465);
            assert(pack.rtk_receiver_id_GET() == (char)13);
            assert(pack.tow_GET() == 2910933702L);
            assert(pack.rtk_rate_GET() == (char)56);
            assert(pack.baseline_c_mm_GET() == -542633819);
        });
        DemoDevice.GPS_RTK p127 = LoopBackDemoChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.baseline_b_mm_SET(-561935272) ;
        p127.baseline_c_mm_SET(-542633819) ;
        p127.accuracy_SET(2789431958L) ;
        p127.iar_num_hypotheses_SET(-992414465) ;
        p127.tow_SET(2910933702L) ;
        p127.baseline_coords_type_SET((char)235) ;
        p127.rtk_health_SET((char)253) ;
        p127.rtk_rate_SET((char)56) ;
        p127.nsats_SET((char)54) ;
        p127.rtk_receiver_id_SET((char)13) ;
        p127.wn_SET((char)6121) ;
        p127.baseline_a_mm_SET(-1567398511) ;
        p127.time_last_baseline_ms_SET(536224390L) ;
        LoopBackDemoChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.baseline_a_mm_GET() == -85961796);
            assert(pack.rtk_health_GET() == (char)90);
            assert(pack.baseline_coords_type_GET() == (char)92);
            assert(pack.baseline_c_mm_GET() == -1238775789);
            assert(pack.rtk_rate_GET() == (char)252);
            assert(pack.iar_num_hypotheses_GET() == 741816993);
            assert(pack.rtk_receiver_id_GET() == (char)63);
            assert(pack.wn_GET() == (char)4522);
            assert(pack.time_last_baseline_ms_GET() == 223417412L);
            assert(pack.baseline_b_mm_GET() == 487389306);
            assert(pack.nsats_GET() == (char)133);
            assert(pack.accuracy_GET() == 3827258954L);
            assert(pack.tow_GET() == 1472049274L);
        });
        DemoDevice.GPS2_RTK p128 = LoopBackDemoChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.time_last_baseline_ms_SET(223417412L) ;
        p128.wn_SET((char)4522) ;
        p128.baseline_c_mm_SET(-1238775789) ;
        p128.rtk_health_SET((char)90) ;
        p128.rtk_receiver_id_SET((char)63) ;
        p128.rtk_rate_SET((char)252) ;
        p128.iar_num_hypotheses_SET(741816993) ;
        p128.baseline_b_mm_SET(487389306) ;
        p128.accuracy_SET(3827258954L) ;
        p128.nsats_SET((char)133) ;
        p128.tow_SET(1472049274L) ;
        p128.baseline_a_mm_SET(-85961796) ;
        p128.baseline_coords_type_SET((char)92) ;
        LoopBackDemoChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.xmag_GET() == (short)28438);
            assert(pack.time_boot_ms_GET() == 2664718493L);
            assert(pack.ymag_GET() == (short) -13009);
            assert(pack.xacc_GET() == (short)4130);
            assert(pack.xgyro_GET() == (short) -24049);
            assert(pack.zmag_GET() == (short)28000);
            assert(pack.zgyro_GET() == (short) -11903);
            assert(pack.zacc_GET() == (short)22304);
            assert(pack.yacc_GET() == (short)10662);
            assert(pack.ygyro_GET() == (short) -31162);
        });
        DemoDevice.SCALED_IMU3 p129 = LoopBackDemoChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.xacc_SET((short)4130) ;
        p129.time_boot_ms_SET(2664718493L) ;
        p129.yacc_SET((short)10662) ;
        p129.xgyro_SET((short) -24049) ;
        p129.xmag_SET((short)28438) ;
        p129.ygyro_SET((short) -31162) ;
        p129.zacc_SET((short)22304) ;
        p129.zmag_SET((short)28000) ;
        p129.zgyro_SET((short) -11903) ;
        p129.ymag_SET((short) -13009) ;
        LoopBackDemoChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.size_GET() == 267313535L);
            assert(pack.packets_GET() == (char)4151);
            assert(pack.jpg_quality_GET() == (char)172);
            assert(pack.width_GET() == (char)52380);
            assert(pack.type_GET() == (char)147);
            assert(pack.payload_GET() == (char)143);
            assert(pack.height_GET() == (char)16573);
        });
        DemoDevice.DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.packets_SET((char)4151) ;
        p130.jpg_quality_SET((char)172) ;
        p130.payload_SET((char)143) ;
        p130.type_SET((char)147) ;
        p130.height_SET((char)16573) ;
        p130.size_SET(267313535L) ;
        p130.width_SET((char)52380) ;
        LoopBackDemoChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)179, (char)21, (char)75, (char)210, (char)11, (char)231, (char)245, (char)78, (char)184, (char)22, (char)111, (char)132, (char)249, (char)153, (char)179, (char)99, (char)184, (char)35, (char)138, (char)160, (char)5, (char)67, (char)249, (char)70, (char)80, (char)44, (char)3, (char)160, (char)12, (char)242, (char)69, (char)212, (char)163, (char)86, (char)219, (char)226, (char)220, (char)35, (char)76, (char)233, (char)170, (char)131, (char)21, (char)22, (char)182, (char)207, (char)53, (char)93, (char)245, (char)130, (char)186, (char)65, (char)133, (char)131, (char)248, (char)219, (char)157, (char)22, (char)50, (char)158, (char)54, (char)243, (char)145, (char)93, (char)32, (char)56, (char)53, (char)108, (char)200, (char)215, (char)166, (char)239, (char)184, (char)69, (char)95, (char)236, (char)114, (char)127, (char)242, (char)36, (char)73, (char)250, (char)217, (char)162, (char)89, (char)141, (char)21, (char)78, (char)204, (char)130, (char)71, (char)94, (char)0, (char)88, (char)81, (char)182, (char)27, (char)199, (char)170, (char)103, (char)231, (char)146, (char)205, (char)243, (char)139, (char)16, (char)174, (char)71, (char)67, (char)144, (char)90, (char)62, (char)207, (char)229, (char)11, (char)60, (char)224, (char)252, (char)16, (char)238, (char)67, (char)84, (char)84, (char)70, (char)178, (char)25, (char)56, (char)201, (char)204, (char)101, (char)221, (char)220, (char)72, (char)10, (char)182, (char)128, (char)231, (char)74, (char)68, (char)227, (char)60, (char)152, (char)3, (char)63, (char)181, (char)148, (char)151, (char)212, (char)38, (char)186, (char)198, (char)187, (char)200, (char)73, (char)144, (char)216, (char)218, (char)43, (char)133, (char)148, (char)101, (char)223, (char)95, (char)102, (char)101, (char)242, (char)219, (char)234, (char)243, (char)101, (char)75, (char)122, (char)167, (char)125, (char)105, (char)2, (char)59, (char)27, (char)142, (char)67, (char)139, (char)121, (char)246, (char)91, (char)148, (char)233, (char)62, (char)235, (char)200, (char)78, (char)215, (char)198, (char)201, (char)180, (char)173, (char)245, (char)255, (char)99, (char)132, (char)49, (char)208, (char)190, (char)176, (char)170, (char)100, (char)246, (char)241, (char)135, (char)251, (char)222, (char)43, (char)11, (char)233, (char)24, (char)159, (char)38, (char)94, (char)24, (char)248, (char)113, (char)227, (char)17, (char)2, (char)46, (char)116, (char)181, (char)100, (char)116, (char)118, (char)87, (char)178, (char)201, (char)34, (char)81, (char)183, (char)134, (char)166, (char)21, (char)144, (char)115, (char)146, (char)43, (char)184, (char)142, (char)243, (char)8, (char)84, (char)191, (char)130, (char)173, (char)114, (char)163, (char)154}));
            assert(pack.seqnr_GET() == (char)47060);
        });
        DemoDevice.ENCAPSULATED_DATA p131 = LoopBackDemoChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.data__SET(new char[] {(char)179, (char)21, (char)75, (char)210, (char)11, (char)231, (char)245, (char)78, (char)184, (char)22, (char)111, (char)132, (char)249, (char)153, (char)179, (char)99, (char)184, (char)35, (char)138, (char)160, (char)5, (char)67, (char)249, (char)70, (char)80, (char)44, (char)3, (char)160, (char)12, (char)242, (char)69, (char)212, (char)163, (char)86, (char)219, (char)226, (char)220, (char)35, (char)76, (char)233, (char)170, (char)131, (char)21, (char)22, (char)182, (char)207, (char)53, (char)93, (char)245, (char)130, (char)186, (char)65, (char)133, (char)131, (char)248, (char)219, (char)157, (char)22, (char)50, (char)158, (char)54, (char)243, (char)145, (char)93, (char)32, (char)56, (char)53, (char)108, (char)200, (char)215, (char)166, (char)239, (char)184, (char)69, (char)95, (char)236, (char)114, (char)127, (char)242, (char)36, (char)73, (char)250, (char)217, (char)162, (char)89, (char)141, (char)21, (char)78, (char)204, (char)130, (char)71, (char)94, (char)0, (char)88, (char)81, (char)182, (char)27, (char)199, (char)170, (char)103, (char)231, (char)146, (char)205, (char)243, (char)139, (char)16, (char)174, (char)71, (char)67, (char)144, (char)90, (char)62, (char)207, (char)229, (char)11, (char)60, (char)224, (char)252, (char)16, (char)238, (char)67, (char)84, (char)84, (char)70, (char)178, (char)25, (char)56, (char)201, (char)204, (char)101, (char)221, (char)220, (char)72, (char)10, (char)182, (char)128, (char)231, (char)74, (char)68, (char)227, (char)60, (char)152, (char)3, (char)63, (char)181, (char)148, (char)151, (char)212, (char)38, (char)186, (char)198, (char)187, (char)200, (char)73, (char)144, (char)216, (char)218, (char)43, (char)133, (char)148, (char)101, (char)223, (char)95, (char)102, (char)101, (char)242, (char)219, (char)234, (char)243, (char)101, (char)75, (char)122, (char)167, (char)125, (char)105, (char)2, (char)59, (char)27, (char)142, (char)67, (char)139, (char)121, (char)246, (char)91, (char)148, (char)233, (char)62, (char)235, (char)200, (char)78, (char)215, (char)198, (char)201, (char)180, (char)173, (char)245, (char)255, (char)99, (char)132, (char)49, (char)208, (char)190, (char)176, (char)170, (char)100, (char)246, (char)241, (char)135, (char)251, (char)222, (char)43, (char)11, (char)233, (char)24, (char)159, (char)38, (char)94, (char)24, (char)248, (char)113, (char)227, (char)17, (char)2, (char)46, (char)116, (char)181, (char)100, (char)116, (char)118, (char)87, (char)178, (char)201, (char)34, (char)81, (char)183, (char)134, (char)166, (char)21, (char)144, (char)115, (char)146, (char)43, (char)184, (char)142, (char)243, (char)8, (char)84, (char)191, (char)130, (char)173, (char)114, (char)163, (char)154}, 0) ;
        p131.seqnr_SET((char)47060) ;
        LoopBackDemoChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_PITCH_90);
            assert(pack.min_distance_GET() == (char)19289);
            assert(pack.covariance_GET() == (char)187);
            assert(pack.id_GET() == (char)69);
            assert(pack.max_distance_GET() == (char)23040);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
            assert(pack.time_boot_ms_GET() == 4002624649L);
            assert(pack.current_distance_GET() == (char)14882);
        });
        DemoDevice.DISTANCE_SENSOR p132 = LoopBackDemoChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.covariance_SET((char)187) ;
        p132.min_distance_SET((char)19289) ;
        p132.time_boot_ms_SET(4002624649L) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_PITCH_90) ;
        p132.current_distance_SET((char)14882) ;
        p132.max_distance_SET((char)23040) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN) ;
        p132.id_SET((char)69) ;
        LoopBackDemoChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 499008766);
            assert(pack.mask_GET() == 5782700542844834701L);
            assert(pack.lon_GET() == -343182067);
            assert(pack.grid_spacing_GET() == (char)58559);
        });
        DemoDevice.TERRAIN_REQUEST p133 = LoopBackDemoChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lon_SET(-343182067) ;
        p133.lat_SET(499008766) ;
        p133.grid_spacing_SET((char)58559) ;
        p133.mask_SET(5782700542844834701L) ;
        LoopBackDemoChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.gridbit_GET() == (char)159);
            assert(pack.lon_GET() == -1208123871);
            assert(pack.grid_spacing_GET() == (char)35867);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short)16091, (short) -23551, (short) -21485, (short) -14960, (short) -25201, (short) -17414, (short) -2677, (short) -25507, (short)21270, (short)9773, (short)5223, (short)15330, (short) -29165, (short) -5808, (short) -26742, (short)20638}));
            assert(pack.lat_GET() == -518270744);
        });
        DemoDevice.TERRAIN_DATA p134 = LoopBackDemoChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.grid_spacing_SET((char)35867) ;
        p134.gridbit_SET((char)159) ;
        p134.lat_SET(-518270744) ;
        p134.data__SET(new short[] {(short)16091, (short) -23551, (short) -21485, (short) -14960, (short) -25201, (short) -17414, (short) -2677, (short) -25507, (short)21270, (short)9773, (short)5223, (short)15330, (short) -29165, (short) -5808, (short) -26742, (short)20638}, 0) ;
        p134.lon_SET(-1208123871) ;
        LoopBackDemoChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 1335520439);
            assert(pack.lon_GET() == -528091187);
        });
        DemoDevice.TERRAIN_CHECK p135 = LoopBackDemoChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(1335520439) ;
        p135.lon_SET(-528091187) ;
        LoopBackDemoChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -1344514134);
            assert(pack.current_height_GET() == 5.76399E36F);
            assert(pack.loaded_GET() == (char)47054);
            assert(pack.pending_GET() == (char)40293);
            assert(pack.lat_GET() == -1139784493);
            assert(pack.terrain_height_GET() == -3.1548783E38F);
            assert(pack.spacing_GET() == (char)63193);
        });
        DemoDevice.TERRAIN_REPORT p136 = LoopBackDemoChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.spacing_SET((char)63193) ;
        p136.pending_SET((char)40293) ;
        p136.lon_SET(-1344514134) ;
        p136.terrain_height_SET(-3.1548783E38F) ;
        p136.loaded_SET((char)47054) ;
        p136.current_height_SET(5.76399E36F) ;
        p136.lat_SET(-1139784493) ;
        LoopBackDemoChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short) -13419);
            assert(pack.press_abs_GET() == 1.197614E37F);
            assert(pack.press_diff_GET() == -5.041027E37F);
            assert(pack.time_boot_ms_GET() == 3908178541L);
        });
        DemoDevice.SCALED_PRESSURE2 p137 = LoopBackDemoChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(3908178541L) ;
        p137.temperature_SET((short) -13419) ;
        p137.press_diff_SET(-5.041027E37F) ;
        p137.press_abs_SET(1.197614E37F) ;
        LoopBackDemoChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.3058411E38F, -3.0859078E38F, -2.7223608E38F, -2.2574707E38F}));
            assert(pack.time_usec_GET() == 5751920335257635547L);
            assert(pack.x_GET() == 1.317814E38F);
            assert(pack.y_GET() == 3.847399E37F);
            assert(pack.z_GET() == 1.1555223E37F);
        });
        DemoDevice.ATT_POS_MOCAP p138 = LoopBackDemoChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.time_usec_SET(5751920335257635547L) ;
        p138.y_SET(3.847399E37F) ;
        p138.x_SET(1.317814E38F) ;
        p138.q_SET(new float[] {1.3058411E38F, -3.0859078E38F, -2.7223608E38F, -2.2574707E38F}, 0) ;
        p138.z_SET(1.1555223E37F) ;
        LoopBackDemoChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)69);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-1.0836032E38F, -2.8337725E38F, 1.6318691E38F, -2.4224866E38F, -1.6341514E38F, -2.311891E38F, 6.751717E37F, 6.887764E36F}));
            assert(pack.group_mlx_GET() == (char)240);
            assert(pack.time_usec_GET() == 2478478224212892625L);
            assert(pack.target_component_GET() == (char)10);
        });
        DemoDevice.SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.target_component_SET((char)10) ;
        p139.time_usec_SET(2478478224212892625L) ;
        p139.group_mlx_SET((char)240) ;
        p139.target_system_SET((char)69) ;
        p139.controls_SET(new float[] {-1.0836032E38F, -2.8337725E38F, 1.6318691E38F, -2.4224866E38F, -1.6341514E38F, -2.311891E38F, 6.751717E37F, 6.887764E36F}, 0) ;
        LoopBackDemoChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 6748693880683623826L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {2.8287222E38F, 1.0082027E38F, 8.904164E37F, 1.8229446E38F, -2.4952033E38F, 2.6166734E38F, 7.8893107E37F, -1.0283492E38F}));
            assert(pack.group_mlx_GET() == (char)52);
        });
        DemoDevice.ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.time_usec_SET(6748693880683623826L) ;
        p140.group_mlx_SET((char)52) ;
        p140.controls_SET(new float[] {2.8287222E38F, 1.0082027E38F, 8.904164E37F, 1.8229446E38F, -2.4952033E38F, 2.6166734E38F, 7.8893107E37F, -1.0283492E38F}, 0) ;
        LoopBackDemoChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.altitude_local_GET() == 3.019981E38F);
            assert(pack.time_usec_GET() == 2586502320181727146L);
            assert(pack.altitude_relative_GET() == 5.7981943E37F);
            assert(pack.altitude_terrain_GET() == -3.3796656E38F);
            assert(pack.altitude_amsl_GET() == 2.444292E38F);
            assert(pack.bottom_clearance_GET() == -2.9312854E38F);
            assert(pack.altitude_monotonic_GET() == -2.3067395E37F);
        });
        DemoDevice.ALTITUDE p141 = LoopBackDemoChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_terrain_SET(-3.3796656E38F) ;
        p141.time_usec_SET(2586502320181727146L) ;
        p141.altitude_amsl_SET(2.444292E38F) ;
        p141.altitude_local_SET(3.019981E38F) ;
        p141.altitude_monotonic_SET(-2.3067395E37F) ;
        p141.bottom_clearance_SET(-2.9312854E38F) ;
        p141.altitude_relative_SET(5.7981943E37F) ;
        LoopBackDemoChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)101, (char)127, (char)126, (char)121, (char)131, (char)198, (char)251, (char)231, (char)246, (char)215, (char)137, (char)12, (char)209, (char)206, (char)127, (char)9, (char)128, (char)86, (char)117, (char)142, (char)56, (char)169, (char)214, (char)110, (char)175, (char)214, (char)234, (char)213, (char)189, (char)180, (char)235, (char)189, (char)250, (char)205, (char)71, (char)33, (char)235, (char)135, (char)162, (char)97, (char)131, (char)153, (char)17, (char)18, (char)50, (char)70, (char)18, (char)151, (char)70, (char)131, (char)152, (char)18, (char)28, (char)138, (char)183, (char)19, (char)20, (char)107, (char)234, (char)241, (char)165, (char)111, (char)163, (char)70, (char)239, (char)254, (char)5, (char)71, (char)85, (char)152, (char)58, (char)163, (char)49, (char)167, (char)235, (char)105, (char)202, (char)139, (char)11, (char)110, (char)66, (char)161, (char)71, (char)25, (char)132, (char)226, (char)147, (char)228, (char)208, (char)251, (char)188, (char)124, (char)184, (char)35, (char)127, (char)10, (char)250, (char)140, (char)142, (char)12, (char)156, (char)26, (char)59, (char)106, (char)126, (char)153, (char)99, (char)28, (char)205, (char)182, (char)53, (char)110, (char)206, (char)169, (char)133, (char)235, (char)32, (char)177, (char)156, (char)100}));
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)156, (char)40, (char)192, (char)239, (char)187, (char)32, (char)108, (char)241, (char)37, (char)217, (char)98, (char)104, (char)202, (char)229, (char)35, (char)36, (char)81, (char)207, (char)219, (char)236, (char)192, (char)58, (char)153, (char)15, (char)104, (char)94, (char)38, (char)121, (char)245, (char)83, (char)73, (char)169, (char)247, (char)17, (char)123, (char)60, (char)9, (char)113, (char)253, (char)165, (char)152, (char)87, (char)79, (char)65, (char)208, (char)182, (char)16, (char)44, (char)48, (char)113, (char)166, (char)117, (char)31, (char)206, (char)92, (char)34, (char)47, (char)140, (char)4, (char)112, (char)62, (char)67, (char)173, (char)229, (char)57, (char)131, (char)136, (char)58, (char)125, (char)107, (char)71, (char)194, (char)133, (char)206, (char)243, (char)236, (char)132, (char)91, (char)23, (char)94, (char)110, (char)174, (char)209, (char)89, (char)211, (char)59, (char)243, (char)196, (char)78, (char)121, (char)195, (char)175, (char)9, (char)69, (char)247, (char)137, (char)45, (char)119, (char)239, (char)131, (char)123, (char)207, (char)38, (char)84, (char)172, (char)255, (char)214, (char)15, (char)86, (char)202, (char)75, (char)168, (char)138, (char)74, (char)74, (char)166, (char)99, (char)129, (char)169, (char)135}));
            assert(pack.transfer_type_GET() == (char)116);
            assert(pack.uri_type_GET() == (char)117);
            assert(pack.request_id_GET() == (char)69);
        });
        DemoDevice.RESOURCE_REQUEST p142 = LoopBackDemoChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.transfer_type_SET((char)116) ;
        p142.storage_SET(new char[] {(char)101, (char)127, (char)126, (char)121, (char)131, (char)198, (char)251, (char)231, (char)246, (char)215, (char)137, (char)12, (char)209, (char)206, (char)127, (char)9, (char)128, (char)86, (char)117, (char)142, (char)56, (char)169, (char)214, (char)110, (char)175, (char)214, (char)234, (char)213, (char)189, (char)180, (char)235, (char)189, (char)250, (char)205, (char)71, (char)33, (char)235, (char)135, (char)162, (char)97, (char)131, (char)153, (char)17, (char)18, (char)50, (char)70, (char)18, (char)151, (char)70, (char)131, (char)152, (char)18, (char)28, (char)138, (char)183, (char)19, (char)20, (char)107, (char)234, (char)241, (char)165, (char)111, (char)163, (char)70, (char)239, (char)254, (char)5, (char)71, (char)85, (char)152, (char)58, (char)163, (char)49, (char)167, (char)235, (char)105, (char)202, (char)139, (char)11, (char)110, (char)66, (char)161, (char)71, (char)25, (char)132, (char)226, (char)147, (char)228, (char)208, (char)251, (char)188, (char)124, (char)184, (char)35, (char)127, (char)10, (char)250, (char)140, (char)142, (char)12, (char)156, (char)26, (char)59, (char)106, (char)126, (char)153, (char)99, (char)28, (char)205, (char)182, (char)53, (char)110, (char)206, (char)169, (char)133, (char)235, (char)32, (char)177, (char)156, (char)100}, 0) ;
        p142.request_id_SET((char)69) ;
        p142.uri_SET(new char[] {(char)156, (char)40, (char)192, (char)239, (char)187, (char)32, (char)108, (char)241, (char)37, (char)217, (char)98, (char)104, (char)202, (char)229, (char)35, (char)36, (char)81, (char)207, (char)219, (char)236, (char)192, (char)58, (char)153, (char)15, (char)104, (char)94, (char)38, (char)121, (char)245, (char)83, (char)73, (char)169, (char)247, (char)17, (char)123, (char)60, (char)9, (char)113, (char)253, (char)165, (char)152, (char)87, (char)79, (char)65, (char)208, (char)182, (char)16, (char)44, (char)48, (char)113, (char)166, (char)117, (char)31, (char)206, (char)92, (char)34, (char)47, (char)140, (char)4, (char)112, (char)62, (char)67, (char)173, (char)229, (char)57, (char)131, (char)136, (char)58, (char)125, (char)107, (char)71, (char)194, (char)133, (char)206, (char)243, (char)236, (char)132, (char)91, (char)23, (char)94, (char)110, (char)174, (char)209, (char)89, (char)211, (char)59, (char)243, (char)196, (char)78, (char)121, (char)195, (char)175, (char)9, (char)69, (char)247, (char)137, (char)45, (char)119, (char)239, (char)131, (char)123, (char)207, (char)38, (char)84, (char)172, (char)255, (char)214, (char)15, (char)86, (char)202, (char)75, (char)168, (char)138, (char)74, (char)74, (char)166, (char)99, (char)129, (char)169, (char)135}, 0) ;
        p142.uri_type_SET((char)117) ;
        LoopBackDemoChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == -1.2120997E38F);
            assert(pack.time_boot_ms_GET() == 3562246268L);
            assert(pack.temperature_GET() == (short)21934);
            assert(pack.press_diff_GET() == -3.764995E37F);
        });
        DemoDevice.SCALED_PRESSURE3 p143 = LoopBackDemoChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.press_abs_SET(-1.2120997E38F) ;
        p143.temperature_SET((short)21934) ;
        p143.time_boot_ms_SET(3562246268L) ;
        p143.press_diff_SET(-3.764995E37F) ;
        LoopBackDemoChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.vel_GET(),  new float[] {1.6107442E38F, -1.0328369E38F, 3.0961214E38F}));
            assert(pack.timestamp_GET() == 1443812271363993320L);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {2.9778966E38F, 2.7907618E38F, -1.9592942E37F}));
            assert(pack.est_capabilities_GET() == (char)11);
            assert(Arrays.equals(pack.rates_GET(),  new float[] {-2.6709155E38F, 4.3328776E36F, 2.5178184E38F}));
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {5.464725E37F, 1.0823643E37F, 2.3137387E37F}));
            assert(pack.lat_GET() == -1426019151);
            assert(pack.custom_state_GET() == 6274346969908407184L);
            assert(pack.alt_GET() == 2.7053595E38F);
            assert(pack.lon_GET() == -1614988142);
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {3.2891905E37F, 2.7203279E38F, 2.6324978E38F, 4.5144755E37F}));
        });
        DemoDevice.FOLLOW_TARGET p144 = LoopBackDemoChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.rates_SET(new float[] {-2.6709155E38F, 4.3328776E36F, 2.5178184E38F}, 0) ;
        p144.lon_SET(-1614988142) ;
        p144.position_cov_SET(new float[] {5.464725E37F, 1.0823643E37F, 2.3137387E37F}, 0) ;
        p144.vel_SET(new float[] {1.6107442E38F, -1.0328369E38F, 3.0961214E38F}, 0) ;
        p144.lat_SET(-1426019151) ;
        p144.timestamp_SET(1443812271363993320L) ;
        p144.acc_SET(new float[] {2.9778966E38F, 2.7907618E38F, -1.9592942E37F}, 0) ;
        p144.custom_state_SET(6274346969908407184L) ;
        p144.alt_SET(2.7053595E38F) ;
        p144.est_capabilities_SET((char)11) ;
        p144.attitude_q_SET(new float[] {3.2891905E37F, 2.7203279E38F, 2.6324978E38F, 4.5144755E37F}, 0) ;
        LoopBackDemoChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.y_acc_GET() == 2.808612E38F);
            assert(pack.z_pos_GET() == 2.1880167E37F);
            assert(pack.z_vel_GET() == -5.373503E36F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {1.8302447E38F, 2.4199553E38F, 2.6383444E38F}));
            assert(pack.y_vel_GET() == -3.0283696E38F);
            assert(pack.x_acc_GET() == 1.2858125E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-6.0512696E36F, 1.630465E38F, -1.1116246E38F, -1.5107733E38F}));
            assert(pack.yaw_rate_GET() == -5.3879303E37F);
            assert(pack.airspeed_GET() == 2.596729E38F);
            assert(pack.time_usec_GET() == 6522924101989694147L);
            assert(pack.x_vel_GET() == 9.970374E37F);
            assert(pack.pitch_rate_GET() == 2.6590014E38F);
            assert(pack.roll_rate_GET() == 6.615205E36F);
            assert(pack.z_acc_GET() == -2.4530798E38F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {5.491986E37F, 1.6231978E37F, -5.9401397E37F}));
            assert(pack.y_pos_GET() == -2.8182364E38F);
            assert(pack.x_pos_GET() == 3.3832615E38F);
        });
        DemoDevice.CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.q_SET(new float[] {-6.0512696E36F, 1.630465E38F, -1.1116246E38F, -1.5107733E38F}, 0) ;
        p146.roll_rate_SET(6.615205E36F) ;
        p146.z_acc_SET(-2.4530798E38F) ;
        p146.vel_variance_SET(new float[] {1.8302447E38F, 2.4199553E38F, 2.6383444E38F}, 0) ;
        p146.x_acc_SET(1.2858125E38F) ;
        p146.y_vel_SET(-3.0283696E38F) ;
        p146.time_usec_SET(6522924101989694147L) ;
        p146.z_vel_SET(-5.373503E36F) ;
        p146.pos_variance_SET(new float[] {5.491986E37F, 1.6231978E37F, -5.9401397E37F}, 0) ;
        p146.y_acc_SET(2.808612E38F) ;
        p146.yaw_rate_SET(-5.3879303E37F) ;
        p146.x_vel_SET(9.970374E37F) ;
        p146.z_pos_SET(2.1880167E37F) ;
        p146.x_pos_SET(3.3832615E38F) ;
        p146.airspeed_SET(2.596729E38F) ;
        p146.y_pos_SET(-2.8182364E38F) ;
        p146.pitch_rate_SET(2.6590014E38F) ;
        LoopBackDemoChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS);
            assert(pack.temperature_GET() == (short) -18099);
            assert(pack.current_consumed_GET() == -1772907782);
            assert(pack.id_GET() == (char)186);
            assert(pack.current_battery_GET() == (short)13861);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)45147, (char)25292, (char)44738, (char)12113, (char)53645, (char)56405, (char)12553, (char)55421, (char)43297, (char)9248}));
            assert(pack.energy_consumed_GET() == -1901960896);
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN);
            assert(pack.battery_remaining_GET() == (byte) - 117);
        });
        DemoDevice.BATTERY_STATUS p147 = LoopBackDemoChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.current_battery_SET((short)13861) ;
        p147.energy_consumed_SET(-1901960896) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN) ;
        p147.current_consumed_SET(-1772907782) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS) ;
        p147.voltages_SET(new char[] {(char)45147, (char)25292, (char)44738, (char)12113, (char)53645, (char)56405, (char)12553, (char)55421, (char)43297, (char)9248}, 0) ;
        p147.temperature_SET((short) -18099) ;
        p147.battery_remaining_SET((byte) - 117) ;
        p147.id_SET((char)186) ;
        LoopBackDemoChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)59, (char)180, (char)69, (char)239, (char)149, (char)211, (char)34, (char)136}));
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)117, (char)8, (char)51, (char)121, (char)253, (char)147, (char)208, (char)29}));
            assert(pack.product_id_GET() == (char)36412);
            assert(pack.board_version_GET() == 4069513852L);
            assert(pack.vendor_id_GET() == (char)55305);
            assert(pack.middleware_sw_version_GET() == 4157075430L);
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)249, (char)165, (char)25, (char)246, (char)220, (char)13, (char)151, (char)102, (char)40, (char)184, (char)235, (char)167, (char)97, (char)235, (char)165, (char)162, (char)171, (char)5}));
            assert(pack.uid_GET() == 5233415831972507517L);
            assert(pack.capabilities_GET() == MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET);
            assert(pack.flight_sw_version_GET() == 1416498550L);
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)11, (char)138, (char)97, (char)110, (char)216, (char)222, (char)157, (char)87}));
            assert(pack.os_sw_version_GET() == 3068879110L);
        });
        DemoDevice.AUTOPILOT_VERSION p148 = LoopBackDemoChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.vendor_id_SET((char)55305) ;
        p148.flight_custom_version_SET(new char[] {(char)11, (char)138, (char)97, (char)110, (char)216, (char)222, (char)157, (char)87}, 0) ;
        p148.os_custom_version_SET(new char[] {(char)117, (char)8, (char)51, (char)121, (char)253, (char)147, (char)208, (char)29}, 0) ;
        p148.middleware_sw_version_SET(4157075430L) ;
        p148.middleware_custom_version_SET(new char[] {(char)59, (char)180, (char)69, (char)239, (char)149, (char)211, (char)34, (char)136}, 0) ;
        p148.board_version_SET(4069513852L) ;
        p148.uid2_SET(new char[] {(char)249, (char)165, (char)25, (char)246, (char)220, (char)13, (char)151, (char)102, (char)40, (char)184, (char)235, (char)167, (char)97, (char)235, (char)165, (char)162, (char)171, (char)5}, 0, PH) ;
        p148.flight_sw_version_SET(1416498550L) ;
        p148.uid_SET(5233415831972507517L) ;
        p148.product_id_SET((char)36412) ;
        p148.os_sw_version_SET(3068879110L) ;
        p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET) ;
        LoopBackDemoChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.distance_GET() == 1.775642E38F);
            assert(pack.position_valid_TRY(ph) == (char)15);
            assert(pack.size_y_GET() == 2.1362164E38F);
            assert(pack.size_x_GET() == -2.1439902E38F);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {3.301182E38F, 2.1789867E38F, 2.8575485E38F, 9.467434E37F}));
            assert(pack.angle_x_GET() == -2.8484642E38F);
            assert(pack.x_TRY(ph) == -1.9556456E38F);
            assert(pack.z_TRY(ph) == 8.662398E37F);
            assert(pack.y_TRY(ph) == 1.8631004E38F);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
            assert(pack.angle_y_GET() == -2.642612E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            assert(pack.time_usec_GET() == 4335220723262225563L);
            assert(pack.target_num_GET() == (char)225);
        });
        DemoDevice.LANDING_TARGET p149 = LoopBackDemoChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.target_num_SET((char)225) ;
        p149.size_x_SET(-2.1439902E38F) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON) ;
        p149.size_y_SET(2.1362164E38F) ;
        p149.q_SET(new float[] {3.301182E38F, 2.1789867E38F, 2.8575485E38F, 9.467434E37F}, 0, PH) ;
        p149.angle_x_SET(-2.8484642E38F) ;
        p149.y_SET(1.8631004E38F, PH) ;
        p149.time_usec_SET(4335220723262225563L) ;
        p149.distance_SET(1.775642E38F) ;
        p149.x_SET(-1.9556456E38F, PH) ;
        p149.angle_y_SET(-2.642612E37F) ;
        p149.position_valid_SET((char)15, PH) ;
        p149.z_SET(8.662398E37F, PH) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        LoopBackDemoChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.pos_vert_ratio_GET() == 1.998289E38F);
            assert(pack.pos_vert_accuracy_GET() == -1.8824173E38F);
            assert(pack.pos_horiz_ratio_GET() == 2.4686224E38F);
            assert(pack.vel_ratio_GET() == -7.390789E37F);
            assert(pack.pos_horiz_accuracy_GET() == -7.8549006E36F);
            assert(pack.flags_GET() == ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE);
            assert(pack.hagl_ratio_GET() == -3.1522025E38F);
            assert(pack.tas_ratio_GET() == -2.8979292E38F);
            assert(pack.mag_ratio_GET() == -1.8050401E38F);
            assert(pack.time_usec_GET() == 2909963772815391858L);
        });
        DemoDevice.ESTIMATOR_STATUS p230 = LoopBackDemoChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.pos_vert_accuracy_SET(-1.8824173E38F) ;
        p230.time_usec_SET(2909963772815391858L) ;
        p230.vel_ratio_SET(-7.390789E37F) ;
        p230.hagl_ratio_SET(-3.1522025E38F) ;
        p230.pos_horiz_ratio_SET(2.4686224E38F) ;
        p230.pos_horiz_accuracy_SET(-7.8549006E36F) ;
        p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE) ;
        p230.pos_vert_ratio_SET(1.998289E38F) ;
        p230.tas_ratio_SET(-2.8979292E38F) ;
        p230.mag_ratio_SET(-1.8050401E38F) ;
        LoopBackDemoChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.vert_accuracy_GET() == -5.729186E37F);
            assert(pack.var_horiz_GET() == -2.7721194E37F);
            assert(pack.wind_x_GET() == -2.6163893E38F);
            assert(pack.var_vert_GET() == -3.1550195E38F);
            assert(pack.wind_y_GET() == 5.4783747E37F);
            assert(pack.horiz_accuracy_GET() == -2.5927174E37F);
            assert(pack.wind_alt_GET() == 3.0800042E38F);
            assert(pack.time_usec_GET() == 484042589334889302L);
            assert(pack.wind_z_GET() == 2.7126303E38F);
        });
        DemoDevice.WIND_COV p231 = LoopBackDemoChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.wind_z_SET(2.7126303E38F) ;
        p231.time_usec_SET(484042589334889302L) ;
        p231.var_vert_SET(-3.1550195E38F) ;
        p231.wind_alt_SET(3.0800042E38F) ;
        p231.horiz_accuracy_SET(-2.5927174E37F) ;
        p231.wind_y_SET(5.4783747E37F) ;
        p231.var_horiz_SET(-2.7721194E37F) ;
        p231.vert_accuracy_SET(-5.729186E37F) ;
        p231.wind_x_SET(-2.6163893E38F) ;
        LoopBackDemoChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.ve_GET() == -2.383463E38F);
            assert(pack.vdop_GET() == -3.3532249E38F);
            assert(pack.hdop_GET() == 1.8375679E38F);
            assert(pack.vn_GET() == -1.4540704E38F);
            assert(pack.speed_accuracy_GET() == -3.2358335E38F);
            assert(pack.lat_GET() == -725346034);
            assert(pack.time_week_GET() == (char)57210);
            assert(pack.time_week_ms_GET() == 3728898091L);
            assert(pack.vert_accuracy_GET() == -1.8893026E38F);
            assert(pack.satellites_visible_GET() == (char)83);
            assert(pack.lon_GET() == 1053783606);
            assert(pack.fix_type_GET() == (char)152);
            assert(pack.vd_GET() == -2.1690449E38F);
            assert(pack.ignore_flags_GET() == GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ);
            assert(pack.alt_GET() == -2.4809932E38F);
            assert(pack.time_usec_GET() == 5529877561249246643L);
            assert(pack.gps_id_GET() == (char)12);
            assert(pack.horiz_accuracy_GET() == 3.0864706E38F);
        });
        DemoDevice.GPS_INPUT p232 = LoopBackDemoChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.ve_SET(-2.383463E38F) ;
        p232.satellites_visible_SET((char)83) ;
        p232.vn_SET(-1.4540704E38F) ;
        p232.vert_accuracy_SET(-1.8893026E38F) ;
        p232.vdop_SET(-3.3532249E38F) ;
        p232.hdop_SET(1.8375679E38F) ;
        p232.lon_SET(1053783606) ;
        p232.alt_SET(-2.4809932E38F) ;
        p232.fix_type_SET((char)152) ;
        p232.time_week_SET((char)57210) ;
        p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ) ;
        p232.speed_accuracy_SET(-3.2358335E38F) ;
        p232.vd_SET(-2.1690449E38F) ;
        p232.horiz_accuracy_SET(3.0864706E38F) ;
        p232.time_week_ms_SET(3728898091L) ;
        p232.time_usec_SET(5529877561249246643L) ;
        p232.lat_SET(-725346034) ;
        p232.gps_id_SET((char)12) ;
        LoopBackDemoChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)0, (char)108, (char)200, (char)244, (char)31, (char)35, (char)46, (char)96, (char)239, (char)113, (char)225, (char)251, (char)13, (char)28, (char)229, (char)238, (char)104, (char)14, (char)169, (char)188, (char)217, (char)195, (char)21, (char)252, (char)242, (char)121, (char)131, (char)88, (char)111, (char)40, (char)186, (char)149, (char)95, (char)110, (char)128, (char)163, (char)209, (char)119, (char)234, (char)122, (char)33, (char)43, (char)218, (char)124, (char)144, (char)16, (char)131, (char)238, (char)165, (char)100, (char)41, (char)118, (char)149, (char)78, (char)197, (char)96, (char)162, (char)58, (char)231, (char)146, (char)163, (char)155, (char)223, (char)3, (char)171, (char)97, (char)191, (char)19, (char)62, (char)175, (char)242, (char)20, (char)205, (char)255, (char)43, (char)140, (char)11, (char)183, (char)172, (char)100, (char)99, (char)118, (char)237, (char)66, (char)11, (char)172, (char)202, (char)50, (char)41, (char)208, (char)71, (char)31, (char)234, (char)157, (char)40, (char)22, (char)99, (char)37, (char)94, (char)5, (char)7, (char)191, (char)175, (char)93, (char)105, (char)23, (char)143, (char)66, (char)116, (char)145, (char)59, (char)212, (char)164, (char)49, (char)118, (char)15, (char)48, (char)106, (char)247, (char)174, (char)86, (char)18, (char)21, (char)199, (char)185, (char)107, (char)76, (char)173, (char)4, (char)198, (char)170, (char)104, (char)179, (char)135, (char)150, (char)55, (char)83, (char)165, (char)94, (char)70, (char)77, (char)215, (char)86, (char)177, (char)10, (char)220, (char)246, (char)223, (char)61, (char)25, (char)198, (char)227, (char)233, (char)167, (char)102, (char)147, (char)209, (char)8, (char)34, (char)153, (char)209, (char)161, (char)235, (char)54, (char)180, (char)197, (char)233, (char)4, (char)79, (char)117, (char)121, (char)40, (char)229, (char)206, (char)25, (char)246, (char)188, (char)83, (char)109, (char)146}));
            assert(pack.len_GET() == (char)9);
            assert(pack.flags_GET() == (char)146);
        });
        DemoDevice.GPS_RTCM_DATA p233 = LoopBackDemoChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.data__SET(new char[] {(char)0, (char)108, (char)200, (char)244, (char)31, (char)35, (char)46, (char)96, (char)239, (char)113, (char)225, (char)251, (char)13, (char)28, (char)229, (char)238, (char)104, (char)14, (char)169, (char)188, (char)217, (char)195, (char)21, (char)252, (char)242, (char)121, (char)131, (char)88, (char)111, (char)40, (char)186, (char)149, (char)95, (char)110, (char)128, (char)163, (char)209, (char)119, (char)234, (char)122, (char)33, (char)43, (char)218, (char)124, (char)144, (char)16, (char)131, (char)238, (char)165, (char)100, (char)41, (char)118, (char)149, (char)78, (char)197, (char)96, (char)162, (char)58, (char)231, (char)146, (char)163, (char)155, (char)223, (char)3, (char)171, (char)97, (char)191, (char)19, (char)62, (char)175, (char)242, (char)20, (char)205, (char)255, (char)43, (char)140, (char)11, (char)183, (char)172, (char)100, (char)99, (char)118, (char)237, (char)66, (char)11, (char)172, (char)202, (char)50, (char)41, (char)208, (char)71, (char)31, (char)234, (char)157, (char)40, (char)22, (char)99, (char)37, (char)94, (char)5, (char)7, (char)191, (char)175, (char)93, (char)105, (char)23, (char)143, (char)66, (char)116, (char)145, (char)59, (char)212, (char)164, (char)49, (char)118, (char)15, (char)48, (char)106, (char)247, (char)174, (char)86, (char)18, (char)21, (char)199, (char)185, (char)107, (char)76, (char)173, (char)4, (char)198, (char)170, (char)104, (char)179, (char)135, (char)150, (char)55, (char)83, (char)165, (char)94, (char)70, (char)77, (char)215, (char)86, (char)177, (char)10, (char)220, (char)246, (char)223, (char)61, (char)25, (char)198, (char)227, (char)233, (char)167, (char)102, (char)147, (char)209, (char)8, (char)34, (char)153, (char)209, (char)161, (char)235, (char)54, (char)180, (char)197, (char)233, (char)4, (char)79, (char)117, (char)121, (char)40, (char)229, (char)206, (char)25, (char)246, (char)188, (char)83, (char)109, (char)146}, 0) ;
        p233.len_SET((char)9) ;
        p233.flags_SET((char)146) ;
        LoopBackDemoChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.airspeed_sp_GET() == (char)62);
            assert(pack.gps_nsat_GET() == (char)15);
            assert(pack.heading_GET() == (char)19178);
            assert(pack.wp_distance_GET() == (char)53651);
            assert(pack.airspeed_GET() == (char)195);
            assert(pack.failsafe_GET() == (char)5);
            assert(pack.temperature_GET() == (byte) - 117);
            assert(pack.custom_mode_GET() == 3899133568L);
            assert(pack.heading_sp_GET() == (short)30553);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED);
            assert(pack.latitude_GET() == -1583460579);
            assert(pack.altitude_amsl_GET() == (short) -27679);
            assert(pack.altitude_sp_GET() == (short)18299);
            assert(pack.pitch_GET() == (short) -14646);
            assert(pack.wp_num_GET() == (char)131);
            assert(pack.groundspeed_GET() == (char)118);
            assert(pack.battery_remaining_GET() == (char)2);
            assert(pack.roll_GET() == (short) -1075);
            assert(pack.throttle_GET() == (byte) - 23);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
            assert(pack.longitude_GET() == 391381479);
            assert(pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED);
            assert(pack.temperature_air_GET() == (byte)57);
            assert(pack.climb_rate_GET() == (byte) - 60);
        });
        DemoDevice.HIGH_LATENCY p234 = LoopBackDemoChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.climb_rate_SET((byte) - 60) ;
        p234.altitude_sp_SET((short)18299) ;
        p234.failsafe_SET((char)5) ;
        p234.pitch_SET((short) -14646) ;
        p234.wp_num_SET((char)131) ;
        p234.airspeed_SET((char)195) ;
        p234.longitude_SET(391381479) ;
        p234.latitude_SET(-1583460579) ;
        p234.throttle_SET((byte) - 23) ;
        p234.custom_mode_SET(3899133568L) ;
        p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED) ;
        p234.heading_sp_SET((short)30553) ;
        p234.roll_SET((short) -1075) ;
        p234.temperature_air_SET((byte)57) ;
        p234.battery_remaining_SET((char)2) ;
        p234.altitude_amsl_SET((short) -27679) ;
        p234.wp_distance_SET((char)53651) ;
        p234.groundspeed_SET((char)118) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF) ;
        p234.airspeed_sp_SET((char)62) ;
        p234.temperature_SET((byte) - 117) ;
        p234.heading_SET((char)19178) ;
        p234.gps_nsat_SET((char)15) ;
        LoopBackDemoChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.vibration_y_GET() == 3.0537368E38F);
            assert(pack.clipping_0_GET() == 2556898799L);
            assert(pack.time_usec_GET() == 656859242384321416L);
            assert(pack.vibration_z_GET() == -1.7144516E38F);
            assert(pack.vibration_x_GET() == -2.248985E38F);
            assert(pack.clipping_1_GET() == 3781701891L);
            assert(pack.clipping_2_GET() == 4205018743L);
        });
        DemoDevice.VIBRATION p241 = LoopBackDemoChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.vibration_x_SET(-2.248985E38F) ;
        p241.time_usec_SET(656859242384321416L) ;
        p241.vibration_z_SET(-1.7144516E38F) ;
        p241.clipping_1_SET(3781701891L) ;
        p241.clipping_2_SET(4205018743L) ;
        p241.clipping_0_SET(2556898799L) ;
        p241.vibration_y_SET(3.0537368E38F) ;
        LoopBackDemoChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.approach_y_GET() == 1.2453704E38F);
            assert(pack.altitude_GET() == 1649662525);
            assert(pack.approach_x_GET() == 3.0793338E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-4.6680483E37F, -1.2819922E38F, -2.1274317E38F, 5.5297023E37F}));
            assert(pack.approach_z_GET() == -3.3440605E37F);
            assert(pack.x_GET() == 3.3338825E38F);
            assert(pack.latitude_GET() == 1735575559);
            assert(pack.y_GET() == 2.5591802E35F);
            assert(pack.longitude_GET() == -562472665);
            assert(pack.time_usec_TRY(ph) == 6551274441048221576L);
            assert(pack.z_GET() == 1.8023901E37F);
        });
        DemoDevice.HOME_POSITION p242 = LoopBackDemoChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.latitude_SET(1735575559) ;
        p242.altitude_SET(1649662525) ;
        p242.time_usec_SET(6551274441048221576L, PH) ;
        p242.q_SET(new float[] {-4.6680483E37F, -1.2819922E38F, -2.1274317E38F, 5.5297023E37F}, 0) ;
        p242.approach_x_SET(3.0793338E38F) ;
        p242.longitude_SET(-562472665) ;
        p242.z_SET(1.8023901E37F) ;
        p242.x_SET(3.3338825E38F) ;
        p242.approach_z_SET(-3.3440605E37F) ;
        p242.approach_y_SET(1.2453704E38F) ;
        p242.y_SET(2.5591802E35F) ;
        LoopBackDemoChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.approach_x_GET() == -2.3063276E38F);
            assert(pack.target_system_GET() == (char)72);
            assert(pack.y_GET() == 1.5772427E38F);
            assert(pack.latitude_GET() == 190716996);
            assert(pack.approach_y_GET() == 1.7524318E38F);
            assert(pack.time_usec_TRY(ph) == 5466341373818627090L);
            assert(pack.x_GET() == 1.7167668E38F);
            assert(pack.approach_z_GET() == 1.8945367E38F);
            assert(pack.altitude_GET() == -872579993);
            assert(pack.z_GET() == -1.0836777E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.026747E38F, -1.5318886E38F, 1.6162551E38F, -3.1332558E38F}));
            assert(pack.longitude_GET() == 23295551);
        });
        DemoDevice.SET_HOME_POSITION p243 = LoopBackDemoChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.latitude_SET(190716996) ;
        p243.altitude_SET(-872579993) ;
        p243.approach_x_SET(-2.3063276E38F) ;
        p243.approach_y_SET(1.7524318E38F) ;
        p243.target_system_SET((char)72) ;
        p243.approach_z_SET(1.8945367E38F) ;
        p243.longitude_SET(23295551) ;
        p243.q_SET(new float[] {3.026747E38F, -1.5318886E38F, 1.6162551E38F, -3.1332558E38F}, 0) ;
        p243.z_SET(-1.0836777E37F) ;
        p243.x_SET(1.7167668E38F) ;
        p243.time_usec_SET(5466341373818627090L, PH) ;
        p243.y_SET(1.5772427E38F) ;
        LoopBackDemoChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.interval_us_GET() == 1633903692);
            assert(pack.message_id_GET() == (char)228);
        });
        DemoDevice.MESSAGE_INTERVAL p244 = LoopBackDemoChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.message_id_SET((char)228) ;
        p244.interval_us_SET(1633903692) ;
        LoopBackDemoChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED);
        });
        DemoDevice.EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED) ;
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR) ;
        LoopBackDemoChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.ver_velocity_GET() == (short)24066);
            assert(pack.ICAO_address_GET() == 3978617712L);
            assert(pack.tslc_GET() == (char)235);
            assert(pack.flags_GET() == ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN);
            assert(pack.lat_GET() == -1357729732);
            assert(pack.hor_velocity_GET() == (char)9174);
            assert(pack.callsign_LEN(ph) == 9);
            assert(pack.callsign_TRY(ph).equals("zsChbftbu"));
            assert(pack.heading_GET() == (char)54706);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_NO_INFO);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
            assert(pack.altitude_GET() == 8856791);
            assert(pack.squawk_GET() == (char)4127);
            assert(pack.lon_GET() == 1943361981);
        });
        DemoDevice.ADSB_VEHICLE p246 = LoopBackDemoChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.ver_velocity_SET((short)24066) ;
        p246.squawk_SET((char)4127) ;
        p246.heading_SET((char)54706) ;
        p246.lon_SET(1943361981) ;
        p246.tslc_SET((char)235) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC) ;
        p246.callsign_SET("zsChbftbu", PH) ;
        p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN) ;
        p246.altitude_SET(8856791) ;
        p246.hor_velocity_SET((char)9174) ;
        p246.lat_SET(-1357729732) ;
        p246.ICAO_address_SET(3978617712L) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_NO_INFO) ;
        LoopBackDemoChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == 236119083L);
            assert(pack.horizontal_minimum_delta_GET() == -1.0156541E38F);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
            assert(pack.altitude_minimum_delta_GET() == -1.0631171E38F);
            assert(pack.time_to_minimum_delta_GET() == -1.1139305E38F);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE);
        });
        DemoDevice.COLLISION p247 = LoopBackDemoChannel.new_COLLISION();
        PH.setPack(p247);
        p247.horizontal_minimum_delta_SET(-1.0156541E38F) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE) ;
        p247.altitude_minimum_delta_SET(-1.0631171E38F) ;
        p247.id_SET(236119083L) ;
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW) ;
        p247.time_to_minimum_delta_SET(-1.1139305E38F) ;
        LoopBackDemoChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)248);
            assert(pack.target_network_GET() == (char)171);
            assert(pack.target_component_GET() == (char)154);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)84, (char)165, (char)201, (char)67, (char)193, (char)71, (char)242, (char)227, (char)171, (char)234, (char)89, (char)186, (char)114, (char)189, (char)181, (char)224, (char)53, (char)234, (char)58, (char)27, (char)173, (char)236, (char)34, (char)155, (char)209, (char)213, (char)69, (char)209, (char)2, (char)167, (char)150, (char)107, (char)200, (char)96, (char)213, (char)249, (char)149, (char)206, (char)213, (char)38, (char)47, (char)95, (char)60, (char)222, (char)23, (char)121, (char)175, (char)87, (char)67, (char)172, (char)2, (char)221, (char)30, (char)101, (char)83, (char)56, (char)164, (char)55, (char)127, (char)20, (char)4, (char)112, (char)32, (char)85, (char)82, (char)58, (char)54, (char)243, (char)143, (char)155, (char)236, (char)155, (char)150, (char)91, (char)127, (char)89, (char)73, (char)62, (char)6, (char)169, (char)151, (char)199, (char)93, (char)13, (char)69, (char)255, (char)67, (char)202, (char)206, (char)19, (char)83, (char)90, (char)222, (char)211, (char)54, (char)201, (char)219, (char)128, (char)160, (char)46, (char)204, (char)144, (char)211, (char)145, (char)85, (char)89, (char)181, (char)188, (char)145, (char)189, (char)119, (char)219, (char)190, (char)243, (char)76, (char)17, (char)240, (char)254, (char)105, (char)192, (char)112, (char)9, (char)82, (char)202, (char)54, (char)151, (char)184, (char)183, (char)44, (char)45, (char)216, (char)196, (char)127, (char)36, (char)15, (char)109, (char)147, (char)221, (char)7, (char)124, (char)195, (char)245, (char)202, (char)195, (char)153, (char)255, (char)194, (char)128, (char)206, (char)181, (char)41, (char)165, (char)94, (char)35, (char)153, (char)155, (char)40, (char)97, (char)148, (char)225, (char)78, (char)44, (char)159, (char)46, (char)1, (char)113, (char)41, (char)139, (char)19, (char)147, (char)110, (char)146, (char)72, (char)18, (char)66, (char)53, (char)193, (char)51, (char)207, (char)69, (char)39, (char)109, (char)42, (char)46, (char)251, (char)248, (char)66, (char)66, (char)152, (char)181, (char)34, (char)84, (char)174, (char)234, (char)208, (char)245, (char)192, (char)18, (char)93, (char)124, (char)118, (char)150, (char)14, (char)148, (char)52, (char)38, (char)52, (char)140, (char)103, (char)251, (char)53, (char)153, (char)213, (char)84, (char)200, (char)50, (char)156, (char)99, (char)71, (char)169, (char)32, (char)165, (char)61, (char)140, (char)103, (char)189, (char)209, (char)193, (char)161, (char)149, (char)245, (char)135, (char)11, (char)222, (char)116, (char)114, (char)254, (char)136, (char)105, (char)114, (char)218, (char)147, (char)156, (char)178, (char)188, (char)35, (char)225, (char)190, (char)180}));
            assert(pack.message_type_GET() == (char)37744);
        });
        DemoDevice.V2_EXTENSION p248 = LoopBackDemoChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_system_SET((char)248) ;
        p248.target_network_SET((char)171) ;
        p248.payload_SET(new char[] {(char)84, (char)165, (char)201, (char)67, (char)193, (char)71, (char)242, (char)227, (char)171, (char)234, (char)89, (char)186, (char)114, (char)189, (char)181, (char)224, (char)53, (char)234, (char)58, (char)27, (char)173, (char)236, (char)34, (char)155, (char)209, (char)213, (char)69, (char)209, (char)2, (char)167, (char)150, (char)107, (char)200, (char)96, (char)213, (char)249, (char)149, (char)206, (char)213, (char)38, (char)47, (char)95, (char)60, (char)222, (char)23, (char)121, (char)175, (char)87, (char)67, (char)172, (char)2, (char)221, (char)30, (char)101, (char)83, (char)56, (char)164, (char)55, (char)127, (char)20, (char)4, (char)112, (char)32, (char)85, (char)82, (char)58, (char)54, (char)243, (char)143, (char)155, (char)236, (char)155, (char)150, (char)91, (char)127, (char)89, (char)73, (char)62, (char)6, (char)169, (char)151, (char)199, (char)93, (char)13, (char)69, (char)255, (char)67, (char)202, (char)206, (char)19, (char)83, (char)90, (char)222, (char)211, (char)54, (char)201, (char)219, (char)128, (char)160, (char)46, (char)204, (char)144, (char)211, (char)145, (char)85, (char)89, (char)181, (char)188, (char)145, (char)189, (char)119, (char)219, (char)190, (char)243, (char)76, (char)17, (char)240, (char)254, (char)105, (char)192, (char)112, (char)9, (char)82, (char)202, (char)54, (char)151, (char)184, (char)183, (char)44, (char)45, (char)216, (char)196, (char)127, (char)36, (char)15, (char)109, (char)147, (char)221, (char)7, (char)124, (char)195, (char)245, (char)202, (char)195, (char)153, (char)255, (char)194, (char)128, (char)206, (char)181, (char)41, (char)165, (char)94, (char)35, (char)153, (char)155, (char)40, (char)97, (char)148, (char)225, (char)78, (char)44, (char)159, (char)46, (char)1, (char)113, (char)41, (char)139, (char)19, (char)147, (char)110, (char)146, (char)72, (char)18, (char)66, (char)53, (char)193, (char)51, (char)207, (char)69, (char)39, (char)109, (char)42, (char)46, (char)251, (char)248, (char)66, (char)66, (char)152, (char)181, (char)34, (char)84, (char)174, (char)234, (char)208, (char)245, (char)192, (char)18, (char)93, (char)124, (char)118, (char)150, (char)14, (char)148, (char)52, (char)38, (char)52, (char)140, (char)103, (char)251, (char)53, (char)153, (char)213, (char)84, (char)200, (char)50, (char)156, (char)99, (char)71, (char)169, (char)32, (char)165, (char)61, (char)140, (char)103, (char)189, (char)209, (char)193, (char)161, (char)149, (char)245, (char)135, (char)11, (char)222, (char)116, (char)114, (char)254, (char)136, (char)105, (char)114, (char)218, (char)147, (char)156, (char)178, (char)188, (char)35, (char)225, (char)190, (char)180}, 0) ;
        p248.message_type_SET((char)37744) ;
        p248.target_component_SET((char)154) ;
        LoopBackDemoChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(pack.ver_GET() == (char)119);
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte)113, (byte) - 46, (byte) - 102, (byte)117, (byte)55, (byte)117, (byte)20, (byte)115, (byte) - 74, (byte) - 53, (byte) - 6, (byte) - 22, (byte)95, (byte)83, (byte) - 64, (byte) - 95, (byte)3, (byte)12, (byte) - 81, (byte) - 19, (byte)69, (byte) - 123, (byte)88, (byte) - 79, (byte) - 96, (byte) - 39, (byte) - 63, (byte)17, (byte) - 15, (byte) - 124, (byte) - 99, (byte)35}));
            assert(pack.type_GET() == (char)132);
            assert(pack.address_GET() == (char)11406);
        });
        DemoDevice.MEMORY_VECT p249 = LoopBackDemoChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.ver_SET((char)119) ;
        p249.value_SET(new byte[] {(byte)113, (byte) - 46, (byte) - 102, (byte)117, (byte)55, (byte)117, (byte)20, (byte)115, (byte) - 74, (byte) - 53, (byte) - 6, (byte) - 22, (byte)95, (byte)83, (byte) - 64, (byte) - 95, (byte)3, (byte)12, (byte) - 81, (byte) - 19, (byte)69, (byte) - 123, (byte)88, (byte) - 79, (byte) - 96, (byte) - 39, (byte) - 63, (byte)17, (byte) - 15, (byte) - 124, (byte) - 99, (byte)35}, 0) ;
        p249.type_SET((char)132) ;
        p249.address_SET((char)11406) ;
        LoopBackDemoChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 6.7703337E37F);
            assert(pack.time_usec_GET() == 8274839242593992576L);
            assert(pack.x_GET() == 1.7756651E38F);
            assert(pack.name_LEN(ph) == 10);
            assert(pack.name_TRY(ph).equals("QhWcgvcmNi"));
            assert(pack.z_GET() == -6.2988483E37F);
        });
        DemoDevice.DEBUG_VECT p250 = LoopBackDemoChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.z_SET(-6.2988483E37F) ;
        p250.name_SET("QhWcgvcmNi", PH) ;
        p250.time_usec_SET(8274839242593992576L) ;
        p250.x_SET(1.7756651E38F) ;
        p250.y_SET(6.7703337E37F) ;
        LoopBackDemoChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2418184291L);
            assert(pack.value_GET() == 1.5099161E37F);
            assert(pack.name_LEN(ph) == 1);
            assert(pack.name_TRY(ph).equals("s"));
        });
        DemoDevice.NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.name_SET("s", PH) ;
        p251.value_SET(1.5099161E37F) ;
        p251.time_boot_ms_SET(2418184291L) ;
        LoopBackDemoChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2060564596L);
            assert(pack.name_LEN(ph) == 5);
            assert(pack.name_TRY(ph).equals("qmyNj"));
            assert(pack.value_GET() == -1816211703);
        });
        DemoDevice.NAMED_VALUE_INT p252 = LoopBackDemoChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.time_boot_ms_SET(2060564596L) ;
        p252.value_SET(-1816211703) ;
        p252.name_SET("qmyNj", PH) ;
        LoopBackDemoChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.text_LEN(ph) == 24);
            assert(pack.text_TRY(ph).equals("yrardunQylhsylwhtmovzyit"));
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_CRITICAL);
        });
        DemoDevice.STATUSTEXT p253 = LoopBackDemoChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_CRITICAL) ;
        p253.text_SET("yrardunQylhsylwhtmovzyit", PH) ;
        LoopBackDemoChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == -2.113254E38F);
            assert(pack.ind_GET() == (char)38);
            assert(pack.time_boot_ms_GET() == 3560537021L);
        });
        DemoDevice.DEBUG p254 = LoopBackDemoChannel.new_DEBUG();
        PH.setPack(p254);
        p254.ind_SET((char)38) ;
        p254.time_boot_ms_SET(3560537021L) ;
        p254.value_SET(-2.113254E38F) ;
        LoopBackDemoChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)169);
            assert(pack.target_system_GET() == (char)178);
            assert(pack.initial_timestamp_GET() == 6645546161201952271L);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)45, (char)60, (char)70, (char)75, (char)150, (char)194, (char)46, (char)63, (char)172, (char)208, (char)80, (char)112, (char)31, (char)141, (char)87, (char)37, (char)205, (char)187, (char)137, (char)139, (char)55, (char)9, (char)227, (char)131, (char)96, (char)248, (char)55, (char)10, (char)192, (char)80, (char)13, (char)222}));
        });
        DemoDevice.SETUP_SIGNING p256 = LoopBackDemoChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_component_SET((char)169) ;
        p256.target_system_SET((char)178) ;
        p256.secret_key_SET(new char[] {(char)45, (char)60, (char)70, (char)75, (char)150, (char)194, (char)46, (char)63, (char)172, (char)208, (char)80, (char)112, (char)31, (char)141, (char)87, (char)37, (char)205, (char)187, (char)137, (char)139, (char)55, (char)9, (char)227, (char)131, (char)96, (char)248, (char)55, (char)10, (char)192, (char)80, (char)13, (char)222}, 0) ;
        p256.initial_timestamp_SET(6645546161201952271L) ;
        LoopBackDemoChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.state_GET() == (char)170);
            assert(pack.time_boot_ms_GET() == 3426613985L);
            assert(pack.last_change_ms_GET() == 2538272328L);
        });
        DemoDevice.BUTTON_CHANGE p257 = LoopBackDemoChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(3426613985L) ;
        p257.last_change_ms_SET(2538272328L) ;
        p257.state_SET((char)170) ;
        LoopBackDemoChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.tune_LEN(ph) == 21);
            assert(pack.tune_TRY(ph).equals("usbUlheiGpkookfxwezzb"));
            assert(pack.target_component_GET() == (char)94);
            assert(pack.target_system_GET() == (char)3);
        });
        DemoDevice.PLAY_TUNE p258 = LoopBackDemoChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.tune_SET("usbUlheiGpkookfxwezzb", PH) ;
        p258.target_component_SET((char)94) ;
        p258.target_system_SET((char)3) ;
        LoopBackDemoChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.resolution_h_GET() == (char)9267);
            assert(pack.focal_length_GET() == -7.5041625E37F);
            assert(pack.cam_definition_version_GET() == (char)57232);
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)20, (char)171, (char)93, (char)66, (char)230, (char)76, (char)175, (char)5, (char)219, (char)54, (char)245, (char)0, (char)210, (char)171, (char)94, (char)211, (char)99, (char)208, (char)118, (char)213, (char)79, (char)8, (char)245, (char)47, (char)74, (char)161, (char)234, (char)72, (char)168, (char)211, (char)38, (char)32}));
            assert(pack.time_boot_ms_GET() == 370788015L);
            assert(pack.flags_GET() == CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE);
            assert(pack.sensor_size_v_GET() == -1.0373853E38F);
            assert(pack.lens_id_GET() == (char)234);
            assert(pack.sensor_size_h_GET() == -8.206179E36F);
            assert(pack.resolution_v_GET() == (char)50688);
            assert(pack.firmware_version_GET() == 3595333582L);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)39, (char)56, (char)202, (char)200, (char)121, (char)115, (char)251, (char)89, (char)148, (char)179, (char)3, (char)135, (char)58, (char)250, (char)161, (char)138, (char)138, (char)3, (char)8, (char)182, (char)126, (char)13, (char)187, (char)190, (char)213, (char)183, (char)14, (char)79, (char)216, (char)134, (char)38, (char)129}));
            assert(pack.cam_definition_uri_LEN(ph) == 2);
            assert(pack.cam_definition_uri_TRY(ph).equals("ah"));
        });
        DemoDevice.CAMERA_INFORMATION p259 = LoopBackDemoChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.cam_definition_uri_SET("ah", PH) ;
        p259.time_boot_ms_SET(370788015L) ;
        p259.focal_length_SET(-7.5041625E37F) ;
        p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE) ;
        p259.sensor_size_h_SET(-8.206179E36F) ;
        p259.vendor_name_SET(new char[] {(char)39, (char)56, (char)202, (char)200, (char)121, (char)115, (char)251, (char)89, (char)148, (char)179, (char)3, (char)135, (char)58, (char)250, (char)161, (char)138, (char)138, (char)3, (char)8, (char)182, (char)126, (char)13, (char)187, (char)190, (char)213, (char)183, (char)14, (char)79, (char)216, (char)134, (char)38, (char)129}, 0) ;
        p259.firmware_version_SET(3595333582L) ;
        p259.lens_id_SET((char)234) ;
        p259.cam_definition_version_SET((char)57232) ;
        p259.sensor_size_v_SET(-1.0373853E38F) ;
        p259.resolution_h_SET((char)9267) ;
        p259.resolution_v_SET((char)50688) ;
        p259.model_name_SET(new char[] {(char)20, (char)171, (char)93, (char)66, (char)230, (char)76, (char)175, (char)5, (char)219, (char)54, (char)245, (char)0, (char)210, (char)171, (char)94, (char)211, (char)99, (char)208, (char)118, (char)213, (char)79, (char)8, (char)245, (char)47, (char)74, (char)161, (char)234, (char)72, (char)168, (char)211, (char)38, (char)32}, 0) ;
        LoopBackDemoChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 433394722L);
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_VIDEO);
        });
        DemoDevice.CAMERA_SETTINGS p260 = LoopBackDemoChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(433394722L) ;
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_VIDEO) ;
        LoopBackDemoChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.used_capacity_GET() == 5.2749446E37F);
            assert(pack.status_GET() == (char)126);
            assert(pack.storage_count_GET() == (char)206);
            assert(pack.time_boot_ms_GET() == 3478137088L);
            assert(pack.read_speed_GET() == -2.6492606E38F);
            assert(pack.total_capacity_GET() == -1.4265164E36F);
            assert(pack.write_speed_GET() == -3.8899692E37F);
            assert(pack.storage_id_GET() == (char)222);
            assert(pack.available_capacity_GET() == 2.6915715E38F);
        });
        DemoDevice.STORAGE_INFORMATION p261 = LoopBackDemoChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.time_boot_ms_SET(3478137088L) ;
        p261.storage_id_SET((char)222) ;
        p261.total_capacity_SET(-1.4265164E36F) ;
        p261.used_capacity_SET(5.2749446E37F) ;
        p261.available_capacity_SET(2.6915715E38F) ;
        p261.status_SET((char)126) ;
        p261.storage_count_SET((char)206) ;
        p261.read_speed_SET(-2.6492606E38F) ;
        p261.write_speed_SET(-3.8899692E37F) ;
        LoopBackDemoChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.image_interval_GET() == 1.35683E38F);
            assert(pack.available_capacity_GET() == 8.725555E37F);
            assert(pack.video_status_GET() == (char)5);
            assert(pack.time_boot_ms_GET() == 2942974811L);
            assert(pack.image_status_GET() == (char)237);
            assert(pack.recording_time_ms_GET() == 370520636L);
        });
        DemoDevice.CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.time_boot_ms_SET(2942974811L) ;
        p262.recording_time_ms_SET(370520636L) ;
        p262.image_status_SET((char)237) ;
        p262.video_status_SET((char)5) ;
        p262.image_interval_SET(1.35683E38F) ;
        p262.available_capacity_SET(8.725555E37F) ;
        LoopBackDemoChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.file_url_LEN(ph) == 197);
            assert(pack.file_url_TRY(ph).equals("bCydRtstsnjgadodkzbdwxesagjdGnfIooelieJCyFdNglxbpshkodhnjauvzqebufkmuriddcYbsasskmdfvquqhpoFPcmfkesLeqEqrbjccwiZfavLlikcdonfjpsrdqttlWjjufcdYugrtlBzyuseczwxcchHwiigsQkzUhZqjxqEdlzxthkzyucxuUZbqurls"));
            assert(pack.alt_GET() == 1234450671);
            assert(pack.camera_id_GET() == (char)197);
            assert(pack.lon_GET() == -1183794617);
            assert(pack.lat_GET() == -597325655);
            assert(pack.image_index_GET() == 1951405382);
            assert(pack.capture_result_GET() == (byte) - 89);
            assert(pack.relative_alt_GET() == -36976129);
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.3835355E38F, 3.283509E38F, 1.8024833E38F, 1.8568183E38F}));
            assert(pack.time_boot_ms_GET() == 2093753543L);
            assert(pack.time_utc_GET() == 7774936320576058447L);
        });
        DemoDevice.CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.camera_id_SET((char)197) ;
        p263.time_boot_ms_SET(2093753543L) ;
        p263.alt_SET(1234450671) ;
        p263.relative_alt_SET(-36976129) ;
        p263.q_SET(new float[] {3.3835355E38F, 3.283509E38F, 1.8024833E38F, 1.8568183E38F}, 0) ;
        p263.lat_SET(-597325655) ;
        p263.file_url_SET("bCydRtstsnjgadodkzbdwxesagjdGnfIooelieJCyFdNglxbpshkodhnjauvzqebufkmuriddcYbsasskmdfvquqhpoFPcmfkesLeqEqrbjccwiZfavLlikcdonfjpsrdqttlWjjufcdYugrtlBzyuseczwxcchHwiigsQkzUhZqjxqEdlzxthkzyucxuUZbqurls", PH) ;
        p263.time_utc_SET(7774936320576058447L) ;
        p263.image_index_SET(1951405382) ;
        p263.capture_result_SET((byte) - 89) ;
        p263.lon_SET(-1183794617) ;
        LoopBackDemoChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.takeoff_time_utc_GET() == 5601774760334522886L);
            assert(pack.arming_time_utc_GET() == 5109826532145067349L);
            assert(pack.flight_uuid_GET() == 2694570181735988269L);
            assert(pack.time_boot_ms_GET() == 1540607937L);
        });
        DemoDevice.FLIGHT_INFORMATION p264 = LoopBackDemoChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.takeoff_time_utc_SET(5601774760334522886L) ;
        p264.arming_time_utc_SET(5109826532145067349L) ;
        p264.flight_uuid_SET(2694570181735988269L) ;
        p264.time_boot_ms_SET(1540607937L) ;
        LoopBackDemoChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 3.0100179E38F);
            assert(pack.time_boot_ms_GET() == 2202368733L);
            assert(pack.roll_GET() == -1.6863035E38F);
            assert(pack.yaw_GET() == -2.4405759E38F);
        });
        DemoDevice.MOUNT_ORIENTATION p265 = LoopBackDemoChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.time_boot_ms_SET(2202368733L) ;
        p265.pitch_SET(3.0100179E38F) ;
        p265.roll_SET(-1.6863035E38F) ;
        p265.yaw_SET(-2.4405759E38F) ;
        LoopBackDemoChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)51, (char)15, (char)7, (char)21, (char)212, (char)66, (char)67, (char)70, (char)217, (char)29, (char)249, (char)214, (char)8, (char)55, (char)54, (char)91, (char)34, (char)64, (char)91, (char)139, (char)5, (char)149, (char)116, (char)197, (char)114, (char)91, (char)100, (char)173, (char)45, (char)77, (char)129, (char)75, (char)200, (char)253, (char)191, (char)194, (char)142, (char)77, (char)178, (char)57, (char)76, (char)42, (char)29, (char)167, (char)218, (char)76, (char)62, (char)73, (char)113, (char)161, (char)113, (char)124, (char)117, (char)167, (char)241, (char)48, (char)48, (char)172, (char)250, (char)37, (char)181, (char)201, (char)94, (char)38, (char)204, (char)205, (char)8, (char)75, (char)43, (char)102, (char)27, (char)134, (char)20, (char)131, (char)46, (char)106, (char)192, (char)221, (char)52, (char)125, (char)212, (char)201, (char)125, (char)155, (char)133, (char)175, (char)223, (char)133, (char)57, (char)242, (char)180, (char)214, (char)89, (char)21, (char)169, (char)142, (char)59, (char)215, (char)242, (char)3, (char)184, (char)217, (char)64, (char)223, (char)243, (char)42, (char)174, (char)225, (char)231, (char)54, (char)204, (char)110, (char)230, (char)71, (char)92, (char)38, (char)121, (char)27, (char)179, (char)15, (char)198, (char)129, (char)135, (char)202, (char)57, (char)164, (char)24, (char)119, (char)229, (char)103, (char)15, (char)27, (char)190, (char)125, (char)181, (char)26, (char)0, (char)66, (char)95, (char)62, (char)136, (char)110, (char)246, (char)137, (char)19, (char)190, (char)127, (char)176, (char)71, (char)124, (char)143, (char)248, (char)147, (char)204, (char)37, (char)241, (char)126, (char)15, (char)121, (char)78, (char)10, (char)111, (char)72, (char)96, (char)54, (char)78, (char)120, (char)59, (char)180, (char)18, (char)48, (char)117, (char)179, (char)93, (char)127, (char)53, (char)139, (char)62, (char)123, (char)61, (char)18, (char)36, (char)101, (char)134, (char)201, (char)123, (char)170, (char)88, (char)84, (char)10, (char)57, (char)71, (char)167, (char)87, (char)209, (char)188, (char)217, (char)238, (char)241, (char)44, (char)6, (char)231, (char)236, (char)166, (char)95, (char)228, (char)127, (char)147, (char)248, (char)42, (char)78, (char)250, (char)180, (char)99, (char)172, (char)40, (char)123, (char)97, (char)107, (char)40, (char)20, (char)210, (char)189, (char)55, (char)233, (char)78, (char)193, (char)5, (char)204, (char)176, (char)86, (char)132, (char)35, (char)223, (char)100, (char)158, (char)13, (char)14, (char)183, (char)112, (char)232, (char)67, (char)121, (char)57, (char)172, (char)144, (char)242, (char)62, (char)252}));
            assert(pack.sequence_GET() == (char)53463);
            assert(pack.first_message_offset_GET() == (char)80);
            assert(pack.length_GET() == (char)244);
            assert(pack.target_component_GET() == (char)20);
            assert(pack.target_system_GET() == (char)250);
        });
        DemoDevice.LOGGING_DATA p266 = LoopBackDemoChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.sequence_SET((char)53463) ;
        p266.length_SET((char)244) ;
        p266.target_component_SET((char)20) ;
        p266.first_message_offset_SET((char)80) ;
        p266.target_system_SET((char)250) ;
        p266.data__SET(new char[] {(char)51, (char)15, (char)7, (char)21, (char)212, (char)66, (char)67, (char)70, (char)217, (char)29, (char)249, (char)214, (char)8, (char)55, (char)54, (char)91, (char)34, (char)64, (char)91, (char)139, (char)5, (char)149, (char)116, (char)197, (char)114, (char)91, (char)100, (char)173, (char)45, (char)77, (char)129, (char)75, (char)200, (char)253, (char)191, (char)194, (char)142, (char)77, (char)178, (char)57, (char)76, (char)42, (char)29, (char)167, (char)218, (char)76, (char)62, (char)73, (char)113, (char)161, (char)113, (char)124, (char)117, (char)167, (char)241, (char)48, (char)48, (char)172, (char)250, (char)37, (char)181, (char)201, (char)94, (char)38, (char)204, (char)205, (char)8, (char)75, (char)43, (char)102, (char)27, (char)134, (char)20, (char)131, (char)46, (char)106, (char)192, (char)221, (char)52, (char)125, (char)212, (char)201, (char)125, (char)155, (char)133, (char)175, (char)223, (char)133, (char)57, (char)242, (char)180, (char)214, (char)89, (char)21, (char)169, (char)142, (char)59, (char)215, (char)242, (char)3, (char)184, (char)217, (char)64, (char)223, (char)243, (char)42, (char)174, (char)225, (char)231, (char)54, (char)204, (char)110, (char)230, (char)71, (char)92, (char)38, (char)121, (char)27, (char)179, (char)15, (char)198, (char)129, (char)135, (char)202, (char)57, (char)164, (char)24, (char)119, (char)229, (char)103, (char)15, (char)27, (char)190, (char)125, (char)181, (char)26, (char)0, (char)66, (char)95, (char)62, (char)136, (char)110, (char)246, (char)137, (char)19, (char)190, (char)127, (char)176, (char)71, (char)124, (char)143, (char)248, (char)147, (char)204, (char)37, (char)241, (char)126, (char)15, (char)121, (char)78, (char)10, (char)111, (char)72, (char)96, (char)54, (char)78, (char)120, (char)59, (char)180, (char)18, (char)48, (char)117, (char)179, (char)93, (char)127, (char)53, (char)139, (char)62, (char)123, (char)61, (char)18, (char)36, (char)101, (char)134, (char)201, (char)123, (char)170, (char)88, (char)84, (char)10, (char)57, (char)71, (char)167, (char)87, (char)209, (char)188, (char)217, (char)238, (char)241, (char)44, (char)6, (char)231, (char)236, (char)166, (char)95, (char)228, (char)127, (char)147, (char)248, (char)42, (char)78, (char)250, (char)180, (char)99, (char)172, (char)40, (char)123, (char)97, (char)107, (char)40, (char)20, (char)210, (char)189, (char)55, (char)233, (char)78, (char)193, (char)5, (char)204, (char)176, (char)86, (char)132, (char)35, (char)223, (char)100, (char)158, (char)13, (char)14, (char)183, (char)112, (char)232, (char)67, (char)121, (char)57, (char)172, (char)144, (char)242, (char)62, (char)252}, 0) ;
        LoopBackDemoChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)97, (char)163, (char)153, (char)115, (char)218, (char)213, (char)241, (char)50, (char)6, (char)225, (char)72, (char)104, (char)52, (char)167, (char)199, (char)195, (char)69, (char)111, (char)43, (char)245, (char)122, (char)102, (char)8, (char)96, (char)20, (char)118, (char)9, (char)38, (char)1, (char)170, (char)143, (char)11, (char)251, (char)137, (char)73, (char)213, (char)244, (char)13, (char)24, (char)110, (char)168, (char)129, (char)82, (char)87, (char)251, (char)86, (char)12, (char)195, (char)137, (char)52, (char)47, (char)42, (char)198, (char)231, (char)113, (char)122, (char)203, (char)238, (char)201, (char)76, (char)252, (char)104, (char)216, (char)116, (char)121, (char)153, (char)169, (char)220, (char)136, (char)37, (char)57, (char)68, (char)106, (char)247, (char)47, (char)9, (char)38, (char)249, (char)183, (char)191, (char)215, (char)130, (char)165, (char)74, (char)209, (char)84, (char)62, (char)11, (char)84, (char)163, (char)64, (char)174, (char)133, (char)100, (char)184, (char)22, (char)108, (char)162, (char)101, (char)66, (char)133, (char)124, (char)131, (char)14, (char)153, (char)225, (char)145, (char)215, (char)241, (char)94, (char)255, (char)103, (char)194, (char)115, (char)228, (char)248, (char)117, (char)78, (char)53, (char)212, (char)82, (char)9, (char)147, (char)107, (char)136, (char)91, (char)193, (char)158, (char)245, (char)175, (char)233, (char)133, (char)249, (char)0, (char)240, (char)99, (char)57, (char)31, (char)111, (char)46, (char)159, (char)150, (char)140, (char)134, (char)4, (char)247, (char)1, (char)208, (char)71, (char)149, (char)23, (char)7, (char)200, (char)229, (char)160, (char)26, (char)209, (char)196, (char)66, (char)150, (char)211, (char)29, (char)71, (char)101, (char)233, (char)154, (char)239, (char)87, (char)242, (char)30, (char)117, (char)11, (char)154, (char)182, (char)157, (char)152, (char)138, (char)119, (char)73, (char)81, (char)51, (char)210, (char)4, (char)13, (char)53, (char)96, (char)7, (char)207, (char)76, (char)74, (char)53, (char)73, (char)29, (char)161, (char)6, (char)178, (char)162, (char)76, (char)23, (char)50, (char)30, (char)193, (char)1, (char)200, (char)197, (char)125, (char)105, (char)241, (char)34, (char)34, (char)174, (char)157, (char)97, (char)129, (char)149, (char)42, (char)20, (char)124, (char)136, (char)72, (char)169, (char)82, (char)151, (char)86, (char)109, (char)183, (char)188, (char)99, (char)198, (char)36, (char)235, (char)119, (char)75, (char)81, (char)254, (char)218, (char)52, (char)205, (char)42, (char)67, (char)230, (char)86, (char)147, (char)47, (char)127, (char)180, (char)15, (char)117, (char)209}));
            assert(pack.length_GET() == (char)183);
            assert(pack.target_system_GET() == (char)80);
            assert(pack.sequence_GET() == (char)17055);
            assert(pack.target_component_GET() == (char)142);
            assert(pack.first_message_offset_GET() == (char)97);
        });
        DemoDevice.LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.length_SET((char)183) ;
        p267.target_system_SET((char)80) ;
        p267.data__SET(new char[] {(char)97, (char)163, (char)153, (char)115, (char)218, (char)213, (char)241, (char)50, (char)6, (char)225, (char)72, (char)104, (char)52, (char)167, (char)199, (char)195, (char)69, (char)111, (char)43, (char)245, (char)122, (char)102, (char)8, (char)96, (char)20, (char)118, (char)9, (char)38, (char)1, (char)170, (char)143, (char)11, (char)251, (char)137, (char)73, (char)213, (char)244, (char)13, (char)24, (char)110, (char)168, (char)129, (char)82, (char)87, (char)251, (char)86, (char)12, (char)195, (char)137, (char)52, (char)47, (char)42, (char)198, (char)231, (char)113, (char)122, (char)203, (char)238, (char)201, (char)76, (char)252, (char)104, (char)216, (char)116, (char)121, (char)153, (char)169, (char)220, (char)136, (char)37, (char)57, (char)68, (char)106, (char)247, (char)47, (char)9, (char)38, (char)249, (char)183, (char)191, (char)215, (char)130, (char)165, (char)74, (char)209, (char)84, (char)62, (char)11, (char)84, (char)163, (char)64, (char)174, (char)133, (char)100, (char)184, (char)22, (char)108, (char)162, (char)101, (char)66, (char)133, (char)124, (char)131, (char)14, (char)153, (char)225, (char)145, (char)215, (char)241, (char)94, (char)255, (char)103, (char)194, (char)115, (char)228, (char)248, (char)117, (char)78, (char)53, (char)212, (char)82, (char)9, (char)147, (char)107, (char)136, (char)91, (char)193, (char)158, (char)245, (char)175, (char)233, (char)133, (char)249, (char)0, (char)240, (char)99, (char)57, (char)31, (char)111, (char)46, (char)159, (char)150, (char)140, (char)134, (char)4, (char)247, (char)1, (char)208, (char)71, (char)149, (char)23, (char)7, (char)200, (char)229, (char)160, (char)26, (char)209, (char)196, (char)66, (char)150, (char)211, (char)29, (char)71, (char)101, (char)233, (char)154, (char)239, (char)87, (char)242, (char)30, (char)117, (char)11, (char)154, (char)182, (char)157, (char)152, (char)138, (char)119, (char)73, (char)81, (char)51, (char)210, (char)4, (char)13, (char)53, (char)96, (char)7, (char)207, (char)76, (char)74, (char)53, (char)73, (char)29, (char)161, (char)6, (char)178, (char)162, (char)76, (char)23, (char)50, (char)30, (char)193, (char)1, (char)200, (char)197, (char)125, (char)105, (char)241, (char)34, (char)34, (char)174, (char)157, (char)97, (char)129, (char)149, (char)42, (char)20, (char)124, (char)136, (char)72, (char)169, (char)82, (char)151, (char)86, (char)109, (char)183, (char)188, (char)99, (char)198, (char)36, (char)235, (char)119, (char)75, (char)81, (char)254, (char)218, (char)52, (char)205, (char)42, (char)67, (char)230, (char)86, (char)147, (char)47, (char)127, (char)180, (char)15, (char)117, (char)209}, 0) ;
        p267.first_message_offset_SET((char)97) ;
        p267.target_component_SET((char)142) ;
        p267.sequence_SET((char)17055) ;
        LoopBackDemoChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.sequence_GET() == (char)63463);
            assert(pack.target_system_GET() == (char)27);
            assert(pack.target_component_GET() == (char)67);
        });
        DemoDevice.LOGGING_ACK p268 = LoopBackDemoChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)27) ;
        p268.target_component_SET((char)67) ;
        p268.sequence_SET((char)63463) ;
        LoopBackDemoChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.camera_id_GET() == (char)36);
            assert(pack.resolution_h_GET() == (char)35475);
            assert(pack.bitrate_GET() == 1405013510L);
            assert(pack.uri_LEN(ph) == 27);
            assert(pack.uri_TRY(ph).equals("ohzznZgUwvecxhnVonAxhnopZqo"));
            assert(pack.status_GET() == (char)79);
            assert(pack.resolution_v_GET() == (char)10437);
            assert(pack.framerate_GET() == -2.8680201E38F);
            assert(pack.rotation_GET() == (char)63013);
        });
        DemoDevice.VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.camera_id_SET((char)36) ;
        p269.bitrate_SET(1405013510L) ;
        p269.resolution_h_SET((char)35475) ;
        p269.uri_SET("ohzznZgUwvecxhnVonAxhnopZqo", PH) ;
        p269.resolution_v_SET((char)10437) ;
        p269.rotation_SET((char)63013) ;
        p269.framerate_SET(-2.8680201E38F) ;
        p269.status_SET((char)79) ;
        LoopBackDemoChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)74);
            assert(pack.camera_id_GET() == (char)116);
            assert(pack.uri_LEN(ph) == 100);
            assert(pack.uri_TRY(ph).equals("BlmyoSfbppukeYFyNcsbxLvscqiwMoysptsjutQDhSnLzqsjTvbnwuumdzoeoebqrgrusjswxxjhwzmdxfevgcrbFkxgklXelpnr"));
            assert(pack.target_component_GET() == (char)11);
            assert(pack.framerate_GET() == -2.7999765E38F);
            assert(pack.bitrate_GET() == 3142049947L);
            assert(pack.rotation_GET() == (char)22615);
            assert(pack.resolution_v_GET() == (char)36160);
            assert(pack.resolution_h_GET() == (char)51331);
        });
        DemoDevice.SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.resolution_h_SET((char)51331) ;
        p270.rotation_SET((char)22615) ;
        p270.framerate_SET(-2.7999765E38F) ;
        p270.camera_id_SET((char)116) ;
        p270.target_component_SET((char)11) ;
        p270.uri_SET("BlmyoSfbppukeYFyNcsbxLvscqiwMoysptsjutQDhSnLzqsjTvbnwuumdzoeoebqrgrusjswxxjhwzmdxfevgcrbFkxgklXelpnr", PH) ;
        p270.resolution_v_SET((char)36160) ;
        p270.target_system_SET((char)74) ;
        p270.bitrate_SET(3142049947L) ;
        LoopBackDemoChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.ssid_LEN(ph) == 3);
            assert(pack.ssid_TRY(ph).equals("vms"));
            assert(pack.password_LEN(ph) == 54);
            assert(pack.password_TRY(ph).equals("lfdkLhuOqtnlqcjrgpftRnpcolrqzxhfjgjpIkjbyqgctndfmbfywl"));
        });
        DemoDevice.WIFI_CONFIG_AP p299 = LoopBackDemoChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("vms", PH) ;
        p299.password_SET("lfdkLhuOqtnlqcjrgpftRnpcolrqzxhfjgjpIkjbyqgctndfmbfywl", PH) ;
        LoopBackDemoChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(pack.version_GET() == (char)54758);
            assert(pack.max_version_GET() == (char)16983);
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)177, (char)37, (char)180, (char)88, (char)45, (char)170, (char)121, (char)18}));
            assert(pack.min_version_GET() == (char)44150);
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)157, (char)59, (char)225, (char)223, (char)140, (char)43, (char)7, (char)225}));
        });
        DemoDevice.PROTOCOL_VERSION p300 = LoopBackDemoChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.min_version_SET((char)44150) ;
        p300.library_version_hash_SET(new char[] {(char)177, (char)37, (char)180, (char)88, (char)45, (char)170, (char)121, (char)18}, 0) ;
        p300.version_SET((char)54758) ;
        p300.max_version_SET((char)16983) ;
        p300.spec_version_hash_SET(new char[] {(char)157, (char)59, (char)225, (char)223, (char)140, (char)43, (char)7, (char)225}, 0) ;
        LoopBackDemoChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE);
            assert(pack.uptime_sec_GET() == 557668542L);
            assert(pack.vendor_specific_status_code_GET() == (char)1560);
            assert(pack.sub_mode_GET() == (char)238);
            assert(pack.time_usec_GET() == 3681658930091861122L);
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK);
        });
        DemoDevice.UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.uptime_sec_SET(557668542L) ;
        p310.vendor_specific_status_code_SET((char)1560) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE) ;
        p310.sub_mode_SET((char)238) ;
        p310.time_usec_SET(3681658930091861122L) ;
        LoopBackDemoChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.sw_version_minor_GET() == (char)248);
            assert(pack.name_LEN(ph) == 55);
            assert(pack.name_TRY(ph).equals("mrecwUaWqtxBlpXedumwenuwgQxtobpjpSzcaoyzepQeCqhjmtqpdxj"));
            assert(pack.time_usec_GET() == 2659276486474300399L);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)135, (char)163, (char)37, (char)13, (char)35, (char)120, (char)152, (char)191, (char)197, (char)89, (char)96, (char)32, (char)123, (char)243, (char)194, (char)131}));
            assert(pack.hw_version_major_GET() == (char)113);
            assert(pack.sw_vcs_commit_GET() == 247821270L);
            assert(pack.uptime_sec_GET() == 3749264285L);
            assert(pack.hw_version_minor_GET() == (char)23);
            assert(pack.sw_version_major_GET() == (char)16);
        });
        DemoDevice.UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.sw_version_minor_SET((char)248) ;
        p311.sw_vcs_commit_SET(247821270L) ;
        p311.sw_version_major_SET((char)16) ;
        p311.hw_unique_id_SET(new char[] {(char)135, (char)163, (char)37, (char)13, (char)35, (char)120, (char)152, (char)191, (char)197, (char)89, (char)96, (char)32, (char)123, (char)243, (char)194, (char)131}, 0) ;
        p311.time_usec_SET(2659276486474300399L) ;
        p311.hw_version_major_SET((char)113) ;
        p311.name_SET("mrecwUaWqtxBlpXedumwenuwgQxtobpjpSzcaoyzepQeCqhjmtqpdxj", PH) ;
        p311.hw_version_minor_SET((char)23) ;
        p311.uptime_sec_SET(3749264285L) ;
        LoopBackDemoChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 6);
            assert(pack.param_id_TRY(ph).equals("gmGPlT"));
            assert(pack.target_component_GET() == (char)225);
            assert(pack.target_system_GET() == (char)119);
            assert(pack.param_index_GET() == (short)21300);
        });
        DemoDevice.PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.param_index_SET((short)21300) ;
        p320.param_id_SET("gmGPlT", PH) ;
        p320.target_component_SET((char)225) ;
        p320.target_system_SET((char)119) ;
        LoopBackDemoChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)176);
            assert(pack.target_component_GET() == (char)19);
        });
        DemoDevice.PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)176) ;
        p321.target_component_SET((char)19) ;
        LoopBackDemoChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_index_GET() == (char)10165);
            assert(pack.param_value_LEN(ph) == 80);
            assert(pack.param_value_TRY(ph).equals("nycsjylqtgldwrkatfwRfgieixjzswvoRfuodtygomjfPkhmwzwopbNrbpCdfdyhgpkskxbmpmeeqmvj"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
            assert(pack.param_id_LEN(ph) == 14);
            assert(pack.param_id_TRY(ph).equals("otasniqenvcwzy"));
            assert(pack.param_count_GET() == (char)51554);
        });
        DemoDevice.PARAM_EXT_VALUE p322 = LoopBackDemoChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_count_SET((char)51554) ;
        p322.param_index_SET((char)10165) ;
        p322.param_value_SET("nycsjylqtgldwrkatfwRfgieixjzswvoRfuodtygomjfPkhmwzwopbNrbpCdfdyhgpkskxbmpmeeqmvj", PH) ;
        p322.param_id_SET("otasniqenvcwzy", PH) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16) ;
        LoopBackDemoChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)123);
            assert(pack.param_value_LEN(ph) == 96);
            assert(pack.param_value_TRY(ph).equals("gfwlroqkcnvsvFunpukslexqdhdtFbeAotwrxjaftnsdlvjhfUhmtomghLgpksikgaoqxdwgxhrmTyczdqzkcsoznmlhtlos"));
            assert(pack.target_system_GET() == (char)117);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16);
            assert(pack.param_id_LEN(ph) == 15);
            assert(pack.param_id_TRY(ph).equals("jxJhdinZxsdjfwt"));
        });
        DemoDevice.PARAM_EXT_SET p323 = LoopBackDemoChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_system_SET((char)117) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16) ;
        p323.target_component_SET((char)123) ;
        p323.param_value_SET("gfwlroqkcnvsvFunpukslexqdhdtFbeAotwrxjaftnsdlvjhfUhmtomghLgpksikgaoqxdwgxhrmTyczdqzkcsoznmlhtlos", PH) ;
        p323.param_id_SET("jxJhdinZxsdjfwt", PH) ;
        LoopBackDemoChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 5);
            assert(pack.param_id_TRY(ph).equals("shfml"));
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_FAILED);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
            assert(pack.param_value_LEN(ph) == 56);
            assert(pack.param_value_TRY(ph).equals("meyzwpygtlmwzbvmmbfqLszyljjuvhzvuudgfzsywmoqGieCnqzzbfkx"));
        });
        DemoDevice.PARAM_EXT_ACK p324 = LoopBackDemoChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_FAILED) ;
        p324.param_id_SET("shfml", PH) ;
        p324.param_value_SET("meyzwpygtlmwzbvmmbfqLszyljjuvhzvuudgfzsywmoqGieCnqzzbfkx", PH) ;
        LoopBackDemoChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
            assert(pack.min_distance_GET() == (char)27575);
            assert(pack.increment_GET() == (char)81);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)23955, (char)48377, (char)49882, (char)1622, (char)63083, (char)32588, (char)32626, (char)4803, (char)21313, (char)23856, (char)54313, (char)22321, (char)29065, (char)60854, (char)32649, (char)61343, (char)45617, (char)60629, (char)38482, (char)16322, (char)2067, (char)52524, (char)56732, (char)7638, (char)700, (char)26996, (char)29129, (char)11748, (char)9381, (char)12292, (char)11073, (char)10517, (char)52078, (char)14260, (char)33748, (char)11698, (char)1849, (char)46480, (char)30922, (char)52515, (char)27734, (char)55862, (char)65422, (char)50824, (char)26908, (char)10810, (char)27869, (char)35780, (char)30693, (char)39260, (char)28456, (char)54071, (char)16637, (char)18662, (char)9493, (char)61314, (char)63414, (char)584, (char)15678, (char)65089, (char)47587, (char)10754, (char)10695, (char)93, (char)62697, (char)23552, (char)40585, (char)36946, (char)41022, (char)37514, (char)35394, (char)17468}));
            assert(pack.max_distance_GET() == (char)26692);
            assert(pack.time_usec_GET() == 6054796966471520290L);
        });
        DemoDevice.OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.distances_SET(new char[] {(char)23955, (char)48377, (char)49882, (char)1622, (char)63083, (char)32588, (char)32626, (char)4803, (char)21313, (char)23856, (char)54313, (char)22321, (char)29065, (char)60854, (char)32649, (char)61343, (char)45617, (char)60629, (char)38482, (char)16322, (char)2067, (char)52524, (char)56732, (char)7638, (char)700, (char)26996, (char)29129, (char)11748, (char)9381, (char)12292, (char)11073, (char)10517, (char)52078, (char)14260, (char)33748, (char)11698, (char)1849, (char)46480, (char)30922, (char)52515, (char)27734, (char)55862, (char)65422, (char)50824, (char)26908, (char)10810, (char)27869, (char)35780, (char)30693, (char)39260, (char)28456, (char)54071, (char)16637, (char)18662, (char)9493, (char)61314, (char)63414, (char)584, (char)15678, (char)65089, (char)47587, (char)10754, (char)10695, (char)93, (char)62697, (char)23552, (char)40585, (char)36946, (char)41022, (char)37514, (char)35394, (char)17468}, 0) ;
        p330.max_distance_SET((char)26692) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER) ;
        p330.time_usec_SET(6054796966471520290L) ;
        p330.increment_SET((char)81) ;
        p330.min_distance_SET((char)27575) ;
        LoopBackDemoChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
    }

}