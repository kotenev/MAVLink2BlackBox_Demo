
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
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_OPENPILOT);
            assert(pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED);
            assert(pack.custom_mode_GET() == 1238365624L);
            assert(pack.mavlink_version_GET() == (char)46);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_ANTENNA_TRACKER);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_ACTIVE);
        });
        DemoDevice.HEARTBEAT p0 = LoopBackDemoChannel.new_HEARTBEAT();
        PH.setPack(p0);
        p0.system_status_SET(MAV_STATE.MAV_STATE_ACTIVE) ;
        p0.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_OPENPILOT) ;
        p0.mavlink_version_SET((char)46) ;
        p0.custom_mode_SET(1238365624L) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_ANTENNA_TRACKER) ;
        LoopBackDemoChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.errors_count3_GET() == (char)13440);
            assert(pack.load_GET() == (char)3359);
            assert(pack.voltage_battery_GET() == (char)52270);
            assert(pack.battery_remaining_GET() == (byte)67);
            assert(pack.errors_count1_GET() == (char)56403);
            assert(pack.onboard_control_sensors_health_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL);
            assert(pack.errors_comm_GET() == (char)30625);
            assert(pack.onboard_control_sensors_enabled_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2);
            assert(pack.errors_count2_GET() == (char)58691);
            assert(pack.onboard_control_sensors_present_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION);
            assert(pack.current_battery_GET() == (short)28467);
            assert(pack.drop_rate_comm_GET() == (char)35539);
            assert(pack.errors_count4_GET() == (char)61380);
        });
        DemoDevice.SYS_STATUS p1 = LoopBackDemoChannel.new_SYS_STATUS();
        PH.setPack(p1);
        p1.onboard_control_sensors_enabled_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2) ;
        p1.errors_comm_SET((char)30625) ;
        p1.battery_remaining_SET((byte)67) ;
        p1.errors_count1_SET((char)56403) ;
        p1.load_SET((char)3359) ;
        p1.drop_rate_comm_SET((char)35539) ;
        p1.voltage_battery_SET((char)52270) ;
        p1.onboard_control_sensors_present_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION) ;
        p1.errors_count4_SET((char)61380) ;
        p1.current_battery_SET((short)28467) ;
        p1.onboard_control_sensors_health_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL) ;
        p1.errors_count3_SET((char)13440) ;
        p1.errors_count2_SET((char)58691) ;
        LoopBackDemoChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 128382738L);
            assert(pack.time_unix_usec_GET() == 4578867094953783594L);
        });
        DemoDevice.SYSTEM_TIME p2 = LoopBackDemoChannel.new_SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_boot_ms_SET(128382738L) ;
        p2.time_unix_usec_SET(4578867094953783594L) ;
        LoopBackDemoChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.type_mask_GET() == (char)19822);
            assert(pack.afz_GET() == 1.2258298E38F);
            assert(pack.yaw_GET() == -2.8082639E38F);
            assert(pack.x_GET() == 6.3971075E37F);
            assert(pack.vz_GET() == 5.2188308E35F);
            assert(pack.vx_GET() == 5.027714E37F);
            assert(pack.afx_GET() == -1.72906E37F);
            assert(pack.afy_GET() == 4.484936E37F);
            assert(pack.time_boot_ms_GET() == 3468624742L);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.y_GET() == 2.6902012E38F);
            assert(pack.z_GET() == 2.3704843E38F);
            assert(pack.vy_GET() == 2.6169493E38F);
            assert(pack.yaw_rate_GET() == 1.7789578E38F);
        });
        DemoDevice.POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.afx_SET(-1.72906E37F) ;
        p3.y_SET(2.6902012E38F) ;
        p3.time_boot_ms_SET(3468624742L) ;
        p3.yaw_rate_SET(1.7789578E38F) ;
        p3.vx_SET(5.027714E37F) ;
        p3.type_mask_SET((char)19822) ;
        p3.yaw_SET(-2.8082639E38F) ;
        p3.z_SET(2.3704843E38F) ;
        p3.vz_SET(5.2188308E35F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p3.afy_SET(4.484936E37F) ;
        p3.x_SET(6.3971075E37F) ;
        p3.afz_SET(1.2258298E38F) ;
        p3.vy_SET(2.6169493E38F) ;
        LoopBackDemoChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)213);
            assert(pack.time_usec_GET() == 8943590343273501502L);
            assert(pack.seq_GET() == 1051366915L);
            assert(pack.target_component_GET() == (char)12);
        });
        DemoDevice.PING p4 = LoopBackDemoChannel.new_PING();
        PH.setPack(p4);
        p4.seq_SET(1051366915L) ;
        p4.target_system_SET((char)213) ;
        p4.target_component_SET((char)12) ;
        p4.time_usec_SET(8943590343273501502L) ;
        LoopBackDemoChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.version_GET() == (char)210);
            assert(pack.control_request_GET() == (char)147);
            assert(pack.passkey_LEN(ph) == 10);
            assert(pack.passkey_TRY(ph).equals("slfnsytpkj"));
            assert(pack.target_system_GET() == (char)179);
        });
        DemoDevice.CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.control_request_SET((char)147) ;
        p5.passkey_SET("slfnsytpkj", PH) ;
        p5.target_system_SET((char)179) ;
        p5.version_SET((char)210) ;
        LoopBackDemoChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.control_request_GET() == (char)188);
            assert(pack.gcs_system_id_GET() == (char)22);
            assert(pack.ack_GET() == (char)101);
        });
        DemoDevice.CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.ack_SET((char)101) ;
        p6.control_request_SET((char)188) ;
        p6.gcs_system_id_SET((char)22) ;
        LoopBackDemoChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 2);
            assert(pack.key_TRY(ph).equals("mv"));
        });
        DemoDevice.AUTH_KEY p7 = LoopBackDemoChannel.new_AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("mv", PH) ;
        LoopBackDemoChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.custom_mode_GET() == 1483511700L);
            assert(pack.target_system_GET() == (char)229);
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
        });
        DemoDevice.SET_MODE p11 = LoopBackDemoChannel.new_SET_MODE();
        PH.setPack(p11);
        p11.base_mode_SET(MAV_MODE.MAV_MODE_STABILIZE_DISARMED) ;
        p11.custom_mode_SET(1483511700L) ;
        p11.target_system_SET((char)229) ;
        LoopBackDemoChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)93);
            assert(pack.param_id_LEN(ph) == 8);
            assert(pack.param_id_TRY(ph).equals("tnupooap"));
            assert(pack.param_index_GET() == (short)26308);
            assert(pack.target_component_GET() == (char)17);
        });
        DemoDevice.PARAM_REQUEST_READ p20 = LoopBackDemoChannel.new_PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.param_index_SET((short)26308) ;
        p20.target_component_SET((char)17) ;
        p20.param_id_SET("tnupooap", PH) ;
        p20.target_system_SET((char)93) ;
        LoopBackDemoChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)107);
            assert(pack.target_component_GET() == (char)196);
        });
        DemoDevice.PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.new_PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_system_SET((char)107) ;
        p21.target_component_SET((char)196) ;
        LoopBackDemoChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_index_GET() == (char)38037);
            assert(pack.param_id_LEN(ph) == 9);
            assert(pack.param_id_TRY(ph).equals("iMaIdmjqL"));
            assert(pack.param_count_GET() == (char)28144);
            assert(pack.param_value_GET() == 2.0698897E38F);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8);
        });
        DemoDevice.PARAM_VALUE p22 = LoopBackDemoChannel.new_PARAM_VALUE();
        PH.setPack(p22);
        p22.param_index_SET((char)38037) ;
        p22.param_count_SET((char)28144) ;
        p22.param_id_SET("iMaIdmjqL", PH) ;
        p22.param_value_SET(2.0698897E38F) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8) ;
        LoopBackDemoChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)35);
            assert(pack.param_value_GET() == 8.489753E37F);
            assert(pack.target_system_GET() == (char)118);
            assert(pack.param_id_LEN(ph) == 12);
            assert(pack.param_id_TRY(ph).equals("ajqBelyvxaxk"));
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32);
        });
        DemoDevice.PARAM_SET p23 = LoopBackDemoChannel.new_PARAM_SET();
        PH.setPack(p23);
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32) ;
        p23.param_id_SET("ajqBelyvxaxk", PH) ;
        p23.target_component_SET((char)35) ;
        p23.target_system_SET((char)118) ;
        p23.param_value_SET(8.489753E37F) ;
        LoopBackDemoChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.vel_acc_TRY(ph) == 2298870412L);
            assert(pack.epv_GET() == (char)34137);
            assert(pack.lon_GET() == -1051612183);
            assert(pack.time_usec_GET() == 7215563595364383442L);
            assert(pack.h_acc_TRY(ph) == 3636391022L);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
            assert(pack.alt_GET() == 1633439737);
            assert(pack.hdg_acc_TRY(ph) == 469836106L);
            assert(pack.lat_GET() == 1209288898);
            assert(pack.vel_GET() == (char)59020);
            assert(pack.satellites_visible_GET() == (char)204);
            assert(pack.cog_GET() == (char)23471);
            assert(pack.alt_ellipsoid_TRY(ph) == 790790214);
            assert(pack.v_acc_TRY(ph) == 2343450567L);
            assert(pack.eph_GET() == (char)11049);
        });
        DemoDevice.GPS_RAW_INT p24 = LoopBackDemoChannel.new_GPS_RAW_INT();
        PH.setPack(p24);
        p24.hdg_acc_SET(469836106L, PH) ;
        p24.cog_SET((char)23471) ;
        p24.alt_SET(1633439737) ;
        p24.v_acc_SET(2343450567L, PH) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX) ;
        p24.eph_SET((char)11049) ;
        p24.epv_SET((char)34137) ;
        p24.time_usec_SET(7215563595364383442L) ;
        p24.alt_ellipsoid_SET(790790214, PH) ;
        p24.lat_SET(1209288898) ;
        p24.vel_acc_SET(2298870412L, PH) ;
        p24.lon_SET(-1051612183) ;
        p24.h_acc_SET(3636391022L, PH) ;
        p24.vel_SET((char)59020) ;
        p24.satellites_visible_SET((char)204) ;
        LoopBackDemoChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)65, (char)171, (char)74, (char)131, (char)72, (char)227, (char)98, (char)98, (char)224, (char)161, (char)92, (char)75, (char)14, (char)178, (char)114, (char)57, (char)98, (char)167, (char)20, (char)146}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)113, (char)83, (char)159, (char)252, (char)71, (char)238, (char)145, (char)240, (char)193, (char)191, (char)237, (char)206, (char)62, (char)224, (char)6, (char)165, (char)203, (char)173, (char)95, (char)23}));
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)111, (char)240, (char)5, (char)230, (char)141, (char)235, (char)161, (char)219, (char)174, (char)24, (char)40, (char)197, (char)158, (char)151, (char)19, (char)88, (char)163, (char)31, (char)134, (char)20}));
            assert(pack.satellites_visible_GET() == (char)43);
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)99, (char)137, (char)7, (char)209, (char)245, (char)93, (char)243, (char)118, (char)119, (char)44, (char)91, (char)235, (char)107, (char)212, (char)36, (char)105, (char)84, (char)165, (char)113, (char)198}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)119, (char)129, (char)59, (char)250, (char)141, (char)210, (char)60, (char)239, (char)129, (char)193, (char)40, (char)225, (char)55, (char)217, (char)194, (char)68, (char)177, (char)198, (char)77, (char)123}));
        });
        DemoDevice.GPS_STATUS p25 = LoopBackDemoChannel.new_GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_snr_SET(new char[] {(char)65, (char)171, (char)74, (char)131, (char)72, (char)227, (char)98, (char)98, (char)224, (char)161, (char)92, (char)75, (char)14, (char)178, (char)114, (char)57, (char)98, (char)167, (char)20, (char)146}, 0) ;
        p25.satellite_azimuth_SET(new char[] {(char)119, (char)129, (char)59, (char)250, (char)141, (char)210, (char)60, (char)239, (char)129, (char)193, (char)40, (char)225, (char)55, (char)217, (char)194, (char)68, (char)177, (char)198, (char)77, (char)123}, 0) ;
        p25.satellite_prn_SET(new char[] {(char)111, (char)240, (char)5, (char)230, (char)141, (char)235, (char)161, (char)219, (char)174, (char)24, (char)40, (char)197, (char)158, (char)151, (char)19, (char)88, (char)163, (char)31, (char)134, (char)20}, 0) ;
        p25.satellite_used_SET(new char[] {(char)99, (char)137, (char)7, (char)209, (char)245, (char)93, (char)243, (char)118, (char)119, (char)44, (char)91, (char)235, (char)107, (char)212, (char)36, (char)105, (char)84, (char)165, (char)113, (char)198}, 0) ;
        p25.satellites_visible_SET((char)43) ;
        p25.satellite_elevation_SET(new char[] {(char)113, (char)83, (char)159, (char)252, (char)71, (char)238, (char)145, (char)240, (char)193, (char)191, (char)237, (char)206, (char)62, (char)224, (char)6, (char)165, (char)203, (char)173, (char)95, (char)23}, 0) ;
        LoopBackDemoChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.xmag_GET() == (short)12194);
            assert(pack.zgyro_GET() == (short) -11909);
            assert(pack.yacc_GET() == (short)29624);
            assert(pack.xacc_GET() == (short) -12121);
            assert(pack.ymag_GET() == (short) -1680);
            assert(pack.zacc_GET() == (short)29804);
            assert(pack.time_boot_ms_GET() == 859317987L);
            assert(pack.ygyro_GET() == (short)4380);
            assert(pack.xgyro_GET() == (short)26935);
            assert(pack.zmag_GET() == (short)21894);
        });
        DemoDevice.SCALED_IMU p26 = LoopBackDemoChannel.new_SCALED_IMU();
        PH.setPack(p26);
        p26.yacc_SET((short)29624) ;
        p26.xacc_SET((short) -12121) ;
        p26.ygyro_SET((short)4380) ;
        p26.zgyro_SET((short) -11909) ;
        p26.time_boot_ms_SET(859317987L) ;
        p26.xgyro_SET((short)26935) ;
        p26.zmag_SET((short)21894) ;
        p26.xmag_SET((short)12194) ;
        p26.zacc_SET((short)29804) ;
        p26.ymag_SET((short) -1680) ;
        LoopBackDemoChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == (short)2275);
            assert(pack.ygyro_GET() == (short) -11167);
            assert(pack.zacc_GET() == (short) -24962);
            assert(pack.zmag_GET() == (short) -11777);
            assert(pack.ymag_GET() == (short) -28829);
            assert(pack.xgyro_GET() == (short)17819);
            assert(pack.xmag_GET() == (short)29481);
            assert(pack.time_usec_GET() == 291806616919124727L);
            assert(pack.zgyro_GET() == (short)17724);
            assert(pack.xacc_GET() == (short) -6086);
        });
        DemoDevice.RAW_IMU p27 = LoopBackDemoChannel.new_RAW_IMU();
        PH.setPack(p27);
        p27.time_usec_SET(291806616919124727L) ;
        p27.xacc_SET((short) -6086) ;
        p27.yacc_SET((short)2275) ;
        p27.zacc_SET((short) -24962) ;
        p27.ymag_SET((short) -28829) ;
        p27.xgyro_SET((short)17819) ;
        p27.zgyro_SET((short)17724) ;
        p27.ygyro_SET((short) -11167) ;
        p27.xmag_SET((short)29481) ;
        p27.zmag_SET((short) -11777) ;
        LoopBackDemoChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short) -32176);
            assert(pack.press_diff2_GET() == (short) -29538);
            assert(pack.time_usec_GET() == 1452208608585014598L);
            assert(pack.press_abs_GET() == (short)10792);
            assert(pack.press_diff1_GET() == (short)4450);
        });
        DemoDevice.RAW_PRESSURE p28 = LoopBackDemoChannel.new_RAW_PRESSURE();
        PH.setPack(p28);
        p28.temperature_SET((short) -32176) ;
        p28.press_diff1_SET((short)4450) ;
        p28.press_abs_SET((short)10792) ;
        p28.press_diff2_SET((short) -29538) ;
        p28.time_usec_SET(1452208608585014598L) ;
        LoopBackDemoChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == 3.3205878E38F);
            assert(pack.press_diff_GET() == -2.4440255E38F);
            assert(pack.temperature_GET() == (short)11336);
            assert(pack.time_boot_ms_GET() == 3256826858L);
        });
        DemoDevice.SCALED_PRESSURE p29 = LoopBackDemoChannel.new_SCALED_PRESSURE();
        PH.setPack(p29);
        p29.press_diff_SET(-2.4440255E38F) ;
        p29.temperature_SET((short)11336) ;
        p29.time_boot_ms_SET(3256826858L) ;
        p29.press_abs_SET(3.3205878E38F) ;
        LoopBackDemoChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == 1.5653443E38F);
            assert(pack.pitchspeed_GET() == -2.3687863E38F);
            assert(pack.yawspeed_GET() == -2.0177193E38F);
            assert(pack.rollspeed_GET() == -1.5488724E38F);
            assert(pack.pitch_GET() == -2.769963E38F);
            assert(pack.yaw_GET() == 1.0324422E38F);
            assert(pack.time_boot_ms_GET() == 3897108507L);
        });
        DemoDevice.ATTITUDE p30 = LoopBackDemoChannel.new_ATTITUDE();
        PH.setPack(p30);
        p30.pitch_SET(-2.769963E38F) ;
        p30.time_boot_ms_SET(3897108507L) ;
        p30.yaw_SET(1.0324422E38F) ;
        p30.rollspeed_SET(-1.5488724E38F) ;
        p30.roll_SET(1.5653443E38F) ;
        p30.pitchspeed_SET(-2.3687863E38F) ;
        p30.yawspeed_SET(-2.0177193E38F) ;
        LoopBackDemoChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.q2_GET() == -1.1339718E38F);
            assert(pack.time_boot_ms_GET() == 509114874L);
            assert(pack.yawspeed_GET() == -3.2504888E38F);
            assert(pack.q1_GET() == 5.8126844E36F);
            assert(pack.q3_GET() == -2.8821677E37F);
            assert(pack.rollspeed_GET() == 4.7931106E37F);
            assert(pack.q4_GET() == 2.197594E38F);
            assert(pack.pitchspeed_GET() == 1.4334834E37F);
        });
        DemoDevice.ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.q2_SET(-1.1339718E38F) ;
        p31.yawspeed_SET(-3.2504888E38F) ;
        p31.rollspeed_SET(4.7931106E37F) ;
        p31.pitchspeed_SET(1.4334834E37F) ;
        p31.q4_SET(2.197594E38F) ;
        p31.time_boot_ms_SET(509114874L) ;
        p31.q1_SET(5.8126844E36F) ;
        p31.q3_SET(-2.8821677E37F) ;
        LoopBackDemoChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 5.868469E37F);
            assert(pack.time_boot_ms_GET() == 3035885722L);
            assert(pack.vz_GET() == 1.0931112E38F);
            assert(pack.vx_GET() == 2.8040285E38F);
            assert(pack.y_GET() == -1.53868E38F);
            assert(pack.x_GET() == -1.8168877E38F);
            assert(pack.vy_GET() == -2.1025427E38F);
        });
        DemoDevice.LOCAL_POSITION_NED p32 = LoopBackDemoChannel.new_LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.z_SET(5.868469E37F) ;
        p32.vx_SET(2.8040285E38F) ;
        p32.time_boot_ms_SET(3035885722L) ;
        p32.vy_SET(-2.1025427E38F) ;
        p32.x_SET(-1.8168877E38F) ;
        p32.vz_SET(1.0931112E38F) ;
        p32.y_SET(-1.53868E38F) ;
        LoopBackDemoChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == (short) -16001);
            assert(pack.alt_GET() == -327558860);
            assert(pack.hdg_GET() == (char)2852);
            assert(pack.time_boot_ms_GET() == 2796498364L);
            assert(pack.vz_GET() == (short) -31127);
            assert(pack.lon_GET() == -133406681);
            assert(pack.relative_alt_GET() == 144423273);
            assert(pack.lat_GET() == 466957784);
            assert(pack.vy_GET() == (short) -14957);
        });
        DemoDevice.GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.time_boot_ms_SET(2796498364L) ;
        p33.hdg_SET((char)2852) ;
        p33.lon_SET(-133406681) ;
        p33.vz_SET((short) -31127) ;
        p33.lat_SET(466957784) ;
        p33.vy_SET((short) -14957) ;
        p33.vx_SET((short) -16001) ;
        p33.relative_alt_SET(144423273) ;
        p33.alt_SET(-327558860) ;
        LoopBackDemoChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan4_scaled_GET() == (short) -4110);
            assert(pack.chan5_scaled_GET() == (short) -90);
            assert(pack.rssi_GET() == (char)107);
            assert(pack.chan7_scaled_GET() == (short) -30910);
            assert(pack.chan6_scaled_GET() == (short)15052);
            assert(pack.chan3_scaled_GET() == (short)13990);
            assert(pack.chan2_scaled_GET() == (short) -5682);
            assert(pack.time_boot_ms_GET() == 4258268004L);
            assert(pack.port_GET() == (char)144);
            assert(pack.chan8_scaled_GET() == (short) -22733);
            assert(pack.chan1_scaled_GET() == (short) -19853);
        });
        DemoDevice.RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.new_RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.chan7_scaled_SET((short) -30910) ;
        p34.chan8_scaled_SET((short) -22733) ;
        p34.chan3_scaled_SET((short)13990) ;
        p34.chan4_scaled_SET((short) -4110) ;
        p34.chan2_scaled_SET((short) -5682) ;
        p34.chan5_scaled_SET((short) -90) ;
        p34.time_boot_ms_SET(4258268004L) ;
        p34.port_SET((char)144) ;
        p34.rssi_SET((char)107) ;
        p34.chan1_scaled_SET((short) -19853) ;
        p34.chan6_scaled_SET((short)15052) ;
        LoopBackDemoChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan8_raw_GET() == (char)42581);
            assert(pack.rssi_GET() == (char)250);
            assert(pack.chan2_raw_GET() == (char)51970);
            assert(pack.port_GET() == (char)101);
            assert(pack.chan4_raw_GET() == (char)55053);
            assert(pack.chan5_raw_GET() == (char)34967);
            assert(pack.time_boot_ms_GET() == 1378572277L);
            assert(pack.chan1_raw_GET() == (char)31417);
            assert(pack.chan7_raw_GET() == (char)42726);
            assert(pack.chan3_raw_GET() == (char)9252);
            assert(pack.chan6_raw_GET() == (char)48214);
        });
        DemoDevice.RC_CHANNELS_RAW p35 = LoopBackDemoChannel.new_RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan7_raw_SET((char)42726) ;
        p35.chan1_raw_SET((char)31417) ;
        p35.chan6_raw_SET((char)48214) ;
        p35.port_SET((char)101) ;
        p35.chan8_raw_SET((char)42581) ;
        p35.chan3_raw_SET((char)9252) ;
        p35.rssi_SET((char)250) ;
        p35.time_boot_ms_SET(1378572277L) ;
        p35.chan4_raw_SET((char)55053) ;
        p35.chan5_raw_SET((char)34967) ;
        p35.chan2_raw_SET((char)51970) ;
        LoopBackDemoChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo1_raw_GET() == (char)22225);
            assert(pack.servo8_raw_GET() == (char)38681);
            assert(pack.servo14_raw_TRY(ph) == (char)12875);
            assert(pack.servo12_raw_TRY(ph) == (char)43250);
            assert(pack.port_GET() == (char)87);
            assert(pack.servo11_raw_TRY(ph) == (char)25561);
            assert(pack.servo10_raw_TRY(ph) == (char)35784);
            assert(pack.servo9_raw_TRY(ph) == (char)3041);
            assert(pack.servo15_raw_TRY(ph) == (char)38498);
            assert(pack.servo6_raw_GET() == (char)18995);
            assert(pack.servo7_raw_GET() == (char)27301);
            assert(pack.servo16_raw_TRY(ph) == (char)46858);
            assert(pack.servo3_raw_GET() == (char)31800);
            assert(pack.servo13_raw_TRY(ph) == (char)57923);
            assert(pack.servo4_raw_GET() == (char)40519);
            assert(pack.time_usec_GET() == 1082912977L);
            assert(pack.servo2_raw_GET() == (char)29361);
            assert(pack.servo5_raw_GET() == (char)14544);
        });
        DemoDevice.SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.new_SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo13_raw_SET((char)57923, PH) ;
        p36.servo14_raw_SET((char)12875, PH) ;
        p36.servo2_raw_SET((char)29361) ;
        p36.servo9_raw_SET((char)3041, PH) ;
        p36.servo7_raw_SET((char)27301) ;
        p36.servo15_raw_SET((char)38498, PH) ;
        p36.servo10_raw_SET((char)35784, PH) ;
        p36.servo8_raw_SET((char)38681) ;
        p36.servo1_raw_SET((char)22225) ;
        p36.time_usec_SET(1082912977L) ;
        p36.servo6_raw_SET((char)18995) ;
        p36.port_SET((char)87) ;
        p36.servo16_raw_SET((char)46858, PH) ;
        p36.servo3_raw_SET((char)31800) ;
        p36.servo5_raw_SET((char)14544) ;
        p36.servo11_raw_SET((char)25561, PH) ;
        p36.servo12_raw_SET((char)43250, PH) ;
        p36.servo4_raw_SET((char)40519) ;
        LoopBackDemoChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)53);
            assert(pack.end_index_GET() == (short)21077);
            assert(pack.target_component_GET() == (char)232);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.start_index_GET() == (short)32546);
        });
        DemoDevice.MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.new_MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.end_index_SET((short)21077) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p37.target_system_SET((char)53) ;
        p37.start_index_SET((short)32546) ;
        p37.target_component_SET((char)232) ;
        LoopBackDemoChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)32);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.end_index_GET() == (short)29970);
            assert(pack.target_system_GET() == (char)199);
            assert(pack.start_index_GET() == (short) -20155);
        });
        DemoDevice.MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.new_MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.target_system_SET((char)199) ;
        p38.target_component_SET((char)32) ;
        p38.start_index_SET((short) -20155) ;
        p38.end_index_SET((short)29970) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        LoopBackDemoChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -1.823998E38F);
            assert(pack.seq_GET() == (char)12322);
            assert(pack.autocontinue_GET() == (char)219);
            assert(pack.param3_GET() == -2.6759869E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            assert(pack.y_GET() == -1.040292E38F);
            assert(pack.x_GET() == -7.0490125E37F);
            assert(pack.param2_GET() == -8.485992E37F);
            assert(pack.target_system_GET() == (char)80);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.current_GET() == (char)152);
            assert(pack.target_component_GET() == (char)179);
            assert(pack.param1_GET() == -1.9121651E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_WAYPOINT_USER_4);
            assert(pack.param4_GET() == 2.380497E37F);
        });
        DemoDevice.MISSION_ITEM p39 = LoopBackDemoChannel.new_MISSION_ITEM();
        PH.setPack(p39);
        p39.y_SET(-1.040292E38F) ;
        p39.x_SET(-7.0490125E37F) ;
        p39.current_SET((char)152) ;
        p39.z_SET(-1.823998E38F) ;
        p39.command_SET(MAV_CMD.MAV_CMD_WAYPOINT_USER_4) ;
        p39.param1_SET(-1.9121651E38F) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p39.autocontinue_SET((char)219) ;
        p39.seq_SET((char)12322) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p39.param2_SET(-8.485992E37F) ;
        p39.target_system_SET((char)80) ;
        p39.param3_SET(-2.6759869E38F) ;
        p39.param4_SET(2.380497E37F) ;
        p39.target_component_SET((char)179) ;
        LoopBackDemoChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)1278);
            assert(pack.target_system_GET() == (char)237);
            assert(pack.target_component_GET() == (char)183);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        });
        DemoDevice.MISSION_REQUEST p40 = LoopBackDemoChannel.new_MISSION_REQUEST();
        PH.setPack(p40);
        p40.seq_SET((char)1278) ;
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p40.target_system_SET((char)237) ;
        p40.target_component_SET((char)183) ;
        LoopBackDemoChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)145);
            assert(pack.seq_GET() == (char)52535);
            assert(pack.target_component_GET() == (char)157);
        });
        DemoDevice.MISSION_SET_CURRENT p41 = LoopBackDemoChannel.new_MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_system_SET((char)145) ;
        p41.seq_SET((char)52535) ;
        p41.target_component_SET((char)157) ;
        LoopBackDemoChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)49824);
        });
        DemoDevice.MISSION_CURRENT p42 = LoopBackDemoChannel.new_MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)49824) ;
        LoopBackDemoChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)48);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_component_GET() == (char)150);
        });
        DemoDevice.MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.new_MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_system_SET((char)48) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p43.target_component_SET((char)150) ;
        LoopBackDemoChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)49584);
            assert(pack.target_component_GET() == (char)248);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_system_GET() == (char)70);
        });
        DemoDevice.MISSION_COUNT p44 = LoopBackDemoChannel.new_MISSION_COUNT();
        PH.setPack(p44);
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p44.count_SET((char)49584) ;
        p44.target_component_SET((char)248) ;
        p44.target_system_SET((char)70) ;
        LoopBackDemoChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)242);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_system_GET() == (char)83);
        });
        DemoDevice.MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.new_MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_system_SET((char)83) ;
        p45.target_component_SET((char)242) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        LoopBackDemoChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)51467);
        });
        DemoDevice.MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.new_MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)51467) ;
        LoopBackDemoChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM3);
            assert(pack.target_component_GET() == (char)170);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_system_GET() == (char)204);
        });
        DemoDevice.MISSION_ACK p47 = LoopBackDemoChannel.new_MISSION_ACK();
        PH.setPack(p47);
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM3) ;
        p47.target_system_SET((char)204) ;
        p47.target_component_SET((char)170) ;
        LoopBackDemoChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)28);
            assert(pack.time_usec_TRY(ph) == 6907491309879298354L);
            assert(pack.longitude_GET() == -1663332851);
            assert(pack.altitude_GET() == 1745296576);
            assert(pack.latitude_GET() == -383636278);
        });
        DemoDevice.SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.new_SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.time_usec_SET(6907491309879298354L, PH) ;
        p48.latitude_SET(-383636278) ;
        p48.longitude_SET(-1663332851) ;
        p48.target_system_SET((char)28) ;
        p48.altitude_SET(1745296576) ;
        LoopBackDemoChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.time_usec_TRY(ph) == 5377980200588695000L);
            assert(pack.altitude_GET() == 996848269);
            assert(pack.latitude_GET() == 1063833864);
            assert(pack.longitude_GET() == 1944911443);
        });
        DemoDevice.GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.new_GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.longitude_SET(1944911443) ;
        p49.time_usec_SET(5377980200588695000L, PH) ;
        p49.latitude_SET(1063833864) ;
        p49.altitude_SET(996848269) ;
        LoopBackDemoChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.scale_GET() == 1.1565717E38F);
            assert(pack.param_value_max_GET() == 1.2780526E38F);
            assert(pack.param_index_GET() == (short) -12767);
            assert(pack.param_id_LEN(ph) == 11);
            assert(pack.param_id_TRY(ph).equals("gsnvwtrLmyv"));
            assert(pack.param_value0_GET() == 1.1541628E38F);
            assert(pack.target_system_GET() == (char)236);
            assert(pack.parameter_rc_channel_index_GET() == (char)66);
            assert(pack.target_component_GET() == (char)176);
            assert(pack.param_value_min_GET() == -2.6646387E38F);
        });
        DemoDevice.PARAM_MAP_RC p50 = LoopBackDemoChannel.new_PARAM_MAP_RC();
        PH.setPack(p50);
        p50.param_value0_SET(1.1541628E38F) ;
        p50.target_component_SET((char)176) ;
        p50.param_value_max_SET(1.2780526E38F) ;
        p50.param_index_SET((short) -12767) ;
        p50.param_value_min_SET(-2.6646387E38F) ;
        p50.param_id_SET("gsnvwtrLmyv", PH) ;
        p50.target_system_SET((char)236) ;
        p50.parameter_rc_channel_index_SET((char)66) ;
        p50.scale_SET(1.1565717E38F) ;
        LoopBackDemoChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_system_GET() == (char)39);
            assert(pack.seq_GET() == (char)62026);
            assert(pack.target_component_GET() == (char)171);
        });
        DemoDevice.MISSION_REQUEST_INT p51 = LoopBackDemoChannel.new_MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.seq_SET((char)62026) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p51.target_component_SET((char)171) ;
        p51.target_system_SET((char)39) ;
        LoopBackDemoChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p1x_GET() == 1.0206071E38F);
            assert(pack.p2z_GET() == 4.719654E37F);
            assert(pack.target_system_GET() == (char)148);
            assert(pack.p1z_GET() == -1.9683823E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.p2y_GET() == 7.504703E37F);
            assert(pack.p1y_GET() == -2.0233476E38F);
            assert(pack.p2x_GET() == 2.8360642E38F);
            assert(pack.target_component_GET() == (char)239);
        });
        DemoDevice.SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.new_SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.p1y_SET(-2.0233476E38F) ;
        p54.target_component_SET((char)239) ;
        p54.p2y_SET(7.504703E37F) ;
        p54.p2x_SET(2.8360642E38F) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p54.p1z_SET(-1.9683823E38F) ;
        p54.p1x_SET(1.0206071E38F) ;
        p54.p2z_SET(4.719654E37F) ;
        p54.target_system_SET((char)148) ;
        LoopBackDemoChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2z_GET() == 5.398413E37F);
            assert(pack.p2x_GET() == -1.7648583E37F);
            assert(pack.p2y_GET() == 2.0425899E37F);
            assert(pack.p1z_GET() == 1.755584E38F);
            assert(pack.p1y_GET() == 2.585118E38F);
            assert(pack.p1x_GET() == 2.5822884E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
        });
        DemoDevice.SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.new_SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p1y_SET(2.585118E38F) ;
        p55.p1z_SET(1.755584E38F) ;
        p55.p2z_SET(5.398413E37F) ;
        p55.p1x_SET(2.5822884E38F) ;
        p55.p2x_SET(-1.7648583E37F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p55.p2y_SET(2.0425899E37F) ;
        LoopBackDemoChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.rollspeed_GET() == -1.4489665E38F);
            assert(pack.pitchspeed_GET() == -1.3776781E38F);
            assert(pack.yawspeed_GET() == -2.8144523E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.828228E38F, -3.3805467E38F, -1.2910894E38F, 1.2775686E38F}));
            assert(pack.time_usec_GET() == 2465999516219331802L);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-2.6294941E38F, 1.1538774E38F, -2.4420163E38F, 3.3083229E38F, -9.309058E37F, -1.5065757E38F, -1.9984986E38F, -1.8880416E38F, -1.9406276E38F}));
        });
        DemoDevice.ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.covariance_SET(new float[] {-2.6294941E38F, 1.1538774E38F, -2.4420163E38F, 3.3083229E38F, -9.309058E37F, -1.5065757E38F, -1.9984986E38F, -1.8880416E38F, -1.9406276E38F}, 0) ;
        p61.q_SET(new float[] {-1.828228E38F, -3.3805467E38F, -1.2910894E38F, 1.2775686E38F}, 0) ;
        p61.rollspeed_SET(-1.4489665E38F) ;
        p61.yawspeed_SET(-2.8144523E37F) ;
        p61.pitchspeed_SET(-1.3776781E38F) ;
        p61.time_usec_SET(2465999516219331802L) ;
        LoopBackDemoChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.alt_error_GET() == 3.2160718E38F);
            assert(pack.nav_roll_GET() == 7.085819E37F);
            assert(pack.nav_pitch_GET() == 1.012133E38F);
            assert(pack.wp_dist_GET() == (char)50420);
            assert(pack.nav_bearing_GET() == (short) -29381);
            assert(pack.aspd_error_GET() == -2.9775827E38F);
            assert(pack.xtrack_error_GET() == 1.7136845E38F);
            assert(pack.target_bearing_GET() == (short)19269);
        });
        DemoDevice.NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.new_NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.nav_pitch_SET(1.012133E38F) ;
        p62.nav_roll_SET(7.085819E37F) ;
        p62.nav_bearing_SET((short) -29381) ;
        p62.target_bearing_SET((short)19269) ;
        p62.alt_error_SET(3.2160718E38F) ;
        p62.aspd_error_SET(-2.9775827E38F) ;
        p62.wp_dist_SET((char)50420) ;
        p62.xtrack_error_SET(1.7136845E38F) ;
        LoopBackDemoChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == -1.3928655E37F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-8.733125E37F, -1.3278997E38F, 2.6156372E38F, 3.7626015E37F, 1.4519084E38F, 3.1489174E37F, 2.794286E38F, 2.5314563E38F, -1.6795791E38F, 3.2524101E38F, 9.953588E37F, 2.0270573E38F, -2.2092728E38F, -2.926891E37F, -8.639188E37F, -1.0471122E38F, -2.9545696E38F, 2.0761454E38F, -1.592544E37F, 3.0617583E38F, 1.7126633E38F, 8.889401E37F, -1.4362782E38F, -3.0585024E38F, 1.7008419E38F, -1.5266453E38F, -3.3585084E38F, -5.951061E37F, 2.4555376E38F, 7.8361115E37F, 1.5476045E38F, -8.189579E37F, 1.4729279E38F, -3.14524E38F, -3.0676974E38F, -2.6287798E38F}));
            assert(pack.time_usec_GET() == 171371841836375644L);
            assert(pack.alt_GET() == -1279920947);
            assert(pack.lon_GET() == 1398019290);
            assert(pack.vy_GET() == -6.8865585E37F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
            assert(pack.lat_GET() == -1309949669);
            assert(pack.relative_alt_GET() == 980032065);
            assert(pack.vx_GET() == 3.1408962E38F);
        });
        DemoDevice.GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.alt_SET(-1279920947) ;
        p63.covariance_SET(new float[] {-8.733125E37F, -1.3278997E38F, 2.6156372E38F, 3.7626015E37F, 1.4519084E38F, 3.1489174E37F, 2.794286E38F, 2.5314563E38F, -1.6795791E38F, 3.2524101E38F, 9.953588E37F, 2.0270573E38F, -2.2092728E38F, -2.926891E37F, -8.639188E37F, -1.0471122E38F, -2.9545696E38F, 2.0761454E38F, -1.592544E37F, 3.0617583E38F, 1.7126633E38F, 8.889401E37F, -1.4362782E38F, -3.0585024E38F, 1.7008419E38F, -1.5266453E38F, -3.3585084E38F, -5.951061E37F, 2.4555376E38F, 7.8361115E37F, 1.5476045E38F, -8.189579E37F, 1.4729279E38F, -3.14524E38F, -3.0676974E38F, -2.6287798E38F}, 0) ;
        p63.vz_SET(-1.3928655E37F) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS) ;
        p63.lat_SET(-1309949669) ;
        p63.relative_alt_SET(980032065) ;
        p63.time_usec_SET(171371841836375644L) ;
        p63.vy_SET(-6.8865585E37F) ;
        p63.lon_SET(1398019290) ;
        p63.vx_SET(3.1408962E38F) ;
        LoopBackDemoChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.az_GET() == 9.529468E37F);
            assert(pack.z_GET() == 5.3331423E36F);
            assert(pack.x_GET() == 1.7383449E38F);
            assert(pack.vx_GET() == 1.6004021E38F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
            assert(pack.time_usec_GET() == 8071116530695342849L);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-1.4722344E38F, -1.9417174E38F, 1.892579E38F, -2.2175992E38F, 3.3057776E38F, 2.7897166E37F, -3.0580779E38F, -1.3316893E38F, -1.0368594E38F, -5.3079674E37F, -9.797779E37F, -3.0027886E38F, -2.692818E38F, -6.2248013E37F, 8.651175E37F, 3.372966E38F, 1.8060776E38F, 3.2236393E38F, 1.5255939E38F, -6.172877E36F, -1.754744E38F, 8.615239E37F, 1.1806093E38F, -1.845425E38F, -1.7701864E38F, 1.777479E38F, -3.076603E38F, 1.1734015E38F, 1.533519E38F, -1.9594784E37F, -1.4129102E38F, 2.7947406E38F, 4.157338E37F, -1.6914146E38F, 1.9536244E38F, 4.942237E37F, -1.02411736E37F, -1.8295522E38F, -1.4686329E38F, 8.1974756E37F, 1.8173946E38F, 3.3821275E37F, -2.8207627E38F, 2.9380013E38F, -3.3614595E38F}));
            assert(pack.ax_GET() == 3.033755E38F);
            assert(pack.vy_GET() == 2.3582976E38F);
            assert(pack.y_GET() == -6.389288E36F);
            assert(pack.ay_GET() == 9.969476E37F);
            assert(pack.vz_GET() == -2.2093456E38F);
        });
        DemoDevice.LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.time_usec_SET(8071116530695342849L) ;
        p64.ay_SET(9.969476E37F) ;
        p64.z_SET(5.3331423E36F) ;
        p64.x_SET(1.7383449E38F) ;
        p64.vx_SET(1.6004021E38F) ;
        p64.ax_SET(3.033755E38F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION) ;
        p64.vy_SET(2.3582976E38F) ;
        p64.vz_SET(-2.2093456E38F) ;
        p64.covariance_SET(new float[] {-1.4722344E38F, -1.9417174E38F, 1.892579E38F, -2.2175992E38F, 3.3057776E38F, 2.7897166E37F, -3.0580779E38F, -1.3316893E38F, -1.0368594E38F, -5.3079674E37F, -9.797779E37F, -3.0027886E38F, -2.692818E38F, -6.2248013E37F, 8.651175E37F, 3.372966E38F, 1.8060776E38F, 3.2236393E38F, 1.5255939E38F, -6.172877E36F, -1.754744E38F, 8.615239E37F, 1.1806093E38F, -1.845425E38F, -1.7701864E38F, 1.777479E38F, -3.076603E38F, 1.1734015E38F, 1.533519E38F, -1.9594784E37F, -1.4129102E38F, 2.7947406E38F, 4.157338E37F, -1.6914146E38F, 1.9536244E38F, 4.942237E37F, -1.02411736E37F, -1.8295522E38F, -1.4686329E38F, 8.1974756E37F, 1.8173946E38F, 3.3821275E37F, -2.8207627E38F, 2.9380013E38F, -3.3614595E38F}, 0) ;
        p64.y_SET(-6.389288E36F) ;
        p64.az_SET(9.529468E37F) ;
        LoopBackDemoChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chancount_GET() == (char)141);
            assert(pack.chan9_raw_GET() == (char)64071);
            assert(pack.chan17_raw_GET() == (char)61893);
            assert(pack.chan12_raw_GET() == (char)15381);
            assert(pack.chan7_raw_GET() == (char)10977);
            assert(pack.chan4_raw_GET() == (char)36303);
            assert(pack.chan10_raw_GET() == (char)11650);
            assert(pack.chan14_raw_GET() == (char)38416);
            assert(pack.chan1_raw_GET() == (char)57973);
            assert(pack.chan2_raw_GET() == (char)29958);
            assert(pack.chan3_raw_GET() == (char)12857);
            assert(pack.time_boot_ms_GET() == 822478734L);
            assert(pack.chan16_raw_GET() == (char)58913);
            assert(pack.chan13_raw_GET() == (char)30042);
            assert(pack.chan5_raw_GET() == (char)56184);
            assert(pack.chan18_raw_GET() == (char)33306);
            assert(pack.chan6_raw_GET() == (char)32649);
            assert(pack.chan8_raw_GET() == (char)46433);
            assert(pack.chan11_raw_GET() == (char)41971);
            assert(pack.rssi_GET() == (char)96);
            assert(pack.chan15_raw_GET() == (char)29337);
        });
        DemoDevice.RC_CHANNELS p65 = LoopBackDemoChannel.new_RC_CHANNELS();
        PH.setPack(p65);
        p65.chan2_raw_SET((char)29958) ;
        p65.chan5_raw_SET((char)56184) ;
        p65.chan7_raw_SET((char)10977) ;
        p65.chan13_raw_SET((char)30042) ;
        p65.chan15_raw_SET((char)29337) ;
        p65.chan17_raw_SET((char)61893) ;
        p65.chan12_raw_SET((char)15381) ;
        p65.chancount_SET((char)141) ;
        p65.chan11_raw_SET((char)41971) ;
        p65.chan10_raw_SET((char)11650) ;
        p65.chan8_raw_SET((char)46433) ;
        p65.chan4_raw_SET((char)36303) ;
        p65.chan14_raw_SET((char)38416) ;
        p65.rssi_SET((char)96) ;
        p65.chan9_raw_SET((char)64071) ;
        p65.chan18_raw_SET((char)33306) ;
        p65.chan3_raw_SET((char)12857) ;
        p65.chan16_raw_SET((char)58913) ;
        p65.time_boot_ms_SET(822478734L) ;
        p65.chan6_raw_SET((char)32649) ;
        p65.chan1_raw_SET((char)57973) ;
        LoopBackDemoChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.req_stream_id_GET() == (char)211);
            assert(pack.target_component_GET() == (char)111);
            assert(pack.req_message_rate_GET() == (char)6383);
            assert(pack.target_system_GET() == (char)91);
            assert(pack.start_stop_GET() == (char)11);
        });
        DemoDevice.REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.new_REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.start_stop_SET((char)11) ;
        p66.req_message_rate_SET((char)6383) ;
        p66.req_stream_id_SET((char)211) ;
        p66.target_component_SET((char)111) ;
        p66.target_system_SET((char)91) ;
        LoopBackDemoChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.stream_id_GET() == (char)250);
            assert(pack.on_off_GET() == (char)168);
            assert(pack.message_rate_GET() == (char)36721);
        });
        DemoDevice.DATA_STREAM p67 = LoopBackDemoChannel.new_DATA_STREAM();
        PH.setPack(p67);
        p67.on_off_SET((char)168) ;
        p67.message_rate_SET((char)36721) ;
        p67.stream_id_SET((char)250) ;
        LoopBackDemoChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == (short) -3976);
            assert(pack.r_GET() == (short)25243);
            assert(pack.z_GET() == (short)7987);
            assert(pack.target_GET() == (char)231);
            assert(pack.x_GET() == (short) -19832);
            assert(pack.buttons_GET() == (char)64686);
        });
        DemoDevice.MANUAL_CONTROL p69 = LoopBackDemoChannel.new_MANUAL_CONTROL();
        PH.setPack(p69);
        p69.target_SET((char)231) ;
        p69.r_SET((short)25243) ;
        p69.z_SET((short)7987) ;
        p69.x_SET((short) -19832) ;
        p69.buttons_SET((char)64686) ;
        p69.y_SET((short) -3976) ;
        LoopBackDemoChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan5_raw_GET() == (char)28251);
            assert(pack.chan2_raw_GET() == (char)29166);
            assert(pack.target_system_GET() == (char)44);
            assert(pack.target_component_GET() == (char)243);
            assert(pack.chan1_raw_GET() == (char)27238);
            assert(pack.chan3_raw_GET() == (char)29243);
            assert(pack.chan7_raw_GET() == (char)32014);
            assert(pack.chan4_raw_GET() == (char)27830);
            assert(pack.chan8_raw_GET() == (char)47623);
            assert(pack.chan6_raw_GET() == (char)13608);
        });
        DemoDevice.RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.new_RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.target_component_SET((char)243) ;
        p70.chan3_raw_SET((char)29243) ;
        p70.chan7_raw_SET((char)32014) ;
        p70.chan5_raw_SET((char)28251) ;
        p70.chan2_raw_SET((char)29166) ;
        p70.chan4_raw_SET((char)27830) ;
        p70.chan1_raw_SET((char)27238) ;
        p70.target_system_SET((char)44) ;
        p70.chan8_raw_SET((char)47623) ;
        p70.chan6_raw_SET((char)13608) ;
        LoopBackDemoChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 999570147);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.x_GET() == -381979366);
            assert(pack.target_system_GET() == (char)254);
            assert(pack.param4_GET() == -7.1590177E37F);
            assert(pack.target_component_GET() == (char)115);
            assert(pack.param2_GET() == 2.099323E38F);
            assert(pack.param1_GET() == -3.1137044E38F);
            assert(pack.z_GET() == -1.2500689E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.param3_GET() == -9.166373E37F);
            assert(pack.current_GET() == (char)126);
            assert(pack.autocontinue_GET() == (char)110);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL);
            assert(pack.seq_GET() == (char)45933);
        });
        DemoDevice.MISSION_ITEM_INT p73 = LoopBackDemoChannel.new_MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.param4_SET(-7.1590177E37F) ;
        p73.y_SET(999570147) ;
        p73.z_SET(-1.2500689E38F) ;
        p73.current_SET((char)126) ;
        p73.param3_SET(-9.166373E37F) ;
        p73.target_system_SET((char)254) ;
        p73.autocontinue_SET((char)110) ;
        p73.target_component_SET((char)115) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p73.seq_SET((char)45933) ;
        p73.param2_SET(2.099323E38F) ;
        p73.x_SET(-381979366) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p73.command_SET(MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL) ;
        p73.param1_SET(-3.1137044E38F) ;
        LoopBackDemoChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.airspeed_GET() == -2.0678626E38F);
            assert(pack.groundspeed_GET() == 9.830876E37F);
            assert(pack.climb_GET() == -1.5864523E38F);
            assert(pack.alt_GET() == 9.669735E37F);
            assert(pack.throttle_GET() == (char)63401);
            assert(pack.heading_GET() == (short) -2229);
        });
        DemoDevice.VFR_HUD p74 = LoopBackDemoChannel.new_VFR_HUD();
        PH.setPack(p74);
        p74.climb_SET(-1.5864523E38F) ;
        p74.groundspeed_SET(9.830876E37F) ;
        p74.alt_SET(9.669735E37F) ;
        p74.throttle_SET((char)63401) ;
        p74.airspeed_SET(-2.0678626E38F) ;
        p74.heading_SET((short) -2229) ;
        LoopBackDemoChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.param4_GET() == -1.7276802E38F);
            assert(pack.autocontinue_GET() == (char)251);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.param3_GET() == -2.8640588E38F);
            assert(pack.y_GET() == 98875287);
            assert(pack.current_GET() == (char)121);
            assert(pack.x_GET() == 1823142556);
            assert(pack.param1_GET() == 3.1829338E38F);
            assert(pack.target_component_GET() == (char)26);
            assert(pack.param2_GET() == -4.4659483E37F);
            assert(pack.z_GET() == 2.0941604E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL);
            assert(pack.target_system_GET() == (char)111);
        });
        DemoDevice.COMMAND_INT p75 = LoopBackDemoChannel.new_COMMAND_INT();
        PH.setPack(p75);
        p75.param3_SET(-2.8640588E38F) ;
        p75.param1_SET(3.1829338E38F) ;
        p75.target_system_SET((char)111) ;
        p75.param2_SET(-4.4659483E37F) ;
        p75.z_SET(2.0941604E38F) ;
        p75.current_SET((char)121) ;
        p75.y_SET(98875287) ;
        p75.command_SET(MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL) ;
        p75.target_component_SET((char)26) ;
        p75.param4_SET(-1.7276802E38F) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p75.x_SET(1823142556) ;
        p75.autocontinue_SET((char)251) ;
        LoopBackDemoChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)232);
            assert(pack.param4_GET() == -9.513226E37F);
            assert(pack.confirmation_GET() == (char)133);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_SPATIAL_USER_1);
            assert(pack.param3_GET() == -4.8977516E37F);
            assert(pack.target_system_GET() == (char)91);
            assert(pack.param5_GET() == -2.9596853E38F);
            assert(pack.param2_GET() == 2.5884344E38F);
            assert(pack.param6_GET() == -1.5444823E38F);
            assert(pack.param1_GET() == 3.229888E38F);
            assert(pack.param7_GET() == 2.5717627E38F);
        });
        DemoDevice.COMMAND_LONG p76 = LoopBackDemoChannel.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.target_system_SET((char)91) ;
        p76.param2_SET(2.5884344E38F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_SPATIAL_USER_1) ;
        p76.confirmation_SET((char)133) ;
        p76.param4_SET(-9.513226E37F) ;
        p76.param1_SET(3.229888E38F) ;
        p76.param6_SET(-1.5444823E38F) ;
        p76.param7_SET(2.5717627E38F) ;
        p76.param5_SET(-2.9596853E38F) ;
        p76.target_component_SET((char)232) ;
        p76.param3_SET(-4.8977516E37F) ;
        LoopBackDemoChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.progress_TRY(ph) == (char)36);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_REPEAT_RELAY);
            assert(pack.target_component_TRY(ph) == (char)199);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_FAILED);
            assert(pack.result_param2_TRY(ph) == -234637807);
            assert(pack.target_system_TRY(ph) == (char)177);
        });
        DemoDevice.COMMAND_ACK p77 = LoopBackDemoChannel.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.result_param2_SET(-234637807, PH) ;
        p77.target_system_SET((char)177, PH) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_FAILED) ;
        p77.target_component_SET((char)199, PH) ;
        p77.progress_SET((char)36, PH) ;
        p77.command_SET(MAV_CMD.MAV_CMD_DO_REPEAT_RELAY) ;
        LoopBackDemoChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 9.619453E37F);
            assert(pack.roll_GET() == 2.2373763E38F);
            assert(pack.pitch_GET() == 1.9227288E38F);
            assert(pack.manual_override_switch_GET() == (char)8);
            assert(pack.time_boot_ms_GET() == 173711093L);
            assert(pack.mode_switch_GET() == (char)62);
            assert(pack.thrust_GET() == -2.4831622E37F);
        });
        DemoDevice.MANUAL_SETPOINT p81 = LoopBackDemoChannel.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.roll_SET(2.2373763E38F) ;
        p81.thrust_SET(-2.4831622E37F) ;
        p81.time_boot_ms_SET(173711093L) ;
        p81.mode_switch_SET((char)62) ;
        p81.manual_override_switch_SET((char)8) ;
        p81.yaw_SET(9.619453E37F) ;
        p81.pitch_SET(1.9227288E38F) ;
        LoopBackDemoChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.thrust_GET() == 2.4730892E38F);
            assert(pack.target_system_GET() == (char)14);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.7468148E37F, -1.0511937E38F, 1.3428299E38F, -2.7320183E37F}));
            assert(pack.body_yaw_rate_GET() == 5.524172E37F);
            assert(pack.body_pitch_rate_GET() == -3.2403843E38F);
            assert(pack.body_roll_rate_GET() == 1.9678525E38F);
            assert(pack.type_mask_GET() == (char)87);
            assert(pack.time_boot_ms_GET() == 1732435946L);
            assert(pack.target_component_GET() == (char)196);
        });
        DemoDevice.SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.q_SET(new float[] {-1.7468148E37F, -1.0511937E38F, 1.3428299E38F, -2.7320183E37F}, 0) ;
        p82.target_component_SET((char)196) ;
        p82.type_mask_SET((char)87) ;
        p82.time_boot_ms_SET(1732435946L) ;
        p82.body_pitch_rate_SET(-3.2403843E38F) ;
        p82.body_roll_rate_SET(1.9678525E38F) ;
        p82.thrust_SET(2.4730892E38F) ;
        p82.target_system_SET((char)14) ;
        p82.body_yaw_rate_SET(5.524172E37F) ;
        LoopBackDemoChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.type_mask_GET() == (char)48);
            assert(pack.thrust_GET() == -3.2245212E38F);
            assert(pack.body_yaw_rate_GET() == 2.0793443E38F);
            assert(pack.body_roll_rate_GET() == -2.5882237E38F);
            assert(pack.time_boot_ms_GET() == 592332783L);
            assert(pack.body_pitch_rate_GET() == 2.8908502E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.6666555E38F, -9.111301E37F, -2.310612E38F, -2.2185424E37F}));
        });
        DemoDevice.ATTITUDE_TARGET p83 = LoopBackDemoChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.type_mask_SET((char)48) ;
        p83.body_pitch_rate_SET(2.8908502E38F) ;
        p83.body_yaw_rate_SET(2.0793443E38F) ;
        p83.body_roll_rate_SET(-2.5882237E38F) ;
        p83.thrust_SET(-3.2245212E38F) ;
        p83.time_boot_ms_SET(592332783L) ;
        p83.q_SET(new float[] {-1.6666555E38F, -9.111301E37F, -2.310612E38F, -2.2185424E37F}, 0) ;
        LoopBackDemoChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == -2.5277858E38F);
            assert(pack.afz_GET() == 2.971962E38F);
            assert(pack.yaw_rate_GET() == -1.5062216E38F);
            assert(pack.y_GET() == 1.3968677E38F);
            assert(pack.vz_GET() == -1.3650169E38F);
            assert(pack.time_boot_ms_GET() == 1197674403L);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.type_mask_GET() == (char)32553);
            assert(pack.x_GET() == 3.3327424E37F);
            assert(pack.target_component_GET() == (char)13);
            assert(pack.z_GET() == -1.7633213E38F);
            assert(pack.vy_GET() == -3.2082312E38F);
            assert(pack.yaw_GET() == 9.308668E37F);
            assert(pack.afx_GET() == 2.6855064E38F);
            assert(pack.afy_GET() == 1.0405011E38F);
            assert(pack.target_system_GET() == (char)52);
        });
        DemoDevice.SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.target_system_SET((char)52) ;
        p84.vy_SET(-3.2082312E38F) ;
        p84.afy_SET(1.0405011E38F) ;
        p84.time_boot_ms_SET(1197674403L) ;
        p84.type_mask_SET((char)32553) ;
        p84.target_component_SET((char)13) ;
        p84.x_SET(3.3327424E37F) ;
        p84.y_SET(1.3968677E38F) ;
        p84.afx_SET(2.6855064E38F) ;
        p84.yaw_rate_SET(-1.5062216E38F) ;
        p84.z_SET(-1.7633213E38F) ;
        p84.vx_SET(-2.5277858E38F) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p84.vz_SET(-1.3650169E38F) ;
        p84.afz_SET(2.971962E38F) ;
        p84.yaw_SET(9.308668E37F) ;
        LoopBackDemoChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            assert(pack.target_component_GET() == (char)125);
            assert(pack.vz_GET() == 4.0323012E37F);
            assert(pack.lon_int_GET() == -1610895933);
            assert(pack.time_boot_ms_GET() == 2294199221L);
            assert(pack.lat_int_GET() == 1227971093);
            assert(pack.afy_GET() == -3.0807867E38F);
            assert(pack.vx_GET() == 1.4146919E38F);
            assert(pack.target_system_GET() == (char)199);
            assert(pack.afx_GET() == -2.209266E38F);
            assert(pack.yaw_rate_GET() == -2.8893889E38F);
            assert(pack.type_mask_GET() == (char)9844);
            assert(pack.alt_GET() == 3.1917718E38F);
            assert(pack.vy_GET() == 2.1516729E37F);
            assert(pack.afz_GET() == -1.5047251E38F);
            assert(pack.yaw_GET() == 3.1728253E37F);
        });
        DemoDevice.SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.type_mask_SET((char)9844) ;
        p86.target_system_SET((char)199) ;
        p86.afz_SET(-1.5047251E38F) ;
        p86.alt_SET(3.1917718E38F) ;
        p86.lat_int_SET(1227971093) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p86.yaw_SET(3.1728253E37F) ;
        p86.target_component_SET((char)125) ;
        p86.afx_SET(-2.209266E38F) ;
        p86.yaw_rate_SET(-2.8893889E38F) ;
        p86.afy_SET(-3.0807867E38F) ;
        p86.vz_SET(4.0323012E37F) ;
        p86.vy_SET(2.1516729E37F) ;
        p86.vx_SET(1.4146919E38F) ;
        p86.lon_int_SET(-1610895933) ;
        p86.time_boot_ms_SET(2294199221L) ;
        LoopBackDemoChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.yaw_rate_GET() == 1.290286E38F);
            assert(pack.yaw_GET() == -5.767663E37F);
            assert(pack.lat_int_GET() == 305795);
            assert(pack.vz_GET() == -1.6861538E38F);
            assert(pack.afx_GET() == -6.3327636E37F);
            assert(pack.alt_GET() == 8.861317E37F);
            assert(pack.afy_GET() == -2.8049654E38F);
            assert(pack.afz_GET() == 3.156744E37F);
            assert(pack.type_mask_GET() == (char)13747);
            assert(pack.vy_GET() == 1.1899258E38F);
            assert(pack.lon_int_GET() == 457926587);
            assert(pack.time_boot_ms_GET() == 227558253L);
            assert(pack.vx_GET() == 2.7974812E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
        });
        DemoDevice.POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.lon_int_SET(457926587) ;
        p87.yaw_rate_SET(1.290286E38F) ;
        p87.afx_SET(-6.3327636E37F) ;
        p87.yaw_SET(-5.767663E37F) ;
        p87.lat_int_SET(305795) ;
        p87.alt_SET(8.861317E37F) ;
        p87.vx_SET(2.7974812E38F) ;
        p87.vz_SET(-1.6861538E38F) ;
        p87.type_mask_SET((char)13747) ;
        p87.vy_SET(1.1899258E38F) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p87.afz_SET(3.156744E37F) ;
        p87.afy_SET(-2.8049654E38F) ;
        p87.time_boot_ms_SET(227558253L) ;
        LoopBackDemoChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -3.2622552E38F);
            assert(pack.y_GET() == 5.911119E37F);
            assert(pack.pitch_GET() == 1.0478737E38F);
            assert(pack.x_GET() == 6.3774386E37F);
            assert(pack.roll_GET() == -2.3601249E38F);
            assert(pack.time_boot_ms_GET() == 1887762333L);
            assert(pack.z_GET() == 1.0808757E38F);
        });
        DemoDevice.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.roll_SET(-2.3601249E38F) ;
        p89.time_boot_ms_SET(1887762333L) ;
        p89.z_SET(1.0808757E38F) ;
        p89.y_SET(5.911119E37F) ;
        p89.pitch_SET(1.0478737E38F) ;
        p89.x_SET(6.3774386E37F) ;
        p89.yaw_SET(-3.2622552E38F) ;
        LoopBackDemoChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -1539604115);
            assert(pack.xacc_GET() == (short) -1666);
            assert(pack.zacc_GET() == (short) -8470);
            assert(pack.pitch_GET() == 2.7165209E38F);
            assert(pack.pitchspeed_GET() == 3.1701356E38F);
            assert(pack.lat_GET() == 13927678);
            assert(pack.vx_GET() == (short) -24069);
            assert(pack.vz_GET() == (short)4611);
            assert(pack.rollspeed_GET() == 3.922142E37F);
            assert(pack.roll_GET() == -1.7992964E38F);
            assert(pack.time_usec_GET() == 2362489508439905990L);
            assert(pack.yawspeed_GET() == 2.8562206E38F);
            assert(pack.yaw_GET() == 1.5988219E38F);
            assert(pack.yacc_GET() == (short)12101);
            assert(pack.alt_GET() == 2061719370);
            assert(pack.vy_GET() == (short)24681);
        });
        DemoDevice.HIL_STATE p90 = LoopBackDemoChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.rollspeed_SET(3.922142E37F) ;
        p90.alt_SET(2061719370) ;
        p90.vz_SET((short)4611) ;
        p90.roll_SET(-1.7992964E38F) ;
        p90.time_usec_SET(2362489508439905990L) ;
        p90.vy_SET((short)24681) ;
        p90.lon_SET(-1539604115) ;
        p90.yawspeed_SET(2.8562206E38F) ;
        p90.yacc_SET((short)12101) ;
        p90.yaw_SET(1.5988219E38F) ;
        p90.vx_SET((short) -24069) ;
        p90.pitch_SET(2.7165209E38F) ;
        p90.zacc_SET((short) -8470) ;
        p90.pitchspeed_SET(3.1701356E38F) ;
        p90.xacc_SET((short) -1666) ;
        p90.lat_SET(13927678) ;
        LoopBackDemoChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.aux2_GET() == 1.6300388E38F);
            assert(pack.nav_mode_GET() == (char)105);
            assert(pack.aux3_GET() == 3.1585905E37F);
            assert(pack.aux4_GET() == 2.7349022E38F);
            assert(pack.aux1_GET() == -1.411264E38F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_GUIDED_DISARMED);
            assert(pack.time_usec_GET() == 5896396692958215244L);
            assert(pack.throttle_GET() == 2.8938618E38F);
            assert(pack.roll_ailerons_GET() == -3.697862E37F);
            assert(pack.yaw_rudder_GET() == -2.086996E38F);
            assert(pack.pitch_elevator_GET() == -3.2106963E37F);
        });
        DemoDevice.HIL_CONTROLS p91 = LoopBackDemoChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.time_usec_SET(5896396692958215244L) ;
        p91.throttle_SET(2.8938618E38F) ;
        p91.aux1_SET(-1.411264E38F) ;
        p91.aux2_SET(1.6300388E38F) ;
        p91.roll_ailerons_SET(-3.697862E37F) ;
        p91.pitch_elevator_SET(-3.2106963E37F) ;
        p91.yaw_rudder_SET(-2.086996E38F) ;
        p91.aux4_SET(2.7349022E38F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_GUIDED_DISARMED) ;
        p91.aux3_SET(3.1585905E37F) ;
        p91.nav_mode_SET((char)105) ;
        LoopBackDemoChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan8_raw_GET() == (char)2479);
            assert(pack.chan7_raw_GET() == (char)32780);
            assert(pack.chan6_raw_GET() == (char)42446);
            assert(pack.chan12_raw_GET() == (char)29787);
            assert(pack.chan1_raw_GET() == (char)38665);
            assert(pack.chan10_raw_GET() == (char)43716);
            assert(pack.chan5_raw_GET() == (char)58785);
            assert(pack.chan11_raw_GET() == (char)60092);
            assert(pack.chan3_raw_GET() == (char)10279);
            assert(pack.time_usec_GET() == 3801913069637654706L);
            assert(pack.chan9_raw_GET() == (char)1154);
            assert(pack.chan2_raw_GET() == (char)47656);
            assert(pack.chan4_raw_GET() == (char)30087);
            assert(pack.rssi_GET() == (char)198);
        });
        DemoDevice.HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan3_raw_SET((char)10279) ;
        p92.rssi_SET((char)198) ;
        p92.chan5_raw_SET((char)58785) ;
        p92.chan11_raw_SET((char)60092) ;
        p92.chan1_raw_SET((char)38665) ;
        p92.chan9_raw_SET((char)1154) ;
        p92.chan2_raw_SET((char)47656) ;
        p92.chan7_raw_SET((char)32780) ;
        p92.chan6_raw_SET((char)42446) ;
        p92.chan12_raw_SET((char)29787) ;
        p92.chan4_raw_SET((char)30087) ;
        p92.time_usec_SET(3801913069637654706L) ;
        p92.chan10_raw_SET((char)43716) ;
        p92.chan8_raw_SET((char)2479) ;
        LoopBackDemoChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {2.196173E38F, 2.2585258E38F, -2.562518E38F, -1.823416E38F, -2.9574706E38F, -1.6156833E38F, 1.0598166E38F, -1.943836E38F, -3.0063713E38F, -2.1825599E38F, 1.9119832E38F, 1.707318E38F, -1.2134577E38F, 1.7299417E37F, 3.8575597E36F, 2.668109E38F}));
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_AUTO_ARMED);
            assert(pack.flags_GET() == 895296554649877125L);
            assert(pack.time_usec_GET() == 6225669437542157527L);
        });
        DemoDevice.HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.time_usec_SET(6225669437542157527L) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_AUTO_ARMED) ;
        p93.flags_SET(895296554649877125L) ;
        p93.controls_SET(new float[] {2.196173E38F, 2.2585258E38F, -2.562518E38F, -1.823416E38F, -2.9574706E38F, -1.6156833E38F, 1.0598166E38F, -1.943836E38F, -3.0063713E38F, -2.1825599E38F, 1.9119832E38F, 1.707318E38F, -1.2134577E38F, 1.7299417E37F, 3.8575597E36F, 2.668109E38F}, 0) ;
        LoopBackDemoChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.flow_x_GET() == (short)19744);
            assert(pack.flow_y_GET() == (short) -16905);
            assert(pack.flow_comp_m_y_GET() == -2.7149034E38F);
            assert(pack.time_usec_GET() == 2318655728976934396L);
            assert(pack.flow_rate_y_TRY(ph) == -1.5876168E38F);
            assert(pack.ground_distance_GET() == -1.8532466E38F);
            assert(pack.flow_rate_x_TRY(ph) == 2.501375E38F);
            assert(pack.sensor_id_GET() == (char)12);
            assert(pack.quality_GET() == (char)172);
            assert(pack.flow_comp_m_x_GET() == -3.5276732E36F);
        });
        DemoDevice.OPTICAL_FLOW p100 = LoopBackDemoChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.flow_x_SET((short)19744) ;
        p100.flow_y_SET((short) -16905) ;
        p100.quality_SET((char)172) ;
        p100.time_usec_SET(2318655728976934396L) ;
        p100.flow_comp_m_x_SET(-3.5276732E36F) ;
        p100.sensor_id_SET((char)12) ;
        p100.flow_comp_m_y_SET(-2.7149034E38F) ;
        p100.flow_rate_y_SET(-1.5876168E38F, PH) ;
        p100.ground_distance_SET(-1.8532466E38F) ;
        p100.flow_rate_x_SET(2.501375E38F, PH) ;
        LoopBackDemoChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -2.8780254E38F);
            assert(pack.x_GET() == -1.249993E38F);
            assert(pack.y_GET() == 2.7139053E37F);
            assert(pack.yaw_GET() == -2.3410284E38F);
            assert(pack.roll_GET() == -3.38839E38F);
            assert(pack.pitch_GET() == 2.8001907E38F);
            assert(pack.usec_GET() == 5423201365703454298L);
        });
        DemoDevice.GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.pitch_SET(2.8001907E38F) ;
        p101.z_SET(-2.8780254E38F) ;
        p101.roll_SET(-3.38839E38F) ;
        p101.usec_SET(5423201365703454298L) ;
        p101.yaw_SET(-2.3410284E38F) ;
        p101.y_SET(2.7139053E37F) ;
        p101.x_SET(-1.249993E38F) ;
        LoopBackDemoChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 6515088209526615442L);
            assert(pack.z_GET() == 3.323796E37F);
            assert(pack.yaw_GET() == -3.3577272E38F);
            assert(pack.roll_GET() == 2.4720132E38F);
            assert(pack.x_GET() == 2.5108856E38F);
            assert(pack.y_GET() == 1.3830667E38F);
            assert(pack.pitch_GET() == -1.3329659E38F);
        });
        DemoDevice.VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.roll_SET(2.4720132E38F) ;
        p102.y_SET(1.3830667E38F) ;
        p102.usec_SET(6515088209526615442L) ;
        p102.x_SET(2.5108856E38F) ;
        p102.pitch_SET(-1.3329659E38F) ;
        p102.z_SET(3.323796E37F) ;
        p102.yaw_SET(-3.3577272E38F) ;
        LoopBackDemoChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 1641111803352245843L);
            assert(pack.z_GET() == 1.4527983E38F);
            assert(pack.y_GET() == -8.673112E37F);
            assert(pack.x_GET() == 2.4087152E38F);
        });
        DemoDevice.VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.z_SET(1.4527983E38F) ;
        p103.x_SET(2.4087152E38F) ;
        p103.y_SET(-8.673112E37F) ;
        p103.usec_SET(1641111803352245843L) ;
        LoopBackDemoChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 8001418888996966530L);
            assert(pack.y_GET() == 2.1010725E38F);
            assert(pack.x_GET() == 2.038498E38F);
            assert(pack.pitch_GET() == -2.4627107E38F);
            assert(pack.yaw_GET() == -7.1643586E37F);
            assert(pack.roll_GET() == 1.945177E38F);
            assert(pack.z_GET() == -2.9490857E38F);
        });
        DemoDevice.VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.z_SET(-2.9490857E38F) ;
        p104.usec_SET(8001418888996966530L) ;
        p104.x_SET(2.038498E38F) ;
        p104.yaw_SET(-7.1643586E37F) ;
        p104.y_SET(2.1010725E38F) ;
        p104.pitch_SET(-2.4627107E38F) ;
        p104.roll_SET(1.945177E38F) ;
        LoopBackDemoChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.zmag_GET() == -1.6627884E38F);
            assert(pack.abs_pressure_GET() == -1.830191E38F);
            assert(pack.zgyro_GET() == 2.0254836E38F);
            assert(pack.xgyro_GET() == -2.579951E38F);
            assert(pack.fields_updated_GET() == (char)21184);
            assert(pack.xmag_GET() == 2.8437514E38F);
            assert(pack.time_usec_GET() == 4038154436635726120L);
            assert(pack.zacc_GET() == 2.0496965E38F);
            assert(pack.xacc_GET() == 2.6683704E38F);
            assert(pack.temperature_GET() == -3.4642067E37F);
            assert(pack.diff_pressure_GET() == -2.8120123E38F);
            assert(pack.yacc_GET() == -2.5143383E38F);
            assert(pack.ygyro_GET() == -2.4669034E38F);
            assert(pack.ymag_GET() == 1.0815006E38F);
            assert(pack.pressure_alt_GET() == -9.793187E37F);
        });
        DemoDevice.HIGHRES_IMU p105 = LoopBackDemoChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.ygyro_SET(-2.4669034E38F) ;
        p105.pressure_alt_SET(-9.793187E37F) ;
        p105.xacc_SET(2.6683704E38F) ;
        p105.xgyro_SET(-2.579951E38F) ;
        p105.time_usec_SET(4038154436635726120L) ;
        p105.temperature_SET(-3.4642067E37F) ;
        p105.abs_pressure_SET(-1.830191E38F) ;
        p105.zmag_SET(-1.6627884E38F) ;
        p105.xmag_SET(2.8437514E38F) ;
        p105.ymag_SET(1.0815006E38F) ;
        p105.zgyro_SET(2.0254836E38F) ;
        p105.diff_pressure_SET(-2.8120123E38F) ;
        p105.fields_updated_SET((char)21184) ;
        p105.yacc_SET(-2.5143383E38F) ;
        p105.zacc_SET(2.0496965E38F) ;
        LoopBackDemoChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.time_delta_distance_us_GET() == 4191134103L);
            assert(pack.sensor_id_GET() == (char)248);
            assert(pack.integrated_y_GET() == 1.816909E38F);
            assert(pack.integrated_ygyro_GET() == -1.8848433E38F);
            assert(pack.integrated_zgyro_GET() == -2.6738795E38F);
            assert(pack.integrated_xgyro_GET() == -3.172661E38F);
            assert(pack.temperature_GET() == (short) -27461);
            assert(pack.integration_time_us_GET() == 2284816596L);
            assert(pack.integrated_x_GET() == 1.4166133E38F);
            assert(pack.quality_GET() == (char)173);
            assert(pack.time_usec_GET() == 2006332467138151622L);
            assert(pack.distance_GET() == -1.2368216E38F);
        });
        DemoDevice.OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.distance_SET(-1.2368216E38F) ;
        p106.time_delta_distance_us_SET(4191134103L) ;
        p106.integrated_xgyro_SET(-3.172661E38F) ;
        p106.integration_time_us_SET(2284816596L) ;
        p106.sensor_id_SET((char)248) ;
        p106.time_usec_SET(2006332467138151622L) ;
        p106.integrated_zgyro_SET(-2.6738795E38F) ;
        p106.quality_SET((char)173) ;
        p106.integrated_ygyro_SET(-1.8848433E38F) ;
        p106.integrated_x_SET(1.4166133E38F) ;
        p106.integrated_y_SET(1.816909E38F) ;
        p106.temperature_SET((short) -27461) ;
        LoopBackDemoChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.zacc_GET() == -1.4068218E38F);
            assert(pack.xgyro_GET() == -7.9832705E37F);
            assert(pack.xacc_GET() == 7.07804E35F);
            assert(pack.pressure_alt_GET() == -2.1788596E38F);
            assert(pack.yacc_GET() == -1.9374007E38F);
            assert(pack.ygyro_GET() == 3.2560297E38F);
            assert(pack.temperature_GET() == 3.001209E38F);
            assert(pack.xmag_GET() == 1.7864373E38F);
            assert(pack.zgyro_GET() == -2.0122315E37F);
            assert(pack.time_usec_GET() == 7477331935543517171L);
            assert(pack.ymag_GET() == 2.5460862E38F);
            assert(pack.diff_pressure_GET() == 1.4722288E38F);
            assert(pack.fields_updated_GET() == 3028553338L);
            assert(pack.abs_pressure_GET() == -9.398952E37F);
            assert(pack.zmag_GET() == 2.7228202E38F);
        });
        DemoDevice.HIL_SENSOR p107 = LoopBackDemoChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.abs_pressure_SET(-9.398952E37F) ;
        p107.xacc_SET(7.07804E35F) ;
        p107.temperature_SET(3.001209E38F) ;
        p107.fields_updated_SET(3028553338L) ;
        p107.yacc_SET(-1.9374007E38F) ;
        p107.pressure_alt_SET(-2.1788596E38F) ;
        p107.time_usec_SET(7477331935543517171L) ;
        p107.xmag_SET(1.7864373E38F) ;
        p107.zmag_SET(2.7228202E38F) ;
        p107.ymag_SET(2.5460862E38F) ;
        p107.xgyro_SET(-7.9832705E37F) ;
        p107.zacc_SET(-1.4068218E38F) ;
        p107.zgyro_SET(-2.0122315E37F) ;
        p107.ygyro_SET(3.2560297E38F) ;
        p107.diff_pressure_SET(1.4722288E38F) ;
        LoopBackDemoChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.ve_GET() == -1.3309533E38F);
            assert(pack.alt_GET() == -1.3915817E38F);
            assert(pack.q2_GET() == -1.2308111E38F);
            assert(pack.q3_GET() == -2.5469709E38F);
            assert(pack.vd_GET() == 6.8554247E36F);
            assert(pack.yacc_GET() == 2.8567938E38F);
            assert(pack.std_dev_vert_GET() == 4.317032E37F);
            assert(pack.xacc_GET() == 2.7923534E38F);
            assert(pack.ygyro_GET() == 2.989982E37F);
            assert(pack.pitch_GET() == 2.9729092E38F);
            assert(pack.roll_GET() == -2.9916702E38F);
            assert(pack.q1_GET() == -2.7995506E38F);
            assert(pack.xgyro_GET() == 5.7450493E37F);
            assert(pack.yaw_GET() == 6.816547E37F);
            assert(pack.vn_GET() == -6.338947E37F);
            assert(pack.zgyro_GET() == 3.2052359E38F);
            assert(pack.zacc_GET() == -2.2767066E38F);
            assert(pack.q4_GET() == -2.2712553E38F);
            assert(pack.lat_GET() == 3.021553E38F);
            assert(pack.lon_GET() == -1.5703516E38F);
            assert(pack.std_dev_horz_GET() == 2.2295848E38F);
        });
        DemoDevice.SIM_STATE p108 = LoopBackDemoChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.zacc_SET(-2.2767066E38F) ;
        p108.zgyro_SET(3.2052359E38F) ;
        p108.alt_SET(-1.3915817E38F) ;
        p108.vn_SET(-6.338947E37F) ;
        p108.pitch_SET(2.9729092E38F) ;
        p108.ve_SET(-1.3309533E38F) ;
        p108.q3_SET(-2.5469709E38F) ;
        p108.q4_SET(-2.2712553E38F) ;
        p108.roll_SET(-2.9916702E38F) ;
        p108.xacc_SET(2.7923534E38F) ;
        p108.std_dev_horz_SET(2.2295848E38F) ;
        p108.lat_SET(3.021553E38F) ;
        p108.vd_SET(6.8554247E36F) ;
        p108.std_dev_vert_SET(4.317032E37F) ;
        p108.yacc_SET(2.8567938E38F) ;
        p108.q1_SET(-2.7995506E38F) ;
        p108.lon_SET(-1.5703516E38F) ;
        p108.xgyro_SET(5.7450493E37F) ;
        p108.q2_SET(-1.2308111E38F) ;
        p108.ygyro_SET(2.989982E37F) ;
        p108.yaw_SET(6.816547E37F) ;
        LoopBackDemoChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.txbuf_GET() == (char)221);
            assert(pack.rssi_GET() == (char)2);
            assert(pack.rxerrors_GET() == (char)22349);
            assert(pack.remnoise_GET() == (char)213);
            assert(pack.remrssi_GET() == (char)67);
            assert(pack.noise_GET() == (char)191);
            assert(pack.fixed__GET() == (char)64202);
        });
        DemoDevice.RADIO_STATUS p109 = LoopBackDemoChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.remrssi_SET((char)67) ;
        p109.fixed__SET((char)64202) ;
        p109.noise_SET((char)191) ;
        p109.txbuf_SET((char)221) ;
        p109.remnoise_SET((char)213) ;
        p109.rxerrors_SET((char)22349) ;
        p109.rssi_SET((char)2) ;
        LoopBackDemoChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)48);
            assert(pack.target_component_GET() == (char)102);
            assert(pack.target_network_GET() == (char)195);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)207, (char)136, (char)21, (char)165, (char)27, (char)169, (char)151, (char)210, (char)166, (char)147, (char)159, (char)239, (char)48, (char)48, (char)216, (char)97, (char)202, (char)198, (char)209, (char)48, (char)9, (char)241, (char)212, (char)214, (char)93, (char)242, (char)175, (char)85, (char)82, (char)112, (char)126, (char)69, (char)129, (char)247, (char)201, (char)239, (char)174, (char)107, (char)204, (char)9, (char)144, (char)112, (char)195, (char)234, (char)198, (char)189, (char)165, (char)238, (char)165, (char)109, (char)228, (char)130, (char)113, (char)154, (char)150, (char)57, (char)152, (char)62, (char)173, (char)208, (char)252, (char)173, (char)146, (char)125, (char)180, (char)118, (char)221, (char)240, (char)45, (char)162, (char)84, (char)139, (char)175, (char)38, (char)148, (char)138, (char)122, (char)146, (char)126, (char)138, (char)124, (char)28, (char)143, (char)3, (char)80, (char)36, (char)190, (char)4, (char)20, (char)156, (char)6, (char)88, (char)83, (char)222, (char)253, (char)178, (char)92, (char)214, (char)216, (char)179, (char)84, (char)82, (char)144, (char)1, (char)139, (char)135, (char)237, (char)82, (char)237, (char)170, (char)67, (char)251, (char)34, (char)35, (char)246, (char)183, (char)240, (char)139, (char)21, (char)29, (char)73, (char)94, (char)105, (char)13, (char)149, (char)99, (char)6, (char)70, (char)23, (char)79, (char)67, (char)175, (char)38, (char)180, (char)155, (char)80, (char)87, (char)183, (char)0, (char)19, (char)68, (char)159, (char)120, (char)225, (char)130, (char)171, (char)157, (char)137, (char)69, (char)146, (char)125, (char)62, (char)183, (char)10, (char)93, (char)207, (char)148, (char)212, (char)218, (char)125, (char)75, (char)203, (char)100, (char)110, (char)24, (char)185, (char)125, (char)155, (char)51, (char)84, (char)224, (char)159, (char)146, (char)151, (char)103, (char)89, (char)154, (char)170, (char)235, (char)15, (char)107, (char)224, (char)98, (char)187, (char)188, (char)204, (char)122, (char)244, (char)10, (char)190, (char)183, (char)132, (char)239, (char)138, (char)254, (char)145, (char)252, (char)180, (char)91, (char)35, (char)151, (char)218, (char)0, (char)97, (char)234, (char)155, (char)236, (char)181, (char)49, (char)222, (char)179, (char)198, (char)137, (char)5, (char)5, (char)172, (char)207, (char)152, (char)91, (char)238, (char)80, (char)39, (char)201, (char)186, (char)234, (char)86, (char)39, (char)0, (char)220, (char)34, (char)206, (char)7, (char)25, (char)10, (char)163, (char)238, (char)200, (char)41, (char)143, (char)237, (char)24, (char)79, (char)239, (char)116, (char)152, (char)228, (char)91, (char)105, (char)114, (char)120, (char)216}));
        });
        DemoDevice.FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_system_SET((char)48) ;
        p110.payload_SET(new char[] {(char)207, (char)136, (char)21, (char)165, (char)27, (char)169, (char)151, (char)210, (char)166, (char)147, (char)159, (char)239, (char)48, (char)48, (char)216, (char)97, (char)202, (char)198, (char)209, (char)48, (char)9, (char)241, (char)212, (char)214, (char)93, (char)242, (char)175, (char)85, (char)82, (char)112, (char)126, (char)69, (char)129, (char)247, (char)201, (char)239, (char)174, (char)107, (char)204, (char)9, (char)144, (char)112, (char)195, (char)234, (char)198, (char)189, (char)165, (char)238, (char)165, (char)109, (char)228, (char)130, (char)113, (char)154, (char)150, (char)57, (char)152, (char)62, (char)173, (char)208, (char)252, (char)173, (char)146, (char)125, (char)180, (char)118, (char)221, (char)240, (char)45, (char)162, (char)84, (char)139, (char)175, (char)38, (char)148, (char)138, (char)122, (char)146, (char)126, (char)138, (char)124, (char)28, (char)143, (char)3, (char)80, (char)36, (char)190, (char)4, (char)20, (char)156, (char)6, (char)88, (char)83, (char)222, (char)253, (char)178, (char)92, (char)214, (char)216, (char)179, (char)84, (char)82, (char)144, (char)1, (char)139, (char)135, (char)237, (char)82, (char)237, (char)170, (char)67, (char)251, (char)34, (char)35, (char)246, (char)183, (char)240, (char)139, (char)21, (char)29, (char)73, (char)94, (char)105, (char)13, (char)149, (char)99, (char)6, (char)70, (char)23, (char)79, (char)67, (char)175, (char)38, (char)180, (char)155, (char)80, (char)87, (char)183, (char)0, (char)19, (char)68, (char)159, (char)120, (char)225, (char)130, (char)171, (char)157, (char)137, (char)69, (char)146, (char)125, (char)62, (char)183, (char)10, (char)93, (char)207, (char)148, (char)212, (char)218, (char)125, (char)75, (char)203, (char)100, (char)110, (char)24, (char)185, (char)125, (char)155, (char)51, (char)84, (char)224, (char)159, (char)146, (char)151, (char)103, (char)89, (char)154, (char)170, (char)235, (char)15, (char)107, (char)224, (char)98, (char)187, (char)188, (char)204, (char)122, (char)244, (char)10, (char)190, (char)183, (char)132, (char)239, (char)138, (char)254, (char)145, (char)252, (char)180, (char)91, (char)35, (char)151, (char)218, (char)0, (char)97, (char)234, (char)155, (char)236, (char)181, (char)49, (char)222, (char)179, (char)198, (char)137, (char)5, (char)5, (char)172, (char)207, (char)152, (char)91, (char)238, (char)80, (char)39, (char)201, (char)186, (char)234, (char)86, (char)39, (char)0, (char)220, (char)34, (char)206, (char)7, (char)25, (char)10, (char)163, (char)238, (char)200, (char)41, (char)143, (char)237, (char)24, (char)79, (char)239, (char)116, (char)152, (char)228, (char)91, (char)105, (char)114, (char)120, (char)216}, 0) ;
        p110.target_network_SET((char)195) ;
        p110.target_component_SET((char)102) ;
        LoopBackDemoChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.tc1_GET() == -5880600077627019097L);
            assert(pack.ts1_GET() == 7275190829752658457L);
        });
        DemoDevice.TIMESYNC p111 = LoopBackDemoChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(-5880600077627019097L) ;
        p111.ts1_SET(7275190829752658457L) ;
        LoopBackDemoChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 593016895L);
            assert(pack.time_usec_GET() == 8127762104220295209L);
        });
        DemoDevice.CAMERA_TRIGGER p112 = LoopBackDemoChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.seq_SET(593016895L) ;
        p112.time_usec_SET(8127762104220295209L) ;
        LoopBackDemoChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.fix_type_GET() == (char)102);
            assert(pack.satellites_visible_GET() == (char)80);
            assert(pack.cog_GET() == (char)34461);
            assert(pack.eph_GET() == (char)57970);
            assert(pack.vd_GET() == (short)30449);
            assert(pack.epv_GET() == (char)53841);
            assert(pack.ve_GET() == (short)4990);
            assert(pack.vel_GET() == (char)62298);
            assert(pack.time_usec_GET() == 5312491409468264964L);
            assert(pack.vn_GET() == (short)25062);
            assert(pack.lat_GET() == 1251075352);
            assert(pack.lon_GET() == 1820273586);
            assert(pack.alt_GET() == -317840769);
        });
        DemoDevice.HIL_GPS p113 = LoopBackDemoChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.ve_SET((short)4990) ;
        p113.eph_SET((char)57970) ;
        p113.satellites_visible_SET((char)80) ;
        p113.vd_SET((short)30449) ;
        p113.vel_SET((char)62298) ;
        p113.lat_SET(1251075352) ;
        p113.alt_SET(-317840769) ;
        p113.cog_SET((char)34461) ;
        p113.epv_SET((char)53841) ;
        p113.time_usec_SET(5312491409468264964L) ;
        p113.lon_SET(1820273586) ;
        p113.fix_type_SET((char)102) ;
        p113.vn_SET((short)25062) ;
        LoopBackDemoChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.integrated_zgyro_GET() == 1.4467306E38F);
            assert(pack.quality_GET() == (char)53);
            assert(pack.integrated_y_GET() == 2.6813226E38F);
            assert(pack.time_delta_distance_us_GET() == 1251597278L);
            assert(pack.sensor_id_GET() == (char)225);
            assert(pack.integration_time_us_GET() == 3867325443L);
            assert(pack.integrated_xgyro_GET() == -2.1742086E38F);
            assert(pack.temperature_GET() == (short)13609);
            assert(pack.time_usec_GET() == 1925259245535985305L);
            assert(pack.integrated_ygyro_GET() == 2.4044312E38F);
            assert(pack.integrated_x_GET() == -2.553875E38F);
            assert(pack.distance_GET() == 1.835471E38F);
        });
        DemoDevice.HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.integrated_x_SET(-2.553875E38F) ;
        p114.integrated_zgyro_SET(1.4467306E38F) ;
        p114.sensor_id_SET((char)225) ;
        p114.integrated_ygyro_SET(2.4044312E38F) ;
        p114.time_usec_SET(1925259245535985305L) ;
        p114.integrated_y_SET(2.6813226E38F) ;
        p114.integrated_xgyro_SET(-2.1742086E38F) ;
        p114.time_delta_distance_us_SET(1251597278L) ;
        p114.integration_time_us_SET(3867325443L) ;
        p114.quality_SET((char)53) ;
        p114.distance_SET(1.835471E38F) ;
        p114.temperature_SET((short)13609) ;
        LoopBackDemoChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == (short)26631);
            assert(pack.vx_GET() == (short) -29901);
            assert(pack.vy_GET() == (short) -2303);
            assert(pack.ind_airspeed_GET() == (char)16714);
            assert(pack.xacc_GET() == (short) -1184);
            assert(pack.zacc_GET() == (short) -24124);
            assert(pack.yawspeed_GET() == -2.283548E38F);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {1.7473724E38F, -2.2522703E37F, -1.1718355E38F, -1.5183113E37F}));
            assert(pack.lon_GET() == 1806727278);
            assert(pack.pitchspeed_GET() == 1.7620254E38F);
            assert(pack.rollspeed_GET() == 1.816206E38F);
            assert(pack.true_airspeed_GET() == (char)46299);
            assert(pack.vz_GET() == (short)7829);
            assert(pack.time_usec_GET() == 8542379299157036326L);
            assert(pack.alt_GET() == -691937514);
            assert(pack.lat_GET() == -420988659);
        });
        DemoDevice.HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.vy_SET((short) -2303) ;
        p115.rollspeed_SET(1.816206E38F) ;
        p115.vx_SET((short) -29901) ;
        p115.yacc_SET((short)26631) ;
        p115.true_airspeed_SET((char)46299) ;
        p115.alt_SET(-691937514) ;
        p115.yawspeed_SET(-2.283548E38F) ;
        p115.time_usec_SET(8542379299157036326L) ;
        p115.lat_SET(-420988659) ;
        p115.zacc_SET((short) -24124) ;
        p115.pitchspeed_SET(1.7620254E38F) ;
        p115.ind_airspeed_SET((char)16714) ;
        p115.xacc_SET((short) -1184) ;
        p115.vz_SET((short)7829) ;
        p115.lon_SET(1806727278) ;
        p115.attitude_quaternion_SET(new float[] {1.7473724E38F, -2.2522703E37F, -1.1718355E38F, -1.5183113E37F}, 0) ;
        LoopBackDemoChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.zmag_GET() == (short) -27632);
            assert(pack.zgyro_GET() == (short)9861);
            assert(pack.ygyro_GET() == (short)11021);
            assert(pack.xacc_GET() == (short)9411);
            assert(pack.zacc_GET() == (short)12593);
            assert(pack.ymag_GET() == (short) -6399);
            assert(pack.xmag_GET() == (short) -19900);
            assert(pack.xgyro_GET() == (short) -27224);
            assert(pack.time_boot_ms_GET() == 1257945413L);
            assert(pack.yacc_GET() == (short)17965);
        });
        DemoDevice.SCALED_IMU2 p116 = LoopBackDemoChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.zgyro_SET((short)9861) ;
        p116.zmag_SET((short) -27632) ;
        p116.time_boot_ms_SET(1257945413L) ;
        p116.xmag_SET((short) -19900) ;
        p116.zacc_SET((short)12593) ;
        p116.xgyro_SET((short) -27224) ;
        p116.yacc_SET((short)17965) ;
        p116.xacc_SET((short)9411) ;
        p116.ygyro_SET((short)11021) ;
        p116.ymag_SET((short) -6399) ;
        LoopBackDemoChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.start_GET() == (char)48474);
            assert(pack.target_component_GET() == (char)46);
            assert(pack.end_GET() == (char)1656);
            assert(pack.target_system_GET() == (char)67);
        });
        DemoDevice.LOG_REQUEST_LIST p117 = LoopBackDemoChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.end_SET((char)1656) ;
        p117.target_component_SET((char)46) ;
        p117.start_SET((char)48474) ;
        p117.target_system_SET((char)67) ;
        LoopBackDemoChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.size_GET() == 1142181938L);
            assert(pack.num_logs_GET() == (char)2082);
            assert(pack.time_utc_GET() == 3777710193L);
            assert(pack.last_log_num_GET() == (char)31574);
            assert(pack.id_GET() == (char)52337);
        });
        DemoDevice.LOG_ENTRY p118 = LoopBackDemoChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.time_utc_SET(3777710193L) ;
        p118.last_log_num_SET((char)31574) ;
        p118.id_SET((char)52337) ;
        p118.size_SET(1142181938L) ;
        p118.num_logs_SET((char)2082) ;
        LoopBackDemoChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)227);
            assert(pack.id_GET() == (char)24904);
            assert(pack.target_system_GET() == (char)39);
            assert(pack.count_GET() == 4032771827L);
            assert(pack.ofs_GET() == 191774599L);
        });
        DemoDevice.LOG_REQUEST_DATA p119 = LoopBackDemoChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_system_SET((char)39) ;
        p119.count_SET(4032771827L) ;
        p119.target_component_SET((char)227) ;
        p119.ofs_SET(191774599L) ;
        p119.id_SET((char)24904) ;
        LoopBackDemoChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)103, (char)112, (char)187, (char)145, (char)112, (char)36, (char)233, (char)100, (char)62, (char)163, (char)16, (char)112, (char)176, (char)166, (char)206, (char)65, (char)205, (char)160, (char)11, (char)30, (char)53, (char)146, (char)95, (char)91, (char)29, (char)14, (char)77, (char)164, (char)149, (char)122, (char)55, (char)226, (char)191, (char)210, (char)91, (char)14, (char)255, (char)63, (char)50, (char)198, (char)9, (char)124, (char)0, (char)51, (char)206, (char)22, (char)142, (char)196, (char)61, (char)30, (char)217, (char)112, (char)131, (char)7, (char)145, (char)45, (char)139, (char)136, (char)117, (char)57, (char)219, (char)193, (char)51, (char)6, (char)88, (char)94, (char)133, (char)7, (char)143, (char)113, (char)29, (char)150, (char)124, (char)157, (char)71, (char)29, (char)38, (char)75, (char)245, (char)231, (char)133, (char)85, (char)232, (char)229, (char)248, (char)39, (char)206, (char)115, (char)74, (char)64}));
            assert(pack.id_GET() == (char)6725);
            assert(pack.ofs_GET() == 1602011012L);
            assert(pack.count_GET() == (char)248);
        });
        DemoDevice.LOG_DATA p120 = LoopBackDemoChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.ofs_SET(1602011012L) ;
        p120.data__SET(new char[] {(char)103, (char)112, (char)187, (char)145, (char)112, (char)36, (char)233, (char)100, (char)62, (char)163, (char)16, (char)112, (char)176, (char)166, (char)206, (char)65, (char)205, (char)160, (char)11, (char)30, (char)53, (char)146, (char)95, (char)91, (char)29, (char)14, (char)77, (char)164, (char)149, (char)122, (char)55, (char)226, (char)191, (char)210, (char)91, (char)14, (char)255, (char)63, (char)50, (char)198, (char)9, (char)124, (char)0, (char)51, (char)206, (char)22, (char)142, (char)196, (char)61, (char)30, (char)217, (char)112, (char)131, (char)7, (char)145, (char)45, (char)139, (char)136, (char)117, (char)57, (char)219, (char)193, (char)51, (char)6, (char)88, (char)94, (char)133, (char)7, (char)143, (char)113, (char)29, (char)150, (char)124, (char)157, (char)71, (char)29, (char)38, (char)75, (char)245, (char)231, (char)133, (char)85, (char)232, (char)229, (char)248, (char)39, (char)206, (char)115, (char)74, (char)64}, 0) ;
        p120.count_SET((char)248) ;
        p120.id_SET((char)6725) ;
        LoopBackDemoChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)71);
            assert(pack.target_component_GET() == (char)159);
        });
        DemoDevice.LOG_ERASE p121 = LoopBackDemoChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)71) ;
        p121.target_component_SET((char)159) ;
        LoopBackDemoChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)74);
            assert(pack.target_component_GET() == (char)162);
        });
        DemoDevice.LOG_REQUEST_END p122 = LoopBackDemoChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_component_SET((char)162) ;
        p122.target_system_SET((char)74) ;
        LoopBackDemoChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)34);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)56, (char)212, (char)52, (char)235, (char)81, (char)169, (char)17, (char)206, (char)124, (char)77, (char)127, (char)89, (char)19, (char)102, (char)217, (char)238, (char)79, (char)92, (char)239, (char)70, (char)164, (char)9, (char)36, (char)138, (char)230, (char)137, (char)142, (char)248, (char)242, (char)154, (char)160, (char)132, (char)150, (char)110, (char)248, (char)167, (char)86, (char)187, (char)44, (char)234, (char)157, (char)58, (char)47, (char)134, (char)133, (char)79, (char)40, (char)79, (char)42, (char)167, (char)224, (char)6, (char)237, (char)76, (char)157, (char)126, (char)21, (char)116, (char)249, (char)75, (char)60, (char)246, (char)111, (char)192, (char)96, (char)34, (char)198, (char)197, (char)71, (char)58, (char)62, (char)26, (char)211, (char)235, (char)164, (char)101, (char)14, (char)44, (char)233, (char)104, (char)232, (char)97, (char)27, (char)219, (char)212, (char)245, (char)182, (char)3, (char)33, (char)228, (char)101, (char)26, (char)217, (char)128, (char)32, (char)159, (char)77, (char)70, (char)121, (char)30, (char)241, (char)237, (char)190, (char)190, (char)217, (char)1, (char)194, (char)242, (char)46, (char)89}));
            assert(pack.len_GET() == (char)26);
            assert(pack.target_component_GET() == (char)112);
        });
        DemoDevice.GPS_INJECT_DATA p123 = LoopBackDemoChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.len_SET((char)26) ;
        p123.target_component_SET((char)112) ;
        p123.target_system_SET((char)34) ;
        p123.data__SET(new char[] {(char)56, (char)212, (char)52, (char)235, (char)81, (char)169, (char)17, (char)206, (char)124, (char)77, (char)127, (char)89, (char)19, (char)102, (char)217, (char)238, (char)79, (char)92, (char)239, (char)70, (char)164, (char)9, (char)36, (char)138, (char)230, (char)137, (char)142, (char)248, (char)242, (char)154, (char)160, (char)132, (char)150, (char)110, (char)248, (char)167, (char)86, (char)187, (char)44, (char)234, (char)157, (char)58, (char)47, (char)134, (char)133, (char)79, (char)40, (char)79, (char)42, (char)167, (char)224, (char)6, (char)237, (char)76, (char)157, (char)126, (char)21, (char)116, (char)249, (char)75, (char)60, (char)246, (char)111, (char)192, (char)96, (char)34, (char)198, (char)197, (char)71, (char)58, (char)62, (char)26, (char)211, (char)235, (char)164, (char)101, (char)14, (char)44, (char)233, (char)104, (char)232, (char)97, (char)27, (char)219, (char)212, (char)245, (char)182, (char)3, (char)33, (char)228, (char)101, (char)26, (char)217, (char)128, (char)32, (char)159, (char)77, (char)70, (char)121, (char)30, (char)241, (char)237, (char)190, (char)190, (char)217, (char)1, (char)194, (char)242, (char)46, (char)89}, 0) ;
        LoopBackDemoChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.cog_GET() == (char)1925);
            assert(pack.satellites_visible_GET() == (char)59);
            assert(pack.lon_GET() == -856964157);
            assert(pack.lat_GET() == -2067230873);
            assert(pack.vel_GET() == (char)58880);
            assert(pack.dgps_age_GET() == 1705898159L);
            assert(pack.alt_GET() == -1146048274);
            assert(pack.eph_GET() == (char)39969);
            assert(pack.time_usec_GET() == 8431052631484100665L);
            assert(pack.dgps_numch_GET() == (char)32);
            assert(pack.epv_GET() == (char)21513);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
        });
        DemoDevice.GPS2_RAW p124 = LoopBackDemoChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.alt_SET(-1146048274) ;
        p124.cog_SET((char)1925) ;
        p124.satellites_visible_SET((char)59) ;
        p124.time_usec_SET(8431052631484100665L) ;
        p124.eph_SET((char)39969) ;
        p124.lat_SET(-2067230873) ;
        p124.dgps_numch_SET((char)32) ;
        p124.vel_SET((char)58880) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC) ;
        p124.epv_SET((char)21513) ;
        p124.dgps_age_SET(1705898159L) ;
        p124.lon_SET(-856964157) ;
        LoopBackDemoChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.Vcc_GET() == (char)24484);
            assert(pack.Vservo_GET() == (char)36555);
            assert(pack.flags_GET() == MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT);
        });
        DemoDevice.POWER_STATUS p125 = LoopBackDemoChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vservo_SET((char)36555) ;
        p125.Vcc_SET((char)24484) ;
        p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT) ;
        LoopBackDemoChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1);
            assert(pack.count_GET() == (char)60);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)16, (char)236, (char)192, (char)242, (char)202, (char)167, (char)163, (char)151, (char)108, (char)52, (char)67, (char)109, (char)146, (char)232, (char)66, (char)64, (char)18, (char)109, (char)190, (char)91, (char)28, (char)104, (char)87, (char)38, (char)109, (char)144, (char)193, (char)232, (char)10, (char)24, (char)7, (char)3, (char)18, (char)191, (char)149, (char)1, (char)161, (char)179, (char)22, (char)183, (char)244, (char)219, (char)62, (char)145, (char)156, (char)2, (char)114, (char)139, (char)236, (char)209, (char)230, (char)156, (char)18, (char)168, (char)79, (char)162, (char)26, (char)181, (char)0, (char)72, (char)100, (char)34, (char)212, (char)171, (char)186, (char)52, (char)0, (char)217, (char)249, (char)3}));
            assert(pack.baudrate_GET() == 4197862061L);
            assert(pack.flags_GET() == SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND);
            assert(pack.timeout_GET() == (char)11401);
        });
        DemoDevice.SERIAL_CONTROL p126 = LoopBackDemoChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.data__SET(new char[] {(char)16, (char)236, (char)192, (char)242, (char)202, (char)167, (char)163, (char)151, (char)108, (char)52, (char)67, (char)109, (char)146, (char)232, (char)66, (char)64, (char)18, (char)109, (char)190, (char)91, (char)28, (char)104, (char)87, (char)38, (char)109, (char)144, (char)193, (char)232, (char)10, (char)24, (char)7, (char)3, (char)18, (char)191, (char)149, (char)1, (char)161, (char)179, (char)22, (char)183, (char)244, (char)219, (char)62, (char)145, (char)156, (char)2, (char)114, (char)139, (char)236, (char)209, (char)230, (char)156, (char)18, (char)168, (char)79, (char)162, (char)26, (char)181, (char)0, (char)72, (char)100, (char)34, (char)212, (char)171, (char)186, (char)52, (char)0, (char)217, (char)249, (char)3}, 0) ;
        p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND) ;
        p126.timeout_SET((char)11401) ;
        p126.baudrate_SET(4197862061L) ;
        p126.count_SET((char)60) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1) ;
        LoopBackDemoChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.time_last_baseline_ms_GET() == 2593799005L);
            assert(pack.tow_GET() == 949382387L);
            assert(pack.rtk_health_GET() == (char)185);
            assert(pack.rtk_rate_GET() == (char)36);
            assert(pack.wn_GET() == (char)65158);
            assert(pack.rtk_receiver_id_GET() == (char)242);
            assert(pack.baseline_a_mm_GET() == 918332613);
            assert(pack.accuracy_GET() == 2302825704L);
            assert(pack.baseline_coords_type_GET() == (char)186);
            assert(pack.baseline_b_mm_GET() == 121289573);
            assert(pack.baseline_c_mm_GET() == 2004321807);
            assert(pack.iar_num_hypotheses_GET() == -1464505762);
            assert(pack.nsats_GET() == (char)190);
        });
        DemoDevice.GPS_RTK p127 = LoopBackDemoChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.accuracy_SET(2302825704L) ;
        p127.baseline_a_mm_SET(918332613) ;
        p127.baseline_coords_type_SET((char)186) ;
        p127.rtk_rate_SET((char)36) ;
        p127.baseline_c_mm_SET(2004321807) ;
        p127.tow_SET(949382387L) ;
        p127.wn_SET((char)65158) ;
        p127.nsats_SET((char)190) ;
        p127.iar_num_hypotheses_SET(-1464505762) ;
        p127.rtk_health_SET((char)185) ;
        p127.rtk_receiver_id_SET((char)242) ;
        p127.baseline_b_mm_SET(121289573) ;
        p127.time_last_baseline_ms_SET(2593799005L) ;
        LoopBackDemoChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.nsats_GET() == (char)181);
            assert(pack.baseline_coords_type_GET() == (char)141);
            assert(pack.tow_GET() == 3746661912L);
            assert(pack.wn_GET() == (char)46768);
            assert(pack.iar_num_hypotheses_GET() == 1443108508);
            assert(pack.time_last_baseline_ms_GET() == 3959474645L);
            assert(pack.rtk_health_GET() == (char)255);
            assert(pack.rtk_receiver_id_GET() == (char)218);
            assert(pack.baseline_a_mm_GET() == 1069442381);
            assert(pack.baseline_c_mm_GET() == 1848937124);
            assert(pack.baseline_b_mm_GET() == 1324607914);
            assert(pack.rtk_rate_GET() == (char)60);
            assert(pack.accuracy_GET() == 1138547412L);
        });
        DemoDevice.GPS2_RTK p128 = LoopBackDemoChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.baseline_coords_type_SET((char)141) ;
        p128.wn_SET((char)46768) ;
        p128.rtk_receiver_id_SET((char)218) ;
        p128.time_last_baseline_ms_SET(3959474645L) ;
        p128.iar_num_hypotheses_SET(1443108508) ;
        p128.rtk_rate_SET((char)60) ;
        p128.rtk_health_SET((char)255) ;
        p128.nsats_SET((char)181) ;
        p128.baseline_c_mm_SET(1848937124) ;
        p128.tow_SET(3746661912L) ;
        p128.baseline_b_mm_SET(1324607914) ;
        p128.accuracy_SET(1138547412L) ;
        p128.baseline_a_mm_SET(1069442381) ;
        LoopBackDemoChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == (short)29613);
            assert(pack.zgyro_GET() == (short)28754);
            assert(pack.xgyro_GET() == (short) -17345);
            assert(pack.yacc_GET() == (short)16819);
            assert(pack.time_boot_ms_GET() == 3935431109L);
            assert(pack.zmag_GET() == (short)2609);
            assert(pack.zacc_GET() == (short) -14315);
            assert(pack.ymag_GET() == (short)5102);
            assert(pack.xacc_GET() == (short)20953);
            assert(pack.xmag_GET() == (short) -28849);
        });
        DemoDevice.SCALED_IMU3 p129 = LoopBackDemoChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.time_boot_ms_SET(3935431109L) ;
        p129.yacc_SET((short)16819) ;
        p129.ygyro_SET((short)29613) ;
        p129.zacc_SET((short) -14315) ;
        p129.xacc_SET((short)20953) ;
        p129.xmag_SET((short) -28849) ;
        p129.xgyro_SET((short) -17345) ;
        p129.ymag_SET((short)5102) ;
        p129.zmag_SET((short)2609) ;
        p129.zgyro_SET((short)28754) ;
        LoopBackDemoChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.height_GET() == (char)4703);
            assert(pack.jpg_quality_GET() == (char)245);
            assert(pack.packets_GET() == (char)55036);
            assert(pack.width_GET() == (char)33519);
            assert(pack.size_GET() == 2011168446L);
            assert(pack.payload_GET() == (char)140);
            assert(pack.type_GET() == (char)106);
        });
        DemoDevice.DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.jpg_quality_SET((char)245) ;
        p130.height_SET((char)4703) ;
        p130.packets_SET((char)55036) ;
        p130.size_SET(2011168446L) ;
        p130.width_SET((char)33519) ;
        p130.type_SET((char)106) ;
        p130.payload_SET((char)140) ;
        LoopBackDemoChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)29, (char)176, (char)18, (char)122, (char)92, (char)223, (char)170, (char)102, (char)126, (char)42, (char)205, (char)32, (char)229, (char)212, (char)166, (char)205, (char)176, (char)192, (char)232, (char)164, (char)54, (char)147, (char)192, (char)90, (char)65, (char)161, (char)79, (char)143, (char)87, (char)22, (char)166, (char)4, (char)154, (char)112, (char)145, (char)34, (char)8, (char)171, (char)109, (char)93, (char)176, (char)143, (char)86, (char)67, (char)130, (char)245, (char)156, (char)151, (char)18, (char)127, (char)9, (char)62, (char)157, (char)73, (char)4, (char)227, (char)75, (char)172, (char)221, (char)166, (char)202, (char)45, (char)188, (char)153, (char)42, (char)60, (char)13, (char)3, (char)184, (char)37, (char)244, (char)155, (char)95, (char)210, (char)116, (char)164, (char)164, (char)7, (char)48, (char)132, (char)173, (char)122, (char)144, (char)13, (char)35, (char)31, (char)200, (char)208, (char)166, (char)241, (char)201, (char)140, (char)71, (char)186, (char)63, (char)9, (char)168, (char)53, (char)155, (char)12, (char)208, (char)96, (char)89, (char)49, (char)151, (char)171, (char)170, (char)123, (char)226, (char)114, (char)43, (char)16, (char)161, (char)127, (char)6, (char)173, (char)97, (char)78, (char)44, (char)118, (char)251, (char)100, (char)130, (char)182, (char)58, (char)152, (char)122, (char)139, (char)159, (char)184, (char)225, (char)173, (char)44, (char)199, (char)120, (char)144, (char)243, (char)170, (char)159, (char)134, (char)26, (char)124, (char)187, (char)28, (char)16, (char)141, (char)195, (char)87, (char)49, (char)81, (char)109, (char)176, (char)181, (char)180, (char)91, (char)221, (char)157, (char)17, (char)230, (char)69, (char)240, (char)184, (char)161, (char)135, (char)63, (char)221, (char)225, (char)151, (char)52, (char)21, (char)42, (char)13, (char)221, (char)122, (char)187, (char)241, (char)137, (char)0, (char)207, (char)63, (char)201, (char)223, (char)246, (char)236, (char)201, (char)249, (char)107, (char)100, (char)181, (char)44, (char)12, (char)86, (char)130, (char)86, (char)69, (char)68, (char)88, (char)193, (char)213, (char)46, (char)93, (char)17, (char)136, (char)246, (char)33, (char)170, (char)221, (char)50, (char)112, (char)145, (char)201, (char)131, (char)60, (char)63, (char)181, (char)10, (char)60, (char)34, (char)222, (char)93, (char)110, (char)242, (char)74, (char)134, (char)101, (char)254, (char)191, (char)206, (char)81, (char)206, (char)246, (char)12, (char)196, (char)0, (char)134, (char)59, (char)230, (char)120, (char)39, (char)102, (char)1, (char)154, (char)128, (char)209, (char)19, (char)150, (char)204, (char)226, (char)176, (char)254, (char)187, (char)253, (char)182}));
            assert(pack.seqnr_GET() == (char)55150);
        });
        DemoDevice.ENCAPSULATED_DATA p131 = LoopBackDemoChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.data__SET(new char[] {(char)29, (char)176, (char)18, (char)122, (char)92, (char)223, (char)170, (char)102, (char)126, (char)42, (char)205, (char)32, (char)229, (char)212, (char)166, (char)205, (char)176, (char)192, (char)232, (char)164, (char)54, (char)147, (char)192, (char)90, (char)65, (char)161, (char)79, (char)143, (char)87, (char)22, (char)166, (char)4, (char)154, (char)112, (char)145, (char)34, (char)8, (char)171, (char)109, (char)93, (char)176, (char)143, (char)86, (char)67, (char)130, (char)245, (char)156, (char)151, (char)18, (char)127, (char)9, (char)62, (char)157, (char)73, (char)4, (char)227, (char)75, (char)172, (char)221, (char)166, (char)202, (char)45, (char)188, (char)153, (char)42, (char)60, (char)13, (char)3, (char)184, (char)37, (char)244, (char)155, (char)95, (char)210, (char)116, (char)164, (char)164, (char)7, (char)48, (char)132, (char)173, (char)122, (char)144, (char)13, (char)35, (char)31, (char)200, (char)208, (char)166, (char)241, (char)201, (char)140, (char)71, (char)186, (char)63, (char)9, (char)168, (char)53, (char)155, (char)12, (char)208, (char)96, (char)89, (char)49, (char)151, (char)171, (char)170, (char)123, (char)226, (char)114, (char)43, (char)16, (char)161, (char)127, (char)6, (char)173, (char)97, (char)78, (char)44, (char)118, (char)251, (char)100, (char)130, (char)182, (char)58, (char)152, (char)122, (char)139, (char)159, (char)184, (char)225, (char)173, (char)44, (char)199, (char)120, (char)144, (char)243, (char)170, (char)159, (char)134, (char)26, (char)124, (char)187, (char)28, (char)16, (char)141, (char)195, (char)87, (char)49, (char)81, (char)109, (char)176, (char)181, (char)180, (char)91, (char)221, (char)157, (char)17, (char)230, (char)69, (char)240, (char)184, (char)161, (char)135, (char)63, (char)221, (char)225, (char)151, (char)52, (char)21, (char)42, (char)13, (char)221, (char)122, (char)187, (char)241, (char)137, (char)0, (char)207, (char)63, (char)201, (char)223, (char)246, (char)236, (char)201, (char)249, (char)107, (char)100, (char)181, (char)44, (char)12, (char)86, (char)130, (char)86, (char)69, (char)68, (char)88, (char)193, (char)213, (char)46, (char)93, (char)17, (char)136, (char)246, (char)33, (char)170, (char)221, (char)50, (char)112, (char)145, (char)201, (char)131, (char)60, (char)63, (char)181, (char)10, (char)60, (char)34, (char)222, (char)93, (char)110, (char)242, (char)74, (char)134, (char)101, (char)254, (char)191, (char)206, (char)81, (char)206, (char)246, (char)12, (char)196, (char)0, (char)134, (char)59, (char)230, (char)120, (char)39, (char)102, (char)1, (char)154, (char)128, (char)209, (char)19, (char)150, (char)204, (char)226, (char)176, (char)254, (char)187, (char)253, (char)182}, 0) ;
        p131.seqnr_SET((char)55150) ;
        LoopBackDemoChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3668197879L);
            assert(pack.max_distance_GET() == (char)30269);
            assert(pack.id_GET() == (char)128);
            assert(pack.current_distance_GET() == (char)57919);
            assert(pack.min_distance_GET() == (char)51754);
            assert(pack.covariance_GET() == (char)62);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_315);
        });
        DemoDevice.DISTANCE_SENSOR p132 = LoopBackDemoChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.time_boot_ms_SET(3668197879L) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR) ;
        p132.id_SET((char)128) ;
        p132.min_distance_SET((char)51754) ;
        p132.covariance_SET((char)62) ;
        p132.max_distance_SET((char)30269) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_315) ;
        p132.current_distance_SET((char)57919) ;
        LoopBackDemoChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 1134499182);
            assert(pack.lon_GET() == -429774864);
            assert(pack.mask_GET() == 5943934562489417024L);
            assert(pack.grid_spacing_GET() == (char)54466);
        });
        DemoDevice.TERRAIN_REQUEST p133 = LoopBackDemoChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.grid_spacing_SET((char)54466) ;
        p133.mask_SET(5943934562489417024L) ;
        p133.lon_SET(-429774864) ;
        p133.lat_SET(1134499182) ;
        LoopBackDemoChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.grid_spacing_GET() == (char)27375);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short) -16290, (short)23517, (short) -15543, (short) -6000, (short) -3725, (short)6034, (short) -4784, (short) -14289, (short)1858, (short) -17648, (short) -17521, (short)26750, (short)18961, (short)6153, (short)19757, (short)11901}));
            assert(pack.lon_GET() == 1489896194);
            assert(pack.gridbit_GET() == (char)231);
            assert(pack.lat_GET() == 611481931);
        });
        DemoDevice.TERRAIN_DATA p134 = LoopBackDemoChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.gridbit_SET((char)231) ;
        p134.grid_spacing_SET((char)27375) ;
        p134.lon_SET(1489896194) ;
        p134.data__SET(new short[] {(short) -16290, (short)23517, (short) -15543, (short) -6000, (short) -3725, (short)6034, (short) -4784, (short) -14289, (short)1858, (short) -17648, (short) -17521, (short)26750, (short)18961, (short)6153, (short)19757, (short)11901}, 0) ;
        p134.lat_SET(611481931) ;
        LoopBackDemoChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -1751206528);
            assert(pack.lat_GET() == -1691371982);
        });
        DemoDevice.TERRAIN_CHECK p135 = LoopBackDemoChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(-1691371982) ;
        p135.lon_SET(-1751206528) ;
        LoopBackDemoChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.loaded_GET() == (char)22242);
            assert(pack.pending_GET() == (char)13600);
            assert(pack.lat_GET() == 92135462);
            assert(pack.lon_GET() == 817140538);
            assert(pack.terrain_height_GET() == 3.0968765E38F);
            assert(pack.spacing_GET() == (char)38316);
            assert(pack.current_height_GET() == 2.3192917E38F);
        });
        DemoDevice.TERRAIN_REPORT p136 = LoopBackDemoChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.spacing_SET((char)38316) ;
        p136.current_height_SET(2.3192917E38F) ;
        p136.loaded_SET((char)22242) ;
        p136.lon_SET(817140538) ;
        p136.lat_SET(92135462) ;
        p136.terrain_height_SET(3.0968765E38F) ;
        p136.pending_SET((char)13600) ;
        LoopBackDemoChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.press_diff_GET() == -3.1787795E38F);
            assert(pack.time_boot_ms_GET() == 1171515657L);
            assert(pack.temperature_GET() == (short) -22492);
            assert(pack.press_abs_GET() == 7.825367E37F);
        });
        DemoDevice.SCALED_PRESSURE2 p137 = LoopBackDemoChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(1171515657L) ;
        p137.temperature_SET((short) -22492) ;
        p137.press_diff_SET(-3.1787795E38F) ;
        p137.press_abs_SET(7.825367E37F) ;
        LoopBackDemoChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 3.119405E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.8836095E38F, -1.8967566E38F, 1.5667133E38F, -2.4419457E38F}));
            assert(pack.time_usec_GET() == 7911564107423108950L);
            assert(pack.z_GET() == 8.764955E37F);
            assert(pack.x_GET() == -5.423228E36F);
        });
        DemoDevice.ATT_POS_MOCAP p138 = LoopBackDemoChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.x_SET(-5.423228E36F) ;
        p138.time_usec_SET(7911564107423108950L) ;
        p138.y_SET(3.119405E37F) ;
        p138.z_SET(8.764955E37F) ;
        p138.q_SET(new float[] {-1.8836095E38F, -1.8967566E38F, 1.5667133E38F, -2.4419457E38F}, 0) ;
        LoopBackDemoChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {7.904904E37F, 1.3278658E38F, 4.1394086E37F, -1.0055155E38F, -1.3477959E38F, 2.5481637E38F, 3.6152918E37F, -4.3378877E37F}));
            assert(pack.group_mlx_GET() == (char)208);
            assert(pack.target_system_GET() == (char)44);
            assert(pack.target_component_GET() == (char)254);
            assert(pack.time_usec_GET() == 1496434208768638780L);
        });
        DemoDevice.SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.time_usec_SET(1496434208768638780L) ;
        p139.group_mlx_SET((char)208) ;
        p139.controls_SET(new float[] {7.904904E37F, 1.3278658E38F, 4.1394086E37F, -1.0055155E38F, -1.3477959E38F, 2.5481637E38F, 3.6152918E37F, -4.3378877E37F}, 0) ;
        p139.target_system_SET((char)44) ;
        p139.target_component_SET((char)254) ;
        LoopBackDemoChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 7913080369310033784L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {2.9594405E38F, -3.0983032E38F, -2.7896408E38F, 1.4446387E38F, -2.7642377E38F, 4.313849E37F, -2.5075685E38F, 2.9317156E38F}));
            assert(pack.group_mlx_GET() == (char)76);
        });
        DemoDevice.ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.time_usec_SET(7913080369310033784L) ;
        p140.group_mlx_SET((char)76) ;
        p140.controls_SET(new float[] {2.9594405E38F, -3.0983032E38F, -2.7896408E38F, 1.4446387E38F, -2.7642377E38F, 4.313849E37F, -2.5075685E38F, 2.9317156E38F}, 0) ;
        LoopBackDemoChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.bottom_clearance_GET() == -6.257039E37F);
            assert(pack.altitude_relative_GET() == 8.787151E37F);
            assert(pack.time_usec_GET() == 6532451791353142033L);
            assert(pack.altitude_amsl_GET() == -7.289895E37F);
            assert(pack.altitude_local_GET() == -1.2022773E38F);
            assert(pack.altitude_terrain_GET() == -1.0200257E38F);
            assert(pack.altitude_monotonic_GET() == 3.3179546E38F);
        });
        DemoDevice.ALTITUDE p141 = LoopBackDemoChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.bottom_clearance_SET(-6.257039E37F) ;
        p141.altitude_relative_SET(8.787151E37F) ;
        p141.altitude_terrain_SET(-1.0200257E38F) ;
        p141.altitude_local_SET(-1.2022773E38F) ;
        p141.altitude_amsl_SET(-7.289895E37F) ;
        p141.time_usec_SET(6532451791353142033L) ;
        p141.altitude_monotonic_SET(3.3179546E38F) ;
        LoopBackDemoChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)113, (char)177, (char)45, (char)155, (char)80, (char)159, (char)235, (char)203, (char)93, (char)224, (char)246, (char)20, (char)7, (char)84, (char)84, (char)52, (char)186, (char)244, (char)183, (char)23, (char)31, (char)227, (char)115, (char)27, (char)55, (char)195, (char)184, (char)150, (char)105, (char)70, (char)223, (char)5, (char)98, (char)107, (char)151, (char)165, (char)18, (char)116, (char)163, (char)145, (char)230, (char)197, (char)107, (char)135, (char)86, (char)70, (char)107, (char)57, (char)179, (char)184, (char)250, (char)222, (char)133, (char)167, (char)72, (char)190, (char)179, (char)180, (char)156, (char)82, (char)39, (char)194, (char)137, (char)34, (char)16, (char)146, (char)136, (char)25, (char)171, (char)236, (char)204, (char)97, (char)30, (char)50, (char)149, (char)198, (char)167, (char)62, (char)178, (char)197, (char)184, (char)81, (char)0, (char)69, (char)156, (char)55, (char)44, (char)113, (char)105, (char)202, (char)158, (char)195, (char)243, (char)205, (char)76, (char)242, (char)151, (char)195, (char)92, (char)69, (char)68, (char)17, (char)142, (char)193, (char)33, (char)251, (char)22, (char)249, (char)22, (char)196, (char)63, (char)50, (char)60, (char)67, (char)82, (char)89, (char)208, (char)67, (char)85, (char)215}));
            assert(pack.transfer_type_GET() == (char)252);
            assert(pack.uri_type_GET() == (char)199);
            assert(pack.request_id_GET() == (char)58);
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)28, (char)208, (char)63, (char)76, (char)54, (char)208, (char)195, (char)56, (char)77, (char)194, (char)197, (char)207, (char)159, (char)67, (char)164, (char)239, (char)90, (char)5, (char)122, (char)143, (char)65, (char)199, (char)18, (char)171, (char)225, (char)33, (char)175, (char)127, (char)73, (char)34, (char)48, (char)101, (char)102, (char)190, (char)41, (char)37, (char)194, (char)128, (char)99, (char)181, (char)47, (char)203, (char)173, (char)98, (char)145, (char)77, (char)234, (char)236, (char)207, (char)231, (char)170, (char)111, (char)255, (char)37, (char)79, (char)206, (char)4, (char)23, (char)44, (char)54, (char)52, (char)200, (char)239, (char)78, (char)187, (char)241, (char)72, (char)112, (char)47, (char)231, (char)122, (char)14, (char)198, (char)97, (char)23, (char)86, (char)236, (char)110, (char)220, (char)225, (char)105, (char)59, (char)117, (char)115, (char)68, (char)82, (char)212, (char)121, (char)78, (char)102, (char)162, (char)117, (char)208, (char)140, (char)143, (char)55, (char)138, (char)181, (char)145, (char)222, (char)211, (char)64, (char)143, (char)247, (char)48, (char)111, (char)112, (char)250, (char)72, (char)108, (char)189, (char)162, (char)228, (char)7, (char)73, (char)82, (char)173, (char)230, (char)99, (char)199}));
        });
        DemoDevice.RESOURCE_REQUEST p142 = LoopBackDemoChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.uri_SET(new char[] {(char)113, (char)177, (char)45, (char)155, (char)80, (char)159, (char)235, (char)203, (char)93, (char)224, (char)246, (char)20, (char)7, (char)84, (char)84, (char)52, (char)186, (char)244, (char)183, (char)23, (char)31, (char)227, (char)115, (char)27, (char)55, (char)195, (char)184, (char)150, (char)105, (char)70, (char)223, (char)5, (char)98, (char)107, (char)151, (char)165, (char)18, (char)116, (char)163, (char)145, (char)230, (char)197, (char)107, (char)135, (char)86, (char)70, (char)107, (char)57, (char)179, (char)184, (char)250, (char)222, (char)133, (char)167, (char)72, (char)190, (char)179, (char)180, (char)156, (char)82, (char)39, (char)194, (char)137, (char)34, (char)16, (char)146, (char)136, (char)25, (char)171, (char)236, (char)204, (char)97, (char)30, (char)50, (char)149, (char)198, (char)167, (char)62, (char)178, (char)197, (char)184, (char)81, (char)0, (char)69, (char)156, (char)55, (char)44, (char)113, (char)105, (char)202, (char)158, (char)195, (char)243, (char)205, (char)76, (char)242, (char)151, (char)195, (char)92, (char)69, (char)68, (char)17, (char)142, (char)193, (char)33, (char)251, (char)22, (char)249, (char)22, (char)196, (char)63, (char)50, (char)60, (char)67, (char)82, (char)89, (char)208, (char)67, (char)85, (char)215}, 0) ;
        p142.transfer_type_SET((char)252) ;
        p142.storage_SET(new char[] {(char)28, (char)208, (char)63, (char)76, (char)54, (char)208, (char)195, (char)56, (char)77, (char)194, (char)197, (char)207, (char)159, (char)67, (char)164, (char)239, (char)90, (char)5, (char)122, (char)143, (char)65, (char)199, (char)18, (char)171, (char)225, (char)33, (char)175, (char)127, (char)73, (char)34, (char)48, (char)101, (char)102, (char)190, (char)41, (char)37, (char)194, (char)128, (char)99, (char)181, (char)47, (char)203, (char)173, (char)98, (char)145, (char)77, (char)234, (char)236, (char)207, (char)231, (char)170, (char)111, (char)255, (char)37, (char)79, (char)206, (char)4, (char)23, (char)44, (char)54, (char)52, (char)200, (char)239, (char)78, (char)187, (char)241, (char)72, (char)112, (char)47, (char)231, (char)122, (char)14, (char)198, (char)97, (char)23, (char)86, (char)236, (char)110, (char)220, (char)225, (char)105, (char)59, (char)117, (char)115, (char)68, (char)82, (char)212, (char)121, (char)78, (char)102, (char)162, (char)117, (char)208, (char)140, (char)143, (char)55, (char)138, (char)181, (char)145, (char)222, (char)211, (char)64, (char)143, (char)247, (char)48, (char)111, (char)112, (char)250, (char)72, (char)108, (char)189, (char)162, (char)228, (char)7, (char)73, (char)82, (char)173, (char)230, (char)99, (char)199}, 0) ;
        p142.request_id_SET((char)58) ;
        p142.uri_type_SET((char)199) ;
        LoopBackDemoChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == 2.4862028E37F);
            assert(pack.time_boot_ms_GET() == 905628839L);
            assert(pack.press_diff_GET() == -1.1937663E38F);
            assert(pack.temperature_GET() == (short) -19287);
        });
        DemoDevice.SCALED_PRESSURE3 p143 = LoopBackDemoChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.temperature_SET((short) -19287) ;
        p143.press_diff_SET(-1.1937663E38F) ;
        p143.press_abs_SET(2.4862028E37F) ;
        p143.time_boot_ms_SET(905628839L) ;
        LoopBackDemoChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {1.8337665E38F, 1.6811655E38F, 1.7327046E38F}));
            assert(pack.timestamp_GET() == 6258789507003485419L);
            assert(pack.est_capabilities_GET() == (char)204);
            assert(pack.custom_state_GET() == 6102306848008442415L);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {2.1565515E38F, 2.1289904E38F, -1.2972412E37F}));
            assert(Arrays.equals(pack.vel_GET(),  new float[] {2.7989912E38F, 6.9384886E37F, -3.3941578E38F}));
            assert(pack.lon_GET() == -49102084);
            assert(Arrays.equals(pack.rates_GET(),  new float[] {-8.053823E37F, 2.3624799E38F, 1.553334E38F}));
            assert(pack.alt_GET() == 1.6893512E38F);
            assert(pack.lat_GET() == 1970730809);
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {-9.263273E37F, 1.7307232E38F, -1.7793239E38F, 1.8533137E38F}));
        });
        DemoDevice.FOLLOW_TARGET p144 = LoopBackDemoChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.alt_SET(1.6893512E38F) ;
        p144.position_cov_SET(new float[] {1.8337665E38F, 1.6811655E38F, 1.7327046E38F}, 0) ;
        p144.lat_SET(1970730809) ;
        p144.est_capabilities_SET((char)204) ;
        p144.lon_SET(-49102084) ;
        p144.rates_SET(new float[] {-8.053823E37F, 2.3624799E38F, 1.553334E38F}, 0) ;
        p144.timestamp_SET(6258789507003485419L) ;
        p144.attitude_q_SET(new float[] {-9.263273E37F, 1.7307232E38F, -1.7793239E38F, 1.8533137E38F}, 0) ;
        p144.vel_SET(new float[] {2.7989912E38F, 6.9384886E37F, -3.3941578E38F}, 0) ;
        p144.acc_SET(new float[] {2.1565515E38F, 2.1289904E38F, -1.2972412E37F}, 0) ;
        p144.custom_state_SET(6102306848008442415L) ;
        LoopBackDemoChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.z_vel_GET() == 1.6164291E38F);
            assert(pack.z_acc_GET() == 1.7722299E38F);
            assert(pack.airspeed_GET() == -3.580716E37F);
            assert(pack.y_vel_GET() == -2.3508325E38F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {-1.4515185E38F, -1.8260164E38F, -2.1860844E38F}));
            assert(pack.x_vel_GET() == -1.7094188E37F);
            assert(pack.z_pos_GET() == 2.2338263E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-4.341354E36F, -2.5536034E38F, 1.8628671E38F, 7.019725E37F}));
            assert(pack.y_pos_GET() == 3.1508196E38F);
            assert(pack.pitch_rate_GET() == -1.429415E38F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {1.1888927E38F, -8.949288E37F, 3.24386E38F}));
            assert(pack.y_acc_GET() == -3.076576E38F);
            assert(pack.yaw_rate_GET() == 9.264611E37F);
            assert(pack.x_acc_GET() == 9.0592834E36F);
            assert(pack.time_usec_GET() == 8458109911798777910L);
            assert(pack.roll_rate_GET() == 3.337518E38F);
            assert(pack.x_pos_GET() == -2.1590113E37F);
        });
        DemoDevice.CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.yaw_rate_SET(9.264611E37F) ;
        p146.pitch_rate_SET(-1.429415E38F) ;
        p146.airspeed_SET(-3.580716E37F) ;
        p146.roll_rate_SET(3.337518E38F) ;
        p146.vel_variance_SET(new float[] {-1.4515185E38F, -1.8260164E38F, -2.1860844E38F}, 0) ;
        p146.x_acc_SET(9.0592834E36F) ;
        p146.x_pos_SET(-2.1590113E37F) ;
        p146.pos_variance_SET(new float[] {1.1888927E38F, -8.949288E37F, 3.24386E38F}, 0) ;
        p146.time_usec_SET(8458109911798777910L) ;
        p146.x_vel_SET(-1.7094188E37F) ;
        p146.y_vel_SET(-2.3508325E38F) ;
        p146.q_SET(new float[] {-4.341354E36F, -2.5536034E38F, 1.8628671E38F, 7.019725E37F}, 0) ;
        p146.y_pos_SET(3.1508196E38F) ;
        p146.z_vel_SET(1.6164291E38F) ;
        p146.y_acc_SET(-3.076576E38F) ;
        p146.z_pos_SET(2.2338263E38F) ;
        p146.z_acc_SET(1.7722299E38F) ;
        LoopBackDemoChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN);
            assert(pack.battery_remaining_GET() == (byte)116);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS);
            assert(pack.current_consumed_GET() == -314415594);
            assert(pack.id_GET() == (char)226);
            assert(pack.temperature_GET() == (short) -27377);
            assert(pack.current_battery_GET() == (short) -3066);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)6523, (char)44277, (char)6237, (char)755, (char)40546, (char)18855, (char)14568, (char)3666, (char)34686, (char)36567}));
            assert(pack.energy_consumed_GET() == 964490378);
        });
        DemoDevice.BATTERY_STATUS p147 = LoopBackDemoChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS) ;
        p147.battery_remaining_SET((byte)116) ;
        p147.current_battery_SET((short) -3066) ;
        p147.id_SET((char)226) ;
        p147.current_consumed_SET(-314415594) ;
        p147.voltages_SET(new char[] {(char)6523, (char)44277, (char)6237, (char)755, (char)40546, (char)18855, (char)14568, (char)3666, (char)34686, (char)36567}, 0) ;
        p147.energy_consumed_SET(964490378) ;
        p147.temperature_SET((short) -27377) ;
        LoopBackDemoChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(pack.flight_sw_version_GET() == 437223496L);
            assert(pack.product_id_GET() == (char)35406);
            assert(pack.uid_GET() == 5292987308733929442L);
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)165, (char)214, (char)221, (char)29, (char)13, (char)129, (char)176, (char)85}));
            assert(pack.vendor_id_GET() == (char)59526);
            assert(pack.board_version_GET() == 1781758249L);
            assert(pack.middleware_sw_version_GET() == 2916095613L);
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)88, (char)21, (char)243, (char)78, (char)9, (char)116, (char)215, (char)245}));
            assert(pack.os_sw_version_GET() == 2115448038L);
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)166, (char)187, (char)86, (char)118, (char)127, (char)151, (char)200, (char)106}));
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)4, (char)176, (char)85, (char)186, (char)190, (char)210, (char)20, (char)126, (char)248, (char)141, (char)126, (char)187, (char)161, (char)66, (char)100, (char)117, (char)115, (char)105}));
            assert(pack.capabilities_GET() == MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT);
        });
        DemoDevice.AUTOPILOT_VERSION p148 = LoopBackDemoChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.uid_SET(5292987308733929442L) ;
        p148.uid2_SET(new char[] {(char)4, (char)176, (char)85, (char)186, (char)190, (char)210, (char)20, (char)126, (char)248, (char)141, (char)126, (char)187, (char)161, (char)66, (char)100, (char)117, (char)115, (char)105}, 0, PH) ;
        p148.vendor_id_SET((char)59526) ;
        p148.middleware_custom_version_SET(new char[] {(char)166, (char)187, (char)86, (char)118, (char)127, (char)151, (char)200, (char)106}, 0) ;
        p148.board_version_SET(1781758249L) ;
        p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT) ;
        p148.flight_sw_version_SET(437223496L) ;
        p148.os_custom_version_SET(new char[] {(char)88, (char)21, (char)243, (char)78, (char)9, (char)116, (char)215, (char)245}, 0) ;
        p148.os_sw_version_SET(2115448038L) ;
        p148.middleware_sw_version_SET(2916095613L) ;
        p148.flight_custom_version_SET(new char[] {(char)165, (char)214, (char)221, (char)29, (char)13, (char)129, (char)176, (char)85}, 0) ;
        p148.product_id_SET((char)35406) ;
        LoopBackDemoChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.x_TRY(ph) == 1.0891417E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON);
            assert(pack.distance_GET() == -5.604832E37F);
            assert(pack.position_valid_TRY(ph) == (char)232);
            assert(pack.angle_x_GET() == 3.1453393E38F);
            assert(pack.size_y_GET() == 3.6853955E37F);
            assert(pack.z_TRY(ph) == 1.5664067E36F);
            assert(pack.angle_y_GET() == -2.6921767E38F);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {2.2455396E38F, -1.8808978E38F, -3.255176E38F, 5.937627E37F}));
            assert(pack.y_TRY(ph) == -1.1174765E38F);
            assert(pack.size_x_GET() == 1.9479492E38F);
            assert(pack.target_num_GET() == (char)4);
            assert(pack.time_usec_GET() == 5095732553365444133L);
        });
        DemoDevice.LANDING_TARGET p149 = LoopBackDemoChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON) ;
        p149.q_SET(new float[] {2.2455396E38F, -1.8808978E38F, -3.255176E38F, 5.937627E37F}, 0, PH) ;
        p149.angle_y_SET(-2.6921767E38F) ;
        p149.size_x_SET(1.9479492E38F) ;
        p149.time_usec_SET(5095732553365444133L) ;
        p149.y_SET(-1.1174765E38F, PH) ;
        p149.x_SET(1.0891417E38F, PH) ;
        p149.position_valid_SET((char)232, PH) ;
        p149.distance_SET(-5.604832E37F) ;
        p149.z_SET(1.5664067E36F, PH) ;
        p149.angle_x_SET(3.1453393E38F) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p149.target_num_SET((char)4) ;
        p149.size_y_SET(3.6853955E37F) ;
        LoopBackDemoChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ARRAY_TEST_0.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.ar_u32_GET(),  new long[] {1038682001L, 1015097589L, 3066037350L, 2619846918L}));
            assert(Arrays.equals(pack.ar_i8_GET(),  new byte[] {(byte) - 29, (byte)3, (byte)113, (byte)13}));
            assert(Arrays.equals(pack.ar_u8_GET(),  new char[] {(char)7, (char)29, (char)190, (char)25}));
            assert(pack.v1_GET() == (char)91);
            assert(Arrays.equals(pack.ar_u16_GET(),  new char[] {(char)53292, (char)26433, (char)43621, (char)28571}));
        });
        DemoDevice.ARRAY_TEST_0 p150 = LoopBackDemoChannel.new_ARRAY_TEST_0();
        PH.setPack(p150);
        p150.v1_SET((char)91) ;
        p150.ar_u8_SET(new char[] {(char)7, (char)29, (char)190, (char)25}, 0) ;
        p150.ar_i8_SET(new byte[] {(byte) - 29, (byte)3, (byte)113, (byte)13}, 0) ;
        p150.ar_u16_SET(new char[] {(char)53292, (char)26433, (char)43621, (char)28571}, 0) ;
        p150.ar_u32_SET(new long[] {1038682001L, 1015097589L, 3066037350L, 2619846918L}, 0) ;
        LoopBackDemoChannel.instance.send(p150);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ARRAY_TEST_1.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.ar_u32_GET(),  new long[] {2024871692L, 3837243709L, 3537535915L, 2204555263L}));
        });
        DemoDevice.ARRAY_TEST_1 p151 = LoopBackDemoChannel.new_ARRAY_TEST_1();
        PH.setPack(p151);
        p151.ar_u32_SET(new long[] {2024871692L, 3837243709L, 3537535915L, 2204555263L}, 0) ;
        LoopBackDemoChannel.instance.send(p151);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ARRAY_TEST_3.add((src, ph, pack) ->
        {
            assert(pack.v_GET() == (char)106);
            assert(Arrays.equals(pack.ar_u32_GET(),  new long[] {1335585267L, 1929901772L, 4067404253L, 511599567L}));
        });
        DemoDevice.ARRAY_TEST_3 p153 = LoopBackDemoChannel.new_ARRAY_TEST_3();
        PH.setPack(p153);
        p153.ar_u32_SET(new long[] {1335585267L, 1929901772L, 4067404253L, 511599567L}, 0) ;
        p153.v_SET((char)106) ;
        LoopBackDemoChannel.instance.send(p153);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ARRAY_TEST_4.add((src, ph, pack) ->
        {
            assert(pack.v_GET() == (char)81);
            assert(Arrays.equals(pack.ar_u32_GET(),  new long[] {1534019490L, 676119884L, 2314107247L, 1640231623L}));
        });
        DemoDevice.ARRAY_TEST_4 p154 = LoopBackDemoChannel.new_ARRAY_TEST_4();
        PH.setPack(p154);
        p154.v_SET((char)81) ;
        p154.ar_u32_SET(new long[] {1534019490L, 676119884L, 2314107247L, 1640231623L}, 0) ;
        LoopBackDemoChannel.instance.send(p154);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ARRAY_TEST_5.add((src, ph, pack) ->
        {
            assert(pack.c1_LEN(ph) == 3);
            assert(pack.c1_TRY(ph).equals("ala"));
            assert(pack.c2_LEN(ph) == 5);
            assert(pack.c2_TRY(ph).equals("svbua"));
        });
        DemoDevice.ARRAY_TEST_5 p155 = LoopBackDemoChannel.new_ARRAY_TEST_5();
        PH.setPack(p155);
        p155.c1_SET("ala", PH) ;
        p155.c2_SET("svbua", PH) ;
        LoopBackDemoChannel.instance.send(p155);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ARRAY_TEST_6.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.ar_f_GET(),  new float[] {2.2740532E38F, -1.7461773E38F}));
            assert(Arrays.equals(pack.ar_u8_GET(),  new char[] {(char)86, (char)7}));
            assert(Arrays.equals(pack.ar_d_GET(),  new double[] {4.925735072193544E307, -1.0515606802822153E308}));
            assert(Arrays.equals(pack.ar_u16_GET(),  new char[] {(char)50713, (char)61477}));
            assert(Arrays.equals(pack.ar_i8_GET(),  new byte[] {(byte) - 62, (byte) - 52}));
            assert(Arrays.equals(pack.ar_i32_GET(),  new int[] {-1535533681, -1640836201}));
            assert(pack.v3_GET() == 2565419155L);
            assert(pack.ar_c_LEN(ph) == 27);
            assert(pack.ar_c_TRY(ph).equals("oqsqzxkpdrmdksonihxecvivesp"));
            assert(pack.v1_GET() == (char)145);
            assert(Arrays.equals(pack.ar_u32_GET(),  new long[] {3411208470L, 4100087630L}));
            assert(pack.v2_GET() == (char)19706);
            assert(Arrays.equals(pack.ar_i16_GET(),  new short[] {(short) -9798, (short)19246}));
        });
        DemoDevice.ARRAY_TEST_6 p156 = LoopBackDemoChannel.new_ARRAY_TEST_6();
        PH.setPack(p156);
        p156.ar_u32_SET(new long[] {3411208470L, 4100087630L}, 0) ;
        p156.ar_u16_SET(new char[] {(char)50713, (char)61477}, 0) ;
        p156.ar_f_SET(new float[] {2.2740532E38F, -1.7461773E38F}, 0) ;
        p156.v2_SET((char)19706) ;
        p156.ar_d_SET(new double[] {4.925735072193544E307, -1.0515606802822153E308}, 0) ;
        p156.ar_i8_SET(new byte[] {(byte) - 62, (byte) - 52}, 0) ;
        p156.ar_c_SET("oqsqzxkpdrmdksonihxecvivesp", PH) ;
        p156.ar_i32_SET(new int[] {-1535533681, -1640836201}, 0) ;
        p156.ar_i16_SET(new short[] {(short) -9798, (short)19246}, 0) ;
        p156.v1_SET((char)145) ;
        p156.v3_SET(2565419155L) ;
        p156.ar_u8_SET(new char[] {(char)86, (char)7}, 0) ;
        LoopBackDemoChannel.instance.send(p156);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ARRAY_TEST_7.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.ar_i8_GET(),  new byte[] {(byte) - 59, (byte)36}));
            assert(pack.ar_c_LEN(ph) == 13);
            assert(pack.ar_c_TRY(ph).equals("hrvXwnMnawcsr"));
            assert(Arrays.equals(pack.ar_d_GET(),  new double[] {-8.402315101532831E306, -1.3548158735393192E308}));
            assert(Arrays.equals(pack.ar_u16_GET(),  new char[] {(char)7682, (char)13109}));
            assert(Arrays.equals(pack.ar_i32_GET(),  new int[] {-1166327419, 701406819}));
            assert(Arrays.equals(pack.ar_u8_GET(),  new char[] {(char)136, (char)70}));
            assert(Arrays.equals(pack.ar_u32_GET(),  new long[] {3662824404L, 2117447263L}));
            assert(Arrays.equals(pack.ar_i16_GET(),  new short[] {(short)27293, (short) -28093}));
            assert(Arrays.equals(pack.ar_f_GET(),  new float[] {1.1141114E38F, 2.0397015E38F}));
        });
        DemoDevice.ARRAY_TEST_7 p157 = LoopBackDemoChannel.new_ARRAY_TEST_7();
        PH.setPack(p157);
        p157.ar_f_SET(new float[] {1.1141114E38F, 2.0397015E38F}, 0) ;
        p157.ar_i16_SET(new short[] {(short)27293, (short) -28093}, 0) ;
        p157.ar_u32_SET(new long[] {3662824404L, 2117447263L}, 0) ;
        p157.ar_i32_SET(new int[] {-1166327419, 701406819}, 0) ;
        p157.ar_d_SET(new double[] {-8.402315101532831E306, -1.3548158735393192E308}, 0) ;
        p157.ar_u16_SET(new char[] {(char)7682, (char)13109}, 0) ;
        p157.ar_u8_SET(new char[] {(char)136, (char)70}, 0) ;
        p157.ar_i8_SET(new byte[] {(byte) - 59, (byte)36}, 0) ;
        p157.ar_c_SET("hrvXwnMnawcsr", PH) ;
        LoopBackDemoChannel.instance.send(p157);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ARRAY_TEST_8.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.ar_d_GET(),  new double[] {1.1570699349265026E308, -1.7663909211031505E307}));
            assert(pack.v3_GET() == 2615053185L);
            assert(Arrays.equals(pack.ar_u16_GET(),  new char[] {(char)21980, (char)490}));
        });
        DemoDevice.ARRAY_TEST_8 p158 = LoopBackDemoChannel.new_ARRAY_TEST_8();
        PH.setPack(p158);
        p158.ar_u16_SET(new char[] {(char)21980, (char)490}, 0) ;
        p158.ar_d_SET(new double[] {1.1570699349265026E308, -1.7663909211031505E307}, 0) ;
        p158.v3_SET(2615053185L) ;
        LoopBackDemoChannel.instance.send(p158);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.vel_ratio_GET() == 2.8343248E38F);
            assert(pack.flags_GET() == ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE);
            assert(pack.mag_ratio_GET() == -2.8752425E38F);
            assert(pack.pos_vert_accuracy_GET() == 1.665989E38F);
            assert(pack.hagl_ratio_GET() == 1.6819221E38F);
            assert(pack.pos_horiz_accuracy_GET() == -8.0061835E37F);
            assert(pack.pos_horiz_ratio_GET() == 3.0082416E38F);
            assert(pack.pos_vert_ratio_GET() == -1.369575E38F);
            assert(pack.tas_ratio_GET() == 2.6392508E38F);
            assert(pack.time_usec_GET() == 6183364000349124662L);
        });
        DemoDevice.ESTIMATOR_STATUS p230 = LoopBackDemoChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.hagl_ratio_SET(1.6819221E38F) ;
        p230.time_usec_SET(6183364000349124662L) ;
        p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE) ;
        p230.vel_ratio_SET(2.8343248E38F) ;
        p230.pos_vert_ratio_SET(-1.369575E38F) ;
        p230.pos_vert_accuracy_SET(1.665989E38F) ;
        p230.pos_horiz_ratio_SET(3.0082416E38F) ;
        p230.tas_ratio_SET(2.6392508E38F) ;
        p230.pos_horiz_accuracy_SET(-8.0061835E37F) ;
        p230.mag_ratio_SET(-2.8752425E38F) ;
        LoopBackDemoChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 5812048635738386441L);
            assert(pack.var_vert_GET() == 3.060119E38F);
            assert(pack.wind_y_GET() == -2.691185E38F);
            assert(pack.horiz_accuracy_GET() == 7.288785E37F);
            assert(pack.wind_alt_GET() == -1.3476311E38F);
            assert(pack.wind_z_GET() == -1.2771382E38F);
            assert(pack.var_horiz_GET() == -9.720963E37F);
            assert(pack.wind_x_GET() == -2.746118E38F);
            assert(pack.vert_accuracy_GET() == 2.7202908E38F);
        });
        DemoDevice.WIND_COV p231 = LoopBackDemoChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.time_usec_SET(5812048635738386441L) ;
        p231.var_horiz_SET(-9.720963E37F) ;
        p231.horiz_accuracy_SET(7.288785E37F) ;
        p231.vert_accuracy_SET(2.7202908E38F) ;
        p231.wind_z_SET(-1.2771382E38F) ;
        p231.wind_y_SET(-2.691185E38F) ;
        p231.var_vert_SET(3.060119E38F) ;
        p231.wind_x_SET(-2.746118E38F) ;
        p231.wind_alt_SET(-1.3476311E38F) ;
        LoopBackDemoChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.satellites_visible_GET() == (char)5);
            assert(pack.time_usec_GET() == 4327086030121200678L);
            assert(pack.fix_type_GET() == (char)110);
            assert(pack.ve_GET() == -3.1346428E38F);
            assert(pack.vdop_GET() == 3.3655775E38F);
            assert(pack.ignore_flags_GET() == GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP);
            assert(pack.vd_GET() == -2.2042434E38F);
            assert(pack.horiz_accuracy_GET() == -1.8564423E38F);
            assert(pack.speed_accuracy_GET() == -1.7919057E38F);
            assert(pack.vert_accuracy_GET() == -5.0599734E37F);
            assert(pack.gps_id_GET() == (char)212);
            assert(pack.lon_GET() == 1224273676);
            assert(pack.lat_GET() == -543034344);
            assert(pack.time_week_GET() == (char)29562);
            assert(pack.hdop_GET() == -6.223363E37F);
            assert(pack.vn_GET() == -1.2249331E38F);
            assert(pack.time_week_ms_GET() == 2301598603L);
            assert(pack.alt_GET() == -3.121134E38F);
        });
        DemoDevice.GPS_INPUT p232 = LoopBackDemoChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.vd_SET(-2.2042434E38F) ;
        p232.ve_SET(-3.1346428E38F) ;
        p232.lon_SET(1224273676) ;
        p232.horiz_accuracy_SET(-1.8564423E38F) ;
        p232.vdop_SET(3.3655775E38F) ;
        p232.lat_SET(-543034344) ;
        p232.vn_SET(-1.2249331E38F) ;
        p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP) ;
        p232.fix_type_SET((char)110) ;
        p232.time_week_ms_SET(2301598603L) ;
        p232.alt_SET(-3.121134E38F) ;
        p232.gps_id_SET((char)212) ;
        p232.speed_accuracy_SET(-1.7919057E38F) ;
        p232.satellites_visible_SET((char)5) ;
        p232.hdop_SET(-6.223363E37F) ;
        p232.time_usec_SET(4327086030121200678L) ;
        p232.vert_accuracy_SET(-5.0599734E37F) ;
        p232.time_week_SET((char)29562) ;
        LoopBackDemoChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(pack.len_GET() == (char)151);
            assert(pack.flags_GET() == (char)123);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)121, (char)252, (char)140, (char)29, (char)8, (char)245, (char)195, (char)95, (char)64, (char)155, (char)193, (char)113, (char)176, (char)9, (char)137, (char)60, (char)58, (char)12, (char)122, (char)19, (char)147, (char)103, (char)221, (char)16, (char)123, (char)63, (char)188, (char)88, (char)8, (char)71, (char)110, (char)35, (char)9, (char)233, (char)243, (char)194, (char)11, (char)150, (char)72, (char)226, (char)195, (char)132, (char)234, (char)151, (char)197, (char)87, (char)205, (char)21, (char)130, (char)35, (char)176, (char)111, (char)48, (char)253, (char)44, (char)125, (char)237, (char)174, (char)120, (char)131, (char)250, (char)110, (char)81, (char)111, (char)50, (char)186, (char)180, (char)88, (char)149, (char)180, (char)225, (char)79, (char)76, (char)115, (char)226, (char)41, (char)22, (char)172, (char)224, (char)12, (char)4, (char)111, (char)43, (char)185, (char)139, (char)44, (char)186, (char)38, (char)57, (char)19, (char)125, (char)16, (char)227, (char)71, (char)226, (char)230, (char)52, (char)52, (char)173, (char)160, (char)202, (char)252, (char)111, (char)205, (char)78, (char)68, (char)79, (char)85, (char)94, (char)178, (char)101, (char)67, (char)118, (char)225, (char)252, (char)159, (char)196, (char)130, (char)107, (char)150, (char)244, (char)228, (char)64, (char)121, (char)185, (char)163, (char)126, (char)187, (char)132, (char)193, (char)129, (char)147, (char)84, (char)153, (char)209, (char)5, (char)209, (char)234, (char)213, (char)41, (char)191, (char)225, (char)172, (char)35, (char)65, (char)97, (char)192, (char)225, (char)110, (char)209, (char)36, (char)223, (char)106, (char)189, (char)83, (char)227, (char)7, (char)183, (char)119, (char)169, (char)25, (char)234, (char)60, (char)6, (char)239, (char)113, (char)17, (char)85, (char)230, (char)162, (char)181, (char)168, (char)133, (char)102, (char)182, (char)16, (char)157, (char)58, (char)193, (char)153}));
        });
        DemoDevice.GPS_RTCM_DATA p233 = LoopBackDemoChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.len_SET((char)151) ;
        p233.data__SET(new char[] {(char)121, (char)252, (char)140, (char)29, (char)8, (char)245, (char)195, (char)95, (char)64, (char)155, (char)193, (char)113, (char)176, (char)9, (char)137, (char)60, (char)58, (char)12, (char)122, (char)19, (char)147, (char)103, (char)221, (char)16, (char)123, (char)63, (char)188, (char)88, (char)8, (char)71, (char)110, (char)35, (char)9, (char)233, (char)243, (char)194, (char)11, (char)150, (char)72, (char)226, (char)195, (char)132, (char)234, (char)151, (char)197, (char)87, (char)205, (char)21, (char)130, (char)35, (char)176, (char)111, (char)48, (char)253, (char)44, (char)125, (char)237, (char)174, (char)120, (char)131, (char)250, (char)110, (char)81, (char)111, (char)50, (char)186, (char)180, (char)88, (char)149, (char)180, (char)225, (char)79, (char)76, (char)115, (char)226, (char)41, (char)22, (char)172, (char)224, (char)12, (char)4, (char)111, (char)43, (char)185, (char)139, (char)44, (char)186, (char)38, (char)57, (char)19, (char)125, (char)16, (char)227, (char)71, (char)226, (char)230, (char)52, (char)52, (char)173, (char)160, (char)202, (char)252, (char)111, (char)205, (char)78, (char)68, (char)79, (char)85, (char)94, (char)178, (char)101, (char)67, (char)118, (char)225, (char)252, (char)159, (char)196, (char)130, (char)107, (char)150, (char)244, (char)228, (char)64, (char)121, (char)185, (char)163, (char)126, (char)187, (char)132, (char)193, (char)129, (char)147, (char)84, (char)153, (char)209, (char)5, (char)209, (char)234, (char)213, (char)41, (char)191, (char)225, (char)172, (char)35, (char)65, (char)97, (char)192, (char)225, (char)110, (char)209, (char)36, (char)223, (char)106, (char)189, (char)83, (char)227, (char)7, (char)183, (char)119, (char)169, (char)25, (char)234, (char)60, (char)6, (char)239, (char)113, (char)17, (char)85, (char)230, (char)162, (char)181, (char)168, (char)133, (char)102, (char)182, (char)16, (char)157, (char)58, (char)193, (char)153}, 0) ;
        p233.flags_SET((char)123) ;
        LoopBackDemoChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.groundspeed_GET() == (char)51);
            assert(pack.temperature_air_GET() == (byte) - 51);
            assert(pack.failsafe_GET() == (char)27);
            assert(pack.gps_nsat_GET() == (char)248);
            assert(pack.roll_GET() == (short)12743);
            assert(pack.wp_num_GET() == (char)184);
            assert(pack.temperature_GET() == (byte) - 83);
            assert(pack.throttle_GET() == (byte) - 69);
            assert(pack.airspeed_GET() == (char)190);
            assert(pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED);
            assert(pack.custom_mode_GET() == 4011800705L);
            assert(pack.latitude_GET() == -2095190200);
            assert(pack.pitch_GET() == (short)16820);
            assert(pack.altitude_amsl_GET() == (short)13821);
            assert(pack.longitude_GET() == 304075203);
            assert(pack.airspeed_sp_GET() == (char)69);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
            assert(pack.wp_distance_GET() == (char)24920);
            assert(pack.climb_rate_GET() == (byte)78);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
            assert(pack.battery_remaining_GET() == (char)61);
            assert(pack.heading_GET() == (char)48057);
            assert(pack.heading_sp_GET() == (short)22158);
            assert(pack.altitude_sp_GET() == (short) -29799);
        });
        DemoDevice.HIGH_LATENCY p234 = LoopBackDemoChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.climb_rate_SET((byte)78) ;
        p234.gps_nsat_SET((char)248) ;
        p234.pitch_SET((short)16820) ;
        p234.roll_SET((short)12743) ;
        p234.custom_mode_SET(4011800705L) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC) ;
        p234.failsafe_SET((char)27) ;
        p234.heading_sp_SET((short)22158) ;
        p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED) ;
        p234.airspeed_sp_SET((char)69) ;
        p234.throttle_SET((byte) - 69) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING) ;
        p234.battery_remaining_SET((char)61) ;
        p234.wp_distance_SET((char)24920) ;
        p234.altitude_sp_SET((short) -29799) ;
        p234.groundspeed_SET((char)51) ;
        p234.latitude_SET(-2095190200) ;
        p234.wp_num_SET((char)184) ;
        p234.heading_SET((char)48057) ;
        p234.longitude_SET(304075203) ;
        p234.temperature_air_SET((byte) - 51) ;
        p234.altitude_amsl_SET((short)13821) ;
        p234.temperature_SET((byte) - 83) ;
        p234.airspeed_SET((char)190) ;
        LoopBackDemoChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.vibration_x_GET() == -4.7913897E37F);
            assert(pack.clipping_0_GET() == 104676036L);
            assert(pack.clipping_2_GET() == 3665627987L);
            assert(pack.clipping_1_GET() == 1189313720L);
            assert(pack.vibration_y_GET() == -2.2899709E38F);
            assert(pack.time_usec_GET() == 4276613458347508475L);
            assert(pack.vibration_z_GET() == -1.6940288E38F);
        });
        DemoDevice.VIBRATION p241 = LoopBackDemoChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.clipping_2_SET(3665627987L) ;
        p241.clipping_0_SET(104676036L) ;
        p241.time_usec_SET(4276613458347508475L) ;
        p241.vibration_x_SET(-4.7913897E37F) ;
        p241.clipping_1_SET(1189313720L) ;
        p241.vibration_y_SET(-2.2899709E38F) ;
        p241.vibration_z_SET(-1.6940288E38F) ;
        LoopBackDemoChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 1.1498375E38F);
            assert(pack.approach_z_GET() == -3.0708935E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {6.5617494E37F, -2.1007279E38F, -1.3503372E37F, 6.2799385E37F}));
            assert(pack.longitude_GET() == -626091944);
            assert(pack.z_GET() == -6.4974907E37F);
            assert(pack.latitude_GET() == -1907487956);
            assert(pack.approach_y_GET() == -6.5386184E37F);
            assert(pack.altitude_GET() == 386974833);
            assert(pack.x_GET() == 1.7309395E37F);
            assert(pack.approach_x_GET() == 1.6227138E38F);
            assert(pack.time_usec_TRY(ph) == 6156970903476030572L);
        });
        DemoDevice.HOME_POSITION p242 = LoopBackDemoChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.approach_z_SET(-3.0708935E38F) ;
        p242.time_usec_SET(6156970903476030572L, PH) ;
        p242.q_SET(new float[] {6.5617494E37F, -2.1007279E38F, -1.3503372E37F, 6.2799385E37F}, 0) ;
        p242.z_SET(-6.4974907E37F) ;
        p242.x_SET(1.7309395E37F) ;
        p242.y_SET(1.1498375E38F) ;
        p242.longitude_SET(-626091944) ;
        p242.approach_y_SET(-6.5386184E37F) ;
        p242.latitude_SET(-1907487956) ;
        p242.altitude_SET(386974833) ;
        p242.approach_x_SET(1.6227138E38F) ;
        LoopBackDemoChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == -1348439398);
            assert(pack.approach_y_GET() == -2.5605727E38F);
            assert(pack.approach_x_GET() == -3.197865E38F);
            assert(pack.x_GET() == -1.1358339E38F);
            assert(pack.z_GET() == 2.7134179E38F);
            assert(pack.time_usec_TRY(ph) == 7073664886251221560L);
            assert(pack.target_system_GET() == (char)166);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.6523644E38F, -2.0418373E38F, -2.8025718E38F, -1.3221791E38F}));
            assert(pack.y_GET() == 1.080219E38F);
            assert(pack.longitude_GET() == -1769976850);
            assert(pack.altitude_GET() == -30804922);
            assert(pack.approach_z_GET() == 3.3965962E38F);
        });
        DemoDevice.SET_HOME_POSITION p243 = LoopBackDemoChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.time_usec_SET(7073664886251221560L, PH) ;
        p243.latitude_SET(-1348439398) ;
        p243.y_SET(1.080219E38F) ;
        p243.target_system_SET((char)166) ;
        p243.approach_y_SET(-2.5605727E38F) ;
        p243.altitude_SET(-30804922) ;
        p243.approach_z_SET(3.3965962E38F) ;
        p243.approach_x_SET(-3.197865E38F) ;
        p243.z_SET(2.7134179E38F) ;
        p243.q_SET(new float[] {1.6523644E38F, -2.0418373E38F, -2.8025718E38F, -1.3221791E38F}, 0) ;
        p243.longitude_SET(-1769976850) ;
        p243.x_SET(-1.1358339E38F) ;
        LoopBackDemoChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.message_id_GET() == (char)5511);
            assert(pack.interval_us_GET() == -753033559);
        });
        DemoDevice.MESSAGE_INTERVAL p244 = LoopBackDemoChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.interval_us_SET(-753033559) ;
        p244.message_id_SET((char)5511) ;
        LoopBackDemoChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_FW);
        });
        DemoDevice.EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_FW) ;
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING) ;
        LoopBackDemoChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.callsign_LEN(ph) == 4);
            assert(pack.callsign_TRY(ph).equals("ruiw"));
            assert(pack.lon_GET() == -640294093);
            assert(pack.heading_GET() == (char)3333);
            assert(pack.tslc_GET() == (char)170);
            assert(pack.flags_GET() == ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK);
            assert(pack.ver_velocity_GET() == (short) -7973);
            assert(pack.altitude_GET() == -1070931487);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSGINED3);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
            assert(pack.hor_velocity_GET() == (char)60793);
            assert(pack.ICAO_address_GET() == 526089577L);
            assert(pack.squawk_GET() == (char)14495);
            assert(pack.lat_GET() == -1294026924);
        });
        DemoDevice.ADSB_VEHICLE p246 = LoopBackDemoChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSGINED3) ;
        p246.squawk_SET((char)14495) ;
        p246.heading_SET((char)3333) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH) ;
        p246.lon_SET(-640294093) ;
        p246.altitude_SET(-1070931487) ;
        p246.ver_velocity_SET((short) -7973) ;
        p246.hor_velocity_SET((char)60793) ;
        p246.lat_SET(-1294026924) ;
        p246.ICAO_address_SET(526089577L) ;
        p246.callsign_SET("ruiw", PH) ;
        p246.tslc_SET((char)170) ;
        p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK) ;
        LoopBackDemoChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
            assert(pack.time_to_minimum_delta_GET() == 2.471934E38F);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
            assert(pack.altitude_minimum_delta_GET() == 2.4794092E37F);
            assert(pack.id_GET() == 135148185L);
            assert(pack.horizontal_minimum_delta_GET() == 1.8637628E38F);
        });
        DemoDevice.COLLISION p247 = LoopBackDemoChannel.new_COLLISION();
        PH.setPack(p247);
        p247.horizontal_minimum_delta_SET(1.8637628E38F) ;
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE) ;
        p247.time_to_minimum_delta_SET(2.471934E38F) ;
        p247.id_SET(135148185L) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR) ;
        p247.altitude_minimum_delta_SET(2.4794092E37F) ;
        LoopBackDemoChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.message_type_GET() == (char)49160);
            assert(pack.target_system_GET() == (char)115);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)177, (char)171, (char)163, (char)94, (char)111, (char)60, (char)80, (char)187, (char)60, (char)41, (char)17, (char)18, (char)162, (char)227, (char)106, (char)10, (char)249, (char)174, (char)132, (char)253, (char)43, (char)39, (char)49, (char)23, (char)111, (char)178, (char)9, (char)142, (char)92, (char)237, (char)77, (char)104, (char)9, (char)156, (char)157, (char)51, (char)181, (char)95, (char)87, (char)82, (char)138, (char)38, (char)40, (char)56, (char)230, (char)153, (char)22, (char)221, (char)75, (char)67, (char)204, (char)219, (char)144, (char)174, (char)2, (char)170, (char)102, (char)80, (char)56, (char)8, (char)145, (char)41, (char)219, (char)98, (char)172, (char)38, (char)165, (char)143, (char)94, (char)123, (char)227, (char)113, (char)116, (char)187, (char)61, (char)1, (char)232, (char)169, (char)174, (char)250, (char)67, (char)125, (char)145, (char)76, (char)46, (char)51, (char)200, (char)21, (char)0, (char)1, (char)172, (char)245, (char)237, (char)55, (char)7, (char)103, (char)150, (char)206, (char)217, (char)143, (char)40, (char)151, (char)176, (char)253, (char)45, (char)84, (char)241, (char)29, (char)254, (char)154, (char)24, (char)143, (char)66, (char)206, (char)203, (char)239, (char)161, (char)21, (char)105, (char)101, (char)209, (char)247, (char)169, (char)114, (char)137, (char)212, (char)91, (char)101, (char)249, (char)158, (char)101, (char)88, (char)179, (char)47, (char)182, (char)190, (char)85, (char)253, (char)142, (char)201, (char)149, (char)189, (char)60, (char)101, (char)75, (char)209, (char)104, (char)21, (char)136, (char)76, (char)77, (char)21, (char)235, (char)194, (char)83, (char)41, (char)68, (char)109, (char)100, (char)120, (char)149, (char)60, (char)182, (char)164, (char)41, (char)126, (char)184, (char)200, (char)57, (char)117, (char)30, (char)181, (char)59, (char)143, (char)132, (char)129, (char)232, (char)76, (char)229, (char)165, (char)38, (char)209, (char)66, (char)89, (char)61, (char)128, (char)22, (char)22, (char)245, (char)108, (char)50, (char)18, (char)228, (char)83, (char)157, (char)243, (char)241, (char)183, (char)237, (char)133, (char)213, (char)106, (char)131, (char)130, (char)165, (char)127, (char)46, (char)197, (char)0, (char)72, (char)109, (char)204, (char)198, (char)165, (char)3, (char)199, (char)160, (char)180, (char)196, (char)5, (char)36, (char)67, (char)70, (char)92, (char)179, (char)31, (char)209, (char)154, (char)117, (char)89, (char)86, (char)41, (char)118, (char)51, (char)184, (char)20, (char)236, (char)183, (char)115, (char)222, (char)118, (char)224, (char)105, (char)214, (char)120, (char)78, (char)164, (char)30, (char)230}));
            assert(pack.target_network_GET() == (char)102);
            assert(pack.target_component_GET() == (char)33);
        });
        DemoDevice.V2_EXTENSION p248 = LoopBackDemoChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_network_SET((char)102) ;
        p248.target_system_SET((char)115) ;
        p248.target_component_SET((char)33) ;
        p248.message_type_SET((char)49160) ;
        p248.payload_SET(new char[] {(char)177, (char)171, (char)163, (char)94, (char)111, (char)60, (char)80, (char)187, (char)60, (char)41, (char)17, (char)18, (char)162, (char)227, (char)106, (char)10, (char)249, (char)174, (char)132, (char)253, (char)43, (char)39, (char)49, (char)23, (char)111, (char)178, (char)9, (char)142, (char)92, (char)237, (char)77, (char)104, (char)9, (char)156, (char)157, (char)51, (char)181, (char)95, (char)87, (char)82, (char)138, (char)38, (char)40, (char)56, (char)230, (char)153, (char)22, (char)221, (char)75, (char)67, (char)204, (char)219, (char)144, (char)174, (char)2, (char)170, (char)102, (char)80, (char)56, (char)8, (char)145, (char)41, (char)219, (char)98, (char)172, (char)38, (char)165, (char)143, (char)94, (char)123, (char)227, (char)113, (char)116, (char)187, (char)61, (char)1, (char)232, (char)169, (char)174, (char)250, (char)67, (char)125, (char)145, (char)76, (char)46, (char)51, (char)200, (char)21, (char)0, (char)1, (char)172, (char)245, (char)237, (char)55, (char)7, (char)103, (char)150, (char)206, (char)217, (char)143, (char)40, (char)151, (char)176, (char)253, (char)45, (char)84, (char)241, (char)29, (char)254, (char)154, (char)24, (char)143, (char)66, (char)206, (char)203, (char)239, (char)161, (char)21, (char)105, (char)101, (char)209, (char)247, (char)169, (char)114, (char)137, (char)212, (char)91, (char)101, (char)249, (char)158, (char)101, (char)88, (char)179, (char)47, (char)182, (char)190, (char)85, (char)253, (char)142, (char)201, (char)149, (char)189, (char)60, (char)101, (char)75, (char)209, (char)104, (char)21, (char)136, (char)76, (char)77, (char)21, (char)235, (char)194, (char)83, (char)41, (char)68, (char)109, (char)100, (char)120, (char)149, (char)60, (char)182, (char)164, (char)41, (char)126, (char)184, (char)200, (char)57, (char)117, (char)30, (char)181, (char)59, (char)143, (char)132, (char)129, (char)232, (char)76, (char)229, (char)165, (char)38, (char)209, (char)66, (char)89, (char)61, (char)128, (char)22, (char)22, (char)245, (char)108, (char)50, (char)18, (char)228, (char)83, (char)157, (char)243, (char)241, (char)183, (char)237, (char)133, (char)213, (char)106, (char)131, (char)130, (char)165, (char)127, (char)46, (char)197, (char)0, (char)72, (char)109, (char)204, (char)198, (char)165, (char)3, (char)199, (char)160, (char)180, (char)196, (char)5, (char)36, (char)67, (char)70, (char)92, (char)179, (char)31, (char)209, (char)154, (char)117, (char)89, (char)86, (char)41, (char)118, (char)51, (char)184, (char)20, (char)236, (char)183, (char)115, (char)222, (char)118, (char)224, (char)105, (char)214, (char)120, (char)78, (char)164, (char)30, (char)230}, 0) ;
        LoopBackDemoChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == (char)58);
            assert(pack.address_GET() == (char)63434);
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte) - 27, (byte)17, (byte)99, (byte) - 63, (byte) - 2, (byte)62, (byte)5, (byte)71, (byte) - 21, (byte) - 52, (byte)1, (byte)57, (byte)67, (byte) - 95, (byte)48, (byte)19, (byte) - 4, (byte) - 59, (byte) - 74, (byte)49, (byte) - 55, (byte) - 96, (byte) - 109, (byte)72, (byte)112, (byte) - 24, (byte) - 32, (byte) - 4, (byte)41, (byte) - 105, (byte)21, (byte) - 33}));
            assert(pack.ver_GET() == (char)236);
        });
        DemoDevice.MEMORY_VECT p249 = LoopBackDemoChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)63434) ;
        p249.type_SET((char)58) ;
        p249.ver_SET((char)236) ;
        p249.value_SET(new byte[] {(byte) - 27, (byte)17, (byte)99, (byte) - 63, (byte) - 2, (byte)62, (byte)5, (byte)71, (byte) - 21, (byte) - 52, (byte)1, (byte)57, (byte)67, (byte) - 95, (byte)48, (byte)19, (byte) - 4, (byte) - 59, (byte) - 74, (byte)49, (byte) - 55, (byte) - 96, (byte) - 109, (byte)72, (byte)112, (byte) - 24, (byte) - 32, (byte) - 4, (byte)41, (byte) - 105, (byte)21, (byte) - 33}, 0) ;
        LoopBackDemoChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 4935329168265157533L);
            assert(pack.x_GET() == 3.381899E38F);
            assert(pack.y_GET() == 3.3475141E38F);
            assert(pack.z_GET() == 4.028248E36F);
            assert(pack.name_LEN(ph) == 2);
            assert(pack.name_TRY(ph).equals("rx"));
        });
        DemoDevice.DEBUG_VECT p250 = LoopBackDemoChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.time_usec_SET(4935329168265157533L) ;
        p250.name_SET("rx", PH) ;
        p250.y_SET(3.3475141E38F) ;
        p250.x_SET(3.381899E38F) ;
        p250.z_SET(4.028248E36F) ;
        LoopBackDemoChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 4);
            assert(pack.name_TRY(ph).equals("jyqk"));
            assert(pack.time_boot_ms_GET() == 1667208384L);
            assert(pack.value_GET() == -3.2606263E38F);
        });
        DemoDevice.NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.value_SET(-3.2606263E38F) ;
        p251.time_boot_ms_SET(1667208384L) ;
        p251.name_SET("jyqk", PH) ;
        LoopBackDemoChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 4104433295L);
            assert(pack.value_GET() == 48479706);
            assert(pack.name_LEN(ph) == 5);
            assert(pack.name_TRY(ph).equals("qwrjs"));
        });
        DemoDevice.NAMED_VALUE_INT p252 = LoopBackDemoChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.value_SET(48479706) ;
        p252.time_boot_ms_SET(4104433295L) ;
        p252.name_SET("qwrjs", PH) ;
        LoopBackDemoChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_DEBUG);
            assert(pack.text_LEN(ph) == 8);
            assert(pack.text_TRY(ph).equals("hwsrkfkd"));
        });
        DemoDevice.STATUSTEXT p253 = LoopBackDemoChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_DEBUG) ;
        p253.text_SET("hwsrkfkd", PH) ;
        LoopBackDemoChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1394113755L);
            assert(pack.ind_GET() == (char)254);
            assert(pack.value_GET() == 2.9440508E38F);
        });
        DemoDevice.DEBUG p254 = LoopBackDemoChannel.new_DEBUG();
        PH.setPack(p254);
        p254.value_SET(2.9440508E38F) ;
        p254.time_boot_ms_SET(1394113755L) ;
        p254.ind_SET((char)254) ;
        LoopBackDemoChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)8, (char)50, (char)44, (char)159, (char)60, (char)98, (char)226, (char)57, (char)67, (char)199, (char)189, (char)36, (char)70, (char)44, (char)240, (char)1, (char)175, (char)114, (char)129, (char)143, (char)199, (char)225, (char)136, (char)219, (char)1, (char)85, (char)26, (char)240, (char)171, (char)249, (char)34, (char)146}));
            assert(pack.initial_timestamp_GET() == 2161953583456418275L);
            assert(pack.target_system_GET() == (char)195);
            assert(pack.target_component_GET() == (char)118);
        });
        DemoDevice.SETUP_SIGNING p256 = LoopBackDemoChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.initial_timestamp_SET(2161953583456418275L) ;
        p256.target_system_SET((char)195) ;
        p256.secret_key_SET(new char[] {(char)8, (char)50, (char)44, (char)159, (char)60, (char)98, (char)226, (char)57, (char)67, (char)199, (char)189, (char)36, (char)70, (char)44, (char)240, (char)1, (char)175, (char)114, (char)129, (char)143, (char)199, (char)225, (char)136, (char)219, (char)1, (char)85, (char)26, (char)240, (char)171, (char)249, (char)34, (char)146}, 0) ;
        p256.target_component_SET((char)118) ;
        LoopBackDemoChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.state_GET() == (char)179);
            assert(pack.last_change_ms_GET() == 2750883823L);
            assert(pack.time_boot_ms_GET() == 132699370L);
        });
        DemoDevice.BUTTON_CHANGE p257 = LoopBackDemoChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.last_change_ms_SET(2750883823L) ;
        p257.state_SET((char)179) ;
        p257.time_boot_ms_SET(132699370L) ;
        LoopBackDemoChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)141);
            assert(pack.tune_LEN(ph) == 6);
            assert(pack.tune_TRY(ph).equals("gpOpro"));
            assert(pack.target_system_GET() == (char)129);
        });
        DemoDevice.PLAY_TUNE p258 = LoopBackDemoChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.tune_SET("gpOpro", PH) ;
        p258.target_component_SET((char)141) ;
        p258.target_system_SET((char)129) ;
        LoopBackDemoChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.cam_definition_uri_LEN(ph) == 110);
            assert(pack.cam_definition_uri_TRY(ph).equals("rtouVrvdqnXqyemUfhxwdohzlxgohnkrmveZiwevvrizirkkdqrdwOqrpwadxhhbuyEglburpmquzHuathrqiprngpxgtqjmnzBKIyyuidyiEg"));
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)149, (char)44, (char)150, (char)198, (char)133, (char)156, (char)148, (char)161, (char)242, (char)195, (char)30, (char)209, (char)111, (char)34, (char)226, (char)37, (char)93, (char)177, (char)105, (char)135, (char)17, (char)216, (char)43, (char)88, (char)248, (char)9, (char)99, (char)68, (char)50, (char)182, (char)168, (char)74}));
            assert(pack.flags_GET() == CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)234, (char)203, (char)32, (char)100, (char)232, (char)101, (char)149, (char)34, (char)200, (char)55, (char)20, (char)44, (char)165, (char)197, (char)188, (char)66, (char)73, (char)241, (char)254, (char)34, (char)224, (char)14, (char)233, (char)200, (char)94, (char)68, (char)58, (char)150, (char)129, (char)149, (char)170, (char)225}));
            assert(pack.time_boot_ms_GET() == 3989524573L);
            assert(pack.resolution_v_GET() == (char)3724);
            assert(pack.sensor_size_v_GET() == 2.6849122E38F);
            assert(pack.lens_id_GET() == (char)180);
            assert(pack.sensor_size_h_GET() == -1.8322648E38F);
            assert(pack.resolution_h_GET() == (char)12001);
            assert(pack.focal_length_GET() == -2.0087473E38F);
            assert(pack.firmware_version_GET() == 841685042L);
            assert(pack.cam_definition_version_GET() == (char)26630);
        });
        DemoDevice.CAMERA_INFORMATION p259 = LoopBackDemoChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.sensor_size_v_SET(2.6849122E38F) ;
        p259.time_boot_ms_SET(3989524573L) ;
        p259.vendor_name_SET(new char[] {(char)234, (char)203, (char)32, (char)100, (char)232, (char)101, (char)149, (char)34, (char)200, (char)55, (char)20, (char)44, (char)165, (char)197, (char)188, (char)66, (char)73, (char)241, (char)254, (char)34, (char)224, (char)14, (char)233, (char)200, (char)94, (char)68, (char)58, (char)150, (char)129, (char)149, (char)170, (char)225}, 0) ;
        p259.focal_length_SET(-2.0087473E38F) ;
        p259.firmware_version_SET(841685042L) ;
        p259.resolution_v_SET((char)3724) ;
        p259.resolution_h_SET((char)12001) ;
        p259.cam_definition_version_SET((char)26630) ;
        p259.lens_id_SET((char)180) ;
        p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES) ;
        p259.cam_definition_uri_SET("rtouVrvdqnXqyemUfhxwdohzlxgohnkrmveZiwevvrizirkkdqrdwOqrpwadxhhbuyEglburpmquzHuathrqiprngpxgtqjmnzBKIyyuidyiEg", PH) ;
        p259.model_name_SET(new char[] {(char)149, (char)44, (char)150, (char)198, (char)133, (char)156, (char)148, (char)161, (char)242, (char)195, (char)30, (char)209, (char)111, (char)34, (char)226, (char)37, (char)93, (char)177, (char)105, (char)135, (char)17, (char)216, (char)43, (char)88, (char)248, (char)9, (char)99, (char)68, (char)50, (char)182, (char)168, (char)74}, 0) ;
        p259.sensor_size_h_SET(-1.8322648E38F) ;
        LoopBackDemoChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_IMAGE);
            assert(pack.time_boot_ms_GET() == 3253714516L);
        });
        DemoDevice.CAMERA_SETTINGS p260 = LoopBackDemoChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE) ;
        p260.time_boot_ms_SET(3253714516L) ;
        LoopBackDemoChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.storage_count_GET() == (char)77);
            assert(pack.write_speed_GET() == 2.2874124E38F);
            assert(pack.read_speed_GET() == -5.8912794E37F);
            assert(pack.available_capacity_GET() == -8.8214804E36F);
            assert(pack.time_boot_ms_GET() == 3450434311L);
            assert(pack.used_capacity_GET() == 3.3536009E38F);
            assert(pack.storage_id_GET() == (char)222);
            assert(pack.status_GET() == (char)223);
            assert(pack.total_capacity_GET() == 2.1337069E38F);
        });
        DemoDevice.STORAGE_INFORMATION p261 = LoopBackDemoChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.time_boot_ms_SET(3450434311L) ;
        p261.storage_count_SET((char)77) ;
        p261.read_speed_SET(-5.8912794E37F) ;
        p261.write_speed_SET(2.2874124E38F) ;
        p261.storage_id_SET((char)222) ;
        p261.total_capacity_SET(2.1337069E38F) ;
        p261.used_capacity_SET(3.3536009E38F) ;
        p261.status_SET((char)223) ;
        p261.available_capacity_SET(-8.8214804E36F) ;
        LoopBackDemoChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.image_interval_GET() == -2.2355941E38F);
            assert(pack.video_status_GET() == (char)71);
            assert(pack.image_status_GET() == (char)28);
            assert(pack.available_capacity_GET() == 3.3696944E38F);
            assert(pack.recording_time_ms_GET() == 38923738L);
            assert(pack.time_boot_ms_GET() == 3407944614L);
        });
        DemoDevice.CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.recording_time_ms_SET(38923738L) ;
        p262.image_interval_SET(-2.2355941E38F) ;
        p262.video_status_SET((char)71) ;
        p262.available_capacity_SET(3.3696944E38F) ;
        p262.time_boot_ms_SET(3407944614L) ;
        p262.image_status_SET((char)28) ;
        LoopBackDemoChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -556001149);
            assert(pack.relative_alt_GET() == 412829839);
            assert(pack.camera_id_GET() == (char)234);
            assert(pack.image_index_GET() == -793975995);
            assert(pack.capture_result_GET() == (byte)79);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.1954475E38F, -2.9310512E38F, 6.4569086E37F, -2.6695608E38F}));
            assert(pack.alt_GET() == 1412326381);
            assert(pack.time_utc_GET() == 8377446345829712350L);
            assert(pack.time_boot_ms_GET() == 15044741L);
            assert(pack.file_url_LEN(ph) == 74);
            assert(pack.file_url_TRY(ph).equals("jttrzutnnntauZvYcdrqouvoqsnvexxvwdvpgxoslddlkcnUwmjseqwkgbmaimwffpbQmexiyo"));
            assert(pack.lon_GET() == 1273448425);
        });
        DemoDevice.CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.image_index_SET(-793975995) ;
        p263.time_boot_ms_SET(15044741L) ;
        p263.file_url_SET("jttrzutnnntauZvYcdrqouvoqsnvexxvwdvpgxoslddlkcnUwmjseqwkgbmaimwffpbQmexiyo", PH) ;
        p263.time_utc_SET(8377446345829712350L) ;
        p263.relative_alt_SET(412829839) ;
        p263.lat_SET(-556001149) ;
        p263.q_SET(new float[] {1.1954475E38F, -2.9310512E38F, 6.4569086E37F, -2.6695608E38F}, 0) ;
        p263.lon_SET(1273448425) ;
        p263.capture_result_SET((byte)79) ;
        p263.alt_SET(1412326381) ;
        p263.camera_id_SET((char)234) ;
        LoopBackDemoChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3109249864L);
            assert(pack.flight_uuid_GET() == 453429326217093033L);
            assert(pack.arming_time_utc_GET() == 5313936508859325277L);
            assert(pack.takeoff_time_utc_GET() == 2644500984786400377L);
        });
        DemoDevice.FLIGHT_INFORMATION p264 = LoopBackDemoChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.flight_uuid_SET(453429326217093033L) ;
        p264.takeoff_time_utc_SET(2644500984786400377L) ;
        p264.arming_time_utc_SET(5313936508859325277L) ;
        p264.time_boot_ms_SET(3109249864L) ;
        LoopBackDemoChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == -1.5665255E38F);
            assert(pack.pitch_GET() == 2.4104514E38F);
            assert(pack.yaw_GET() == -2.8144949E38F);
            assert(pack.time_boot_ms_GET() == 1746212272L);
        });
        DemoDevice.MOUNT_ORIENTATION p265 = LoopBackDemoChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.time_boot_ms_SET(1746212272L) ;
        p265.pitch_SET(2.4104514E38F) ;
        p265.roll_SET(-1.5665255E38F) ;
        p265.yaw_SET(-2.8144949E38F) ;
        LoopBackDemoChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)175, (char)115, (char)199, (char)128, (char)29, (char)117, (char)12, (char)184, (char)52, (char)123, (char)48, (char)182, (char)144, (char)243, (char)48, (char)45, (char)17, (char)42, (char)173, (char)62, (char)92, (char)24, (char)23, (char)40, (char)215, (char)228, (char)167, (char)254, (char)151, (char)86, (char)15, (char)44, (char)124, (char)155, (char)254, (char)65, (char)20, (char)203, (char)39, (char)130, (char)31, (char)44, (char)126, (char)16, (char)36, (char)208, (char)38, (char)33, (char)7, (char)1, (char)141, (char)160, (char)67, (char)21, (char)54, (char)228, (char)118, (char)254, (char)185, (char)96, (char)177, (char)47, (char)212, (char)29, (char)135, (char)184, (char)95, (char)170, (char)21, (char)212, (char)22, (char)72, (char)29, (char)105, (char)206, (char)86, (char)245, (char)81, (char)120, (char)188, (char)174, (char)77, (char)241, (char)127, (char)213, (char)90, (char)181, (char)39, (char)205, (char)67, (char)177, (char)97, (char)46, (char)213, (char)107, (char)254, (char)181, (char)161, (char)191, (char)126, (char)21, (char)42, (char)167, (char)81, (char)90, (char)203, (char)250, (char)69, (char)68, (char)40, (char)119, (char)153, (char)210, (char)87, (char)11, (char)211, (char)40, (char)247, (char)122, (char)176, (char)180, (char)192, (char)196, (char)92, (char)179, (char)21, (char)195, (char)244, (char)182, (char)145, (char)58, (char)231, (char)22, (char)140, (char)116, (char)209, (char)253, (char)162, (char)53, (char)94, (char)31, (char)140, (char)69, (char)173, (char)187, (char)244, (char)99, (char)171, (char)161, (char)109, (char)86, (char)209, (char)33, (char)196, (char)123, (char)248, (char)133, (char)178, (char)146, (char)187, (char)203, (char)143, (char)77, (char)28, (char)242, (char)99, (char)146, (char)213, (char)164, (char)218, (char)72, (char)207, (char)126, (char)251, (char)105, (char)45, (char)144, (char)14, (char)3, (char)4, (char)37, (char)69, (char)85, (char)186, (char)36, (char)243, (char)225, (char)235, (char)137, (char)177, (char)54, (char)150, (char)30, (char)246, (char)89, (char)8, (char)231, (char)68, (char)74, (char)130, (char)228, (char)189, (char)227, (char)157, (char)66, (char)130, (char)67, (char)50, (char)95, (char)196, (char)56, (char)75, (char)116, (char)251, (char)251, (char)30, (char)140, (char)223, (char)172, (char)154, (char)158, (char)113, (char)18, (char)111, (char)235, (char)104, (char)78, (char)151, (char)199, (char)15, (char)136, (char)40, (char)169, (char)136, (char)8, (char)74, (char)172, (char)145, (char)59, (char)81, (char)83, (char)125, (char)197, (char)149, (char)225, (char)93, (char)15, (char)80, (char)124}));
            assert(pack.target_component_GET() == (char)102);
            assert(pack.first_message_offset_GET() == (char)14);
            assert(pack.target_system_GET() == (char)1);
            assert(pack.sequence_GET() == (char)20352);
            assert(pack.length_GET() == (char)215);
        });
        DemoDevice.LOGGING_DATA p266 = LoopBackDemoChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.data__SET(new char[] {(char)175, (char)115, (char)199, (char)128, (char)29, (char)117, (char)12, (char)184, (char)52, (char)123, (char)48, (char)182, (char)144, (char)243, (char)48, (char)45, (char)17, (char)42, (char)173, (char)62, (char)92, (char)24, (char)23, (char)40, (char)215, (char)228, (char)167, (char)254, (char)151, (char)86, (char)15, (char)44, (char)124, (char)155, (char)254, (char)65, (char)20, (char)203, (char)39, (char)130, (char)31, (char)44, (char)126, (char)16, (char)36, (char)208, (char)38, (char)33, (char)7, (char)1, (char)141, (char)160, (char)67, (char)21, (char)54, (char)228, (char)118, (char)254, (char)185, (char)96, (char)177, (char)47, (char)212, (char)29, (char)135, (char)184, (char)95, (char)170, (char)21, (char)212, (char)22, (char)72, (char)29, (char)105, (char)206, (char)86, (char)245, (char)81, (char)120, (char)188, (char)174, (char)77, (char)241, (char)127, (char)213, (char)90, (char)181, (char)39, (char)205, (char)67, (char)177, (char)97, (char)46, (char)213, (char)107, (char)254, (char)181, (char)161, (char)191, (char)126, (char)21, (char)42, (char)167, (char)81, (char)90, (char)203, (char)250, (char)69, (char)68, (char)40, (char)119, (char)153, (char)210, (char)87, (char)11, (char)211, (char)40, (char)247, (char)122, (char)176, (char)180, (char)192, (char)196, (char)92, (char)179, (char)21, (char)195, (char)244, (char)182, (char)145, (char)58, (char)231, (char)22, (char)140, (char)116, (char)209, (char)253, (char)162, (char)53, (char)94, (char)31, (char)140, (char)69, (char)173, (char)187, (char)244, (char)99, (char)171, (char)161, (char)109, (char)86, (char)209, (char)33, (char)196, (char)123, (char)248, (char)133, (char)178, (char)146, (char)187, (char)203, (char)143, (char)77, (char)28, (char)242, (char)99, (char)146, (char)213, (char)164, (char)218, (char)72, (char)207, (char)126, (char)251, (char)105, (char)45, (char)144, (char)14, (char)3, (char)4, (char)37, (char)69, (char)85, (char)186, (char)36, (char)243, (char)225, (char)235, (char)137, (char)177, (char)54, (char)150, (char)30, (char)246, (char)89, (char)8, (char)231, (char)68, (char)74, (char)130, (char)228, (char)189, (char)227, (char)157, (char)66, (char)130, (char)67, (char)50, (char)95, (char)196, (char)56, (char)75, (char)116, (char)251, (char)251, (char)30, (char)140, (char)223, (char)172, (char)154, (char)158, (char)113, (char)18, (char)111, (char)235, (char)104, (char)78, (char)151, (char)199, (char)15, (char)136, (char)40, (char)169, (char)136, (char)8, (char)74, (char)172, (char)145, (char)59, (char)81, (char)83, (char)125, (char)197, (char)149, (char)225, (char)93, (char)15, (char)80, (char)124}, 0) ;
        p266.target_component_SET((char)102) ;
        p266.sequence_SET((char)20352) ;
        p266.first_message_offset_SET((char)14) ;
        p266.target_system_SET((char)1) ;
        p266.length_SET((char)215) ;
        LoopBackDemoChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)130);
            assert(pack.target_component_GET() == (char)23);
            assert(pack.first_message_offset_GET() == (char)206);
            assert(pack.sequence_GET() == (char)1372);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)234, (char)154, (char)103, (char)104, (char)241, (char)113, (char)164, (char)56, (char)107, (char)56, (char)252, (char)111, (char)229, (char)176, (char)48, (char)190, (char)90, (char)45, (char)44, (char)114, (char)124, (char)166, (char)119, (char)165, (char)224, (char)119, (char)250, (char)12, (char)136, (char)190, (char)14, (char)49, (char)229, (char)168, (char)11, (char)211, (char)208, (char)135, (char)98, (char)95, (char)2, (char)62, (char)65, (char)26, (char)109, (char)116, (char)158, (char)23, (char)172, (char)196, (char)211, (char)107, (char)187, (char)153, (char)134, (char)34, (char)23, (char)153, (char)186, (char)151, (char)100, (char)219, (char)16, (char)255, (char)178, (char)48, (char)215, (char)131, (char)66, (char)20, (char)79, (char)143, (char)229, (char)41, (char)230, (char)191, (char)124, (char)190, (char)28, (char)113, (char)126, (char)205, (char)135, (char)139, (char)201, (char)235, (char)195, (char)118, (char)118, (char)210, (char)41, (char)14, (char)118, (char)49, (char)215, (char)133, (char)187, (char)108, (char)61, (char)76, (char)132, (char)255, (char)224, (char)47, (char)148, (char)154, (char)182, (char)119, (char)171, (char)71, (char)153, (char)194, (char)35, (char)139, (char)3, (char)194, (char)39, (char)131, (char)7, (char)90, (char)75, (char)172, (char)238, (char)214, (char)43, (char)18, (char)46, (char)109, (char)115, (char)83, (char)61, (char)119, (char)28, (char)163, (char)143, (char)38, (char)200, (char)165, (char)3, (char)205, (char)207, (char)76, (char)240, (char)147, (char)174, (char)227, (char)210, (char)193, (char)93, (char)248, (char)112, (char)54, (char)59, (char)163, (char)108, (char)236, (char)208, (char)233, (char)227, (char)249, (char)251, (char)231, (char)146, (char)125, (char)224, (char)172, (char)44, (char)9, (char)68, (char)204, (char)208, (char)67, (char)94, (char)140, (char)170, (char)93, (char)178, (char)185, (char)0, (char)180, (char)113, (char)221, (char)72, (char)102, (char)215, (char)153, (char)181, (char)188, (char)198, (char)2, (char)29, (char)99, (char)157, (char)9, (char)121, (char)32, (char)130, (char)74, (char)184, (char)47, (char)158, (char)187, (char)16, (char)223, (char)159, (char)194, (char)22, (char)103, (char)66, (char)247, (char)243, (char)88, (char)223, (char)141, (char)141, (char)218, (char)239, (char)123, (char)153, (char)199, (char)9, (char)203, (char)180, (char)55, (char)200, (char)142, (char)208, (char)211, (char)45, (char)228, (char)25, (char)130, (char)45, (char)253, (char)90, (char)171, (char)181, (char)95, (char)229, (char)133, (char)161, (char)72, (char)54, (char)169, (char)85, (char)177, (char)129, (char)160, (char)84}));
            assert(pack.length_GET() == (char)92);
        });
        DemoDevice.LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.first_message_offset_SET((char)206) ;
        p267.sequence_SET((char)1372) ;
        p267.target_component_SET((char)23) ;
        p267.length_SET((char)92) ;
        p267.target_system_SET((char)130) ;
        p267.data__SET(new char[] {(char)234, (char)154, (char)103, (char)104, (char)241, (char)113, (char)164, (char)56, (char)107, (char)56, (char)252, (char)111, (char)229, (char)176, (char)48, (char)190, (char)90, (char)45, (char)44, (char)114, (char)124, (char)166, (char)119, (char)165, (char)224, (char)119, (char)250, (char)12, (char)136, (char)190, (char)14, (char)49, (char)229, (char)168, (char)11, (char)211, (char)208, (char)135, (char)98, (char)95, (char)2, (char)62, (char)65, (char)26, (char)109, (char)116, (char)158, (char)23, (char)172, (char)196, (char)211, (char)107, (char)187, (char)153, (char)134, (char)34, (char)23, (char)153, (char)186, (char)151, (char)100, (char)219, (char)16, (char)255, (char)178, (char)48, (char)215, (char)131, (char)66, (char)20, (char)79, (char)143, (char)229, (char)41, (char)230, (char)191, (char)124, (char)190, (char)28, (char)113, (char)126, (char)205, (char)135, (char)139, (char)201, (char)235, (char)195, (char)118, (char)118, (char)210, (char)41, (char)14, (char)118, (char)49, (char)215, (char)133, (char)187, (char)108, (char)61, (char)76, (char)132, (char)255, (char)224, (char)47, (char)148, (char)154, (char)182, (char)119, (char)171, (char)71, (char)153, (char)194, (char)35, (char)139, (char)3, (char)194, (char)39, (char)131, (char)7, (char)90, (char)75, (char)172, (char)238, (char)214, (char)43, (char)18, (char)46, (char)109, (char)115, (char)83, (char)61, (char)119, (char)28, (char)163, (char)143, (char)38, (char)200, (char)165, (char)3, (char)205, (char)207, (char)76, (char)240, (char)147, (char)174, (char)227, (char)210, (char)193, (char)93, (char)248, (char)112, (char)54, (char)59, (char)163, (char)108, (char)236, (char)208, (char)233, (char)227, (char)249, (char)251, (char)231, (char)146, (char)125, (char)224, (char)172, (char)44, (char)9, (char)68, (char)204, (char)208, (char)67, (char)94, (char)140, (char)170, (char)93, (char)178, (char)185, (char)0, (char)180, (char)113, (char)221, (char)72, (char)102, (char)215, (char)153, (char)181, (char)188, (char)198, (char)2, (char)29, (char)99, (char)157, (char)9, (char)121, (char)32, (char)130, (char)74, (char)184, (char)47, (char)158, (char)187, (char)16, (char)223, (char)159, (char)194, (char)22, (char)103, (char)66, (char)247, (char)243, (char)88, (char)223, (char)141, (char)141, (char)218, (char)239, (char)123, (char)153, (char)199, (char)9, (char)203, (char)180, (char)55, (char)200, (char)142, (char)208, (char)211, (char)45, (char)228, (char)25, (char)130, (char)45, (char)253, (char)90, (char)171, (char)181, (char)95, (char)229, (char)133, (char)161, (char)72, (char)54, (char)169, (char)85, (char)177, (char)129, (char)160, (char)84}, 0) ;
        LoopBackDemoChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)12);
            assert(pack.target_system_GET() == (char)143);
            assert(pack.sequence_GET() == (char)52685);
        });
        DemoDevice.LOGGING_ACK p268 = LoopBackDemoChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)143) ;
        p268.sequence_SET((char)52685) ;
        p268.target_component_SET((char)12) ;
        LoopBackDemoChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.status_GET() == (char)212);
            assert(pack.bitrate_GET() == 347720949L);
            assert(pack.rotation_GET() == (char)7889);
            assert(pack.resolution_h_GET() == (char)19141);
            assert(pack.uri_LEN(ph) == 107);
            assert(pack.uri_TRY(ph).equals("kqssmnxwfkhmohlytrciiUwiAhhptwachjvfNniyvowqnjtrRpksyeuagpahxmhXtjppgdgvMygamraoNsdmlNzttxoebFfbkdttyuvpcqj"));
            assert(pack.camera_id_GET() == (char)97);
            assert(pack.framerate_GET() == 3.392658E38F);
            assert(pack.resolution_v_GET() == (char)48302);
        });
        DemoDevice.VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.resolution_h_SET((char)19141) ;
        p269.uri_SET("kqssmnxwfkhmohlytrciiUwiAhhptwachjvfNniyvowqnjtrRpksyeuagpahxmhXtjppgdgvMygamraoNsdmlNzttxoebFfbkdttyuvpcqj", PH) ;
        p269.camera_id_SET((char)97) ;
        p269.resolution_v_SET((char)48302) ;
        p269.framerate_SET(3.392658E38F) ;
        p269.rotation_SET((char)7889) ;
        p269.bitrate_SET(347720949L) ;
        p269.status_SET((char)212) ;
        LoopBackDemoChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.framerate_GET() == -5.7522785E37F);
            assert(pack.target_system_GET() == (char)251);
            assert(pack.bitrate_GET() == 314396306L);
            assert(pack.resolution_h_GET() == (char)18946);
            assert(pack.camera_id_GET() == (char)104);
            assert(pack.uri_LEN(ph) == 73);
            assert(pack.uri_TRY(ph).equals("XyyezdCbpjmjntNduztplevmkysfksblwuzhxmpWwaqpbkepepxjvsphueeBKjmckcBnGshQy"));
            assert(pack.target_component_GET() == (char)33);
            assert(pack.resolution_v_GET() == (char)16568);
            assert(pack.rotation_GET() == (char)43568);
        });
        DemoDevice.SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.target_system_SET((char)251) ;
        p270.framerate_SET(-5.7522785E37F) ;
        p270.camera_id_SET((char)104) ;
        p270.resolution_h_SET((char)18946) ;
        p270.rotation_SET((char)43568) ;
        p270.bitrate_SET(314396306L) ;
        p270.uri_SET("XyyezdCbpjmjntNduztplevmkysfksblwuzhxmpWwaqpbkepepxjvsphueeBKjmckcBnGshQy", PH) ;
        p270.resolution_v_SET((char)16568) ;
        p270.target_component_SET((char)33) ;
        LoopBackDemoChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.ssid_LEN(ph) == 30);
            assert(pack.ssid_TRY(ph).equals("gdboehPqwjidWqfqxjtibwrAtwbkpj"));
            assert(pack.password_LEN(ph) == 34);
            assert(pack.password_TRY(ph).equals("inVrebypkpeyxkzUnbUhydfoxEjmEjwkep"));
        });
        DemoDevice.WIFI_CONFIG_AP p299 = LoopBackDemoChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("gdboehPqwjidWqfqxjtibwrAtwbkpj", PH) ;
        p299.password_SET("inVrebypkpeyxkzUnbUhydfoxEjmEjwkep", PH) ;
        LoopBackDemoChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(pack.version_GET() == (char)47443);
            assert(pack.min_version_GET() == (char)15451);
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)54, (char)183, (char)240, (char)206, (char)71, (char)253, (char)66, (char)3}));
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)90, (char)201, (char)22, (char)198, (char)107, (char)84, (char)60, (char)75}));
            assert(pack.max_version_GET() == (char)56272);
        });
        DemoDevice.PROTOCOL_VERSION p300 = LoopBackDemoChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.spec_version_hash_SET(new char[] {(char)54, (char)183, (char)240, (char)206, (char)71, (char)253, (char)66, (char)3}, 0) ;
        p300.version_SET((char)47443) ;
        p300.library_version_hash_SET(new char[] {(char)90, (char)201, (char)22, (char)198, (char)107, (char)84, (char)60, (char)75}, 0) ;
        p300.max_version_SET((char)56272) ;
        p300.min_version_SET((char)15451) ;
        LoopBackDemoChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.sub_mode_GET() == (char)91);
            assert(pack.time_usec_GET() == 4056895644010144962L);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE);
            assert(pack.vendor_specific_status_code_GET() == (char)63978);
            assert(pack.uptime_sec_GET() == 3497853396L);
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING);
        });
        DemoDevice.UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.uptime_sec_SET(3497853396L) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE) ;
        p310.vendor_specific_status_code_SET((char)63978) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING) ;
        p310.time_usec_SET(4056895644010144962L) ;
        p310.sub_mode_SET((char)91) ;
        LoopBackDemoChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.hw_version_major_GET() == (char)16);
            assert(pack.sw_vcs_commit_GET() == 508338363L);
            assert(pack.sw_version_major_GET() == (char)36);
            assert(pack.hw_version_minor_GET() == (char)22);
            assert(pack.name_LEN(ph) == 54);
            assert(pack.name_TRY(ph).equals("hhzVeqevbifybtqoqfnoqgqrpfzaBdxhckjyqkgkvPbzqcjizhzDii"));
            assert(pack.time_usec_GET() == 4035707416929110176L);
            assert(pack.uptime_sec_GET() == 2596989213L);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)119, (char)145, (char)201, (char)27, (char)232, (char)134, (char)1, (char)2, (char)197, (char)133, (char)121, (char)43, (char)30, (char)24, (char)242, (char)126}));
            assert(pack.sw_version_minor_GET() == (char)189);
        });
        DemoDevice.UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.hw_version_major_SET((char)16) ;
        p311.uptime_sec_SET(2596989213L) ;
        p311.sw_version_minor_SET((char)189) ;
        p311.sw_version_major_SET((char)36) ;
        p311.hw_unique_id_SET(new char[] {(char)119, (char)145, (char)201, (char)27, (char)232, (char)134, (char)1, (char)2, (char)197, (char)133, (char)121, (char)43, (char)30, (char)24, (char)242, (char)126}, 0) ;
        p311.sw_vcs_commit_SET(508338363L) ;
        p311.name_SET("hhzVeqevbifybtqoqfnoqgqrpfzaBdxhckjyqkgkvPbzqcjizhzDii", PH) ;
        p311.time_usec_SET(4035707416929110176L) ;
        p311.hw_version_minor_SET((char)22) ;
        LoopBackDemoChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)195);
            assert(pack.target_system_GET() == (char)31);
            assert(pack.param_index_GET() == (short)31536);
            assert(pack.param_id_LEN(ph) == 2);
            assert(pack.param_id_TRY(ph).equals("fz"));
        });
        DemoDevice.PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_system_SET((char)31) ;
        p320.target_component_SET((char)195) ;
        p320.param_id_SET("fz", PH) ;
        p320.param_index_SET((short)31536) ;
        LoopBackDemoChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)76);
            assert(pack.target_component_GET() == (char)240);
        });
        DemoDevice.PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)76) ;
        p321.target_component_SET((char)240) ;
        LoopBackDemoChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_index_GET() == (char)15899);
            assert(pack.param_count_GET() == (char)17774);
            assert(pack.param_id_LEN(ph) == 7);
            assert(pack.param_id_TRY(ph).equals("frfwlvl"));
            assert(pack.param_value_LEN(ph) == 123);
            assert(pack.param_value_TRY(ph).equals("HqtefdnfboctEoynfnebuksjwdplaafbjaAiiwsjyLvzvuxVgYccprwyoKwnsdmqlfasgsHubiafkejqkuwfaeymdrstelqpLmncohpmcaLqxstdnscmmgmrijq"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16);
        });
        DemoDevice.PARAM_EXT_VALUE p322 = LoopBackDemoChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16) ;
        p322.param_id_SET("frfwlvl", PH) ;
        p322.param_index_SET((char)15899) ;
        p322.param_count_SET((char)17774) ;
        p322.param_value_SET("HqtefdnfboctEoynfnebuksjwdplaafbjaAiiwsjyLvzvuxVgYccprwyoKwnsdmqlfasgsHubiafkejqkuwfaeymdrstelqpLmncohpmcaLqxstdnscmmgmrijq", PH) ;
        LoopBackDemoChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)189);
            assert(pack.param_value_LEN(ph) == 88);
            assert(pack.param_value_TRY(ph).equals("BftotsgqcjGtvnsmzbtrlthcybbtkujwazyQqtfihfoahEqihGkofjbdqhwjCuvyoddpnagadiuejuykqkbIsdtr"));
            assert(pack.param_id_LEN(ph) == 12);
            assert(pack.param_id_TRY(ph).equals("gkwzzriyghDf"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
            assert(pack.target_component_GET() == (char)23);
        });
        DemoDevice.PARAM_EXT_SET p323 = LoopBackDemoChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_component_SET((char)23) ;
        p323.param_value_SET("BftotsgqcjGtvnsmzbtrlthcybbtkujwazyQqtfihfoahEqihGkofjbdqhwjCuvyoddpnagadiuejuykqkbIsdtr", PH) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8) ;
        p323.param_id_SET("gkwzzriyghDf", PH) ;
        p323.target_system_SET((char)189) ;
        LoopBackDemoChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM);
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_FAILED);
            assert(pack.param_id_LEN(ph) == 9);
            assert(pack.param_id_TRY(ph).equals("mauujaoJl"));
            assert(pack.param_value_LEN(ph) == 12);
            assert(pack.param_value_TRY(ph).equals("oarfyibeljew"));
        });
        DemoDevice.PARAM_EXT_ACK p324 = LoopBackDemoChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM) ;
        p324.param_value_SET("oarfyibeljew", PH) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_FAILED) ;
        p324.param_id_SET("mauujaoJl", PH) ;
        LoopBackDemoChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)56657, (char)41778, (char)46006, (char)48561, (char)7408, (char)50218, (char)59729, (char)61451, (char)57240, (char)37759, (char)42473, (char)11900, (char)29157, (char)24839, (char)37256, (char)55719, (char)51404, (char)40352, (char)2153, (char)39417, (char)49071, (char)55712, (char)497, (char)51685, (char)23725, (char)53149, (char)53296, (char)13813, (char)65354, (char)32336, (char)46492, (char)22355, (char)51692, (char)40714, (char)14821, (char)8886, (char)4984, (char)1590, (char)54700, (char)62555, (char)49255, (char)62418, (char)45670, (char)1046, (char)10296, (char)37885, (char)51918, (char)35051, (char)1652, (char)3565, (char)18893, (char)47669, (char)55303, (char)35538, (char)49991, (char)4856, (char)25859, (char)19505, (char)36074, (char)20055, (char)63983, (char)62459, (char)5866, (char)35404, (char)37053, (char)25207, (char)48312, (char)4748, (char)36584, (char)52222, (char)56669, (char)58562}));
            assert(pack.increment_GET() == (char)209);
            assert(pack.max_distance_GET() == (char)15120);
            assert(pack.min_distance_GET() == (char)65413);
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
            assert(pack.time_usec_GET() == 3545390413207196634L);
        });
        DemoDevice.OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.min_distance_SET((char)65413) ;
        p330.time_usec_SET(3545390413207196634L) ;
        p330.increment_SET((char)209) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER) ;
        p330.distances_SET(new char[] {(char)56657, (char)41778, (char)46006, (char)48561, (char)7408, (char)50218, (char)59729, (char)61451, (char)57240, (char)37759, (char)42473, (char)11900, (char)29157, (char)24839, (char)37256, (char)55719, (char)51404, (char)40352, (char)2153, (char)39417, (char)49071, (char)55712, (char)497, (char)51685, (char)23725, (char)53149, (char)53296, (char)13813, (char)65354, (char)32336, (char)46492, (char)22355, (char)51692, (char)40714, (char)14821, (char)8886, (char)4984, (char)1590, (char)54700, (char)62555, (char)49255, (char)62418, (char)45670, (char)1046, (char)10296, (char)37885, (char)51918, (char)35051, (char)1652, (char)3565, (char)18893, (char)47669, (char)55303, (char)35538, (char)49991, (char)4856, (char)25859, (char)19505, (char)36074, (char)20055, (char)63983, (char)62459, (char)5866, (char)35404, (char)37053, (char)25207, (char)48312, (char)4748, (char)36584, (char)52222, (char)56669, (char)58562}, 0) ;
        p330.max_distance_SET((char)15120) ;
        LoopBackDemoChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
    }

}