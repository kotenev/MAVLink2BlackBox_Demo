
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
            assert(pack.mavlink_version_GET() == (char)36);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_RESERVED);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_CALIBRATING);
            assert(pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED);
            assert(pack.custom_mode_GET() == 3804217544L);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_FREE_BALLOON);
        });
        DemoDevice.HEARTBEAT p0 = LoopBackDemoChannel.new_HEARTBEAT();
        PH.setPack(p0);
        p0.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_CALIBRATING) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_RESERVED) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_FREE_BALLOON) ;
        p0.custom_mode_SET(3804217544L) ;
        p0.mavlink_version_SET((char)36) ;
        LoopBackDemoChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.errors_count1_GET() == (char)21010);
            assert(pack.onboard_control_sensors_enabled_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL);
            assert(pack.errors_comm_GET() == (char)9831);
            assert(pack.onboard_control_sensors_health_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING);
            assert(pack.current_battery_GET() == (short)14717);
            assert(pack.load_GET() == (char)1267);
            assert(pack.errors_count4_GET() == (char)49658);
            assert(pack.voltage_battery_GET() == (char)6271);
            assert(pack.errors_count2_GET() == (char)11867);
            assert(pack.drop_rate_comm_GET() == (char)38611);
            assert(pack.battery_remaining_GET() == (byte) - 56);
            assert(pack.errors_count3_GET() == (char)59667);
            assert(pack.onboard_control_sensors_present_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2);
        });
        DemoDevice.SYS_STATUS p1 = LoopBackDemoChannel.new_SYS_STATUS();
        PH.setPack(p1);
        p1.voltage_battery_SET((char)6271) ;
        p1.onboard_control_sensors_present_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2) ;
        p1.errors_count1_SET((char)21010) ;
        p1.errors_count3_SET((char)59667) ;
        p1.errors_count2_SET((char)11867) ;
        p1.load_SET((char)1267) ;
        p1.battery_remaining_SET((byte) - 56) ;
        p1.drop_rate_comm_SET((char)38611) ;
        p1.current_battery_SET((short)14717) ;
        p1.errors_count4_SET((char)49658) ;
        p1.errors_comm_SET((char)9831) ;
        p1.onboard_control_sensors_health_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING) ;
        p1.onboard_control_sensors_enabled_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL) ;
        LoopBackDemoChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_unix_usec_GET() == 83706561703371514L);
            assert(pack.time_boot_ms_GET() == 1528772028L);
        });
        DemoDevice.SYSTEM_TIME p2 = LoopBackDemoChannel.new_SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_boot_ms_SET(1528772028L) ;
        p2.time_unix_usec_SET(83706561703371514L) ;
        LoopBackDemoChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -1.0386436E38F);
            assert(pack.vz_GET() == 1.3595489E38F);
            assert(pack.yaw_GET() == -3.1612482E38F);
            assert(pack.vy_GET() == -1.7845243E38F);
            assert(pack.type_mask_GET() == (char)32430);
            assert(pack.y_GET() == 7.040555E37F);
            assert(pack.time_boot_ms_GET() == 772650316L);
            assert(pack.afy_GET() == -2.6833216E38F);
            assert(pack.vx_GET() == -1.1281865E38F);
            assert(pack.z_GET() == -2.6979755E38F);
            assert(pack.afx_GET() == -1.7574174E38F);
            assert(pack.yaw_rate_GET() == -3.089715E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            assert(pack.afz_GET() == 3.3129915E38F);
        });
        DemoDevice.POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT) ;
        p3.type_mask_SET((char)32430) ;
        p3.afx_SET(-1.7574174E38F) ;
        p3.vz_SET(1.3595489E38F) ;
        p3.afz_SET(3.3129915E38F) ;
        p3.z_SET(-2.6979755E38F) ;
        p3.x_SET(-1.0386436E38F) ;
        p3.yaw_SET(-3.1612482E38F) ;
        p3.afy_SET(-2.6833216E38F) ;
        p3.yaw_rate_SET(-3.089715E38F) ;
        p3.vy_SET(-1.7845243E38F) ;
        p3.vx_SET(-1.1281865E38F) ;
        p3.time_boot_ms_SET(772650316L) ;
        p3.y_SET(7.040555E37F) ;
        LoopBackDemoChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 5843687974872425353L);
            assert(pack.target_component_GET() == (char)55);
            assert(pack.seq_GET() == 1099914839L);
            assert(pack.target_system_GET() == (char)152);
        });
        DemoDevice.PING p4 = LoopBackDemoChannel.new_PING();
        PH.setPack(p4);
        p4.target_system_SET((char)152) ;
        p4.seq_SET(1099914839L) ;
        p4.target_component_SET((char)55) ;
        p4.time_usec_SET(5843687974872425353L) ;
        LoopBackDemoChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.control_request_GET() == (char)160);
            assert(pack.version_GET() == (char)65);
            assert(pack.target_system_GET() == (char)185);
            assert(pack.passkey_LEN(ph) == 10);
            assert(pack.passkey_TRY(ph).equals("MeIsulklkv"));
        });
        DemoDevice.CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.control_request_SET((char)160) ;
        p5.passkey_SET("MeIsulklkv", PH) ;
        p5.target_system_SET((char)185) ;
        p5.version_SET((char)65) ;
        LoopBackDemoChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.gcs_system_id_GET() == (char)7);
            assert(pack.ack_GET() == (char)219);
            assert(pack.control_request_GET() == (char)103);
        });
        DemoDevice.CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.ack_SET((char)219) ;
        p6.control_request_SET((char)103) ;
        p6.gcs_system_id_SET((char)7) ;
        LoopBackDemoChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 23);
            assert(pack.key_TRY(ph).equals("gdqeqqpttrtdkxngxczgogp"));
        });
        DemoDevice.AUTH_KEY p7 = LoopBackDemoChannel.new_AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("gdqeqqpttrtdkxngxczgogp", PH) ;
        LoopBackDemoChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
            assert(pack.target_system_GET() == (char)180);
            assert(pack.custom_mode_GET() == 90756116L);
        });
        DemoDevice.SET_MODE p11 = LoopBackDemoChannel.new_SET_MODE();
        PH.setPack(p11);
        p11.custom_mode_SET(90756116L) ;
        p11.target_system_SET((char)180) ;
        p11.base_mode_SET(MAV_MODE.MAV_MODE_STABILIZE_DISARMED) ;
        LoopBackDemoChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)226);
            assert(pack.target_component_GET() == (char)129);
            assert(pack.param_index_GET() == (short)11972);
            assert(pack.param_id_LEN(ph) == 10);
            assert(pack.param_id_TRY(ph).equals("iuweovxquj"));
        });
        DemoDevice.PARAM_REQUEST_READ p20 = LoopBackDemoChannel.new_PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.param_index_SET((short)11972) ;
        p20.param_id_SET("iuweovxquj", PH) ;
        p20.target_system_SET((char)226) ;
        p20.target_component_SET((char)129) ;
        LoopBackDemoChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)18);
            assert(pack.target_component_GET() == (char)169);
        });
        DemoDevice.PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.new_PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_component_SET((char)169) ;
        p21.target_system_SET((char)18) ;
        LoopBackDemoChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_index_GET() == (char)29913);
            assert(pack.param_value_GET() == 2.9305755E38F);
            assert(pack.param_count_GET() == (char)375);
            assert(pack.param_id_LEN(ph) == 5);
            assert(pack.param_id_TRY(ph).equals("dwyyg"));
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8);
        });
        DemoDevice.PARAM_VALUE p22 = LoopBackDemoChannel.new_PARAM_VALUE();
        PH.setPack(p22);
        p22.param_count_SET((char)375) ;
        p22.param_value_SET(2.9305755E38F) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8) ;
        p22.param_index_SET((char)29913) ;
        p22.param_id_SET("dwyyg", PH) ;
        LoopBackDemoChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 8);
            assert(pack.param_id_TRY(ph).equals("nsqasifq"));
            assert(pack.param_value_GET() == -5.633952E36F);
            assert(pack.target_component_GET() == (char)121);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8);
            assert(pack.target_system_GET() == (char)75);
        });
        DemoDevice.PARAM_SET p23 = LoopBackDemoChannel.new_PARAM_SET();
        PH.setPack(p23);
        p23.target_component_SET((char)121) ;
        p23.target_system_SET((char)75) ;
        p23.param_value_SET(-5.633952E36F) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8) ;
        p23.param_id_SET("nsqasifq", PH) ;
        LoopBackDemoChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.epv_GET() == (char)62519);
            assert(pack.vel_acc_TRY(ph) == 4193895069L);
            assert(pack.time_usec_GET() == 5751524182490975351L);
            assert(pack.lon_GET() == 1987883162);
            assert(pack.eph_GET() == (char)59171);
            assert(pack.h_acc_TRY(ph) == 1723916683L);
            assert(pack.hdg_acc_TRY(ph) == 4109750972L);
            assert(pack.satellites_visible_GET() == (char)47);
            assert(pack.vel_GET() == (char)28603);
            assert(pack.v_acc_TRY(ph) == 268907364L);
            assert(pack.alt_GET() == 577073060);
            assert(pack.lat_GET() == 1027132780);
            assert(pack.alt_ellipsoid_TRY(ph) == -1787393749);
            assert(pack.cog_GET() == (char)35055);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED);
        });
        DemoDevice.GPS_RAW_INT p24 = LoopBackDemoChannel.new_GPS_RAW_INT();
        PH.setPack(p24);
        p24.lon_SET(1987883162) ;
        p24.cog_SET((char)35055) ;
        p24.vel_acc_SET(4193895069L, PH) ;
        p24.alt_SET(577073060) ;
        p24.lat_SET(1027132780) ;
        p24.time_usec_SET(5751524182490975351L) ;
        p24.v_acc_SET(268907364L, PH) ;
        p24.alt_ellipsoid_SET(-1787393749, PH) ;
        p24.hdg_acc_SET(4109750972L, PH) ;
        p24.epv_SET((char)62519) ;
        p24.eph_SET((char)59171) ;
        p24.vel_SET((char)28603) ;
        p24.h_acc_SET(1723916683L, PH) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED) ;
        p24.satellites_visible_SET((char)47) ;
        LoopBackDemoChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)35, (char)189, (char)234, (char)251, (char)76, (char)15, (char)153, (char)16, (char)190, (char)42, (char)174, (char)252, (char)55, (char)147, (char)39, (char)237, (char)197, (char)182, (char)197, (char)53}));
            assert(pack.satellites_visible_GET() == (char)45);
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)255, (char)97, (char)193, (char)0, (char)81, (char)14, (char)140, (char)182, (char)87, (char)230, (char)13, (char)38, (char)5, (char)134, (char)230, (char)242, (char)113, (char)168, (char)159, (char)214}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)247, (char)111, (char)177, (char)189, (char)204, (char)120, (char)147, (char)83, (char)46, (char)148, (char)201, (char)69, (char)188, (char)130, (char)13, (char)31, (char)18, (char)19, (char)138, (char)140}));
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)169, (char)11, (char)233, (char)117, (char)216, (char)76, (char)113, (char)41, (char)42, (char)128, (char)7, (char)227, (char)60, (char)118, (char)32, (char)10, (char)35, (char)225, (char)238, (char)49}));
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)181, (char)139, (char)201, (char)58, (char)167, (char)56, (char)162, (char)234, (char)61, (char)92, (char)4, (char)18, (char)125, (char)75, (char)37, (char)212, (char)53, (char)157, (char)206, (char)129}));
        });
        DemoDevice.GPS_STATUS p25 = LoopBackDemoChannel.new_GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_used_SET(new char[] {(char)35, (char)189, (char)234, (char)251, (char)76, (char)15, (char)153, (char)16, (char)190, (char)42, (char)174, (char)252, (char)55, (char)147, (char)39, (char)237, (char)197, (char)182, (char)197, (char)53}, 0) ;
        p25.satellite_snr_SET(new char[] {(char)181, (char)139, (char)201, (char)58, (char)167, (char)56, (char)162, (char)234, (char)61, (char)92, (char)4, (char)18, (char)125, (char)75, (char)37, (char)212, (char)53, (char)157, (char)206, (char)129}, 0) ;
        p25.satellite_azimuth_SET(new char[] {(char)247, (char)111, (char)177, (char)189, (char)204, (char)120, (char)147, (char)83, (char)46, (char)148, (char)201, (char)69, (char)188, (char)130, (char)13, (char)31, (char)18, (char)19, (char)138, (char)140}, 0) ;
        p25.satellite_elevation_SET(new char[] {(char)255, (char)97, (char)193, (char)0, (char)81, (char)14, (char)140, (char)182, (char)87, (char)230, (char)13, (char)38, (char)5, (char)134, (char)230, (char)242, (char)113, (char)168, (char)159, (char)214}, 0) ;
        p25.satellites_visible_SET((char)45) ;
        p25.satellite_prn_SET(new char[] {(char)169, (char)11, (char)233, (char)117, (char)216, (char)76, (char)113, (char)41, (char)42, (char)128, (char)7, (char)227, (char)60, (char)118, (char)32, (char)10, (char)35, (char)225, (char)238, (char)49}, 0) ;
        LoopBackDemoChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == (short)13955);
            assert(pack.zmag_GET() == (short) -7130);
            assert(pack.ymag_GET() == (short) -13222);
            assert(pack.xmag_GET() == (short) -1293);
            assert(pack.zacc_GET() == (short)31016);
            assert(pack.ygyro_GET() == (short)11545);
            assert(pack.zgyro_GET() == (short)24978);
            assert(pack.yacc_GET() == (short)16181);
            assert(pack.xgyro_GET() == (short)12670);
            assert(pack.time_boot_ms_GET() == 3977183092L);
        });
        DemoDevice.SCALED_IMU p26 = LoopBackDemoChannel.new_SCALED_IMU();
        PH.setPack(p26);
        p26.zmag_SET((short) -7130) ;
        p26.time_boot_ms_SET(3977183092L) ;
        p26.zgyro_SET((short)24978) ;
        p26.ymag_SET((short) -13222) ;
        p26.ygyro_SET((short)11545) ;
        p26.yacc_SET((short)16181) ;
        p26.xmag_SET((short) -1293) ;
        p26.xgyro_SET((short)12670) ;
        p26.zacc_SET((short)31016) ;
        p26.xacc_SET((short)13955) ;
        LoopBackDemoChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == (short)19147);
            assert(pack.time_usec_GET() == 8159231857229662415L);
            assert(pack.xgyro_GET() == (short)1917);
            assert(pack.ymag_GET() == (short) -21703);
            assert(pack.zacc_GET() == (short) -7984);
            assert(pack.xacc_GET() == (short) -803);
            assert(pack.zmag_GET() == (short)24985);
            assert(pack.yacc_GET() == (short)5484);
            assert(pack.ygyro_GET() == (short)11878);
            assert(pack.xmag_GET() == (short)12013);
        });
        DemoDevice.RAW_IMU p27 = LoopBackDemoChannel.new_RAW_IMU();
        PH.setPack(p27);
        p27.ygyro_SET((short)11878) ;
        p27.ymag_SET((short) -21703) ;
        p27.xgyro_SET((short)1917) ;
        p27.zacc_SET((short) -7984) ;
        p27.time_usec_SET(8159231857229662415L) ;
        p27.yacc_SET((short)5484) ;
        p27.zgyro_SET((short)19147) ;
        p27.zmag_SET((short)24985) ;
        p27.xacc_SET((short) -803) ;
        p27.xmag_SET((short)12013) ;
        LoopBackDemoChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == (short)9065);
            assert(pack.time_usec_GET() == 7007246621701035732L);
            assert(pack.press_diff1_GET() == (short) -14677);
            assert(pack.temperature_GET() == (short)1220);
            assert(pack.press_diff2_GET() == (short) -1932);
        });
        DemoDevice.RAW_PRESSURE p28 = LoopBackDemoChannel.new_RAW_PRESSURE();
        PH.setPack(p28);
        p28.time_usec_SET(7007246621701035732L) ;
        p28.temperature_SET((short)1220) ;
        p28.press_abs_SET((short)9065) ;
        p28.press_diff1_SET((short) -14677) ;
        p28.press_diff2_SET((short) -1932) ;
        LoopBackDemoChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1074040517L);
            assert(pack.temperature_GET() == (short)18768);
            assert(pack.press_diff_GET() == 2.2115518E38F);
            assert(pack.press_abs_GET() == 1.4843773E38F);
        });
        DemoDevice.SCALED_PRESSURE p29 = LoopBackDemoChannel.new_SCALED_PRESSURE();
        PH.setPack(p29);
        p29.press_abs_SET(1.4843773E38F) ;
        p29.time_boot_ms_SET(1074040517L) ;
        p29.press_diff_SET(2.2115518E38F) ;
        p29.temperature_SET((short)18768) ;
        LoopBackDemoChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 643321577L);
            assert(pack.pitchspeed_GET() == 3.420571E37F);
            assert(pack.rollspeed_GET() == -1.953214E38F);
            assert(pack.yaw_GET() == 2.6211901E38F);
            assert(pack.roll_GET() == 2.2146247E38F);
            assert(pack.yawspeed_GET() == -6.465884E36F);
            assert(pack.pitch_GET() == 8.1762627E37F);
        });
        DemoDevice.ATTITUDE p30 = LoopBackDemoChannel.new_ATTITUDE();
        PH.setPack(p30);
        p30.time_boot_ms_SET(643321577L) ;
        p30.rollspeed_SET(-1.953214E38F) ;
        p30.roll_SET(2.2146247E38F) ;
        p30.pitchspeed_SET(3.420571E37F) ;
        p30.yawspeed_SET(-6.465884E36F) ;
        p30.pitch_SET(8.1762627E37F) ;
        p30.yaw_SET(2.6211901E38F) ;
        LoopBackDemoChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.yawspeed_GET() == -2.2582725E38F);
            assert(pack.q3_GET() == -2.8716633E38F);
            assert(pack.rollspeed_GET() == 6.672698E37F);
            assert(pack.q1_GET() == -1.6216242E38F);
            assert(pack.pitchspeed_GET() == 1.2623124E38F);
            assert(pack.q4_GET() == 2.5492912E38F);
            assert(pack.time_boot_ms_GET() == 279537085L);
            assert(pack.q2_GET() == 1.6310706E38F);
        });
        DemoDevice.ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.rollspeed_SET(6.672698E37F) ;
        p31.pitchspeed_SET(1.2623124E38F) ;
        p31.yawspeed_SET(-2.2582725E38F) ;
        p31.time_boot_ms_SET(279537085L) ;
        p31.q4_SET(2.5492912E38F) ;
        p31.q3_SET(-2.8716633E38F) ;
        p31.q2_SET(1.6310706E38F) ;
        p31.q1_SET(-1.6216242E38F) ;
        LoopBackDemoChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 3.2088374E38F);
            assert(pack.vz_GET() == -1.914658E38F);
            assert(pack.z_GET() == 3.0207019E38F);
            assert(pack.vy_GET() == -5.909062E37F);
            assert(pack.vx_GET() == -1.5274297E38F);
            assert(pack.time_boot_ms_GET() == 985410950L);
            assert(pack.x_GET() == -3.1180347E38F);
        });
        DemoDevice.LOCAL_POSITION_NED p32 = LoopBackDemoChannel.new_LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.time_boot_ms_SET(985410950L) ;
        p32.vx_SET(-1.5274297E38F) ;
        p32.y_SET(3.2088374E38F) ;
        p32.z_SET(3.0207019E38F) ;
        p32.vz_SET(-1.914658E38F) ;
        p32.x_SET(-3.1180347E38F) ;
        p32.vy_SET(-5.909062E37F) ;
        LoopBackDemoChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == (short) -29415);
            assert(pack.lon_GET() == 1227529594);
            assert(pack.relative_alt_GET() == -1964069048);
            assert(pack.alt_GET() == 1004290900);
            assert(pack.lat_GET() == 1988933338);
            assert(pack.time_boot_ms_GET() == 3189954210L);
            assert(pack.vy_GET() == (short) -28528);
            assert(pack.vx_GET() == (short) -15072);
            assert(pack.hdg_GET() == (char)28392);
        });
        DemoDevice.GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.vz_SET((short) -29415) ;
        p33.lon_SET(1227529594) ;
        p33.hdg_SET((char)28392) ;
        p33.alt_SET(1004290900) ;
        p33.vy_SET((short) -28528) ;
        p33.vx_SET((short) -15072) ;
        p33.relative_alt_SET(-1964069048) ;
        p33.lat_SET(1988933338) ;
        p33.time_boot_ms_SET(3189954210L) ;
        LoopBackDemoChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.port_GET() == (char)242);
            assert(pack.rssi_GET() == (char)122);
            assert(pack.chan5_scaled_GET() == (short)17946);
            assert(pack.time_boot_ms_GET() == 3776913747L);
            assert(pack.chan1_scaled_GET() == (short)7384);
            assert(pack.chan6_scaled_GET() == (short) -18672);
            assert(pack.chan3_scaled_GET() == (short) -26708);
            assert(pack.chan2_scaled_GET() == (short) -24387);
            assert(pack.chan8_scaled_GET() == (short) -8068);
            assert(pack.chan7_scaled_GET() == (short) -30256);
            assert(pack.chan4_scaled_GET() == (short) -8685);
        });
        DemoDevice.RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.new_RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.chan6_scaled_SET((short) -18672) ;
        p34.chan5_scaled_SET((short)17946) ;
        p34.port_SET((char)242) ;
        p34.chan2_scaled_SET((short) -24387) ;
        p34.chan4_scaled_SET((short) -8685) ;
        p34.time_boot_ms_SET(3776913747L) ;
        p34.rssi_SET((char)122) ;
        p34.chan7_scaled_SET((short) -30256) ;
        p34.chan8_scaled_SET((short) -8068) ;
        p34.chan1_scaled_SET((short)7384) ;
        p34.chan3_scaled_SET((short) -26708) ;
        LoopBackDemoChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan1_raw_GET() == (char)39074);
            assert(pack.rssi_GET() == (char)58);
            assert(pack.port_GET() == (char)170);
            assert(pack.chan5_raw_GET() == (char)29708);
            assert(pack.chan8_raw_GET() == (char)1119);
            assert(pack.time_boot_ms_GET() == 1263309698L);
            assert(pack.chan4_raw_GET() == (char)16556);
            assert(pack.chan3_raw_GET() == (char)35845);
            assert(pack.chan6_raw_GET() == (char)58020);
            assert(pack.chan7_raw_GET() == (char)23014);
            assert(pack.chan2_raw_GET() == (char)51031);
        });
        DemoDevice.RC_CHANNELS_RAW p35 = LoopBackDemoChannel.new_RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.port_SET((char)170) ;
        p35.chan7_raw_SET((char)23014) ;
        p35.chan1_raw_SET((char)39074) ;
        p35.chan5_raw_SET((char)29708) ;
        p35.chan4_raw_SET((char)16556) ;
        p35.chan6_raw_SET((char)58020) ;
        p35.rssi_SET((char)58) ;
        p35.chan2_raw_SET((char)51031) ;
        p35.chan3_raw_SET((char)35845) ;
        p35.time_boot_ms_SET(1263309698L) ;
        p35.chan8_raw_SET((char)1119) ;
        LoopBackDemoChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo3_raw_GET() == (char)42494);
            assert(pack.servo9_raw_TRY(ph) == (char)55311);
            assert(pack.servo11_raw_TRY(ph) == (char)35520);
            assert(pack.time_usec_GET() == 2779173752L);
            assert(pack.servo2_raw_GET() == (char)53009);
            assert(pack.servo14_raw_TRY(ph) == (char)57799);
            assert(pack.servo1_raw_GET() == (char)41687);
            assert(pack.servo4_raw_GET() == (char)37879);
            assert(pack.servo5_raw_GET() == (char)10395);
            assert(pack.servo15_raw_TRY(ph) == (char)20858);
            assert(pack.servo13_raw_TRY(ph) == (char)45595);
            assert(pack.servo8_raw_GET() == (char)7867);
            assert(pack.servo7_raw_GET() == (char)48240);
            assert(pack.servo10_raw_TRY(ph) == (char)16388);
            assert(pack.servo16_raw_TRY(ph) == (char)45057);
            assert(pack.servo12_raw_TRY(ph) == (char)21762);
            assert(pack.port_GET() == (char)241);
            assert(pack.servo6_raw_GET() == (char)49278);
        });
        DemoDevice.SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.new_SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo10_raw_SET((char)16388, PH) ;
        p36.servo15_raw_SET((char)20858, PH) ;
        p36.servo16_raw_SET((char)45057, PH) ;
        p36.servo13_raw_SET((char)45595, PH) ;
        p36.port_SET((char)241) ;
        p36.servo8_raw_SET((char)7867) ;
        p36.servo1_raw_SET((char)41687) ;
        p36.servo7_raw_SET((char)48240) ;
        p36.servo14_raw_SET((char)57799, PH) ;
        p36.servo5_raw_SET((char)10395) ;
        p36.servo4_raw_SET((char)37879) ;
        p36.servo2_raw_SET((char)53009) ;
        p36.servo12_raw_SET((char)21762, PH) ;
        p36.time_usec_SET(2779173752L) ;
        p36.servo11_raw_SET((char)35520, PH) ;
        p36.servo9_raw_SET((char)55311, PH) ;
        p36.servo6_raw_SET((char)49278) ;
        p36.servo3_raw_SET((char)42494) ;
        LoopBackDemoChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.start_index_GET() == (short) -31291);
            assert(pack.target_component_GET() == (char)158);
            assert(pack.end_index_GET() == (short)29243);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_system_GET() == (char)158);
        });
        DemoDevice.MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.new_MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.end_index_SET((short)29243) ;
        p37.start_index_SET((short) -31291) ;
        p37.target_system_SET((char)158) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p37.target_component_SET((char)158) ;
        LoopBackDemoChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)19);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.end_index_GET() == (short)2476);
            assert(pack.target_system_GET() == (char)139);
            assert(pack.start_index_GET() == (short)17450);
        });
        DemoDevice.MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.new_MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.target_system_SET((char)139) ;
        p38.end_index_SET((short)2476) ;
        p38.start_index_SET((short)17450) ;
        p38.target_component_SET((char)19) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        LoopBackDemoChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -9.43347E37F);
            assert(pack.param2_GET() == -3.079981E38F);
            assert(pack.param4_GET() == 2.8487628E38F);
            assert(pack.target_component_GET() == (char)27);
            assert(pack.param3_GET() == -8.558472E37F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.z_GET() == 1.9279339E38F);
            assert(pack.current_GET() == (char)143);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_USER_2);
            assert(pack.param1_GET() == -3.1203043E38F);
            assert(pack.seq_GET() == (char)63304);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.x_GET() == 2.9860668E38F);
            assert(pack.target_system_GET() == (char)221);
            assert(pack.autocontinue_GET() == (char)175);
        });
        DemoDevice.MISSION_ITEM p39 = LoopBackDemoChannel.new_MISSION_ITEM();
        PH.setPack(p39);
        p39.y_SET(-9.43347E37F) ;
        p39.param4_SET(2.8487628E38F) ;
        p39.target_system_SET((char)221) ;
        p39.target_component_SET((char)27) ;
        p39.autocontinue_SET((char)175) ;
        p39.x_SET(2.9860668E38F) ;
        p39.z_SET(1.9279339E38F) ;
        p39.current_SET((char)143) ;
        p39.param3_SET(-8.558472E37F) ;
        p39.param1_SET(-3.1203043E38F) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p39.seq_SET((char)63304) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p39.command_SET(MAV_CMD.MAV_CMD_USER_2) ;
        p39.param2_SET(-3.079981E38F) ;
        LoopBackDemoChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)102);
            assert(pack.seq_GET() == (char)50000);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_component_GET() == (char)246);
        });
        DemoDevice.MISSION_REQUEST p40 = LoopBackDemoChannel.new_MISSION_REQUEST();
        PH.setPack(p40);
        p40.target_system_SET((char)102) ;
        p40.seq_SET((char)50000) ;
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p40.target_component_SET((char)246) ;
        LoopBackDemoChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)53);
            assert(pack.target_component_GET() == (char)56);
            assert(pack.seq_GET() == (char)31215);
        });
        DemoDevice.MISSION_SET_CURRENT p41 = LoopBackDemoChannel.new_MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_component_SET((char)56) ;
        p41.seq_SET((char)31215) ;
        p41.target_system_SET((char)53) ;
        LoopBackDemoChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)18230);
        });
        DemoDevice.MISSION_CURRENT p42 = LoopBackDemoChannel.new_MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)18230) ;
        LoopBackDemoChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)218);
            assert(pack.target_system_GET() == (char)240);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        });
        DemoDevice.MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.new_MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_component_SET((char)218) ;
        p43.target_system_SET((char)240) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        LoopBackDemoChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)6077);
            assert(pack.target_system_GET() == (char)103);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_component_GET() == (char)69);
        });
        DemoDevice.MISSION_COUNT p44 = LoopBackDemoChannel.new_MISSION_COUNT();
        PH.setPack(p44);
        p44.target_component_SET((char)69) ;
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p44.count_SET((char)6077) ;
        p44.target_system_SET((char)103) ;
        LoopBackDemoChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)25);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_component_GET() == (char)57);
        });
        DemoDevice.MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.new_MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_component_SET((char)57) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p45.target_system_SET((char)25) ;
        LoopBackDemoChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)64159);
        });
        DemoDevice.MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.new_MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)64159) ;
        LoopBackDemoChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED);
            assert(pack.target_system_GET() == (char)96);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_component_GET() == (char)116);
        });
        DemoDevice.MISSION_ACK p47 = LoopBackDemoChannel.new_MISSION_ACK();
        PH.setPack(p47);
        p47.target_system_SET((char)96) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED) ;
        p47.target_component_SET((char)116) ;
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        LoopBackDemoChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == -1702987222);
            assert(pack.time_usec_TRY(ph) == 3405036355145186397L);
            assert(pack.altitude_GET() == -720328571);
            assert(pack.latitude_GET() == -938287296);
            assert(pack.target_system_GET() == (char)110);
        });
        DemoDevice.SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.new_SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.latitude_SET(-938287296) ;
        p48.time_usec_SET(3405036355145186397L, PH) ;
        p48.altitude_SET(-720328571) ;
        p48.longitude_SET(-1702987222) ;
        p48.target_system_SET((char)110) ;
        LoopBackDemoChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == -2005337605);
            assert(pack.time_usec_TRY(ph) == 2268680237109235462L);
            assert(pack.longitude_GET() == 134855695);
            assert(pack.altitude_GET() == -428784477);
        });
        DemoDevice.GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.new_GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.time_usec_SET(2268680237109235462L, PH) ;
        p49.latitude_SET(-2005337605) ;
        p49.altitude_SET(-428784477) ;
        p49.longitude_SET(134855695) ;
        LoopBackDemoChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.param_value_max_GET() == -1.1154677E38F);
            assert(pack.parameter_rc_channel_index_GET() == (char)225);
            assert(pack.param_index_GET() == (short)26850);
            assert(pack.param_value0_GET() == -2.5109532E38F);
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("ours"));
            assert(pack.target_system_GET() == (char)6);
            assert(pack.param_value_min_GET() == -3.2737048E38F);
            assert(pack.scale_GET() == -1.8926642E38F);
            assert(pack.target_component_GET() == (char)50);
        });
        DemoDevice.PARAM_MAP_RC p50 = LoopBackDemoChannel.new_PARAM_MAP_RC();
        PH.setPack(p50);
        p50.param_index_SET((short)26850) ;
        p50.param_id_SET("ours", PH) ;
        p50.param_value0_SET(-2.5109532E38F) ;
        p50.target_component_SET((char)50) ;
        p50.param_value_max_SET(-1.1154677E38F) ;
        p50.param_value_min_SET(-3.2737048E38F) ;
        p50.parameter_rc_channel_index_SET((char)225) ;
        p50.scale_SET(-1.8926642E38F) ;
        p50.target_system_SET((char)6) ;
        LoopBackDemoChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)7);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_system_GET() == (char)203);
            assert(pack.seq_GET() == (char)19904);
        });
        DemoDevice.MISSION_REQUEST_INT p51 = LoopBackDemoChannel.new_MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p51.target_component_SET((char)7) ;
        p51.target_system_SET((char)203) ;
        p51.seq_SET((char)19904) ;
        LoopBackDemoChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)133);
            assert(pack.p2z_GET() == 1.996389E38F);
            assert(pack.p2x_GET() == 2.745394E38F);
            assert(pack.p2y_GET() == -1.6620681E38F);
            assert(pack.target_system_GET() == (char)69);
            assert(pack.p1z_GET() == -3.1470558E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.p1x_GET() == 1.9197903E38F);
            assert(pack.p1y_GET() == -1.9866959E38F);
        });
        DemoDevice.SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.new_SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.p1z_SET(-3.1470558E38F) ;
        p54.target_system_SET((char)69) ;
        p54.p2x_SET(2.745394E38F) ;
        p54.p2y_SET(-1.6620681E38F) ;
        p54.p2z_SET(1.996389E38F) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p54.p1x_SET(1.9197903E38F) ;
        p54.target_component_SET((char)133) ;
        p54.p1y_SET(-1.9866959E38F) ;
        LoopBackDemoChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p1z_GET() == -1.7659458E38F);
            assert(pack.p2z_GET() == -5.8287873E37F);
            assert(pack.p2y_GET() == -9.610355E37F);
            assert(pack.p1y_GET() == -1.1177146E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.p1x_GET() == 3.4021656E37F);
            assert(pack.p2x_GET() == -2.17139E38F);
        });
        DemoDevice.SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.new_SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p1y_SET(-1.1177146E38F) ;
        p55.p2y_SET(-9.610355E37F) ;
        p55.p1x_SET(3.4021656E37F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p55.p2z_SET(-5.8287873E37F) ;
        p55.p1z_SET(-1.7659458E38F) ;
        p55.p2x_SET(-2.17139E38F) ;
        LoopBackDemoChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.723572E38F, 2.8958525E38F, 1.1435612E38F, -5.4831126E37F}));
            assert(pack.yawspeed_GET() == -3.2125012E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {4.373231E37F, 2.760854E38F, 1.6886955E38F, -2.6830571E38F, -8.181636E37F, -2.8078753E38F, -3.0128479E38F, -3.1453229E38F, 1.0265111E38F}));
            assert(pack.time_usec_GET() == 6261896970748301005L);
            assert(pack.pitchspeed_GET() == 2.2936965E38F);
            assert(pack.rollspeed_GET() == -3.2225157E38F);
        });
        DemoDevice.ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.pitchspeed_SET(2.2936965E38F) ;
        p61.rollspeed_SET(-3.2225157E38F) ;
        p61.time_usec_SET(6261896970748301005L) ;
        p61.covariance_SET(new float[] {4.373231E37F, 2.760854E38F, 1.6886955E38F, -2.6830571E38F, -8.181636E37F, -2.8078753E38F, -3.0128479E38F, -3.1453229E38F, 1.0265111E38F}, 0) ;
        p61.q_SET(new float[] {-1.723572E38F, 2.8958525E38F, 1.1435612E38F, -5.4831126E37F}, 0) ;
        p61.yawspeed_SET(-3.2125012E38F) ;
        LoopBackDemoChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.nav_bearing_GET() == (short)6226);
            assert(pack.aspd_error_GET() == -9.033565E37F);
            assert(pack.alt_error_GET() == 1.9423304E38F);
            assert(pack.nav_pitch_GET() == -2.6068882E38F);
            assert(pack.wp_dist_GET() == (char)4980);
            assert(pack.nav_roll_GET() == -5.6914693E37F);
            assert(pack.xtrack_error_GET() == -1.5307355E38F);
            assert(pack.target_bearing_GET() == (short) -23215);
        });
        DemoDevice.NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.new_NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.nav_pitch_SET(-2.6068882E38F) ;
        p62.wp_dist_SET((char)4980) ;
        p62.nav_roll_SET(-5.6914693E37F) ;
        p62.aspd_error_SET(-9.033565E37F) ;
        p62.xtrack_error_SET(-1.5307355E38F) ;
        p62.nav_bearing_SET((short)6226) ;
        p62.target_bearing_SET((short) -23215) ;
        p62.alt_error_SET(1.9423304E38F) ;
        LoopBackDemoChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-8.2685775E36F, 8.52503E37F, 2.5787713E38F, -2.539462E38F, 1.0469473E38F, 5.492622E37F, -2.9107055E38F, 2.8907427E38F, -5.814046E37F, 2.2244502E38F, 1.2528372E38F, -1.9568177E38F, -6.2586674E37F, 3.05743E38F, 9.816046E37F, 4.3172437E37F, 7.27886E37F, -1.3679488E38F, 1.453474E38F, -2.5134597E38F, -1.4613672E38F, -2.008692E38F, 2.7153175E38F, -2.5497796E38F, 2.2776996E38F, -8.823272E37F, 3.1277634E38F, 1.7933877E38F, -1.5217374E38F, 2.2172499E38F, 3.2497677E38F, 2.5657356E38F, -4.026455E37F, 4.3923145E37F, 5.0614962E36F, -8.526312E37F}));
            assert(pack.vx_GET() == -1.5113215E38F);
            assert(pack.lat_GET() == 848501935);
            assert(pack.time_usec_GET() == 7480289367154095440L);
            assert(pack.vz_GET() == 3.183029E38F);
            assert(pack.vy_GET() == -8.4527003E37F);
            assert(pack.lon_GET() == -1710267278);
            assert(pack.relative_alt_GET() == -737057056);
            assert(pack.alt_GET() == 154906382);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE);
        });
        DemoDevice.GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.lon_SET(-1710267278) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE) ;
        p63.vy_SET(-8.4527003E37F) ;
        p63.relative_alt_SET(-737057056) ;
        p63.lat_SET(848501935) ;
        p63.covariance_SET(new float[] {-8.2685775E36F, 8.52503E37F, 2.5787713E38F, -2.539462E38F, 1.0469473E38F, 5.492622E37F, -2.9107055E38F, 2.8907427E38F, -5.814046E37F, 2.2244502E38F, 1.2528372E38F, -1.9568177E38F, -6.2586674E37F, 3.05743E38F, 9.816046E37F, 4.3172437E37F, 7.27886E37F, -1.3679488E38F, 1.453474E38F, -2.5134597E38F, -1.4613672E38F, -2.008692E38F, 2.7153175E38F, -2.5497796E38F, 2.2776996E38F, -8.823272E37F, 3.1277634E38F, 1.7933877E38F, -1.5217374E38F, 2.2172499E38F, 3.2497677E38F, 2.5657356E38F, -4.026455E37F, 4.3923145E37F, 5.0614962E36F, -8.526312E37F}, 0) ;
        p63.vx_SET(-1.5113215E38F) ;
        p63.time_usec_SET(7480289367154095440L) ;
        p63.alt_SET(154906382) ;
        p63.vz_SET(3.183029E38F) ;
        LoopBackDemoChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == -2.5632362E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-1.8717127E38F, -2.87363E38F, -2.573491E38F, -2.957232E38F, -2.9966222E38F, -1.5752378E38F, 1.6993071E38F, -6.1095445E37F, 1.9244403E38F, 3.1063683E38F, -1.5667059E38F, 3.263042E38F, 2.288599E38F, 2.667129E38F, -1.8041313E38F, -1.9551866E38F, -3.2264509E38F, 2.954989E38F, -3.0428472E38F, 5.010885E37F, 6.983358E37F, 3.389791E38F, -9.01843E37F, 3.3335807E38F, -1.0687713E38F, -1.9696966E38F, 1.1522879E37F, 2.6375108E38F, 4.428771E37F, -3.346712E38F, 7.865562E37F, 2.114214E38F, 9.719061E37F, -1.2143479E38F, 2.263022E38F, -1.4263706E38F, 1.9282294E38F, 1.4214931E38F, -1.880382E38F, 5.9614505E37F, 2.7396954E38F, 3.3071448E38F, -1.0500729E38F, -1.1773099E38F, 7.233801E37F}));
            assert(pack.y_GET() == 3.3576663E38F);
            assert(pack.ax_GET() == 1.4797333E38F);
            assert(pack.az_GET() == 2.263165E38F);
            assert(pack.z_GET() == -2.0696627E38F);
            assert(pack.ay_GET() == -2.8741282E38F);
            assert(pack.time_usec_GET() == 168305957821285432L);
            assert(pack.vz_GET() == -1.0015068E38F);
            assert(pack.x_GET() == -1.1225476E38F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
            assert(pack.vx_GET() == 1.61865E38F);
        });
        DemoDevice.LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.vz_SET(-1.0015068E38F) ;
        p64.ay_SET(-2.8741282E38F) ;
        p64.ax_SET(1.4797333E38F) ;
        p64.x_SET(-1.1225476E38F) ;
        p64.time_usec_SET(168305957821285432L) ;
        p64.vy_SET(-2.5632362E38F) ;
        p64.covariance_SET(new float[] {-1.8717127E38F, -2.87363E38F, -2.573491E38F, -2.957232E38F, -2.9966222E38F, -1.5752378E38F, 1.6993071E38F, -6.1095445E37F, 1.9244403E38F, 3.1063683E38F, -1.5667059E38F, 3.263042E38F, 2.288599E38F, 2.667129E38F, -1.8041313E38F, -1.9551866E38F, -3.2264509E38F, 2.954989E38F, -3.0428472E38F, 5.010885E37F, 6.983358E37F, 3.389791E38F, -9.01843E37F, 3.3335807E38F, -1.0687713E38F, -1.9696966E38F, 1.1522879E37F, 2.6375108E38F, 4.428771E37F, -3.346712E38F, 7.865562E37F, 2.114214E38F, 9.719061E37F, -1.2143479E38F, 2.263022E38F, -1.4263706E38F, 1.9282294E38F, 1.4214931E38F, -1.880382E38F, 5.9614505E37F, 2.7396954E38F, 3.3071448E38F, -1.0500729E38F, -1.1773099E38F, 7.233801E37F}, 0) ;
        p64.z_SET(-2.0696627E38F) ;
        p64.y_SET(3.3576663E38F) ;
        p64.az_SET(2.263165E38F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS) ;
        p64.vx_SET(1.61865E38F) ;
        LoopBackDemoChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan14_raw_GET() == (char)17043);
            assert(pack.chancount_GET() == (char)234);
            assert(pack.chan16_raw_GET() == (char)6322);
            assert(pack.chan5_raw_GET() == (char)35476);
            assert(pack.chan15_raw_GET() == (char)47754);
            assert(pack.chan3_raw_GET() == (char)3547);
            assert(pack.chan1_raw_GET() == (char)10470);
            assert(pack.chan6_raw_GET() == (char)30523);
            assert(pack.chan10_raw_GET() == (char)57534);
            assert(pack.chan17_raw_GET() == (char)14829);
            assert(pack.chan9_raw_GET() == (char)58671);
            assert(pack.time_boot_ms_GET() == 1046217589L);
            assert(pack.chan8_raw_GET() == (char)21493);
            assert(pack.chan2_raw_GET() == (char)26083);
            assert(pack.chan13_raw_GET() == (char)1374);
            assert(pack.chan4_raw_GET() == (char)47237);
            assert(pack.chan11_raw_GET() == (char)34810);
            assert(pack.chan18_raw_GET() == (char)56666);
            assert(pack.rssi_GET() == (char)198);
            assert(pack.chan7_raw_GET() == (char)48708);
            assert(pack.chan12_raw_GET() == (char)65379);
        });
        DemoDevice.RC_CHANNELS p65 = LoopBackDemoChannel.new_RC_CHANNELS();
        PH.setPack(p65);
        p65.rssi_SET((char)198) ;
        p65.chan16_raw_SET((char)6322) ;
        p65.chan18_raw_SET((char)56666) ;
        p65.chancount_SET((char)234) ;
        p65.chan17_raw_SET((char)14829) ;
        p65.chan15_raw_SET((char)47754) ;
        p65.chan8_raw_SET((char)21493) ;
        p65.chan11_raw_SET((char)34810) ;
        p65.chan5_raw_SET((char)35476) ;
        p65.time_boot_ms_SET(1046217589L) ;
        p65.chan3_raw_SET((char)3547) ;
        p65.chan6_raw_SET((char)30523) ;
        p65.chan14_raw_SET((char)17043) ;
        p65.chan9_raw_SET((char)58671) ;
        p65.chan13_raw_SET((char)1374) ;
        p65.chan12_raw_SET((char)65379) ;
        p65.chan7_raw_SET((char)48708) ;
        p65.chan10_raw_SET((char)57534) ;
        p65.chan1_raw_SET((char)10470) ;
        p65.chan2_raw_SET((char)26083) ;
        p65.chan4_raw_SET((char)47237) ;
        LoopBackDemoChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)225);
            assert(pack.req_stream_id_GET() == (char)200);
            assert(pack.req_message_rate_GET() == (char)61422);
            assert(pack.start_stop_GET() == (char)186);
            assert(pack.target_system_GET() == (char)180);
        });
        DemoDevice.REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.new_REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.req_stream_id_SET((char)200) ;
        p66.start_stop_SET((char)186) ;
        p66.target_system_SET((char)180) ;
        p66.target_component_SET((char)225) ;
        p66.req_message_rate_SET((char)61422) ;
        LoopBackDemoChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.message_rate_GET() == (char)55285);
            assert(pack.stream_id_GET() == (char)242);
            assert(pack.on_off_GET() == (char)199);
        });
        DemoDevice.DATA_STREAM p67 = LoopBackDemoChannel.new_DATA_STREAM();
        PH.setPack(p67);
        p67.message_rate_SET((char)55285) ;
        p67.stream_id_SET((char)242) ;
        p67.on_off_SET((char)199) ;
        LoopBackDemoChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.buttons_GET() == (char)40648);
            assert(pack.y_GET() == (short) -11876);
            assert(pack.x_GET() == (short)26794);
            assert(pack.z_GET() == (short) -9314);
            assert(pack.r_GET() == (short)25803);
            assert(pack.target_GET() == (char)94);
        });
        DemoDevice.MANUAL_CONTROL p69 = LoopBackDemoChannel.new_MANUAL_CONTROL();
        PH.setPack(p69);
        p69.y_SET((short) -11876) ;
        p69.x_SET((short)26794) ;
        p69.target_SET((char)94) ;
        p69.buttons_SET((char)40648) ;
        p69.r_SET((short)25803) ;
        p69.z_SET((short) -9314) ;
        LoopBackDemoChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)193);
            assert(pack.chan7_raw_GET() == (char)12345);
            assert(pack.target_system_GET() == (char)194);
            assert(pack.chan8_raw_GET() == (char)22492);
            assert(pack.chan3_raw_GET() == (char)42569);
            assert(pack.chan2_raw_GET() == (char)4234);
            assert(pack.chan6_raw_GET() == (char)42522);
            assert(pack.chan1_raw_GET() == (char)60144);
            assert(pack.chan5_raw_GET() == (char)12596);
            assert(pack.chan4_raw_GET() == (char)65148);
        });
        DemoDevice.RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.new_RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.target_component_SET((char)193) ;
        p70.chan5_raw_SET((char)12596) ;
        p70.chan4_raw_SET((char)65148) ;
        p70.chan3_raw_SET((char)42569) ;
        p70.chan1_raw_SET((char)60144) ;
        p70.chan6_raw_SET((char)42522) ;
        p70.chan8_raw_SET((char)22492) ;
        p70.chan7_raw_SET((char)12345) ;
        p70.chan2_raw_SET((char)4234) ;
        p70.target_system_SET((char)194) ;
        LoopBackDemoChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.autocontinue_GET() == (char)10);
            assert(pack.z_GET() == 1.9742521E37F);
            assert(pack.param1_GET() == -1.0974741E38F);
            assert(pack.target_system_GET() == (char)23);
            assert(pack.current_GET() == (char)214);
            assert(pack.x_GET() == 1101928699);
            assert(pack.param4_GET() == -2.5058552E38F);
            assert(pack.y_GET() == -1613575241);
            assert(pack.param2_GET() == -8.4820814E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.seq_GET() == (char)9891);
            assert(pack.target_component_GET() == (char)84);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_USER_3);
            assert(pack.param3_GET() == -2.3014675E38F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        });
        DemoDevice.MISSION_ITEM_INT p73 = LoopBackDemoChannel.new_MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.current_SET((char)214) ;
        p73.param1_SET(-1.0974741E38F) ;
        p73.target_component_SET((char)84) ;
        p73.z_SET(1.9742521E37F) ;
        p73.param2_SET(-8.4820814E37F) ;
        p73.target_system_SET((char)23) ;
        p73.y_SET(-1613575241) ;
        p73.param3_SET(-2.3014675E38F) ;
        p73.x_SET(1101928699) ;
        p73.param4_SET(-2.5058552E38F) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p73.command_SET(MAV_CMD.MAV_CMD_USER_3) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p73.autocontinue_SET((char)10) ;
        p73.seq_SET((char)9891) ;
        LoopBackDemoChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.groundspeed_GET() == -1.1965401E37F);
            assert(pack.airspeed_GET() == 2.4946198E38F);
            assert(pack.climb_GET() == -1.6625698E37F);
            assert(pack.alt_GET() == 2.268228E38F);
            assert(pack.throttle_GET() == (char)35416);
            assert(pack.heading_GET() == (short) -3661);
        });
        DemoDevice.VFR_HUD p74 = LoopBackDemoChannel.new_VFR_HUD();
        PH.setPack(p74);
        p74.airspeed_SET(2.4946198E38F) ;
        p74.groundspeed_SET(-1.1965401E37F) ;
        p74.heading_SET((short) -3661) ;
        p74.alt_SET(2.268228E38F) ;
        p74.climb_SET(-1.6625698E37F) ;
        p74.throttle_SET((char)35416) ;
        LoopBackDemoChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)72);
            assert(pack.param1_GET() == -7.2283957E37F);
            assert(pack.target_component_GET() == (char)72);
            assert(pack.current_GET() == (char)61);
            assert(pack.x_GET() == 1558791362);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.param3_GET() == 7.840528E37F);
            assert(pack.param4_GET() == -1.1542978E38F);
            assert(pack.z_GET() == -3.3810375E38F);
            assert(pack.y_GET() == -1909652511);
            assert(pack.autocontinue_GET() == (char)107);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE);
            assert(pack.param2_GET() == 2.768485E38F);
        });
        DemoDevice.COMMAND_INT p75 = LoopBackDemoChannel.new_COMMAND_INT();
        PH.setPack(p75);
        p75.autocontinue_SET((char)107) ;
        p75.current_SET((char)61) ;
        p75.param1_SET(-7.2283957E37F) ;
        p75.z_SET(-3.3810375E38F) ;
        p75.target_component_SET((char)72) ;
        p75.param4_SET(-1.1542978E38F) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p75.y_SET(-1909652511) ;
        p75.param2_SET(2.768485E38F) ;
        p75.param3_SET(7.840528E37F) ;
        p75.target_system_SET((char)72) ;
        p75.command_SET(MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE) ;
        p75.x_SET(1558791362) ;
        LoopBackDemoChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)61);
            assert(pack.param7_GET() == -1.4677573E38F);
            assert(pack.target_component_GET() == (char)6);
            assert(pack.param3_GET() == 2.671423E38F);
            assert(pack.param4_GET() == 2.4498782E38F);
            assert(pack.confirmation_GET() == (char)162);
            assert(pack.param6_GET() == -3.5069863E37F);
            assert(pack.param1_GET() == 3.2224165E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE);
            assert(pack.param5_GET() == -1.0698381E38F);
            assert(pack.param2_GET() == 1.5951618E38F);
        });
        DemoDevice.COMMAND_LONG p76 = LoopBackDemoChannel.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.param4_SET(2.4498782E38F) ;
        p76.param3_SET(2.671423E38F) ;
        p76.param6_SET(-3.5069863E37F) ;
        p76.confirmation_SET((char)162) ;
        p76.param2_SET(1.5951618E38F) ;
        p76.target_system_SET((char)61) ;
        p76.param5_SET(-1.0698381E38F) ;
        p76.param1_SET(3.2224165E38F) ;
        p76.param7_SET(-1.4677573E38F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE) ;
        p76.target_component_SET((char)6) ;
        LoopBackDemoChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_TRY(ph) == (char)12);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING);
            assert(pack.target_system_TRY(ph) == (char)165);
            assert(pack.result_param2_TRY(ph) == -1243190514);
            assert(pack.progress_TRY(ph) == (char)88);
        });
        DemoDevice.COMMAND_ACK p77 = LoopBackDemoChannel.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.command_SET(MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING) ;
        p77.target_system_SET((char)165, PH) ;
        p77.target_component_SET((char)12, PH) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED) ;
        p77.result_param2_SET(-1243190514, PH) ;
        p77.progress_SET((char)88, PH) ;
        LoopBackDemoChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.mode_switch_GET() == (char)107);
            assert(pack.manual_override_switch_GET() == (char)39);
            assert(pack.time_boot_ms_GET() == 3459087757L);
            assert(pack.pitch_GET() == 1.3360203E38F);
            assert(pack.thrust_GET() == -3.3550339E38F);
            assert(pack.roll_GET() == 2.79871E38F);
            assert(pack.yaw_GET() == 2.0415312E38F);
        });
        DemoDevice.MANUAL_SETPOINT p81 = LoopBackDemoChannel.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.mode_switch_SET((char)107) ;
        p81.thrust_SET(-3.3550339E38F) ;
        p81.pitch_SET(1.3360203E38F) ;
        p81.roll_SET(2.79871E38F) ;
        p81.time_boot_ms_SET(3459087757L) ;
        p81.yaw_SET(2.0415312E38F) ;
        p81.manual_override_switch_SET((char)39) ;
        LoopBackDemoChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 4079683052L);
            assert(pack.type_mask_GET() == (char)85);
            assert(pack.body_yaw_rate_GET() == 8.514158E37F);
            assert(pack.body_roll_rate_GET() == 9.971654E37F);
            assert(pack.body_pitch_rate_GET() == 2.9260708E38F);
            assert(pack.target_component_GET() == (char)168);
            assert(pack.thrust_GET() == 1.7960033E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.6704678E38F, -5.130471E37F, -1.3145102E38F, -2.9733491E38F}));
            assert(pack.target_system_GET() == (char)240);
        });
        DemoDevice.SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.time_boot_ms_SET(4079683052L) ;
        p82.body_pitch_rate_SET(2.9260708E38F) ;
        p82.type_mask_SET((char)85) ;
        p82.q_SET(new float[] {-2.6704678E38F, -5.130471E37F, -1.3145102E38F, -2.9733491E38F}, 0) ;
        p82.thrust_SET(1.7960033E38F) ;
        p82.target_system_SET((char)240) ;
        p82.body_roll_rate_SET(9.971654E37F) ;
        p82.target_component_SET((char)168) ;
        p82.body_yaw_rate_SET(8.514158E37F) ;
        LoopBackDemoChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.type_mask_GET() == (char)218);
            assert(pack.body_roll_rate_GET() == -1.6033905E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {5.014176E37F, 2.3658096E38F, 2.8570084E38F, -3.3209306E38F}));
            assert(pack.body_pitch_rate_GET() == -7.8009727E37F);
            assert(pack.thrust_GET() == 4.5955554E36F);
            assert(pack.time_boot_ms_GET() == 2790809301L);
            assert(pack.body_yaw_rate_GET() == 1.3046861E38F);
        });
        DemoDevice.ATTITUDE_TARGET p83 = LoopBackDemoChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.thrust_SET(4.5955554E36F) ;
        p83.q_SET(new float[] {5.014176E37F, 2.3658096E38F, 2.8570084E38F, -3.3209306E38F}, 0) ;
        p83.body_yaw_rate_SET(1.3046861E38F) ;
        p83.time_boot_ms_SET(2790809301L) ;
        p83.type_mask_SET((char)218) ;
        p83.body_pitch_rate_SET(-7.8009727E37F) ;
        p83.body_roll_rate_SET(-1.6033905E38F) ;
        LoopBackDemoChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -1.6085646E37F);
            assert(pack.type_mask_GET() == (char)6220);
            assert(pack.target_component_GET() == (char)101);
            assert(pack.vz_GET() == -2.664596E38F);
            assert(pack.afz_GET() == -1.2507385E37F);
            assert(pack.x_GET() == 1.3275095E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.afx_GET() == -3.3213626E38F);
            assert(pack.time_boot_ms_GET() == 2149903507L);
            assert(pack.yaw_GET() == 1.1528517E37F);
            assert(pack.afy_GET() == -1.372822E38F);
            assert(pack.y_GET() == -3.1708542E38F);
            assert(pack.vy_GET() == 1.5648247E38F);
            assert(pack.yaw_rate_GET() == 1.8522675E38F);
            assert(pack.target_system_GET() == (char)153);
            assert(pack.vx_GET() == -3.1165507E37F);
        });
        DemoDevice.SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.afx_SET(-3.3213626E38F) ;
        p84.target_component_SET((char)101) ;
        p84.vx_SET(-3.1165507E37F) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p84.x_SET(1.3275095E38F) ;
        p84.y_SET(-3.1708542E38F) ;
        p84.afz_SET(-1.2507385E37F) ;
        p84.type_mask_SET((char)6220) ;
        p84.yaw_SET(1.1528517E37F) ;
        p84.vz_SET(-2.664596E38F) ;
        p84.z_SET(-1.6085646E37F) ;
        p84.yaw_rate_SET(1.8522675E38F) ;
        p84.time_boot_ms_SET(2149903507L) ;
        p84.afy_SET(-1.372822E38F) ;
        p84.vy_SET(1.5648247E38F) ;
        p84.target_system_SET((char)153) ;
        LoopBackDemoChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.yaw_rate_GET() == 1.4665999E38F);
            assert(pack.lon_int_GET() == 1451172756);
            assert(pack.type_mask_GET() == (char)1064);
            assert(pack.alt_GET() == 2.1728698E38F);
            assert(pack.afx_GET() == 4.0408984E36F);
            assert(pack.yaw_GET() == -2.4229249E38F);
            assert(pack.afz_GET() == 6.494871E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.vz_GET() == 1.121517E38F);
            assert(pack.vx_GET() == 1.1735231E38F);
            assert(pack.target_system_GET() == (char)105);
            assert(pack.time_boot_ms_GET() == 4062445833L);
            assert(pack.lat_int_GET() == -1420291809);
            assert(pack.afy_GET() == 3.1224402E38F);
            assert(pack.vy_GET() == -2.8138253E38F);
            assert(pack.target_component_GET() == (char)166);
        });
        DemoDevice.SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.target_component_SET((char)166) ;
        p86.vx_SET(1.1735231E38F) ;
        p86.alt_SET(2.1728698E38F) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p86.yaw_SET(-2.4229249E38F) ;
        p86.time_boot_ms_SET(4062445833L) ;
        p86.yaw_rate_SET(1.4665999E38F) ;
        p86.afy_SET(3.1224402E38F) ;
        p86.afz_SET(6.494871E37F) ;
        p86.vz_SET(1.121517E38F) ;
        p86.vy_SET(-2.8138253E38F) ;
        p86.target_system_SET((char)105) ;
        p86.lon_int_SET(1451172756) ;
        p86.afx_SET(4.0408984E36F) ;
        p86.lat_int_SET(-1420291809) ;
        p86.type_mask_SET((char)1064) ;
        LoopBackDemoChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == -1.8900204E38F);
            assert(pack.time_boot_ms_GET() == 1217232257L);
            assert(pack.afx_GET() == 1.6328514E38F);
            assert(pack.vz_GET() == 5.941403E37F);
            assert(pack.lat_int_GET() == 1525032620);
            assert(pack.lon_int_GET() == 1408166122);
            assert(pack.afy_GET() == -3.1239515E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
            assert(pack.yaw_rate_GET() == 3.3878727E38F);
            assert(pack.type_mask_GET() == (char)52676);
            assert(pack.vx_GET() == -1.6099124E38F);
            assert(pack.vy_GET() == -1.4496814E38F);
            assert(pack.yaw_GET() == 2.1566637E38F);
            assert(pack.afz_GET() == -3.3775556E38F);
        });
        DemoDevice.POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.lon_int_SET(1408166122) ;
        p87.yaw_SET(2.1566637E38F) ;
        p87.alt_SET(-1.8900204E38F) ;
        p87.type_mask_SET((char)52676) ;
        p87.vx_SET(-1.6099124E38F) ;
        p87.yaw_rate_SET(3.3878727E38F) ;
        p87.vz_SET(5.941403E37F) ;
        p87.time_boot_ms_SET(1217232257L) ;
        p87.afz_SET(-3.3775556E38F) ;
        p87.vy_SET(-1.4496814E38F) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU) ;
        p87.afy_SET(-3.1239515E38F) ;
        p87.afx_SET(1.6328514E38F) ;
        p87.lat_int_SET(1525032620) ;
        LoopBackDemoChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == -9.96466E37F);
            assert(pack.time_boot_ms_GET() == 2800437750L);
            assert(pack.y_GET() == -2.5135149E38F);
            assert(pack.yaw_GET() == -1.0538142E38F);
            assert(pack.z_GET() == -1.744586E38F);
            assert(pack.pitch_GET() == -2.0457962E38F);
            assert(pack.x_GET() == 3.224337E38F);
        });
        DemoDevice.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.roll_SET(-9.96466E37F) ;
        p89.pitch_SET(-2.0457962E38F) ;
        p89.time_boot_ms_SET(2800437750L) ;
        p89.x_SET(3.224337E38F) ;
        p89.yaw_SET(-1.0538142E38F) ;
        p89.z_SET(-1.744586E38F) ;
        p89.y_SET(-2.5135149E38F) ;
        LoopBackDemoChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -2.9575765E38F);
            assert(pack.alt_GET() == 1532287870);
            assert(pack.roll_GET() == -6.23216E37F);
            assert(pack.zacc_GET() == (short)22773);
            assert(pack.time_usec_GET() == 5466112933908499161L);
            assert(pack.xacc_GET() == (short)6349);
            assert(pack.vx_GET() == (short)968);
            assert(pack.yawspeed_GET() == 2.3554901E38F);
            assert(pack.lat_GET() == -191256825);
            assert(pack.vy_GET() == (short) -10898);
            assert(pack.yacc_GET() == (short)12603);
            assert(pack.pitch_GET() == -2.7398242E38F);
            assert(pack.rollspeed_GET() == 1.878741E38F);
            assert(pack.lon_GET() == -1758309591);
            assert(pack.pitchspeed_GET() == -1.5317045E37F);
            assert(pack.vz_GET() == (short)31055);
        });
        DemoDevice.HIL_STATE p90 = LoopBackDemoChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.roll_SET(-6.23216E37F) ;
        p90.vz_SET((short)31055) ;
        p90.yaw_SET(-2.9575765E38F) ;
        p90.pitchspeed_SET(-1.5317045E37F) ;
        p90.xacc_SET((short)6349) ;
        p90.rollspeed_SET(1.878741E38F) ;
        p90.vx_SET((short)968) ;
        p90.vy_SET((short) -10898) ;
        p90.lon_SET(-1758309591) ;
        p90.yacc_SET((short)12603) ;
        p90.alt_SET(1532287870) ;
        p90.time_usec_SET(5466112933908499161L) ;
        p90.lat_SET(-191256825) ;
        p90.pitch_SET(-2.7398242E38F) ;
        p90.yawspeed_SET(2.3554901E38F) ;
        p90.zacc_SET((short)22773) ;
        LoopBackDemoChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.yaw_rudder_GET() == -1.7232623E38F);
            assert(pack.aux3_GET() == 5.72573E37F);
            assert(pack.aux1_GET() == -3.0933337E37F);
            assert(pack.aux2_GET() == 1.9664788E38F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_TEST_ARMED);
            assert(pack.pitch_elevator_GET() == -4.9216524E37F);
            assert(pack.time_usec_GET() == 4024300193813222664L);
            assert(pack.roll_ailerons_GET() == 1.4745781E38F);
            assert(pack.nav_mode_GET() == (char)77);
            assert(pack.throttle_GET() == -3.3867255E38F);
            assert(pack.aux4_GET() == 1.6911276E38F);
        });
        DemoDevice.HIL_CONTROLS p91 = LoopBackDemoChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.mode_SET(MAV_MODE.MAV_MODE_TEST_ARMED) ;
        p91.aux1_SET(-3.0933337E37F) ;
        p91.pitch_elevator_SET(-4.9216524E37F) ;
        p91.aux4_SET(1.6911276E38F) ;
        p91.nav_mode_SET((char)77) ;
        p91.time_usec_SET(4024300193813222664L) ;
        p91.roll_ailerons_SET(1.4745781E38F) ;
        p91.throttle_SET(-3.3867255E38F) ;
        p91.aux3_SET(5.72573E37F) ;
        p91.yaw_rudder_SET(-1.7232623E38F) ;
        p91.aux2_SET(1.9664788E38F) ;
        LoopBackDemoChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan2_raw_GET() == (char)43348);
            assert(pack.chan6_raw_GET() == (char)51389);
            assert(pack.chan4_raw_GET() == (char)56117);
            assert(pack.chan9_raw_GET() == (char)53121);
            assert(pack.chan3_raw_GET() == (char)44686);
            assert(pack.chan5_raw_GET() == (char)33500);
            assert(pack.chan8_raw_GET() == (char)49328);
            assert(pack.chan7_raw_GET() == (char)36457);
            assert(pack.chan12_raw_GET() == (char)10188);
            assert(pack.chan1_raw_GET() == (char)3214);
            assert(pack.chan10_raw_GET() == (char)63324);
            assert(pack.time_usec_GET() == 1099078196476063983L);
            assert(pack.rssi_GET() == (char)104);
            assert(pack.chan11_raw_GET() == (char)9500);
        });
        DemoDevice.HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan7_raw_SET((char)36457) ;
        p92.chan6_raw_SET((char)51389) ;
        p92.chan12_raw_SET((char)10188) ;
        p92.chan3_raw_SET((char)44686) ;
        p92.chan1_raw_SET((char)3214) ;
        p92.rssi_SET((char)104) ;
        p92.chan10_raw_SET((char)63324) ;
        p92.time_usec_SET(1099078196476063983L) ;
        p92.chan11_raw_SET((char)9500) ;
        p92.chan9_raw_SET((char)53121) ;
        p92.chan5_raw_SET((char)33500) ;
        p92.chan2_raw_SET((char)43348) ;
        p92.chan4_raw_SET((char)56117) ;
        p92.chan8_raw_SET((char)49328) ;
        LoopBackDemoChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 5281390204389747458L);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_GUIDED_DISARMED);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {2.6572794E38F, 2.6577214E38F, 1.9645818E38F, 2.8609974E38F, -5.9533897E37F, 1.0917518E38F, 1.4058156E38F, 3.3696558E38F, -1.3065704E38F, 1.6829748E38F, 3.0374334E38F, 1.1580547E38F, 2.0969087E38F, -2.1641804E38F, -3.2510662E38F, -2.357682E38F}));
            assert(pack.flags_GET() == 8058535847425380159L);
        });
        DemoDevice.HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.time_usec_SET(5281390204389747458L) ;
        p93.controls_SET(new float[] {2.6572794E38F, 2.6577214E38F, 1.9645818E38F, 2.8609974E38F, -5.9533897E37F, 1.0917518E38F, 1.4058156E38F, 3.3696558E38F, -1.3065704E38F, 1.6829748E38F, 3.0374334E38F, 1.1580547E38F, 2.0969087E38F, -2.1641804E38F, -3.2510662E38F, -2.357682E38F}, 0) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_GUIDED_DISARMED) ;
        p93.flags_SET(8058535847425380159L) ;
        LoopBackDemoChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.quality_GET() == (char)243);
            assert(pack.ground_distance_GET() == -1.3904622E37F);
            assert(pack.sensor_id_GET() == (char)50);
            assert(pack.time_usec_GET() == 6613544811760052492L);
            assert(pack.flow_rate_y_TRY(ph) == -1.9495667E38F);
            assert(pack.flow_x_GET() == (short) -9018);
            assert(pack.flow_y_GET() == (short)2528);
            assert(pack.flow_comp_m_x_GET() == 1.0052985E38F);
            assert(pack.flow_comp_m_y_GET() == 3.1596915E38F);
            assert(pack.flow_rate_x_TRY(ph) == 2.4477844E38F);
        });
        DemoDevice.OPTICAL_FLOW p100 = LoopBackDemoChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.quality_SET((char)243) ;
        p100.flow_comp_m_x_SET(1.0052985E38F) ;
        p100.flow_y_SET((short)2528) ;
        p100.time_usec_SET(6613544811760052492L) ;
        p100.ground_distance_SET(-1.3904622E37F) ;
        p100.flow_comp_m_y_SET(3.1596915E38F) ;
        p100.sensor_id_SET((char)50) ;
        p100.flow_rate_x_SET(2.4477844E38F, PH) ;
        p100.flow_rate_y_SET(-1.9495667E38F, PH) ;
        p100.flow_x_SET((short) -9018) ;
        LoopBackDemoChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == -3.2787848E38F);
            assert(pack.x_GET() == -1.2907694E38F);
            assert(pack.yaw_GET() == 2.325917E37F);
            assert(pack.y_GET() == -3.0441984E38F);
            assert(pack.z_GET() == 2.431107E38F);
            assert(pack.roll_GET() == 2.5216947E38F);
            assert(pack.usec_GET() == 4446649825156828480L);
        });
        DemoDevice.GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.y_SET(-3.0441984E38F) ;
        p101.x_SET(-1.2907694E38F) ;
        p101.usec_SET(4446649825156828480L) ;
        p101.pitch_SET(-3.2787848E38F) ;
        p101.roll_SET(2.5216947E38F) ;
        p101.z_SET(2.431107E38F) ;
        p101.yaw_SET(2.325917E37F) ;
        LoopBackDemoChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -3.0493621E38F);
            assert(pack.roll_GET() == 3.257103E37F);
            assert(pack.x_GET() == 1.3308116E36F);
            assert(pack.y_GET() == -9.940606E37F);
            assert(pack.usec_GET() == 5295411382795446005L);
            assert(pack.pitch_GET() == 4.446398E37F);
            assert(pack.z_GET() == 1.154825E38F);
        });
        DemoDevice.VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.roll_SET(3.257103E37F) ;
        p102.z_SET(1.154825E38F) ;
        p102.yaw_SET(-3.0493621E38F) ;
        p102.usec_SET(5295411382795446005L) ;
        p102.pitch_SET(4.446398E37F) ;
        p102.y_SET(-9.940606E37F) ;
        p102.x_SET(1.3308116E36F) ;
        LoopBackDemoChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -1.136804E37F);
            assert(pack.usec_GET() == 5653412247552776752L);
            assert(pack.x_GET() == -3.2609343E37F);
            assert(pack.z_GET() == -2.8195947E38F);
        });
        DemoDevice.VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.usec_SET(5653412247552776752L) ;
        p103.z_SET(-2.8195947E38F) ;
        p103.x_SET(-3.2609343E37F) ;
        p103.y_SET(-1.136804E37F) ;
        LoopBackDemoChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -3.2881597E38F);
            assert(pack.z_GET() == 2.943899E38F);
            assert(pack.usec_GET() == 1974993773649938937L);
            assert(pack.pitch_GET() == 1.7243918E38F);
            assert(pack.x_GET() == 2.191069E38F);
            assert(pack.roll_GET() == 2.757556E38F);
            assert(pack.y_GET() == -3.559176E37F);
        });
        DemoDevice.VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.usec_SET(1974993773649938937L) ;
        p104.x_SET(2.191069E38F) ;
        p104.z_SET(2.943899E38F) ;
        p104.y_SET(-3.559176E37F) ;
        p104.pitch_SET(1.7243918E38F) ;
        p104.roll_SET(2.757556E38F) ;
        p104.yaw_SET(-3.2881597E38F) ;
        LoopBackDemoChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == -1.8231152E38F);
            assert(pack.pressure_alt_GET() == -1.0147521E38F);
            assert(pack.xgyro_GET() == 7.605352E37F);
            assert(pack.xmag_GET() == -4.5086798E36F);
            assert(pack.temperature_GET() == 2.1088023E38F);
            assert(pack.ymag_GET() == 2.1919727E38F);
            assert(pack.yacc_GET() == -3.2505809E38F);
            assert(pack.zmag_GET() == 3.3541923E37F);
            assert(pack.xacc_GET() == -1.8951754E38F);
            assert(pack.abs_pressure_GET() == 3.1703512E38F);
            assert(pack.time_usec_GET() == 5796315986357663982L);
            assert(pack.zgyro_GET() == 3.1868892E38F);
            assert(pack.diff_pressure_GET() == -1.7314748E38F);
            assert(pack.zacc_GET() == -2.4722217E38F);
            assert(pack.fields_updated_GET() == (char)5051);
        });
        DemoDevice.HIGHRES_IMU p105 = LoopBackDemoChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.diff_pressure_SET(-1.7314748E38F) ;
        p105.pressure_alt_SET(-1.0147521E38F) ;
        p105.zacc_SET(-2.4722217E38F) ;
        p105.abs_pressure_SET(3.1703512E38F) ;
        p105.time_usec_SET(5796315986357663982L) ;
        p105.yacc_SET(-3.2505809E38F) ;
        p105.fields_updated_SET((char)5051) ;
        p105.temperature_SET(2.1088023E38F) ;
        p105.zgyro_SET(3.1868892E38F) ;
        p105.xacc_SET(-1.8951754E38F) ;
        p105.ygyro_SET(-1.8231152E38F) ;
        p105.ymag_SET(2.1919727E38F) ;
        p105.zmag_SET(3.3541923E37F) ;
        p105.xmag_SET(-4.5086798E36F) ;
        p105.xgyro_SET(7.605352E37F) ;
        LoopBackDemoChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.integrated_y_GET() == 2.5766503E38F);
            assert(pack.integrated_x_GET() == -4.025887E37F);
            assert(pack.time_delta_distance_us_GET() == 3102052526L);
            assert(pack.integrated_xgyro_GET() == 2.0057593E38F);
            assert(pack.quality_GET() == (char)49);
            assert(pack.integrated_zgyro_GET() == 4.3645514E37F);
            assert(pack.time_usec_GET() == 8994706893233067605L);
            assert(pack.sensor_id_GET() == (char)24);
            assert(pack.distance_GET() == 3.2514824E38F);
            assert(pack.temperature_GET() == (short)4334);
            assert(pack.integrated_ygyro_GET() == -1.2463824E38F);
            assert(pack.integration_time_us_GET() == 407986861L);
        });
        DemoDevice.OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.distance_SET(3.2514824E38F) ;
        p106.integrated_ygyro_SET(-1.2463824E38F) ;
        p106.sensor_id_SET((char)24) ;
        p106.integrated_y_SET(2.5766503E38F) ;
        p106.temperature_SET((short)4334) ;
        p106.quality_SET((char)49) ;
        p106.integration_time_us_SET(407986861L) ;
        p106.integrated_x_SET(-4.025887E37F) ;
        p106.time_delta_distance_us_SET(3102052526L) ;
        p106.integrated_zgyro_SET(4.3645514E37F) ;
        p106.integrated_xgyro_SET(2.0057593E38F) ;
        p106.time_usec_SET(8994706893233067605L) ;
        LoopBackDemoChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == 3.0150163E38F);
            assert(pack.xacc_GET() == -7.228999E37F);
            assert(pack.time_usec_GET() == 2397975439078259807L);
            assert(pack.zacc_GET() == 4.2882176E36F);
            assert(pack.diff_pressure_GET() == 2.9187574E37F);
            assert(pack.ymag_GET() == -3.2029164E36F);
            assert(pack.xgyro_GET() == -2.032144E38F);
            assert(pack.zgyro_GET() == 1.0598285E38F);
            assert(pack.ygyro_GET() == -1.7339848E38F);
            assert(pack.yacc_GET() == 1.8141607E38F);
            assert(pack.fields_updated_GET() == 2187752466L);
            assert(pack.zmag_GET() == -2.8467628E38F);
            assert(pack.xmag_GET() == -1.4993145E38F);
            assert(pack.abs_pressure_GET() == -8.0369236E37F);
            assert(pack.pressure_alt_GET() == -2.5719846E38F);
        });
        DemoDevice.HIL_SENSOR p107 = LoopBackDemoChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.xacc_SET(-7.228999E37F) ;
        p107.pressure_alt_SET(-2.5719846E38F) ;
        p107.zgyro_SET(1.0598285E38F) ;
        p107.zacc_SET(4.2882176E36F) ;
        p107.abs_pressure_SET(-8.0369236E37F) ;
        p107.xgyro_SET(-2.032144E38F) ;
        p107.fields_updated_SET(2187752466L) ;
        p107.time_usec_SET(2397975439078259807L) ;
        p107.diff_pressure_SET(2.9187574E37F) ;
        p107.xmag_SET(-1.4993145E38F) ;
        p107.yacc_SET(1.8141607E38F) ;
        p107.ymag_SET(-3.2029164E36F) ;
        p107.temperature_SET(3.0150163E38F) ;
        p107.ygyro_SET(-1.7339848E38F) ;
        p107.zmag_SET(-2.8467628E38F) ;
        LoopBackDemoChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.q3_GET() == 3.2027643E38F);
            assert(pack.std_dev_vert_GET() == -7.278568E37F);
            assert(pack.zgyro_GET() == 1.767893E38F);
            assert(pack.ygyro_GET() == -4.261393E37F);
            assert(pack.yacc_GET() == -2.0436375E38F);
            assert(pack.yaw_GET() == 1.9769999E38F);
            assert(pack.pitch_GET() == -2.5755308E38F);
            assert(pack.xgyro_GET() == -1.1899669E38F);
            assert(pack.xacc_GET() == 3.2690796E35F);
            assert(pack.std_dev_horz_GET() == 8.2464013E37F);
            assert(pack.vd_GET() == -1.6811926E38F);
            assert(pack.vn_GET() == 3.848139E37F);
            assert(pack.q2_GET() == 1.402061E38F);
            assert(pack.roll_GET() == 1.716997E38F);
            assert(pack.alt_GET() == -1.2468441E38F);
            assert(pack.ve_GET() == 3.00929E38F);
            assert(pack.zacc_GET() == 4.30558E37F);
            assert(pack.q1_GET() == -1.2650968E37F);
            assert(pack.lon_GET() == -2.7079293E38F);
            assert(pack.q4_GET() == 2.698357E37F);
            assert(pack.lat_GET() == 2.1752294E38F);
        });
        DemoDevice.SIM_STATE p108 = LoopBackDemoChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.alt_SET(-1.2468441E38F) ;
        p108.vn_SET(3.848139E37F) ;
        p108.ygyro_SET(-4.261393E37F) ;
        p108.ve_SET(3.00929E38F) ;
        p108.q2_SET(1.402061E38F) ;
        p108.pitch_SET(-2.5755308E38F) ;
        p108.vd_SET(-1.6811926E38F) ;
        p108.q1_SET(-1.2650968E37F) ;
        p108.q4_SET(2.698357E37F) ;
        p108.roll_SET(1.716997E38F) ;
        p108.yacc_SET(-2.0436375E38F) ;
        p108.std_dev_horz_SET(8.2464013E37F) ;
        p108.yaw_SET(1.9769999E38F) ;
        p108.xacc_SET(3.2690796E35F) ;
        p108.lat_SET(2.1752294E38F) ;
        p108.lon_SET(-2.7079293E38F) ;
        p108.xgyro_SET(-1.1899669E38F) ;
        p108.std_dev_vert_SET(-7.278568E37F) ;
        p108.zgyro_SET(1.767893E38F) ;
        p108.zacc_SET(4.30558E37F) ;
        p108.q3_SET(3.2027643E38F) ;
        LoopBackDemoChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.rssi_GET() == (char)87);
            assert(pack.remrssi_GET() == (char)84);
            assert(pack.remnoise_GET() == (char)46);
            assert(pack.noise_GET() == (char)186);
            assert(pack.txbuf_GET() == (char)71);
            assert(pack.fixed__GET() == (char)59843);
            assert(pack.rxerrors_GET() == (char)34922);
        });
        DemoDevice.RADIO_STATUS p109 = LoopBackDemoChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.rxerrors_SET((char)34922) ;
        p109.fixed__SET((char)59843) ;
        p109.rssi_SET((char)87) ;
        p109.remnoise_SET((char)46) ;
        p109.noise_SET((char)186) ;
        p109.txbuf_SET((char)71) ;
        p109.remrssi_SET((char)84) ;
        LoopBackDemoChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)51);
            assert(pack.target_system_GET() == (char)188);
            assert(pack.target_network_GET() == (char)42);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)118, (char)176, (char)124, (char)153, (char)253, (char)51, (char)196, (char)29, (char)197, (char)252, (char)127, (char)117, (char)88, (char)198, (char)111, (char)244, (char)28, (char)29, (char)246, (char)220, (char)95, (char)101, (char)247, (char)40, (char)2, (char)132, (char)221, (char)3, (char)230, (char)11, (char)149, (char)68, (char)60, (char)31, (char)23, (char)17, (char)161, (char)86, (char)194, (char)91, (char)15, (char)248, (char)184, (char)102, (char)197, (char)133, (char)110, (char)62, (char)160, (char)240, (char)22, (char)80, (char)97, (char)129, (char)83, (char)134, (char)44, (char)68, (char)69, (char)207, (char)86, (char)123, (char)90, (char)247, (char)53, (char)179, (char)141, (char)50, (char)111, (char)176, (char)12, (char)171, (char)210, (char)167, (char)69, (char)237, (char)254, (char)42, (char)219, (char)222, (char)14, (char)170, (char)74, (char)189, (char)111, (char)48, (char)73, (char)87, (char)198, (char)82, (char)153, (char)83, (char)146, (char)199, (char)152, (char)147, (char)20, (char)2, (char)127, (char)74, (char)113, (char)126, (char)158, (char)119, (char)204, (char)40, (char)176, (char)247, (char)239, (char)150, (char)98, (char)98, (char)189, (char)20, (char)45, (char)199, (char)65, (char)242, (char)25, (char)51, (char)145, (char)235, (char)204, (char)90, (char)0, (char)93, (char)204, (char)241, (char)222, (char)75, (char)61, (char)2, (char)156, (char)56, (char)90, (char)106, (char)29, (char)65, (char)68, (char)46, (char)81, (char)181, (char)18, (char)218, (char)189, (char)14, (char)11, (char)249, (char)170, (char)228, (char)160, (char)142, (char)102, (char)96, (char)235, (char)189, (char)27, (char)90, (char)35, (char)12, (char)171, (char)32, (char)199, (char)76, (char)236, (char)245, (char)164, (char)133, (char)193, (char)204, (char)35, (char)103, (char)3, (char)91, (char)74, (char)176, (char)14, (char)33, (char)226, (char)121, (char)150, (char)68, (char)23, (char)253, (char)194, (char)13, (char)74, (char)134, (char)233, (char)144, (char)185, (char)23, (char)111, (char)226, (char)162, (char)204, (char)131, (char)254, (char)50, (char)174, (char)64, (char)117, (char)160, (char)185, (char)201, (char)88, (char)219, (char)95, (char)181, (char)8, (char)148, (char)188, (char)165, (char)103, (char)155, (char)121, (char)157, (char)93, (char)167, (char)251, (char)208, (char)85, (char)7, (char)78, (char)160, (char)221, (char)225, (char)137, (char)36, (char)72, (char)117, (char)251, (char)98, (char)85, (char)47, (char)223, (char)238, (char)88, (char)19, (char)75, (char)205, (char)202, (char)166, (char)162, (char)221, (char)195, (char)127, (char)38, (char)29, (char)179, (char)59}));
        });
        DemoDevice.FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_network_SET((char)42) ;
        p110.target_component_SET((char)51) ;
        p110.payload_SET(new char[] {(char)118, (char)176, (char)124, (char)153, (char)253, (char)51, (char)196, (char)29, (char)197, (char)252, (char)127, (char)117, (char)88, (char)198, (char)111, (char)244, (char)28, (char)29, (char)246, (char)220, (char)95, (char)101, (char)247, (char)40, (char)2, (char)132, (char)221, (char)3, (char)230, (char)11, (char)149, (char)68, (char)60, (char)31, (char)23, (char)17, (char)161, (char)86, (char)194, (char)91, (char)15, (char)248, (char)184, (char)102, (char)197, (char)133, (char)110, (char)62, (char)160, (char)240, (char)22, (char)80, (char)97, (char)129, (char)83, (char)134, (char)44, (char)68, (char)69, (char)207, (char)86, (char)123, (char)90, (char)247, (char)53, (char)179, (char)141, (char)50, (char)111, (char)176, (char)12, (char)171, (char)210, (char)167, (char)69, (char)237, (char)254, (char)42, (char)219, (char)222, (char)14, (char)170, (char)74, (char)189, (char)111, (char)48, (char)73, (char)87, (char)198, (char)82, (char)153, (char)83, (char)146, (char)199, (char)152, (char)147, (char)20, (char)2, (char)127, (char)74, (char)113, (char)126, (char)158, (char)119, (char)204, (char)40, (char)176, (char)247, (char)239, (char)150, (char)98, (char)98, (char)189, (char)20, (char)45, (char)199, (char)65, (char)242, (char)25, (char)51, (char)145, (char)235, (char)204, (char)90, (char)0, (char)93, (char)204, (char)241, (char)222, (char)75, (char)61, (char)2, (char)156, (char)56, (char)90, (char)106, (char)29, (char)65, (char)68, (char)46, (char)81, (char)181, (char)18, (char)218, (char)189, (char)14, (char)11, (char)249, (char)170, (char)228, (char)160, (char)142, (char)102, (char)96, (char)235, (char)189, (char)27, (char)90, (char)35, (char)12, (char)171, (char)32, (char)199, (char)76, (char)236, (char)245, (char)164, (char)133, (char)193, (char)204, (char)35, (char)103, (char)3, (char)91, (char)74, (char)176, (char)14, (char)33, (char)226, (char)121, (char)150, (char)68, (char)23, (char)253, (char)194, (char)13, (char)74, (char)134, (char)233, (char)144, (char)185, (char)23, (char)111, (char)226, (char)162, (char)204, (char)131, (char)254, (char)50, (char)174, (char)64, (char)117, (char)160, (char)185, (char)201, (char)88, (char)219, (char)95, (char)181, (char)8, (char)148, (char)188, (char)165, (char)103, (char)155, (char)121, (char)157, (char)93, (char)167, (char)251, (char)208, (char)85, (char)7, (char)78, (char)160, (char)221, (char)225, (char)137, (char)36, (char)72, (char)117, (char)251, (char)98, (char)85, (char)47, (char)223, (char)238, (char)88, (char)19, (char)75, (char)205, (char)202, (char)166, (char)162, (char)221, (char)195, (char)127, (char)38, (char)29, (char)179, (char)59}, 0) ;
        p110.target_system_SET((char)188) ;
        LoopBackDemoChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.ts1_GET() == -243509557057940656L);
            assert(pack.tc1_GET() == -1889397776372825786L);
        });
        DemoDevice.TIMESYNC p111 = LoopBackDemoChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.ts1_SET(-243509557057940656L) ;
        p111.tc1_SET(-1889397776372825786L) ;
        LoopBackDemoChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 7508926377029861127L);
            assert(pack.seq_GET() == 3212362972L);
        });
        DemoDevice.CAMERA_TRIGGER p112 = LoopBackDemoChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.seq_SET(3212362972L) ;
        p112.time_usec_SET(7508926377029861127L) ;
        LoopBackDemoChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.fix_type_GET() == (char)65);
            assert(pack.ve_GET() == (short)26338);
            assert(pack.vd_GET() == (short) -29438);
            assert(pack.satellites_visible_GET() == (char)245);
            assert(pack.alt_GET() == -283343119);
            assert(pack.lon_GET() == -92147939);
            assert(pack.cog_GET() == (char)63592);
            assert(pack.vel_GET() == (char)62931);
            assert(pack.lat_GET() == -1021290080);
            assert(pack.time_usec_GET() == 5153334122828124738L);
            assert(pack.epv_GET() == (char)39067);
            assert(pack.vn_GET() == (short) -19118);
            assert(pack.eph_GET() == (char)12492);
        });
        DemoDevice.HIL_GPS p113 = LoopBackDemoChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.time_usec_SET(5153334122828124738L) ;
        p113.lon_SET(-92147939) ;
        p113.ve_SET((short)26338) ;
        p113.vd_SET((short) -29438) ;
        p113.alt_SET(-283343119) ;
        p113.fix_type_SET((char)65) ;
        p113.cog_SET((char)63592) ;
        p113.satellites_visible_SET((char)245) ;
        p113.vel_SET((char)62931) ;
        p113.eph_SET((char)12492) ;
        p113.vn_SET((short) -19118) ;
        p113.epv_SET((char)39067) ;
        p113.lat_SET(-1021290080) ;
        LoopBackDemoChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.quality_GET() == (char)17);
            assert(pack.integrated_xgyro_GET() == 2.4461267E38F);
            assert(pack.time_delta_distance_us_GET() == 2312967707L);
            assert(pack.integrated_y_GET() == -2.2624468E38F);
            assert(pack.integrated_ygyro_GET() == 2.0763271E37F);
            assert(pack.time_usec_GET() == 61069538313619747L);
            assert(pack.temperature_GET() == (short) -19882);
            assert(pack.distance_GET() == 1.2690759E38F);
            assert(pack.integrated_zgyro_GET() == 1.5188658E38F);
            assert(pack.integrated_x_GET() == -2.9946095E38F);
            assert(pack.sensor_id_GET() == (char)158);
            assert(pack.integration_time_us_GET() == 2963984938L);
        });
        DemoDevice.HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.temperature_SET((short) -19882) ;
        p114.integrated_y_SET(-2.2624468E38F) ;
        p114.distance_SET(1.2690759E38F) ;
        p114.time_delta_distance_us_SET(2312967707L) ;
        p114.time_usec_SET(61069538313619747L) ;
        p114.integrated_zgyro_SET(1.5188658E38F) ;
        p114.integration_time_us_SET(2963984938L) ;
        p114.integrated_ygyro_SET(2.0763271E37F) ;
        p114.integrated_x_SET(-2.9946095E38F) ;
        p114.sensor_id_SET((char)158) ;
        p114.integrated_xgyro_SET(2.4461267E38F) ;
        p114.quality_SET((char)17) ;
        LoopBackDemoChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == (short) -15204);
            assert(pack.true_airspeed_GET() == (char)14371);
            assert(pack.rollspeed_GET() == -5.749009E37F);
            assert(pack.vz_GET() == (short)184);
            assert(pack.zacc_GET() == (short)13987);
            assert(pack.time_usec_GET() == 3721327802443233396L);
            assert(pack.lat_GET() == 822352675);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {2.3505207E38F, -2.2001107E38F, 1.4386663E38F, 7.61638E37F}));
            assert(pack.xacc_GET() == (short)22854);
            assert(pack.yacc_GET() == (short)29826);
            assert(pack.vy_GET() == (short) -17952);
            assert(pack.pitchspeed_GET() == -3.3809258E38F);
            assert(pack.alt_GET() == -1356006743);
            assert(pack.ind_airspeed_GET() == (char)48441);
            assert(pack.yawspeed_GET() == 1.4807644E38F);
            assert(pack.lon_GET() == -2124727102);
        });
        DemoDevice.HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.attitude_quaternion_SET(new float[] {2.3505207E38F, -2.2001107E38F, 1.4386663E38F, 7.61638E37F}, 0) ;
        p115.ind_airspeed_SET((char)48441) ;
        p115.time_usec_SET(3721327802443233396L) ;
        p115.xacc_SET((short)22854) ;
        p115.yacc_SET((short)29826) ;
        p115.lon_SET(-2124727102) ;
        p115.alt_SET(-1356006743) ;
        p115.vy_SET((short) -17952) ;
        p115.vx_SET((short) -15204) ;
        p115.zacc_SET((short)13987) ;
        p115.pitchspeed_SET(-3.3809258E38F) ;
        p115.true_airspeed_SET((char)14371) ;
        p115.rollspeed_SET(-5.749009E37F) ;
        p115.vz_SET((short)184) ;
        p115.yawspeed_SET(1.4807644E38F) ;
        p115.lat_SET(822352675) ;
        LoopBackDemoChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.xgyro_GET() == (short) -10808);
            assert(pack.zgyro_GET() == (short) -1541);
            assert(pack.xacc_GET() == (short) -15714);
            assert(pack.ygyro_GET() == (short)19314);
            assert(pack.zmag_GET() == (short)26596);
            assert(pack.yacc_GET() == (short) -27936);
            assert(pack.time_boot_ms_GET() == 1968100674L);
            assert(pack.ymag_GET() == (short) -6671);
            assert(pack.zacc_GET() == (short)9737);
            assert(pack.xmag_GET() == (short) -23809);
        });
        DemoDevice.SCALED_IMU2 p116 = LoopBackDemoChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.ygyro_SET((short)19314) ;
        p116.xmag_SET((short) -23809) ;
        p116.yacc_SET((short) -27936) ;
        p116.time_boot_ms_SET(1968100674L) ;
        p116.zgyro_SET((short) -1541) ;
        p116.xacc_SET((short) -15714) ;
        p116.xgyro_SET((short) -10808) ;
        p116.ymag_SET((short) -6671) ;
        p116.zacc_SET((short)9737) ;
        p116.zmag_SET((short)26596) ;
        LoopBackDemoChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)252);
            assert(pack.end_GET() == (char)9290);
            assert(pack.target_system_GET() == (char)185);
            assert(pack.start_GET() == (char)64106);
        });
        DemoDevice.LOG_REQUEST_LIST p117 = LoopBackDemoChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_component_SET((char)252) ;
        p117.end_SET((char)9290) ;
        p117.start_SET((char)64106) ;
        p117.target_system_SET((char)185) ;
        LoopBackDemoChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.num_logs_GET() == (char)4276);
            assert(pack.id_GET() == (char)57190);
            assert(pack.time_utc_GET() == 1074052912L);
            assert(pack.last_log_num_GET() == (char)46964);
            assert(pack.size_GET() == 2134048637L);
        });
        DemoDevice.LOG_ENTRY p118 = LoopBackDemoChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.size_SET(2134048637L) ;
        p118.id_SET((char)57190) ;
        p118.num_logs_SET((char)4276) ;
        p118.last_log_num_SET((char)46964) ;
        p118.time_utc_SET(1074052912L) ;
        LoopBackDemoChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == 3335118143L);
            assert(pack.id_GET() == (char)58814);
            assert(pack.ofs_GET() == 614368749L);
            assert(pack.target_component_GET() == (char)184);
            assert(pack.target_system_GET() == (char)226);
        });
        DemoDevice.LOG_REQUEST_DATA p119 = LoopBackDemoChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.ofs_SET(614368749L) ;
        p119.count_SET(3335118143L) ;
        p119.target_component_SET((char)184) ;
        p119.target_system_SET((char)226) ;
        p119.id_SET((char)58814) ;
        LoopBackDemoChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)50812);
            assert(pack.ofs_GET() == 3230578295L);
            assert(pack.count_GET() == (char)231);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)159, (char)90, (char)197, (char)71, (char)119, (char)103, (char)17, (char)88, (char)124, (char)121, (char)99, (char)213, (char)177, (char)204, (char)39, (char)122, (char)93, (char)236, (char)69, (char)146, (char)62, (char)120, (char)68, (char)210, (char)239, (char)93, (char)123, (char)208, (char)12, (char)31, (char)45, (char)51, (char)67, (char)180, (char)106, (char)71, (char)240, (char)176, (char)194, (char)209, (char)186, (char)113, (char)57, (char)35, (char)13, (char)211, (char)20, (char)139, (char)34, (char)91, (char)0, (char)91, (char)164, (char)177, (char)43, (char)160, (char)116, (char)205, (char)236, (char)102, (char)239, (char)239, (char)195, (char)84, (char)237, (char)95, (char)74, (char)224, (char)54, (char)227, (char)10, (char)120, (char)214, (char)160, (char)162, (char)20, (char)238, (char)54, (char)150, (char)172, (char)16, (char)161, (char)206, (char)158, (char)202, (char)165, (char)150, (char)75, (char)234, (char)147}));
        });
        DemoDevice.LOG_DATA p120 = LoopBackDemoChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.data__SET(new char[] {(char)159, (char)90, (char)197, (char)71, (char)119, (char)103, (char)17, (char)88, (char)124, (char)121, (char)99, (char)213, (char)177, (char)204, (char)39, (char)122, (char)93, (char)236, (char)69, (char)146, (char)62, (char)120, (char)68, (char)210, (char)239, (char)93, (char)123, (char)208, (char)12, (char)31, (char)45, (char)51, (char)67, (char)180, (char)106, (char)71, (char)240, (char)176, (char)194, (char)209, (char)186, (char)113, (char)57, (char)35, (char)13, (char)211, (char)20, (char)139, (char)34, (char)91, (char)0, (char)91, (char)164, (char)177, (char)43, (char)160, (char)116, (char)205, (char)236, (char)102, (char)239, (char)239, (char)195, (char)84, (char)237, (char)95, (char)74, (char)224, (char)54, (char)227, (char)10, (char)120, (char)214, (char)160, (char)162, (char)20, (char)238, (char)54, (char)150, (char)172, (char)16, (char)161, (char)206, (char)158, (char)202, (char)165, (char)150, (char)75, (char)234, (char)147}, 0) ;
        p120.count_SET((char)231) ;
        p120.id_SET((char)50812) ;
        p120.ofs_SET(3230578295L) ;
        LoopBackDemoChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)125);
            assert(pack.target_component_GET() == (char)98);
        });
        DemoDevice.LOG_ERASE p121 = LoopBackDemoChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_component_SET((char)98) ;
        p121.target_system_SET((char)125) ;
        LoopBackDemoChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)27);
            assert(pack.target_component_GET() == (char)1);
        });
        DemoDevice.LOG_REQUEST_END p122 = LoopBackDemoChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)27) ;
        p122.target_component_SET((char)1) ;
        LoopBackDemoChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.len_GET() == (char)57);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)25, (char)189, (char)68, (char)157, (char)16, (char)125, (char)102, (char)156, (char)85, (char)112, (char)29, (char)255, (char)115, (char)179, (char)34, (char)36, (char)57, (char)201, (char)55, (char)10, (char)137, (char)42, (char)173, (char)201, (char)24, (char)141, (char)175, (char)28, (char)155, (char)243, (char)194, (char)181, (char)248, (char)208, (char)185, (char)63, (char)168, (char)147, (char)241, (char)26, (char)80, (char)214, (char)194, (char)139, (char)64, (char)46, (char)32, (char)118, (char)40, (char)108, (char)28, (char)192, (char)29, (char)181, (char)198, (char)91, (char)153, (char)243, (char)9, (char)84, (char)25, (char)156, (char)98, (char)36, (char)1, (char)45, (char)27, (char)108, (char)19, (char)125, (char)165, (char)67, (char)92, (char)118, (char)90, (char)209, (char)245, (char)236, (char)197, (char)222, (char)179, (char)15, (char)115, (char)148, (char)74, (char)20, (char)113, (char)195, (char)11, (char)44, (char)78, (char)143, (char)243, (char)70, (char)39, (char)210, (char)68, (char)80, (char)158, (char)221, (char)183, (char)200, (char)3, (char)152, (char)107, (char)47, (char)119, (char)57, (char)10, (char)184}));
            assert(pack.target_component_GET() == (char)205);
            assert(pack.target_system_GET() == (char)134);
        });
        DemoDevice.GPS_INJECT_DATA p123 = LoopBackDemoChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_component_SET((char)205) ;
        p123.target_system_SET((char)134) ;
        p123.len_SET((char)57) ;
        p123.data__SET(new char[] {(char)25, (char)189, (char)68, (char)157, (char)16, (char)125, (char)102, (char)156, (char)85, (char)112, (char)29, (char)255, (char)115, (char)179, (char)34, (char)36, (char)57, (char)201, (char)55, (char)10, (char)137, (char)42, (char)173, (char)201, (char)24, (char)141, (char)175, (char)28, (char)155, (char)243, (char)194, (char)181, (char)248, (char)208, (char)185, (char)63, (char)168, (char)147, (char)241, (char)26, (char)80, (char)214, (char)194, (char)139, (char)64, (char)46, (char)32, (char)118, (char)40, (char)108, (char)28, (char)192, (char)29, (char)181, (char)198, (char)91, (char)153, (char)243, (char)9, (char)84, (char)25, (char)156, (char)98, (char)36, (char)1, (char)45, (char)27, (char)108, (char)19, (char)125, (char)165, (char)67, (char)92, (char)118, (char)90, (char)209, (char)245, (char)236, (char)197, (char)222, (char)179, (char)15, (char)115, (char)148, (char)74, (char)20, (char)113, (char)195, (char)11, (char)44, (char)78, (char)143, (char)243, (char)70, (char)39, (char)210, (char)68, (char)80, (char)158, (char)221, (char)183, (char)200, (char)3, (char)152, (char)107, (char)47, (char)119, (char)57, (char)10, (char)184}, 0) ;
        LoopBackDemoChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
            assert(pack.vel_GET() == (char)62171);
            assert(pack.satellites_visible_GET() == (char)246);
            assert(pack.cog_GET() == (char)9430);
            assert(pack.lon_GET() == -1389432348);
            assert(pack.alt_GET() == -131436671);
            assert(pack.eph_GET() == (char)8707);
            assert(pack.time_usec_GET() == 3209720862173361270L);
            assert(pack.epv_GET() == (char)55549);
            assert(pack.dgps_numch_GET() == (char)140);
            assert(pack.lat_GET() == -184921955);
            assert(pack.dgps_age_GET() == 1837942392L);
        });
        DemoDevice.GPS2_RAW p124 = LoopBackDemoChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.lon_SET(-1389432348) ;
        p124.time_usec_SET(3209720862173361270L) ;
        p124.epv_SET((char)55549) ;
        p124.satellites_visible_SET((char)246) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX) ;
        p124.eph_SET((char)8707) ;
        p124.lat_SET(-184921955) ;
        p124.alt_SET(-131436671) ;
        p124.dgps_numch_SET((char)140) ;
        p124.cog_SET((char)9430) ;
        p124.vel_SET((char)62171) ;
        p124.dgps_age_SET(1837942392L) ;
        LoopBackDemoChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID);
            assert(pack.Vcc_GET() == (char)65207);
            assert(pack.Vservo_GET() == (char)48451);
        });
        DemoDevice.POWER_STATUS p125 = LoopBackDemoChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID) ;
        p125.Vcc_SET((char)65207) ;
        p125.Vservo_SET((char)48451) ;
        LoopBackDemoChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.timeout_GET() == (char)42988);
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2);
            assert(pack.count_GET() == (char)35);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)222, (char)52, (char)115, (char)145, (char)11, (char)234, (char)101, (char)161, (char)19, (char)206, (char)226, (char)58, (char)116, (char)175, (char)105, (char)11, (char)19, (char)26, (char)101, (char)10, (char)124, (char)244, (char)196, (char)87, (char)24, (char)114, (char)169, (char)1, (char)96, (char)14, (char)83, (char)206, (char)164, (char)47, (char)145, (char)59, (char)83, (char)136, (char)219, (char)247, (char)125, (char)122, (char)124, (char)205, (char)171, (char)227, (char)24, (char)63, (char)82, (char)176, (char)83, (char)50, (char)28, (char)47, (char)223, (char)232, (char)248, (char)153, (char)53, (char)60, (char)57, (char)58, (char)57, (char)99, (char)200, (char)188, (char)11, (char)96, (char)218, (char)5}));
            assert(pack.baudrate_GET() == 2182143366L);
            assert(pack.flags_GET() == SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY);
        });
        DemoDevice.SERIAL_CONTROL p126 = LoopBackDemoChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.baudrate_SET(2182143366L) ;
        p126.timeout_SET((char)42988) ;
        p126.count_SET((char)35) ;
        p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY) ;
        p126.data__SET(new char[] {(char)222, (char)52, (char)115, (char)145, (char)11, (char)234, (char)101, (char)161, (char)19, (char)206, (char)226, (char)58, (char)116, (char)175, (char)105, (char)11, (char)19, (char)26, (char)101, (char)10, (char)124, (char)244, (char)196, (char)87, (char)24, (char)114, (char)169, (char)1, (char)96, (char)14, (char)83, (char)206, (char)164, (char)47, (char)145, (char)59, (char)83, (char)136, (char)219, (char)247, (char)125, (char)122, (char)124, (char)205, (char)171, (char)227, (char)24, (char)63, (char)82, (char)176, (char)83, (char)50, (char)28, (char)47, (char)223, (char)232, (char)248, (char)153, (char)53, (char)60, (char)57, (char)58, (char)57, (char)99, (char)200, (char)188, (char)11, (char)96, (char)218, (char)5}, 0) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2) ;
        LoopBackDemoChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.rtk_rate_GET() == (char)225);
            assert(pack.baseline_coords_type_GET() == (char)5);
            assert(pack.accuracy_GET() == 3389344502L);
            assert(pack.rtk_health_GET() == (char)85);
            assert(pack.nsats_GET() == (char)123);
            assert(pack.iar_num_hypotheses_GET() == -551154862);
            assert(pack.baseline_c_mm_GET() == 415228640);
            assert(pack.baseline_b_mm_GET() == 1586144432);
            assert(pack.baseline_a_mm_GET() == 1724754187);
            assert(pack.tow_GET() == 3921151582L);
            assert(pack.wn_GET() == (char)26407);
            assert(pack.rtk_receiver_id_GET() == (char)96);
            assert(pack.time_last_baseline_ms_GET() == 3900423410L);
        });
        DemoDevice.GPS_RTK p127 = LoopBackDemoChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.rtk_receiver_id_SET((char)96) ;
        p127.nsats_SET((char)123) ;
        p127.baseline_coords_type_SET((char)5) ;
        p127.wn_SET((char)26407) ;
        p127.rtk_rate_SET((char)225) ;
        p127.baseline_c_mm_SET(415228640) ;
        p127.tow_SET(3921151582L) ;
        p127.baseline_b_mm_SET(1586144432) ;
        p127.iar_num_hypotheses_SET(-551154862) ;
        p127.time_last_baseline_ms_SET(3900423410L) ;
        p127.baseline_a_mm_SET(1724754187) ;
        p127.accuracy_SET(3389344502L) ;
        p127.rtk_health_SET((char)85) ;
        LoopBackDemoChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.baseline_b_mm_GET() == -1296085901);
            assert(pack.baseline_coords_type_GET() == (char)199);
            assert(pack.rtk_health_GET() == (char)162);
            assert(pack.accuracy_GET() == 642772508L);
            assert(pack.rtk_rate_GET() == (char)183);
            assert(pack.tow_GET() == 2372697867L);
            assert(pack.wn_GET() == (char)19777);
            assert(pack.nsats_GET() == (char)245);
            assert(pack.rtk_receiver_id_GET() == (char)236);
            assert(pack.baseline_c_mm_GET() == -2068243817);
            assert(pack.iar_num_hypotheses_GET() == 1305636951);
            assert(pack.baseline_a_mm_GET() == -288488076);
            assert(pack.time_last_baseline_ms_GET() == 3539759017L);
        });
        DemoDevice.GPS2_RTK p128 = LoopBackDemoChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.accuracy_SET(642772508L) ;
        p128.rtk_health_SET((char)162) ;
        p128.baseline_coords_type_SET((char)199) ;
        p128.baseline_c_mm_SET(-2068243817) ;
        p128.time_last_baseline_ms_SET(3539759017L) ;
        p128.tow_SET(2372697867L) ;
        p128.nsats_SET((char)245) ;
        p128.baseline_a_mm_SET(-288488076) ;
        p128.iar_num_hypotheses_SET(1305636951) ;
        p128.rtk_receiver_id_SET((char)236) ;
        p128.baseline_b_mm_SET(-1296085901) ;
        p128.wn_SET((char)19777) ;
        p128.rtk_rate_SET((char)183) ;
        LoopBackDemoChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.xgyro_GET() == (short)15343);
            assert(pack.zmag_GET() == (short)12767);
            assert(pack.xacc_GET() == (short) -25337);
            assert(pack.ygyro_GET() == (short) -29228);
            assert(pack.zgyro_GET() == (short)32284);
            assert(pack.time_boot_ms_GET() == 1029513461L);
            assert(pack.yacc_GET() == (short) -6016);
            assert(pack.zacc_GET() == (short)5508);
            assert(pack.ymag_GET() == (short)1964);
            assert(pack.xmag_GET() == (short) -22960);
        });
        DemoDevice.SCALED_IMU3 p129 = LoopBackDemoChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.ygyro_SET((short) -29228) ;
        p129.zacc_SET((short)5508) ;
        p129.xacc_SET((short) -25337) ;
        p129.time_boot_ms_SET(1029513461L) ;
        p129.yacc_SET((short) -6016) ;
        p129.xgyro_SET((short)15343) ;
        p129.ymag_SET((short)1964) ;
        p129.zmag_SET((short)12767) ;
        p129.zgyro_SET((short)32284) ;
        p129.xmag_SET((short) -22960) ;
        LoopBackDemoChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == (char)151);
            assert(pack.packets_GET() == (char)49270);
            assert(pack.jpg_quality_GET() == (char)92);
            assert(pack.size_GET() == 3620266671L);
            assert(pack.payload_GET() == (char)167);
            assert(pack.width_GET() == (char)63929);
            assert(pack.height_GET() == (char)29248);
        });
        DemoDevice.DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.height_SET((char)29248) ;
        p130.jpg_quality_SET((char)92) ;
        p130.size_SET(3620266671L) ;
        p130.packets_SET((char)49270) ;
        p130.width_SET((char)63929) ;
        p130.type_SET((char)151) ;
        p130.payload_SET((char)167) ;
        LoopBackDemoChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)62, (char)131, (char)44, (char)218, (char)233, (char)237, (char)225, (char)237, (char)13, (char)8, (char)48, (char)243, (char)208, (char)142, (char)233, (char)164, (char)33, (char)219, (char)128, (char)19, (char)160, (char)199, (char)8, (char)29, (char)183, (char)67, (char)188, (char)81, (char)226, (char)207, (char)172, (char)127, (char)12, (char)181, (char)249, (char)20, (char)28, (char)10, (char)89, (char)106, (char)92, (char)65, (char)41, (char)13, (char)176, (char)232, (char)95, (char)235, (char)194, (char)152, (char)184, (char)132, (char)7, (char)79, (char)239, (char)161, (char)95, (char)146, (char)91, (char)216, (char)171, (char)9, (char)118, (char)233, (char)91, (char)213, (char)66, (char)237, (char)202, (char)204, (char)46, (char)59, (char)122, (char)140, (char)36, (char)57, (char)25, (char)20, (char)202, (char)134, (char)59, (char)112, (char)132, (char)100, (char)215, (char)29, (char)230, (char)158, (char)244, (char)4, (char)19, (char)216, (char)19, (char)117, (char)226, (char)108, (char)59, (char)76, (char)115, (char)116, (char)101, (char)229, (char)157, (char)190, (char)57, (char)34, (char)204, (char)226, (char)44, (char)176, (char)24, (char)213, (char)135, (char)139, (char)236, (char)193, (char)245, (char)32, (char)235, (char)107, (char)20, (char)31, (char)214, (char)202, (char)45, (char)177, (char)58, (char)107, (char)106, (char)80, (char)55, (char)22, (char)211, (char)34, (char)237, (char)179, (char)24, (char)211, (char)61, (char)118, (char)114, (char)67, (char)66, (char)33, (char)193, (char)179, (char)4, (char)152, (char)134, (char)24, (char)142, (char)202, (char)66, (char)111, (char)150, (char)210, (char)214, (char)64, (char)65, (char)227, (char)37, (char)53, (char)177, (char)171, (char)108, (char)224, (char)206, (char)230, (char)59, (char)123, (char)226, (char)42, (char)252, (char)11, (char)115, (char)183, (char)186, (char)183, (char)227, (char)33, (char)86, (char)49, (char)168, (char)251, (char)224, (char)160, (char)24, (char)139, (char)73, (char)51, (char)136, (char)223, (char)132, (char)232, (char)94, (char)138, (char)245, (char)175, (char)19, (char)240, (char)117, (char)214, (char)135, (char)229, (char)208, (char)18, (char)27, (char)134, (char)156, (char)45, (char)141, (char)237, (char)223, (char)134, (char)35, (char)144, (char)144, (char)56, (char)102, (char)159, (char)219, (char)138, (char)37, (char)128, (char)238, (char)229, (char)87, (char)144, (char)45, (char)239, (char)164, (char)79, (char)69, (char)110, (char)212, (char)228, (char)174, (char)108, (char)27, (char)103, (char)102, (char)247, (char)191, (char)185, (char)129, (char)22, (char)250, (char)196, (char)48, (char)53, (char)211, (char)170, (char)230}));
            assert(pack.seqnr_GET() == (char)43259);
        });
        DemoDevice.ENCAPSULATED_DATA p131 = LoopBackDemoChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)43259) ;
        p131.data__SET(new char[] {(char)62, (char)131, (char)44, (char)218, (char)233, (char)237, (char)225, (char)237, (char)13, (char)8, (char)48, (char)243, (char)208, (char)142, (char)233, (char)164, (char)33, (char)219, (char)128, (char)19, (char)160, (char)199, (char)8, (char)29, (char)183, (char)67, (char)188, (char)81, (char)226, (char)207, (char)172, (char)127, (char)12, (char)181, (char)249, (char)20, (char)28, (char)10, (char)89, (char)106, (char)92, (char)65, (char)41, (char)13, (char)176, (char)232, (char)95, (char)235, (char)194, (char)152, (char)184, (char)132, (char)7, (char)79, (char)239, (char)161, (char)95, (char)146, (char)91, (char)216, (char)171, (char)9, (char)118, (char)233, (char)91, (char)213, (char)66, (char)237, (char)202, (char)204, (char)46, (char)59, (char)122, (char)140, (char)36, (char)57, (char)25, (char)20, (char)202, (char)134, (char)59, (char)112, (char)132, (char)100, (char)215, (char)29, (char)230, (char)158, (char)244, (char)4, (char)19, (char)216, (char)19, (char)117, (char)226, (char)108, (char)59, (char)76, (char)115, (char)116, (char)101, (char)229, (char)157, (char)190, (char)57, (char)34, (char)204, (char)226, (char)44, (char)176, (char)24, (char)213, (char)135, (char)139, (char)236, (char)193, (char)245, (char)32, (char)235, (char)107, (char)20, (char)31, (char)214, (char)202, (char)45, (char)177, (char)58, (char)107, (char)106, (char)80, (char)55, (char)22, (char)211, (char)34, (char)237, (char)179, (char)24, (char)211, (char)61, (char)118, (char)114, (char)67, (char)66, (char)33, (char)193, (char)179, (char)4, (char)152, (char)134, (char)24, (char)142, (char)202, (char)66, (char)111, (char)150, (char)210, (char)214, (char)64, (char)65, (char)227, (char)37, (char)53, (char)177, (char)171, (char)108, (char)224, (char)206, (char)230, (char)59, (char)123, (char)226, (char)42, (char)252, (char)11, (char)115, (char)183, (char)186, (char)183, (char)227, (char)33, (char)86, (char)49, (char)168, (char)251, (char)224, (char)160, (char)24, (char)139, (char)73, (char)51, (char)136, (char)223, (char)132, (char)232, (char)94, (char)138, (char)245, (char)175, (char)19, (char)240, (char)117, (char)214, (char)135, (char)229, (char)208, (char)18, (char)27, (char)134, (char)156, (char)45, (char)141, (char)237, (char)223, (char)134, (char)35, (char)144, (char)144, (char)56, (char)102, (char)159, (char)219, (char)138, (char)37, (char)128, (char)238, (char)229, (char)87, (char)144, (char)45, (char)239, (char)164, (char)79, (char)69, (char)110, (char)212, (char)228, (char)174, (char)108, (char)27, (char)103, (char)102, (char)247, (char)191, (char)185, (char)129, (char)22, (char)250, (char)196, (char)48, (char)53, (char)211, (char)170, (char)230}, 0) ;
        LoopBackDemoChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2806786050L);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
            assert(pack.current_distance_GET() == (char)18157);
            assert(pack.id_GET() == (char)33);
            assert(pack.min_distance_GET() == (char)50602);
            assert(pack.covariance_GET() == (char)213);
            assert(pack.max_distance_GET() == (char)2130);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180);
        });
        DemoDevice.DISTANCE_SENSOR p132 = LoopBackDemoChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.min_distance_SET((char)50602) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR) ;
        p132.max_distance_SET((char)2130) ;
        p132.current_distance_SET((char)18157) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180) ;
        p132.id_SET((char)33) ;
        p132.time_boot_ms_SET(2806786050L) ;
        p132.covariance_SET((char)213) ;
        LoopBackDemoChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -2037023024);
            assert(pack.mask_GET() == 1324199549522210226L);
            assert(pack.lat_GET() == 1730401165);
            assert(pack.grid_spacing_GET() == (char)56676);
        });
        DemoDevice.TERRAIN_REQUEST p133 = LoopBackDemoChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lon_SET(-2037023024) ;
        p133.lat_SET(1730401165) ;
        p133.mask_SET(1324199549522210226L) ;
        p133.grid_spacing_SET((char)56676) ;
        LoopBackDemoChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.grid_spacing_GET() == (char)28948);
            assert(pack.lat_GET() == -885544716);
            assert(pack.gridbit_GET() == (char)111);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short)10307, (short) -24359, (short)12629, (short) -7448, (short)3875, (short) -30879, (short) -9973, (short) -14229, (short)1674, (short) -4030, (short)12195, (short)19733, (short)12655, (short)12345, (short)7686, (short)22230}));
            assert(pack.lon_GET() == -1552569083);
        });
        DemoDevice.TERRAIN_DATA p134 = LoopBackDemoChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.lat_SET(-885544716) ;
        p134.gridbit_SET((char)111) ;
        p134.grid_spacing_SET((char)28948) ;
        p134.data__SET(new short[] {(short)10307, (short) -24359, (short)12629, (short) -7448, (short)3875, (short) -30879, (short) -9973, (short) -14229, (short)1674, (short) -4030, (short)12195, (short)19733, (short)12655, (short)12345, (short)7686, (short)22230}, 0) ;
        p134.lon_SET(-1552569083) ;
        LoopBackDemoChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 1012881774);
            assert(pack.lon_GET() == -1109509528);
        });
        DemoDevice.TERRAIN_CHECK p135 = LoopBackDemoChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(1012881774) ;
        p135.lon_SET(-1109509528) ;
        LoopBackDemoChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.pending_GET() == (char)36963);
            assert(pack.spacing_GET() == (char)11078);
            assert(pack.terrain_height_GET() == -1.0525976E38F);
            assert(pack.lat_GET() == -1206201027);
            assert(pack.loaded_GET() == (char)14310);
            assert(pack.current_height_GET() == 1.7982303E38F);
            assert(pack.lon_GET() == 946062589);
        });
        DemoDevice.TERRAIN_REPORT p136 = LoopBackDemoChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.spacing_SET((char)11078) ;
        p136.loaded_SET((char)14310) ;
        p136.pending_SET((char)36963) ;
        p136.current_height_SET(1.7982303E38F) ;
        p136.lon_SET(946062589) ;
        p136.terrain_height_SET(-1.0525976E38F) ;
        p136.lat_SET(-1206201027) ;
        LoopBackDemoChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.press_diff_GET() == -1.3349284E38F);
            assert(pack.press_abs_GET() == 2.1014791E38F);
            assert(pack.time_boot_ms_GET() == 2662185538L);
            assert(pack.temperature_GET() == (short)20939);
        });
        DemoDevice.SCALED_PRESSURE2 p137 = LoopBackDemoChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.press_abs_SET(2.1014791E38F) ;
        p137.press_diff_SET(-1.3349284E38F) ;
        p137.temperature_SET((short)20939) ;
        p137.time_boot_ms_SET(2662185538L) ;
        LoopBackDemoChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 8.619023E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-6.1137683E37F, -3.5583553E37F, -2.7273424E38F, 1.0723318E38F}));
            assert(pack.z_GET() == 7.0666374E37F);
            assert(pack.x_GET() == -1.1455119E38F);
            assert(pack.time_usec_GET() == 8558961984478136577L);
        });
        DemoDevice.ATT_POS_MOCAP p138 = LoopBackDemoChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.time_usec_SET(8558961984478136577L) ;
        p138.y_SET(8.619023E37F) ;
        p138.z_SET(7.0666374E37F) ;
        p138.x_SET(-1.1455119E38F) ;
        p138.q_SET(new float[] {-6.1137683E37F, -3.5583553E37F, -2.7273424E38F, 1.0723318E38F}, 0) ;
        LoopBackDemoChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-2.0237336E38F, -5.633676E36F, 7.467143E37F, -9.557603E37F, -1.372498E38F, -1.9921117E38F, 1.805902E38F, 2.3932619E38F}));
            assert(pack.time_usec_GET() == 4456378301031792316L);
            assert(pack.group_mlx_GET() == (char)12);
            assert(pack.target_system_GET() == (char)231);
            assert(pack.target_component_GET() == (char)205);
        });
        DemoDevice.SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.target_system_SET((char)231) ;
        p139.group_mlx_SET((char)12) ;
        p139.controls_SET(new float[] {-2.0237336E38F, -5.633676E36F, 7.467143E37F, -9.557603E37F, -1.372498E38F, -1.9921117E38F, 1.805902E38F, 2.3932619E38F}, 0) ;
        p139.time_usec_SET(4456378301031792316L) ;
        p139.target_component_SET((char)205) ;
        LoopBackDemoChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {1.9805619E38F, -2.505004E38F, 3.2001996E38F, -2.8275074E38F, 1.0348925E38F, 9.723188E37F, 2.4380055E38F, 2.4851643E38F}));
            assert(pack.time_usec_GET() == 5434005408083690180L);
            assert(pack.group_mlx_GET() == (char)248);
        });
        DemoDevice.ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.time_usec_SET(5434005408083690180L) ;
        p140.controls_SET(new float[] {1.9805619E38F, -2.505004E38F, 3.2001996E38F, -2.8275074E38F, 1.0348925E38F, 9.723188E37F, 2.4380055E38F, 2.4851643E38F}, 0) ;
        p140.group_mlx_SET((char)248) ;
        LoopBackDemoChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.bottom_clearance_GET() == 1.5959261E38F);
            assert(pack.altitude_terrain_GET() == 2.2491854E38F);
            assert(pack.time_usec_GET() == 8208195150821579150L);
            assert(pack.altitude_relative_GET() == 1.3199193E38F);
            assert(pack.altitude_monotonic_GET() == 1.4094027E38F);
            assert(pack.altitude_local_GET() == -3.19394E38F);
            assert(pack.altitude_amsl_GET() == 1.0270074E37F);
        });
        DemoDevice.ALTITUDE p141 = LoopBackDemoChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.bottom_clearance_SET(1.5959261E38F) ;
        p141.altitude_terrain_SET(2.2491854E38F) ;
        p141.altitude_local_SET(-3.19394E38F) ;
        p141.altitude_amsl_SET(1.0270074E37F) ;
        p141.altitude_monotonic_SET(1.4094027E38F) ;
        p141.time_usec_SET(8208195150821579150L) ;
        p141.altitude_relative_SET(1.3199193E38F) ;
        LoopBackDemoChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)177, (char)162, (char)170, (char)83, (char)130, (char)207, (char)173, (char)93, (char)87, (char)216, (char)146, (char)221, (char)186, (char)112, (char)233, (char)73, (char)11, (char)120, (char)221, (char)14, (char)122, (char)141, (char)38, (char)55, (char)76, (char)27, (char)15, (char)181, (char)33, (char)200, (char)129, (char)102, (char)38, (char)253, (char)78, (char)166, (char)59, (char)242, (char)175, (char)224, (char)183, (char)159, (char)5, (char)59, (char)117, (char)154, (char)235, (char)24, (char)118, (char)96, (char)189, (char)212, (char)77, (char)225, (char)212, (char)47, (char)251, (char)83, (char)85, (char)129, (char)143, (char)204, (char)16, (char)147, (char)162, (char)7, (char)46, (char)36, (char)222, (char)163, (char)180, (char)67, (char)253, (char)128, (char)165, (char)213, (char)127, (char)215, (char)58, (char)188, (char)78, (char)22, (char)170, (char)87, (char)161, (char)213, (char)250, (char)222, (char)191, (char)223, (char)44, (char)53, (char)251, (char)107, (char)42, (char)173, (char)142, (char)192, (char)137, (char)102, (char)64, (char)92, (char)51, (char)235, (char)117, (char)130, (char)229, (char)165, (char)139, (char)32, (char)121, (char)148, (char)93, (char)29, (char)204, (char)133, (char)35, (char)111, (char)161, (char)158}));
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)75, (char)7, (char)130, (char)17, (char)192, (char)32, (char)194, (char)226, (char)39, (char)65, (char)94, (char)187, (char)185, (char)70, (char)125, (char)124, (char)147, (char)220, (char)53, (char)204, (char)1, (char)143, (char)19, (char)147, (char)115, (char)52, (char)125, (char)106, (char)165, (char)20, (char)219, (char)73, (char)162, (char)54, (char)215, (char)111, (char)235, (char)57, (char)180, (char)19, (char)220, (char)119, (char)87, (char)212, (char)96, (char)58, (char)12, (char)27, (char)88, (char)97, (char)9, (char)51, (char)92, (char)124, (char)149, (char)200, (char)183, (char)32, (char)250, (char)243, (char)22, (char)251, (char)215, (char)62, (char)34, (char)190, (char)221, (char)231, (char)139, (char)139, (char)47, (char)21, (char)150, (char)240, (char)177, (char)13, (char)108, (char)172, (char)137, (char)92, (char)141, (char)195, (char)192, (char)244, (char)127, (char)39, (char)227, (char)190, (char)157, (char)189, (char)122, (char)171, (char)70, (char)159, (char)51, (char)173, (char)238, (char)245, (char)96, (char)215, (char)116, (char)4, (char)118, (char)140, (char)15, (char)100, (char)41, (char)12, (char)97, (char)34, (char)189, (char)144, (char)113, (char)251, (char)69, (char)16, (char)39, (char)233, (char)21, (char)223}));
            assert(pack.transfer_type_GET() == (char)105);
            assert(pack.uri_type_GET() == (char)143);
            assert(pack.request_id_GET() == (char)103);
        });
        DemoDevice.RESOURCE_REQUEST p142 = LoopBackDemoChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.uri_type_SET((char)143) ;
        p142.uri_SET(new char[] {(char)177, (char)162, (char)170, (char)83, (char)130, (char)207, (char)173, (char)93, (char)87, (char)216, (char)146, (char)221, (char)186, (char)112, (char)233, (char)73, (char)11, (char)120, (char)221, (char)14, (char)122, (char)141, (char)38, (char)55, (char)76, (char)27, (char)15, (char)181, (char)33, (char)200, (char)129, (char)102, (char)38, (char)253, (char)78, (char)166, (char)59, (char)242, (char)175, (char)224, (char)183, (char)159, (char)5, (char)59, (char)117, (char)154, (char)235, (char)24, (char)118, (char)96, (char)189, (char)212, (char)77, (char)225, (char)212, (char)47, (char)251, (char)83, (char)85, (char)129, (char)143, (char)204, (char)16, (char)147, (char)162, (char)7, (char)46, (char)36, (char)222, (char)163, (char)180, (char)67, (char)253, (char)128, (char)165, (char)213, (char)127, (char)215, (char)58, (char)188, (char)78, (char)22, (char)170, (char)87, (char)161, (char)213, (char)250, (char)222, (char)191, (char)223, (char)44, (char)53, (char)251, (char)107, (char)42, (char)173, (char)142, (char)192, (char)137, (char)102, (char)64, (char)92, (char)51, (char)235, (char)117, (char)130, (char)229, (char)165, (char)139, (char)32, (char)121, (char)148, (char)93, (char)29, (char)204, (char)133, (char)35, (char)111, (char)161, (char)158}, 0) ;
        p142.storage_SET(new char[] {(char)75, (char)7, (char)130, (char)17, (char)192, (char)32, (char)194, (char)226, (char)39, (char)65, (char)94, (char)187, (char)185, (char)70, (char)125, (char)124, (char)147, (char)220, (char)53, (char)204, (char)1, (char)143, (char)19, (char)147, (char)115, (char)52, (char)125, (char)106, (char)165, (char)20, (char)219, (char)73, (char)162, (char)54, (char)215, (char)111, (char)235, (char)57, (char)180, (char)19, (char)220, (char)119, (char)87, (char)212, (char)96, (char)58, (char)12, (char)27, (char)88, (char)97, (char)9, (char)51, (char)92, (char)124, (char)149, (char)200, (char)183, (char)32, (char)250, (char)243, (char)22, (char)251, (char)215, (char)62, (char)34, (char)190, (char)221, (char)231, (char)139, (char)139, (char)47, (char)21, (char)150, (char)240, (char)177, (char)13, (char)108, (char)172, (char)137, (char)92, (char)141, (char)195, (char)192, (char)244, (char)127, (char)39, (char)227, (char)190, (char)157, (char)189, (char)122, (char)171, (char)70, (char)159, (char)51, (char)173, (char)238, (char)245, (char)96, (char)215, (char)116, (char)4, (char)118, (char)140, (char)15, (char)100, (char)41, (char)12, (char)97, (char)34, (char)189, (char)144, (char)113, (char)251, (char)69, (char)16, (char)39, (char)233, (char)21, (char)223}, 0) ;
        p142.transfer_type_SET((char)105) ;
        p142.request_id_SET((char)103) ;
        LoopBackDemoChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.press_diff_GET() == -7.9753984E37F);
            assert(pack.temperature_GET() == (short) -10422);
            assert(pack.press_abs_GET() == 2.1477962E38F);
            assert(pack.time_boot_ms_GET() == 1980711840L);
        });
        DemoDevice.SCALED_PRESSURE3 p143 = LoopBackDemoChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.temperature_SET((short) -10422) ;
        p143.press_diff_SET(-7.9753984E37F) ;
        p143.time_boot_ms_SET(1980711840L) ;
        p143.press_abs_SET(2.1477962E38F) ;
        LoopBackDemoChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(pack.custom_state_GET() == 7187406858627883172L);
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {2.8075903E38F, 2.2527145E38F, -2.465862E37F, -1.0627194E38F}));
            assert(Arrays.equals(pack.vel_GET(),  new float[] {1.8855757E38F, -1.7325839E38F, -2.7648875E38F}));
            assert(pack.est_capabilities_GET() == (char)57);
            assert(pack.lat_GET() == -30910338);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {-2.085459E38F, -2.1152553E37F, 1.339585E38F}));
            assert(pack.alt_GET() == 7.157039E37F);
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {-2.5816331E38F, 1.1469664E38F, 3.2636922E38F}));
            assert(pack.timestamp_GET() == 3123900199000515795L);
            assert(Arrays.equals(pack.rates_GET(),  new float[] {-3.3372356E38F, -5.629507E37F, -2.2642432E38F}));
            assert(pack.lon_GET() == -1172540266);
        });
        DemoDevice.FOLLOW_TARGET p144 = LoopBackDemoChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.est_capabilities_SET((char)57) ;
        p144.acc_SET(new float[] {-2.085459E38F, -2.1152553E37F, 1.339585E38F}, 0) ;
        p144.attitude_q_SET(new float[] {2.8075903E38F, 2.2527145E38F, -2.465862E37F, -1.0627194E38F}, 0) ;
        p144.rates_SET(new float[] {-3.3372356E38F, -5.629507E37F, -2.2642432E38F}, 0) ;
        p144.timestamp_SET(3123900199000515795L) ;
        p144.custom_state_SET(7187406858627883172L) ;
        p144.vel_SET(new float[] {1.8855757E38F, -1.7325839E38F, -2.7648875E38F}, 0) ;
        p144.position_cov_SET(new float[] {-2.5816331E38F, 1.1469664E38F, 3.2636922E38F}, 0) ;
        p144.lat_SET(-30910338) ;
        p144.lon_SET(-1172540266) ;
        p144.alt_SET(7.157039E37F) ;
        LoopBackDemoChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {-2.2140913E38F, 3.3361608E37F, -5.386054E37F}));
            assert(pack.y_vel_GET() == -2.231564E38F);
            assert(pack.time_usec_GET() == 4102336169298061507L);
            assert(pack.pitch_rate_GET() == 9.963278E37F);
            assert(pack.z_acc_GET() == 1.3207553E37F);
            assert(pack.z_pos_GET() == -2.7951726E38F);
            assert(pack.y_pos_GET() == 2.669154E37F);
            assert(pack.x_pos_GET() == 3.5406729E37F);
            assert(pack.roll_rate_GET() == -4.8793743E37F);
            assert(pack.yaw_rate_GET() == 1.9487017E38F);
            assert(pack.z_vel_GET() == 2.0368725E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.239712E38F, 3.8325923E37F, -5.2074792E36F, -2.7406525E38F}));
            assert(pack.y_acc_GET() == -3.312734E38F);
            assert(pack.x_acc_GET() == 2.1291833E36F);
            assert(pack.x_vel_GET() == 2.1644045E38F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {3.0368992E38F, 3.0373837E38F, 2.3979118E38F}));
            assert(pack.airspeed_GET() == 7.5348655E37F);
        });
        DemoDevice.CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.pitch_rate_SET(9.963278E37F) ;
        p146.z_pos_SET(-2.7951726E38F) ;
        p146.roll_rate_SET(-4.8793743E37F) ;
        p146.yaw_rate_SET(1.9487017E38F) ;
        p146.y_acc_SET(-3.312734E38F) ;
        p146.y_pos_SET(2.669154E37F) ;
        p146.pos_variance_SET(new float[] {3.0368992E38F, 3.0373837E38F, 2.3979118E38F}, 0) ;
        p146.airspeed_SET(7.5348655E37F) ;
        p146.q_SET(new float[] {-2.239712E38F, 3.8325923E37F, -5.2074792E36F, -2.7406525E38F}, 0) ;
        p146.y_vel_SET(-2.231564E38F) ;
        p146.vel_variance_SET(new float[] {-2.2140913E38F, 3.3361608E37F, -5.386054E37F}, 0) ;
        p146.time_usec_SET(4102336169298061507L) ;
        p146.z_acc_SET(1.3207553E37F) ;
        p146.x_pos_SET(3.5406729E37F) ;
        p146.x_acc_SET(2.1291833E36F) ;
        p146.z_vel_SET(2.0368725E38F) ;
        p146.x_vel_SET(2.1644045E38F) ;
        LoopBackDemoChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.battery_remaining_GET() == (byte)29);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)9721, (char)37966, (char)25151, (char)54941, (char)15584, (char)60214, (char)8269, (char)13450, (char)50852, (char)5664}));
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH);
            assert(pack.id_GET() == (char)150);
            assert(pack.temperature_GET() == (short)6906);
            assert(pack.energy_consumed_GET() == 1136501716);
            assert(pack.current_battery_GET() == (short)25554);
            assert(pack.current_consumed_GET() == 277063449);
        });
        DemoDevice.BATTERY_STATUS p147 = LoopBackDemoChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.id_SET((char)150) ;
        p147.temperature_SET((short)6906) ;
        p147.voltages_SET(new char[] {(char)9721, (char)37966, (char)25151, (char)54941, (char)15584, (char)60214, (char)8269, (char)13450, (char)50852, (char)5664}, 0) ;
        p147.current_battery_SET((short)25554) ;
        p147.energy_consumed_SET(1136501716) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD) ;
        p147.current_consumed_SET(277063449) ;
        p147.battery_remaining_SET((byte)29) ;
        LoopBackDemoChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(pack.uid_GET() == 8792677961002306703L);
            assert(pack.os_sw_version_GET() == 3543841293L);
            assert(pack.middleware_sw_version_GET() == 97868672L);
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)47, (char)76, (char)222, (char)99, (char)187, (char)37, (char)67, (char)97, (char)191, (char)100, (char)130, (char)110, (char)221, (char)24, (char)44, (char)16, (char)124, (char)96}));
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)7, (char)30, (char)130, (char)61, (char)161, (char)190, (char)31, (char)82}));
            assert(pack.board_version_GET() == 3989292736L);
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)223, (char)78, (char)197, (char)143, (char)119, (char)221, (char)51, (char)33}));
            assert(pack.flight_sw_version_GET() == 3016270908L);
            assert(pack.capabilities_GET() == MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT);
            assert(pack.vendor_id_GET() == (char)18837);
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)50, (char)161, (char)107, (char)46, (char)241, (char)37, (char)231, (char)21}));
            assert(pack.product_id_GET() == (char)10105);
        });
        DemoDevice.AUTOPILOT_VERSION p148 = LoopBackDemoChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.board_version_SET(3989292736L) ;
        p148.os_sw_version_SET(3543841293L) ;
        p148.middleware_custom_version_SET(new char[] {(char)223, (char)78, (char)197, (char)143, (char)119, (char)221, (char)51, (char)33}, 0) ;
        p148.vendor_id_SET((char)18837) ;
        p148.product_id_SET((char)10105) ;
        p148.middleware_sw_version_SET(97868672L) ;
        p148.flight_custom_version_SET(new char[] {(char)50, (char)161, (char)107, (char)46, (char)241, (char)37, (char)231, (char)21}, 0) ;
        p148.uid_SET(8792677961002306703L) ;
        p148.flight_sw_version_SET(3016270908L) ;
        p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT) ;
        p148.uid2_SET(new char[] {(char)47, (char)76, (char)222, (char)99, (char)187, (char)37, (char)67, (char)97, (char)191, (char)100, (char)130, (char)110, (char)221, (char)24, (char)44, (char)16, (char)124, (char)96}, 0, PH) ;
        p148.os_custom_version_SET(new char[] {(char)7, (char)30, (char)130, (char)61, (char)161, (char)190, (char)31, (char)82}, 0) ;
        LoopBackDemoChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.size_x_GET() == 1.7760087E38F);
            assert(pack.size_y_GET() == -5.954388E36F);
            assert(pack.y_TRY(ph) == 3.3955058E38F);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {1.4706537E38F, -3.3831604E37F, 2.1812995E38F, 1.0014493E38F}));
            assert(pack.angle_y_GET() == 2.6787427E38F);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.x_TRY(ph) == 2.0676531E38F);
            assert(pack.target_num_GET() == (char)61);
            assert(pack.position_valid_TRY(ph) == (char)206);
            assert(pack.z_TRY(ph) == -2.4459953E38F);
            assert(pack.time_usec_GET() == 8539949663290238375L);
            assert(pack.angle_x_GET() == 2.7111594E38F);
            assert(pack.distance_GET() == -1.6156578E38F);
        });
        DemoDevice.LANDING_TARGET p149 = LoopBackDemoChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.size_y_SET(-5.954388E36F) ;
        p149.size_x_SET(1.7760087E38F) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL) ;
        p149.target_num_SET((char)61) ;
        p149.time_usec_SET(8539949663290238375L) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p149.angle_x_SET(2.7111594E38F) ;
        p149.distance_SET(-1.6156578E38F) ;
        p149.q_SET(new float[] {1.4706537E38F, -3.3831604E37F, 2.1812995E38F, 1.0014493E38F}, 0, PH) ;
        p149.angle_y_SET(2.6787427E38F) ;
        p149.z_SET(-2.4459953E38F, PH) ;
        p149.y_SET(3.3955058E38F, PH) ;
        p149.position_valid_SET((char)206, PH) ;
        p149.x_SET(2.0676531E38F, PH) ;
        LoopBackDemoChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SENS_POWER.add((src, ph, pack) ->
        {
            assert(pack.adc121_cs2_amp_GET() == 3.3684614E38F);
            assert(pack.adc121_cs1_amp_GET() == 6.6784326E37F);
            assert(pack.adc121_cspb_amp_GET() == -1.4255756E38F);
            assert(pack.adc121_vspb_volt_GET() == 2.1060487E38F);
        });
        DemoDevice.SENS_POWER p201 = LoopBackDemoChannel.new_SENS_POWER();
        PH.setPack(p201);
        p201.adc121_vspb_volt_SET(2.1060487E38F) ;
        p201.adc121_cs1_amp_SET(6.6784326E37F) ;
        p201.adc121_cs2_amp_SET(3.3684614E38F) ;
        p201.adc121_cspb_amp_SET(-1.4255756E38F) ;
        LoopBackDemoChannel.instance.send(p201);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SENS_MPPT.add((src, ph, pack) ->
        {
            assert(pack.mppt3_status_GET() == (char)28);
            assert(pack.mppt2_amp_GET() == 2.15783E38F);
            assert(pack.mppt3_amp_GET() == -1.4009371E38F);
            assert(pack.mppt3_volt_GET() == -2.7190783E38F);
            assert(pack.mppt_timestamp_GET() == 2494150155960890421L);
            assert(pack.mppt2_pwm_GET() == (char)59276);
            assert(pack.mppt1_volt_GET() == 3.0280792E38F);
            assert(pack.mppt1_pwm_GET() == (char)46725);
            assert(pack.mppt2_volt_GET() == -2.6811808E38F);
            assert(pack.mppt1_amp_GET() == 2.0361019E37F);
            assert(pack.mppt2_status_GET() == (char)97);
            assert(pack.mppt3_pwm_GET() == (char)836);
            assert(pack.mppt1_status_GET() == (char)232);
        });
        DemoDevice.SENS_MPPT p202 = LoopBackDemoChannel.new_SENS_MPPT();
        PH.setPack(p202);
        p202.mppt1_pwm_SET((char)46725) ;
        p202.mppt3_volt_SET(-2.7190783E38F) ;
        p202.mppt3_pwm_SET((char)836) ;
        p202.mppt2_status_SET((char)97) ;
        p202.mppt1_amp_SET(2.0361019E37F) ;
        p202.mppt1_volt_SET(3.0280792E38F) ;
        p202.mppt3_amp_SET(-1.4009371E38F) ;
        p202.mppt_timestamp_SET(2494150155960890421L) ;
        p202.mppt1_status_SET((char)232) ;
        p202.mppt2_pwm_SET((char)59276) ;
        p202.mppt3_status_SET((char)28) ;
        p202.mppt2_amp_SET(2.15783E38F) ;
        p202.mppt2_volt_SET(-2.6811808E38F) ;
        LoopBackDemoChannel.instance.send(p202);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ASLCTRL_DATA.add((src, ph, pack) ->
        {
            assert(pack.uElev_GET() == 1.1122025E38F);
            assert(pack.q_GET() == 9.699518E37F);
            assert(pack.uRud_GET() == -1.9184644E38F);
            assert(pack.uThrot_GET() == 3.1883145E38F);
            assert(pack.RollAngleRef_GET() == 1.80231E38F);
            assert(pack.YawAngle_GET() == 5.5945574E37F);
            assert(pack.hRef_t_GET() == 1.5444026E38F);
            assert(pack.PitchAngleRef_GET() == -3.2316235E38F);
            assert(pack.timestamp_GET() == 8528187957958251734L);
            assert(pack.PitchAngle_GET() == 1.8197877E38F);
            assert(pack.RollAngle_GET() == -3.0052438E38F);
            assert(pack.uThrot2_GET() == 3.3026718E38F);
            assert(pack.pRef_GET() == -1.6547729E38F);
            assert(pack.nZ_GET() == 4.7452087E37F);
            assert(pack.h_GET() == 2.9995558E38F);
            assert(pack.uAil_GET() == -1.8714086E38F);
            assert(pack.SpoilersEngaged_GET() == (char)83);
            assert(pack.aslctrl_mode_GET() == (char)205);
            assert(pack.AirspeedRef_GET() == 1.4439454E38F);
            assert(pack.hRef_GET() == 7.298316E37F);
            assert(pack.qRef_GET() == 3.1646514E38F);
            assert(pack.YawAngleRef_GET() == -2.0414276E38F);
            assert(pack.rRef_GET() == -1.7845632E38F);
            assert(pack.r_GET() == 2.6726845E38F);
            assert(pack.p_GET() == 1.4730806E38F);
        });
        DemoDevice.ASLCTRL_DATA p203 = LoopBackDemoChannel.new_ASLCTRL_DATA();
        PH.setPack(p203);
        p203.YawAngleRef_SET(-2.0414276E38F) ;
        p203.p_SET(1.4730806E38F) ;
        p203.q_SET(9.699518E37F) ;
        p203.RollAngleRef_SET(1.80231E38F) ;
        p203.rRef_SET(-1.7845632E38F) ;
        p203.uThrot_SET(3.1883145E38F) ;
        p203.h_SET(2.9995558E38F) ;
        p203.qRef_SET(3.1646514E38F) ;
        p203.pRef_SET(-1.6547729E38F) ;
        p203.uElev_SET(1.1122025E38F) ;
        p203.hRef_SET(7.298316E37F) ;
        p203.uThrot2_SET(3.3026718E38F) ;
        p203.PitchAngleRef_SET(-3.2316235E38F) ;
        p203.nZ_SET(4.7452087E37F) ;
        p203.YawAngle_SET(5.5945574E37F) ;
        p203.uRud_SET(-1.9184644E38F) ;
        p203.aslctrl_mode_SET((char)205) ;
        p203.RollAngle_SET(-3.0052438E38F) ;
        p203.timestamp_SET(8528187957958251734L) ;
        p203.hRef_t_SET(1.5444026E38F) ;
        p203.AirspeedRef_SET(1.4439454E38F) ;
        p203.PitchAngle_SET(1.8197877E38F) ;
        p203.SpoilersEngaged_SET((char)83) ;
        p203.r_SET(2.6726845E38F) ;
        p203.uAil_SET(-1.8714086E38F) ;
        LoopBackDemoChannel.instance.send(p203);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ASLCTRL_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.i8_1_GET() == (char)89);
            assert(pack.f_1_GET() == 1.505383E38F);
            assert(pack.f_6_GET() == 1.945754E38F);
            assert(pack.i8_2_GET() == (char)45);
            assert(pack.f_3_GET() == -7.242873E37F);
            assert(pack.f_7_GET() == 1.6623824E38F);
            assert(pack.f_8_GET() == -3.3173935E37F);
            assert(pack.f_5_GET() == 3.1832546E38F);
            assert(pack.f_4_GET() == 3.0316945E38F);
            assert(pack.i32_1_GET() == 1798290010L);
            assert(pack.f_2_GET() == 1.6266219E38F);
        });
        DemoDevice.ASLCTRL_DEBUG p204 = LoopBackDemoChannel.new_ASLCTRL_DEBUG();
        PH.setPack(p204);
        p204.f_7_SET(1.6623824E38F) ;
        p204.f_5_SET(3.1832546E38F) ;
        p204.f_4_SET(3.0316945E38F) ;
        p204.f_1_SET(1.505383E38F) ;
        p204.f_6_SET(1.945754E38F) ;
        p204.f_3_SET(-7.242873E37F) ;
        p204.i8_2_SET((char)45) ;
        p204.i8_1_SET((char)89) ;
        p204.f_2_SET(1.6266219E38F) ;
        p204.f_8_SET(-3.3173935E37F) ;
        p204.i32_1_SET(1798290010L) ;
        LoopBackDemoChannel.instance.send(p204);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ASLUAV_STATUS.add((src, ph, pack) ->
        {
            assert(pack.SATCOM_status_GET() == (char)177);
            assert(pack.LED_status_GET() == (char)99);
            assert(Arrays.equals(pack.Servo_status_GET(),  new char[] {(char)177, (char)73, (char)121, (char)75, (char)231, (char)182, (char)78, (char)142}));
            assert(pack.Motor_rpm_GET() == -1.860812E38F);
        });
        DemoDevice.ASLUAV_STATUS p205 = LoopBackDemoChannel.new_ASLUAV_STATUS();
        PH.setPack(p205);
        p205.Servo_status_SET(new char[] {(char)177, (char)73, (char)121, (char)75, (char)231, (char)182, (char)78, (char)142}, 0) ;
        p205.SATCOM_status_SET((char)177) ;
        p205.LED_status_SET((char)99) ;
        p205.Motor_rpm_SET(-1.860812E38F) ;
        LoopBackDemoChannel.instance.send(p205);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_EKF_EXT.add((src, ph, pack) ->
        {
            assert(pack.alpha_GET() == 1.1605948E38F);
            assert(pack.WindZ_GET() == -9.057487E37F);
            assert(pack.beta_GET() == -1.2030534E38F);
            assert(pack.timestamp_GET() == 4203932541525308710L);
            assert(pack.Windspeed_GET() == -5.052816E37F);
            assert(pack.Airspeed_GET() == -2.286577E38F);
            assert(pack.WindDir_GET() == 3.2260325E38F);
        });
        DemoDevice.EKF_EXT p206 = LoopBackDemoChannel.new_EKF_EXT();
        PH.setPack(p206);
        p206.Windspeed_SET(-5.052816E37F) ;
        p206.WindDir_SET(3.2260325E38F) ;
        p206.beta_SET(-1.2030534E38F) ;
        p206.timestamp_SET(4203932541525308710L) ;
        p206.WindZ_SET(-9.057487E37F) ;
        p206.Airspeed_SET(-2.286577E38F) ;
        p206.alpha_SET(1.1605948E38F) ;
        LoopBackDemoChannel.instance.send(p206);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ASL_OBCTRL.add((src, ph, pack) ->
        {
            assert(pack.uRud_GET() == -4.388772E37F);
            assert(pack.timestamp_GET() == 9122373536412938974L);
            assert(pack.uThrot_GET() == 2.2320173E38F);
            assert(pack.uThrot2_GET() == 2.6833865E38F);
            assert(pack.uAilL_GET() == -8.277287E37F);
            assert(pack.uElev_GET() == 2.7216817E38F);
            assert(pack.uAilR_GET() == -2.6540389E38F);
            assert(pack.obctrl_status_GET() == (char)197);
        });
        DemoDevice.ASL_OBCTRL p207 = LoopBackDemoChannel.new_ASL_OBCTRL();
        PH.setPack(p207);
        p207.uThrot2_SET(2.6833865E38F) ;
        p207.uRud_SET(-4.388772E37F) ;
        p207.uAilL_SET(-8.277287E37F) ;
        p207.obctrl_status_SET((char)197) ;
        p207.uAilR_SET(-2.6540389E38F) ;
        p207.uElev_SET(2.7216817E38F) ;
        p207.timestamp_SET(9122373536412938974L) ;
        p207.uThrot_SET(2.2320173E38F) ;
        LoopBackDemoChannel.instance.send(p207);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SENS_ATMOS.add((src, ph, pack) ->
        {
            assert(pack.Humidity_GET() == -1.476731E38F);
            assert(pack.TempAmbient_GET() == -2.7313297E37F);
        });
        DemoDevice.SENS_ATMOS p208 = LoopBackDemoChannel.new_SENS_ATMOS();
        PH.setPack(p208);
        p208.Humidity_SET(-1.476731E38F) ;
        p208.TempAmbient_SET(-2.7313297E37F) ;
        LoopBackDemoChannel.instance.send(p208);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SENS_BATMON.add((src, ph, pack) ->
        {
            assert(pack.batterystatus_GET() == (char)48079);
            assert(pack.hostfetcontrol_GET() == (char)23954);
            assert(pack.serialnumber_GET() == (char)45057);
            assert(pack.cellvoltage1_GET() == (char)64364);
            assert(pack.cellvoltage5_GET() == (char)27628);
            assert(pack.cellvoltage4_GET() == (char)61601);
            assert(pack.SoC_GET() == (char)164);
            assert(pack.temperature_GET() == -3.1905944E37F);
            assert(pack.cellvoltage3_GET() == (char)46599);
            assert(pack.cellvoltage2_GET() == (char)18176);
            assert(pack.current_GET() == (short)9780);
            assert(pack.voltage_GET() == (char)18069);
            assert(pack.cellvoltage6_GET() == (char)48824);
        });
        DemoDevice.SENS_BATMON p209 = LoopBackDemoChannel.new_SENS_BATMON();
        PH.setPack(p209);
        p209.cellvoltage2_SET((char)18176) ;
        p209.temperature_SET(-3.1905944E37F) ;
        p209.cellvoltage1_SET((char)64364) ;
        p209.cellvoltage4_SET((char)61601) ;
        p209.cellvoltage3_SET((char)46599) ;
        p209.current_SET((short)9780) ;
        p209.batterystatus_SET((char)48079) ;
        p209.cellvoltage6_SET((char)48824) ;
        p209.SoC_SET((char)164) ;
        p209.cellvoltage5_SET((char)27628) ;
        p209.voltage_SET((char)18069) ;
        p209.hostfetcontrol_SET((char)23954) ;
        p209.serialnumber_SET((char)45057) ;
        LoopBackDemoChannel.instance.send(p209);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FW_SOARING_DATA.add((src, ph, pack) ->
        {
            assert(pack.ControlMode_GET() == (char)221);
            assert(pack.valid_GET() == (char)137);
            assert(pack.xW_GET() == 1.1507758E38F);
            assert(pack.LoiterRadius_GET() == 2.2856107E38F);
            assert(pack.TSE_dot_GET() == -3.0016246E38F);
            assert(pack.ThermalGSNorth_GET() == 2.871114E37F);
            assert(pack.LoiterDirection_GET() == 3.3721402E37F);
            assert(pack.VarR_GET() == -9.070665E37F);
            assert(pack.xLat_GET() == 1.29228794E36F);
            assert(pack.DebugVar1_GET() == -2.514128E38F);
            assert(pack.z2_DeltaRoll_GET() == 2.0183267E38F);
            assert(pack.VarLat_GET() == 2.9216087E38F);
            assert(pack.VarW_GET() == 2.571487E38F);
            assert(pack.timestamp_GET() == 88021643187473444L);
            assert(pack.z1_LocalUpdraftSpeed_GET() == -3.3711421E38F);
            assert(pack.VarLon_GET() == 6.612517E37F);
            assert(pack.ThermalGSEast_GET() == -2.4618375E38F);
            assert(pack.xR_GET() == -1.2143364E38F);
            assert(pack.z2_exp_GET() == 5.1597015E37F);
            assert(pack.DebugVar2_GET() == -1.0007756E38F);
            assert(pack.timestampModeChanged_GET() == 7866458711672500324L);
            assert(pack.DistToSoarPoint_GET() == -1.5404731E37F);
            assert(pack.vSinkExp_GET() == -1.7634468E38F);
            assert(pack.xLon_GET() == 1.5631861E38F);
            assert(pack.z1_exp_GET() == -1.4125254E38F);
        });
        DemoDevice.FW_SOARING_DATA p210 = LoopBackDemoChannel.new_FW_SOARING_DATA();
        PH.setPack(p210);
        p210.DebugVar1_SET(-2.514128E38F) ;
        p210.timestamp_SET(88021643187473444L) ;
        p210.vSinkExp_SET(-1.7634468E38F) ;
        p210.z1_exp_SET(-1.4125254E38F) ;
        p210.VarLat_SET(2.9216087E38F) ;
        p210.z1_LocalUpdraftSpeed_SET(-3.3711421E38F) ;
        p210.VarLon_SET(6.612517E37F) ;
        p210.ControlMode_SET((char)221) ;
        p210.ThermalGSEast_SET(-2.4618375E38F) ;
        p210.timestampModeChanged_SET(7866458711672500324L) ;
        p210.xLon_SET(1.5631861E38F) ;
        p210.VarR_SET(-9.070665E37F) ;
        p210.z2_DeltaRoll_SET(2.0183267E38F) ;
        p210.z2_exp_SET(5.1597015E37F) ;
        p210.VarW_SET(2.571487E38F) ;
        p210.xLat_SET(1.29228794E36F) ;
        p210.DistToSoarPoint_SET(-1.5404731E37F) ;
        p210.valid_SET((char)137) ;
        p210.LoiterDirection_SET(3.3721402E37F) ;
        p210.xW_SET(1.1507758E38F) ;
        p210.LoiterRadius_SET(2.2856107E38F) ;
        p210.xR_SET(-1.2143364E38F) ;
        p210.DebugVar2_SET(-1.0007756E38F) ;
        p210.TSE_dot_SET(-3.0016246E38F) ;
        p210.ThermalGSNorth_SET(2.871114E37F) ;
        LoopBackDemoChannel.instance.send(p210);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SENSORPOD_STATUS.add((src, ph, pack) ->
        {
            assert(pack.visensor_rate_3_GET() == (char)77);
            assert(pack.visensor_rate_1_GET() == (char)120);
            assert(pack.visensor_rate_4_GET() == (char)71);
            assert(pack.cpu_temp_GET() == (char)106);
            assert(pack.visensor_rate_2_GET() == (char)0);
            assert(pack.free_space_GET() == (char)42104);
            assert(pack.timestamp_GET() == 1426996347204818661L);
            assert(pack.recording_nodes_count_GET() == (char)203);
        });
        DemoDevice.SENSORPOD_STATUS p211 = LoopBackDemoChannel.new_SENSORPOD_STATUS();
        PH.setPack(p211);
        p211.visensor_rate_2_SET((char)0) ;
        p211.visensor_rate_4_SET((char)71) ;
        p211.timestamp_SET(1426996347204818661L) ;
        p211.recording_nodes_count_SET((char)203) ;
        p211.cpu_temp_SET((char)106) ;
        p211.free_space_SET((char)42104) ;
        p211.visensor_rate_1_SET((char)120) ;
        p211.visensor_rate_3_SET((char)77) ;
        LoopBackDemoChannel.instance.send(p211);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SENS_POWER_BOARD.add((src, ph, pack) ->
        {
            assert(pack.pwr_brd_system_volt_GET() == 5.5634715E37F);
            assert(pack.pwr_brd_servo_1_amp_GET() == -4.3430054E37F);
            assert(pack.pwr_brd_servo_3_amp_GET() == -3.1851253E38F);
            assert(pack.pwr_brd_mot_r_amp_GET() == 1.051998E38F);
            assert(pack.pwr_brd_led_status_GET() == (char)156);
            assert(pack.pwr_brd_mot_l_amp_GET() == 2.9238974E38F);
            assert(pack.timestamp_GET() == 7616425793965555618L);
            assert(pack.pwr_brd_aux_amp_GET() == 1.524067E38F);
            assert(pack.pwr_brd_servo_2_amp_GET() == 5.7312735E37F);
            assert(pack.pwr_brd_servo_volt_GET() == 6.59253E37F);
            assert(pack.pwr_brd_servo_4_amp_GET() == 3.0520568E38F);
            assert(pack.pwr_brd_status_GET() == (char)111);
        });
        DemoDevice.SENS_POWER_BOARD p212 = LoopBackDemoChannel.new_SENS_POWER_BOARD();
        PH.setPack(p212);
        p212.pwr_brd_led_status_SET((char)156) ;
        p212.pwr_brd_mot_r_amp_SET(1.051998E38F) ;
        p212.pwr_brd_status_SET((char)111) ;
        p212.pwr_brd_servo_3_amp_SET(-3.1851253E38F) ;
        p212.timestamp_SET(7616425793965555618L) ;
        p212.pwr_brd_servo_volt_SET(6.59253E37F) ;
        p212.pwr_brd_servo_1_amp_SET(-4.3430054E37F) ;
        p212.pwr_brd_servo_2_amp_SET(5.7312735E37F) ;
        p212.pwr_brd_mot_l_amp_SET(2.9238974E38F) ;
        p212.pwr_brd_servo_4_amp_SET(3.0520568E38F) ;
        p212.pwr_brd_aux_amp_SET(1.524067E38F) ;
        p212.pwr_brd_system_volt_SET(5.5634715E37F) ;
        LoopBackDemoChannel.instance.send(p212);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.hagl_ratio_GET() == -5.514261E37F);
            assert(pack.vel_ratio_GET() == -6.465015E37F);
            assert(pack.mag_ratio_GET() == -7.226473E37F);
            assert(pack.pos_vert_ratio_GET() == 2.4559325E38F);
            assert(pack.pos_horiz_accuracy_GET() == -1.569876E38F);
            assert(pack.pos_horiz_ratio_GET() == 1.7305333E38F);
            assert(pack.pos_vert_accuracy_GET() == 8.67813E34F);
            assert(pack.time_usec_GET() == 6769186406112769892L);
            assert(pack.tas_ratio_GET() == 1.9704466E38F);
            assert(pack.flags_GET() == ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS);
        });
        DemoDevice.ESTIMATOR_STATUS p230 = LoopBackDemoChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.pos_vert_ratio_SET(2.4559325E38F) ;
        p230.vel_ratio_SET(-6.465015E37F) ;
        p230.mag_ratio_SET(-7.226473E37F) ;
        p230.hagl_ratio_SET(-5.514261E37F) ;
        p230.tas_ratio_SET(1.9704466E38F) ;
        p230.pos_horiz_accuracy_SET(-1.569876E38F) ;
        p230.time_usec_SET(6769186406112769892L) ;
        p230.pos_vert_accuracy_SET(8.67813E34F) ;
        p230.pos_horiz_ratio_SET(1.7305333E38F) ;
        p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS) ;
        LoopBackDemoChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 4574218680189647608L);
            assert(pack.var_horiz_GET() == -2.8000806E38F);
            assert(pack.vert_accuracy_GET() == -2.8202275E38F);
            assert(pack.var_vert_GET() == -2.5057203E38F);
            assert(pack.wind_x_GET() == -3.3035588E37F);
            assert(pack.wind_alt_GET() == 2.2261632E38F);
            assert(pack.wind_z_GET() == -2.2843964E38F);
            assert(pack.wind_y_GET() == 2.770943E37F);
            assert(pack.horiz_accuracy_GET() == 5.824851E37F);
        });
        DemoDevice.WIND_COV p231 = LoopBackDemoChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.wind_y_SET(2.770943E37F) ;
        p231.wind_x_SET(-3.3035588E37F) ;
        p231.wind_alt_SET(2.2261632E38F) ;
        p231.horiz_accuracy_SET(5.824851E37F) ;
        p231.time_usec_SET(4574218680189647608L) ;
        p231.var_vert_SET(-2.5057203E38F) ;
        p231.var_horiz_SET(-2.8000806E38F) ;
        p231.wind_z_SET(-2.2843964E38F) ;
        p231.vert_accuracy_SET(-2.8202275E38F) ;
        LoopBackDemoChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.hdop_GET() == -3.2069814E38F);
            assert(pack.lat_GET() == -1134218770);
            assert(pack.fix_type_GET() == (char)222);
            assert(pack.alt_GET() == 2.3597908E38F);
            assert(pack.time_usec_GET() == 2159301620864523065L);
            assert(pack.vd_GET() == -8.462431E37F);
            assert(pack.time_week_GET() == (char)60511);
            assert(pack.speed_accuracy_GET() == 2.5181914E38F);
            assert(pack.vert_accuracy_GET() == -1.8973997E38F);
            assert(pack.time_week_ms_GET() == 1226588682L);
            assert(pack.vn_GET() == 9.991802E37F);
            assert(pack.ve_GET() == 1.0016886E38F);
            assert(pack.ignore_flags_GET() == GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT);
            assert(pack.horiz_accuracy_GET() == 2.9141533E38F);
            assert(pack.lon_GET() == 1564527600);
            assert(pack.gps_id_GET() == (char)64);
            assert(pack.satellites_visible_GET() == (char)254);
            assert(pack.vdop_GET() == 2.9028076E37F);
        });
        DemoDevice.GPS_INPUT p232 = LoopBackDemoChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.lat_SET(-1134218770) ;
        p232.time_week_ms_SET(1226588682L) ;
        p232.hdop_SET(-3.2069814E38F) ;
        p232.lon_SET(1564527600) ;
        p232.gps_id_SET((char)64) ;
        p232.speed_accuracy_SET(2.5181914E38F) ;
        p232.satellites_visible_SET((char)254) ;
        p232.vd_SET(-8.462431E37F) ;
        p232.vert_accuracy_SET(-1.8973997E38F) ;
        p232.time_week_SET((char)60511) ;
        p232.time_usec_SET(2159301620864523065L) ;
        p232.vn_SET(9.991802E37F) ;
        p232.fix_type_SET((char)222) ;
        p232.vdop_SET(2.9028076E37F) ;
        p232.ve_SET(1.0016886E38F) ;
        p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT) ;
        p232.alt_SET(2.3597908E38F) ;
        p232.horiz_accuracy_SET(2.9141533E38F) ;
        LoopBackDemoChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)9, (char)10, (char)99, (char)232, (char)155, (char)218, (char)71, (char)69, (char)40, (char)186, (char)3, (char)154, (char)113, (char)139, (char)243, (char)39, (char)255, (char)169, (char)206, (char)194, (char)205, (char)116, (char)190, (char)51, (char)88, (char)145, (char)42, (char)140, (char)19, (char)26, (char)46, (char)254, (char)9, (char)64, (char)148, (char)184, (char)221, (char)19, (char)173, (char)31, (char)237, (char)97, (char)137, (char)30, (char)92, (char)246, (char)72, (char)169, (char)51, (char)182, (char)125, (char)92, (char)255, (char)12, (char)6, (char)169, (char)155, (char)116, (char)236, (char)24, (char)80, (char)85, (char)79, (char)13, (char)111, (char)63, (char)215, (char)22, (char)177, (char)89, (char)248, (char)2, (char)32, (char)218, (char)253, (char)191, (char)51, (char)143, (char)59, (char)17, (char)77, (char)120, (char)236, (char)231, (char)19, (char)59, (char)130, (char)91, (char)150, (char)221, (char)194, (char)122, (char)248, (char)61, (char)129, (char)75, (char)91, (char)12, (char)77, (char)56, (char)36, (char)17, (char)182, (char)162, (char)17, (char)71, (char)250, (char)208, (char)215, (char)15, (char)45, (char)245, (char)26, (char)132, (char)189, (char)107, (char)125, (char)120, (char)25, (char)130, (char)194, (char)232, (char)100, (char)233, (char)226, (char)106, (char)223, (char)94, (char)17, (char)234, (char)228, (char)21, (char)67, (char)153, (char)145, (char)199, (char)121, (char)82, (char)144, (char)255, (char)151, (char)181, (char)159, (char)155, (char)181, (char)239, (char)66, (char)38, (char)61, (char)138, (char)203, (char)82, (char)63, (char)253, (char)153, (char)89, (char)244, (char)92, (char)173, (char)222, (char)76, (char)235, (char)224, (char)70, (char)29, (char)55, (char)41, (char)184, (char)57, (char)14, (char)79, (char)22, (char)205, (char)230, (char)237, (char)66, (char)96, (char)55, (char)140, (char)30}));
            assert(pack.flags_GET() == (char)90);
            assert(pack.len_GET() == (char)67);
        });
        DemoDevice.GPS_RTCM_DATA p233 = LoopBackDemoChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.len_SET((char)67) ;
        p233.flags_SET((char)90) ;
        p233.data__SET(new char[] {(char)9, (char)10, (char)99, (char)232, (char)155, (char)218, (char)71, (char)69, (char)40, (char)186, (char)3, (char)154, (char)113, (char)139, (char)243, (char)39, (char)255, (char)169, (char)206, (char)194, (char)205, (char)116, (char)190, (char)51, (char)88, (char)145, (char)42, (char)140, (char)19, (char)26, (char)46, (char)254, (char)9, (char)64, (char)148, (char)184, (char)221, (char)19, (char)173, (char)31, (char)237, (char)97, (char)137, (char)30, (char)92, (char)246, (char)72, (char)169, (char)51, (char)182, (char)125, (char)92, (char)255, (char)12, (char)6, (char)169, (char)155, (char)116, (char)236, (char)24, (char)80, (char)85, (char)79, (char)13, (char)111, (char)63, (char)215, (char)22, (char)177, (char)89, (char)248, (char)2, (char)32, (char)218, (char)253, (char)191, (char)51, (char)143, (char)59, (char)17, (char)77, (char)120, (char)236, (char)231, (char)19, (char)59, (char)130, (char)91, (char)150, (char)221, (char)194, (char)122, (char)248, (char)61, (char)129, (char)75, (char)91, (char)12, (char)77, (char)56, (char)36, (char)17, (char)182, (char)162, (char)17, (char)71, (char)250, (char)208, (char)215, (char)15, (char)45, (char)245, (char)26, (char)132, (char)189, (char)107, (char)125, (char)120, (char)25, (char)130, (char)194, (char)232, (char)100, (char)233, (char)226, (char)106, (char)223, (char)94, (char)17, (char)234, (char)228, (char)21, (char)67, (char)153, (char)145, (char)199, (char)121, (char)82, (char)144, (char)255, (char)151, (char)181, (char)159, (char)155, (char)181, (char)239, (char)66, (char)38, (char)61, (char)138, (char)203, (char)82, (char)63, (char)253, (char)153, (char)89, (char)244, (char)92, (char)173, (char)222, (char)76, (char)235, (char)224, (char)70, (char)29, (char)55, (char)41, (char)184, (char)57, (char)14, (char)79, (char)22, (char)205, (char)230, (char)237, (char)66, (char)96, (char)55, (char)140, (char)30}, 0) ;
        LoopBackDemoChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == (short) -2064);
            assert(pack.longitude_GET() == -1067743905);
            assert(pack.throttle_GET() == (byte)17);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
            assert(pack.latitude_GET() == 548332767);
            assert(pack.altitude_sp_GET() == (short)28507);
            assert(pack.airspeed_sp_GET() == (char)109);
            assert(pack.failsafe_GET() == (char)5);
            assert(pack.airspeed_GET() == (char)49);
            assert(pack.heading_GET() == (char)29215);
            assert(pack.temperature_GET() == (byte) - 96);
            assert(pack.groundspeed_GET() == (char)171);
            assert(pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);
            assert(pack.wp_num_GET() == (char)236);
            assert(pack.battery_remaining_GET() == (char)93);
            assert(pack.climb_rate_GET() == (byte) - 29);
            assert(pack.wp_distance_GET() == (char)23142);
            assert(pack.altitude_amsl_GET() == (short)374);
            assert(pack.temperature_air_GET() == (byte)34);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
            assert(pack.custom_mode_GET() == 2255651923L);
            assert(pack.pitch_GET() == (short)4716);
            assert(pack.heading_sp_GET() == (short) -16211);
            assert(pack.gps_nsat_GET() == (char)138);
        });
        DemoDevice.HIGH_LATENCY p234 = LoopBackDemoChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.climb_rate_SET((byte) - 29) ;
        p234.roll_SET((short) -2064) ;
        p234.groundspeed_SET((char)171) ;
        p234.failsafe_SET((char)5) ;
        p234.battery_remaining_SET((char)93) ;
        p234.longitude_SET(-1067743905) ;
        p234.wp_num_SET((char)236) ;
        p234.altitude_amsl_SET((short)374) ;
        p234.custom_mode_SET(2255651923L) ;
        p234.pitch_SET((short)4716) ;
        p234.airspeed_SET((char)49) ;
        p234.latitude_SET(548332767) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING) ;
        p234.heading_SET((char)29215) ;
        p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED) ;
        p234.temperature_SET((byte) - 96) ;
        p234.wp_distance_SET((char)23142) ;
        p234.airspeed_sp_SET((char)109) ;
        p234.throttle_SET((byte)17) ;
        p234.heading_sp_SET((short) -16211) ;
        p234.gps_nsat_SET((char)138) ;
        p234.altitude_sp_SET((short)28507) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_PPP) ;
        p234.temperature_air_SET((byte)34) ;
        LoopBackDemoChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.clipping_0_GET() == 3290517099L);
            assert(pack.clipping_2_GET() == 3884881209L);
            assert(pack.vibration_z_GET() == -2.781277E38F);
            assert(pack.time_usec_GET() == 7040132560738038824L);
            assert(pack.vibration_x_GET() == 2.069184E38F);
            assert(pack.clipping_1_GET() == 914341546L);
            assert(pack.vibration_y_GET() == 1.0822805E38F);
        });
        DemoDevice.VIBRATION p241 = LoopBackDemoChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.vibration_y_SET(1.0822805E38F) ;
        p241.vibration_x_SET(2.069184E38F) ;
        p241.vibration_z_SET(-2.781277E38F) ;
        p241.clipping_2_SET(3884881209L) ;
        p241.clipping_0_SET(3290517099L) ;
        p241.clipping_1_SET(914341546L) ;
        p241.time_usec_SET(7040132560738038824L) ;
        LoopBackDemoChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 3.3947544E37F);
            assert(pack.approach_y_GET() == -1.73073E38F);
            assert(pack.longitude_GET() == -1002070035);
            assert(pack.altitude_GET() == 886086160);
            assert(pack.approach_z_GET() == 1.6324652E38F);
            assert(pack.approach_x_GET() == 1.225049E38F);
            assert(pack.z_GET() == -3.2432252E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.9927576E38F, -1.1151991E38F, -2.2313563E38F, 1.6411611E38F}));
            assert(pack.latitude_GET() == 1109975695);
            assert(pack.x_GET() == 3.0337933E38F);
            assert(pack.time_usec_TRY(ph) == 989676478937671964L);
        });
        DemoDevice.HOME_POSITION p242 = LoopBackDemoChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.approach_z_SET(1.6324652E38F) ;
        p242.latitude_SET(1109975695) ;
        p242.q_SET(new float[] {2.9927576E38F, -1.1151991E38F, -2.2313563E38F, 1.6411611E38F}, 0) ;
        p242.altitude_SET(886086160) ;
        p242.approach_y_SET(-1.73073E38F) ;
        p242.z_SET(-3.2432252E38F) ;
        p242.x_SET(3.0337933E38F) ;
        p242.longitude_SET(-1002070035) ;
        p242.y_SET(3.3947544E37F) ;
        p242.approach_x_SET(1.225049E38F) ;
        p242.time_usec_SET(989676478937671964L, PH) ;
        LoopBackDemoChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -2.1473275E38F);
            assert(pack.altitude_GET() == -1345069512);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.8693256E38F, -4.8785457E36F, 1.8824098E38F, 1.660398E38F}));
            assert(pack.x_GET() == -2.2853755E38F);
            assert(pack.longitude_GET() == -1155280830);
            assert(pack.approach_x_GET() == 2.9173368E38F);
            assert(pack.target_system_GET() == (char)53);
            assert(pack.latitude_GET() == 239145640);
            assert(pack.approach_z_GET() == 1.0990293E38F);
            assert(pack.time_usec_TRY(ph) == 5965085475232769555L);
            assert(pack.approach_y_GET() == 8.3683184E37F);
            assert(pack.z_GET() == 2.6786206E38F);
        });
        DemoDevice.SET_HOME_POSITION p243 = LoopBackDemoChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.q_SET(new float[] {1.8693256E38F, -4.8785457E36F, 1.8824098E38F, 1.660398E38F}, 0) ;
        p243.latitude_SET(239145640) ;
        p243.approach_x_SET(2.9173368E38F) ;
        p243.target_system_SET((char)53) ;
        p243.approach_z_SET(1.0990293E38F) ;
        p243.longitude_SET(-1155280830) ;
        p243.approach_y_SET(8.3683184E37F) ;
        p243.time_usec_SET(5965085475232769555L, PH) ;
        p243.z_SET(2.6786206E38F) ;
        p243.altitude_SET(-1345069512) ;
        p243.x_SET(-2.2853755E38F) ;
        p243.y_SET(-2.1473275E38F) ;
        LoopBackDemoChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.message_id_GET() == (char)10390);
            assert(pack.interval_us_GET() == 1861622678);
        });
        DemoDevice.MESSAGE_INTERVAL p244 = LoopBackDemoChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.message_id_SET((char)10390) ;
        p244.interval_us_SET(1861622678) ;
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
            assert(pack.ver_velocity_GET() == (short) -12174);
            assert(pack.heading_GET() == (char)748);
            assert(pack.tslc_GET() == (char)114);
            assert(pack.hor_velocity_GET() == (char)11229);
            assert(pack.squawk_GET() == (char)13297);
            assert(pack.lon_GET() == -1079250303);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSIGNED2);
            assert(pack.altitude_GET() == -902237264);
            assert(pack.lat_GET() == -1958686437);
            assert(pack.callsign_LEN(ph) == 7);
            assert(pack.callsign_TRY(ph).equals("vqkeiuz"));
            assert(pack.ICAO_address_GET() == 3155550226L);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
            assert(pack.flags_GET() == ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE);
        });
        DemoDevice.ADSB_VEHICLE p246 = LoopBackDemoChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE) ;
        p246.heading_SET((char)748) ;
        p246.hor_velocity_SET((char)11229) ;
        p246.ver_velocity_SET((short) -12174) ;
        p246.lat_SET(-1958686437) ;
        p246.squawk_SET((char)13297) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH) ;
        p246.ICAO_address_SET(3155550226L) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSIGNED2) ;
        p246.callsign_SET("vqkeiuz", PH) ;
        p246.altitude_SET(-902237264) ;
        p246.lon_SET(-1079250303) ;
        p246.tslc_SET((char)114) ;
        LoopBackDemoChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
            assert(pack.id_GET() == 1342058284L);
            assert(pack.time_to_minimum_delta_GET() == 1.256472E38F);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_REPORT);
            assert(pack.horizontal_minimum_delta_GET() == 2.3152707E38F);
            assert(pack.altitude_minimum_delta_GET() == 1.2685877E38F);
        });
        DemoDevice.COLLISION p247 = LoopBackDemoChannel.new_COLLISION();
        PH.setPack(p247);
        p247.id_SET(1342058284L) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_REPORT) ;
        p247.altitude_minimum_delta_SET(1.2685877E38F) ;
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB) ;
        p247.time_to_minimum_delta_SET(1.256472E38F) ;
        p247.horizontal_minimum_delta_SET(2.3152707E38F) ;
        LoopBackDemoChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.target_network_GET() == (char)32);
            assert(pack.message_type_GET() == (char)29713);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)218, (char)31, (char)212, (char)91, (char)130, (char)197, (char)153, (char)122, (char)228, (char)214, (char)56, (char)209, (char)47, (char)113, (char)68, (char)224, (char)13, (char)68, (char)132, (char)77, (char)200, (char)225, (char)140, (char)115, (char)149, (char)201, (char)10, (char)1, (char)155, (char)177, (char)153, (char)39, (char)140, (char)98, (char)127, (char)15, (char)45, (char)57, (char)129, (char)183, (char)63, (char)175, (char)216, (char)11, (char)34, (char)40, (char)52, (char)212, (char)186, (char)37, (char)218, (char)227, (char)29, (char)10, (char)55, (char)236, (char)233, (char)110, (char)77, (char)98, (char)39, (char)114, (char)76, (char)40, (char)132, (char)45, (char)209, (char)18, (char)184, (char)22, (char)90, (char)113, (char)40, (char)79, (char)45, (char)172, (char)95, (char)140, (char)20, (char)14, (char)147, (char)119, (char)66, (char)7, (char)245, (char)33, (char)84, (char)93, (char)201, (char)1, (char)39, (char)197, (char)160, (char)45, (char)7, (char)18, (char)128, (char)210, (char)127, (char)190, (char)225, (char)215, (char)186, (char)193, (char)56, (char)137, (char)23, (char)17, (char)251, (char)25, (char)216, (char)243, (char)2, (char)203, (char)199, (char)111, (char)239, (char)101, (char)31, (char)220, (char)228, (char)218, (char)60, (char)232, (char)144, (char)95, (char)248, (char)228, (char)99, (char)239, (char)14, (char)2, (char)228, (char)226, (char)42, (char)153, (char)101, (char)52, (char)109, (char)93, (char)20, (char)227, (char)185, (char)51, (char)105, (char)197, (char)174, (char)146, (char)115, (char)194, (char)180, (char)201, (char)124, (char)232, (char)198, (char)193, (char)162, (char)131, (char)220, (char)209, (char)246, (char)219, (char)233, (char)109, (char)103, (char)108, (char)205, (char)145, (char)99, (char)193, (char)7, (char)131, (char)112, (char)171, (char)29, (char)20, (char)41, (char)254, (char)140, (char)184, (char)116, (char)247, (char)254, (char)30, (char)191, (char)71, (char)101, (char)138, (char)153, (char)120, (char)83, (char)232, (char)54, (char)12, (char)148, (char)66, (char)14, (char)105, (char)135, (char)155, (char)56, (char)103, (char)192, (char)33, (char)39, (char)50, (char)122, (char)150, (char)217, (char)49, (char)76, (char)183, (char)19, (char)163, (char)49, (char)55, (char)215, (char)66, (char)117, (char)139, (char)191, (char)112, (char)19, (char)118, (char)236, (char)185, (char)236, (char)209, (char)148, (char)185, (char)211, (char)137, (char)252, (char)111, (char)125, (char)158, (char)220, (char)30, (char)215, (char)209, (char)198, (char)175, (char)214, (char)2, (char)191, (char)102, (char)152, (char)19, (char)86}));
            assert(pack.target_system_GET() == (char)164);
            assert(pack.target_component_GET() == (char)156);
        });
        DemoDevice.V2_EXTENSION p248 = LoopBackDemoChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_component_SET((char)156) ;
        p248.target_network_SET((char)32) ;
        p248.message_type_SET((char)29713) ;
        p248.payload_SET(new char[] {(char)218, (char)31, (char)212, (char)91, (char)130, (char)197, (char)153, (char)122, (char)228, (char)214, (char)56, (char)209, (char)47, (char)113, (char)68, (char)224, (char)13, (char)68, (char)132, (char)77, (char)200, (char)225, (char)140, (char)115, (char)149, (char)201, (char)10, (char)1, (char)155, (char)177, (char)153, (char)39, (char)140, (char)98, (char)127, (char)15, (char)45, (char)57, (char)129, (char)183, (char)63, (char)175, (char)216, (char)11, (char)34, (char)40, (char)52, (char)212, (char)186, (char)37, (char)218, (char)227, (char)29, (char)10, (char)55, (char)236, (char)233, (char)110, (char)77, (char)98, (char)39, (char)114, (char)76, (char)40, (char)132, (char)45, (char)209, (char)18, (char)184, (char)22, (char)90, (char)113, (char)40, (char)79, (char)45, (char)172, (char)95, (char)140, (char)20, (char)14, (char)147, (char)119, (char)66, (char)7, (char)245, (char)33, (char)84, (char)93, (char)201, (char)1, (char)39, (char)197, (char)160, (char)45, (char)7, (char)18, (char)128, (char)210, (char)127, (char)190, (char)225, (char)215, (char)186, (char)193, (char)56, (char)137, (char)23, (char)17, (char)251, (char)25, (char)216, (char)243, (char)2, (char)203, (char)199, (char)111, (char)239, (char)101, (char)31, (char)220, (char)228, (char)218, (char)60, (char)232, (char)144, (char)95, (char)248, (char)228, (char)99, (char)239, (char)14, (char)2, (char)228, (char)226, (char)42, (char)153, (char)101, (char)52, (char)109, (char)93, (char)20, (char)227, (char)185, (char)51, (char)105, (char)197, (char)174, (char)146, (char)115, (char)194, (char)180, (char)201, (char)124, (char)232, (char)198, (char)193, (char)162, (char)131, (char)220, (char)209, (char)246, (char)219, (char)233, (char)109, (char)103, (char)108, (char)205, (char)145, (char)99, (char)193, (char)7, (char)131, (char)112, (char)171, (char)29, (char)20, (char)41, (char)254, (char)140, (char)184, (char)116, (char)247, (char)254, (char)30, (char)191, (char)71, (char)101, (char)138, (char)153, (char)120, (char)83, (char)232, (char)54, (char)12, (char)148, (char)66, (char)14, (char)105, (char)135, (char)155, (char)56, (char)103, (char)192, (char)33, (char)39, (char)50, (char)122, (char)150, (char)217, (char)49, (char)76, (char)183, (char)19, (char)163, (char)49, (char)55, (char)215, (char)66, (char)117, (char)139, (char)191, (char)112, (char)19, (char)118, (char)236, (char)185, (char)236, (char)209, (char)148, (char)185, (char)211, (char)137, (char)252, (char)111, (char)125, (char)158, (char)220, (char)30, (char)215, (char)209, (char)198, (char)175, (char)214, (char)2, (char)191, (char)102, (char)152, (char)19, (char)86}, 0) ;
        p248.target_system_SET((char)164) ;
        LoopBackDemoChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == (char)129);
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte) - 16, (byte) - 99, (byte) - 94, (byte)68, (byte)89, (byte)35, (byte) - 127, (byte)12, (byte)1, (byte) - 65, (byte)3, (byte) - 8, (byte) - 54, (byte) - 86, (byte) - 121, (byte) - 36, (byte) - 112, (byte)16, (byte) - 116, (byte)20, (byte) - 25, (byte)51, (byte) - 75, (byte) - 126, (byte)0, (byte)101, (byte)46, (byte) - 121, (byte)98, (byte)50, (byte) - 119, (byte) - 20}));
            assert(pack.ver_GET() == (char)18);
            assert(pack.address_GET() == (char)30802);
        });
        DemoDevice.MEMORY_VECT p249 = LoopBackDemoChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.value_SET(new byte[] {(byte) - 16, (byte) - 99, (byte) - 94, (byte)68, (byte)89, (byte)35, (byte) - 127, (byte)12, (byte)1, (byte) - 65, (byte)3, (byte) - 8, (byte) - 54, (byte) - 86, (byte) - 121, (byte) - 36, (byte) - 112, (byte)16, (byte) - 116, (byte)20, (byte) - 25, (byte)51, (byte) - 75, (byte) - 126, (byte)0, (byte)101, (byte)46, (byte) - 121, (byte)98, (byte)50, (byte) - 119, (byte) - 20}, 0) ;
        p249.type_SET((char)129) ;
        p249.address_SET((char)30802) ;
        p249.ver_SET((char)18) ;
        LoopBackDemoChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 2797596790715305826L);
            assert(pack.y_GET() == -8.4068007E37F);
            assert(pack.name_LEN(ph) == 2);
            assert(pack.name_TRY(ph).equals("kr"));
            assert(pack.x_GET() == -2.983013E38F);
            assert(pack.z_GET() == 1.4985806E38F);
        });
        DemoDevice.DEBUG_VECT p250 = LoopBackDemoChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.time_usec_SET(2797596790715305826L) ;
        p250.y_SET(-8.4068007E37F) ;
        p250.name_SET("kr", PH) ;
        p250.x_SET(-2.983013E38F) ;
        p250.z_SET(1.4985806E38F) ;
        LoopBackDemoChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3968726170L);
            assert(pack.value_GET() == -2.5631185E38F);
            assert(pack.name_LEN(ph) == 2);
            assert(pack.name_TRY(ph).equals("pp"));
        });
        DemoDevice.NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.value_SET(-2.5631185E38F) ;
        p251.time_boot_ms_SET(3968726170L) ;
        p251.name_SET("pp", PH) ;
        LoopBackDemoChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == 1134433474);
            assert(pack.name_LEN(ph) == 7);
            assert(pack.name_TRY(ph).equals("ntOnsqp"));
            assert(pack.time_boot_ms_GET() == 4058977740L);
        });
        DemoDevice.NAMED_VALUE_INT p252 = LoopBackDemoChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.time_boot_ms_SET(4058977740L) ;
        p252.value_SET(1134433474) ;
        p252.name_SET("ntOnsqp", PH) ;
        LoopBackDemoChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.text_LEN(ph) == 6);
            assert(pack.text_TRY(ph).equals("eMieIb"));
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_DEBUG);
        });
        DemoDevice.STATUSTEXT p253 = LoopBackDemoChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_DEBUG) ;
        p253.text_SET("eMieIb", PH) ;
        LoopBackDemoChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2905068479L);
            assert(pack.ind_GET() == (char)53);
            assert(pack.value_GET() == 5.2344777E37F);
        });
        DemoDevice.DEBUG p254 = LoopBackDemoChannel.new_DEBUG();
        PH.setPack(p254);
        p254.value_SET(5.2344777E37F) ;
        p254.time_boot_ms_SET(2905068479L) ;
        p254.ind_SET((char)53) ;
        LoopBackDemoChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)114);
            assert(pack.target_component_GET() == (char)211);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)104, (char)231, (char)196, (char)65, (char)46, (char)117, (char)109, (char)170, (char)39, (char)7, (char)134, (char)113, (char)44, (char)199, (char)195, (char)134, (char)34, (char)20, (char)18, (char)73, (char)15, (char)219, (char)219, (char)91, (char)24, (char)76, (char)93, (char)226, (char)239, (char)236, (char)243, (char)51}));
            assert(pack.initial_timestamp_GET() == 5601377794824270226L);
        });
        DemoDevice.SETUP_SIGNING p256 = LoopBackDemoChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.initial_timestamp_SET(5601377794824270226L) ;
        p256.target_component_SET((char)211) ;
        p256.target_system_SET((char)114) ;
        p256.secret_key_SET(new char[] {(char)104, (char)231, (char)196, (char)65, (char)46, (char)117, (char)109, (char)170, (char)39, (char)7, (char)134, (char)113, (char)44, (char)199, (char)195, (char)134, (char)34, (char)20, (char)18, (char)73, (char)15, (char)219, (char)219, (char)91, (char)24, (char)76, (char)93, (char)226, (char)239, (char)236, (char)243, (char)51}, 0) ;
        LoopBackDemoChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2101110786L);
            assert(pack.last_change_ms_GET() == 2876135997L);
            assert(pack.state_GET() == (char)87);
        });
        DemoDevice.BUTTON_CHANGE p257 = LoopBackDemoChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(2101110786L) ;
        p257.last_change_ms_SET(2876135997L) ;
        p257.state_SET((char)87) ;
        LoopBackDemoChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.tune_LEN(ph) == 21);
            assert(pack.tune_TRY(ph).equals("VjosckyhpfhasGghoGWlw"));
            assert(pack.target_system_GET() == (char)59);
            assert(pack.target_component_GET() == (char)155);
        });
        DemoDevice.PLAY_TUNE p258 = LoopBackDemoChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)59) ;
        p258.tune_SET("VjosckyhpfhasGghoGWlw", PH) ;
        p258.target_component_SET((char)155) ;
        LoopBackDemoChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.resolution_h_GET() == (char)11192);
            assert(pack.focal_length_GET() == 2.504678E38F);
            assert(pack.cam_definition_version_GET() == (char)15811);
            assert(pack.time_boot_ms_GET() == 3246354120L);
            assert(pack.lens_id_GET() == (char)139);
            assert(pack.sensor_size_h_GET() == -2.8019346E38F);
            assert(pack.flags_GET() == CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE);
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)71, (char)116, (char)1, (char)9, (char)128, (char)119, (char)224, (char)136, (char)48, (char)240, (char)2, (char)249, (char)79, (char)37, (char)255, (char)162, (char)17, (char)45, (char)5, (char)115, (char)181, (char)132, (char)173, (char)106, (char)110, (char)10, (char)17, (char)97, (char)226, (char)150, (char)212, (char)49}));
            assert(pack.sensor_size_v_GET() == 1.3262392E38F);
            assert(pack.resolution_v_GET() == (char)43753);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)223, (char)52, (char)159, (char)188, (char)127, (char)220, (char)11, (char)40, (char)108, (char)168, (char)119, (char)70, (char)203, (char)141, (char)193, (char)55, (char)61, (char)117, (char)191, (char)140, (char)163, (char)11, (char)243, (char)89, (char)140, (char)30, (char)43, (char)139, (char)211, (char)226, (char)171, (char)131}));
            assert(pack.firmware_version_GET() == 842639956L);
            assert(pack.cam_definition_uri_LEN(ph) == 47);
            assert(pack.cam_definition_uri_TRY(ph).equals("lPixfyshsgVhZtttTGXqmwkwgpQrbOlgqrwqtwuafcycynn"));
        });
        DemoDevice.CAMERA_INFORMATION p259 = LoopBackDemoChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.model_name_SET(new char[] {(char)71, (char)116, (char)1, (char)9, (char)128, (char)119, (char)224, (char)136, (char)48, (char)240, (char)2, (char)249, (char)79, (char)37, (char)255, (char)162, (char)17, (char)45, (char)5, (char)115, (char)181, (char)132, (char)173, (char)106, (char)110, (char)10, (char)17, (char)97, (char)226, (char)150, (char)212, (char)49}, 0) ;
        p259.sensor_size_v_SET(1.3262392E38F) ;
        p259.resolution_h_SET((char)11192) ;
        p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE) ;
        p259.time_boot_ms_SET(3246354120L) ;
        p259.cam_definition_uri_SET("lPixfyshsgVhZtttTGXqmwkwgpQrbOlgqrwqtwuafcycynn", PH) ;
        p259.lens_id_SET((char)139) ;
        p259.vendor_name_SET(new char[] {(char)223, (char)52, (char)159, (char)188, (char)127, (char)220, (char)11, (char)40, (char)108, (char)168, (char)119, (char)70, (char)203, (char)141, (char)193, (char)55, (char)61, (char)117, (char)191, (char)140, (char)163, (char)11, (char)243, (char)89, (char)140, (char)30, (char)43, (char)139, (char)211, (char)226, (char)171, (char)131}, 0) ;
        p259.firmware_version_SET(842639956L) ;
        p259.resolution_v_SET((char)43753) ;
        p259.focal_length_SET(2.504678E38F) ;
        p259.cam_definition_version_SET((char)15811) ;
        p259.sensor_size_h_SET(-2.8019346E38F) ;
        LoopBackDemoChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1510523106L);
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_IMAGE);
        });
        DemoDevice.CAMERA_SETTINGS p260 = LoopBackDemoChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(1510523106L) ;
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE) ;
        LoopBackDemoChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.write_speed_GET() == -2.1689644E38F);
            assert(pack.total_capacity_GET() == 3.561809E37F);
            assert(pack.storage_id_GET() == (char)170);
            assert(pack.storage_count_GET() == (char)247);
            assert(pack.available_capacity_GET() == 1.6780623E38F);
            assert(pack.time_boot_ms_GET() == 1464925867L);
            assert(pack.used_capacity_GET() == -1.1315447E37F);
            assert(pack.read_speed_GET() == 2.7847925E38F);
            assert(pack.status_GET() == (char)160);
        });
        DemoDevice.STORAGE_INFORMATION p261 = LoopBackDemoChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.read_speed_SET(2.7847925E38F) ;
        p261.storage_count_SET((char)247) ;
        p261.time_boot_ms_SET(1464925867L) ;
        p261.write_speed_SET(-2.1689644E38F) ;
        p261.status_SET((char)160) ;
        p261.storage_id_SET((char)170) ;
        p261.total_capacity_SET(3.561809E37F) ;
        p261.used_capacity_SET(-1.1315447E37F) ;
        p261.available_capacity_SET(1.6780623E38F) ;
        LoopBackDemoChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.image_interval_GET() == 1.2419311E38F);
            assert(pack.time_boot_ms_GET() == 2384943544L);
            assert(pack.recording_time_ms_GET() == 151592130L);
            assert(pack.image_status_GET() == (char)130);
            assert(pack.available_capacity_GET() == -2.46761E38F);
            assert(pack.video_status_GET() == (char)107);
        });
        DemoDevice.CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.recording_time_ms_SET(151592130L) ;
        p262.time_boot_ms_SET(2384943544L) ;
        p262.image_interval_SET(1.2419311E38F) ;
        p262.image_status_SET((char)130) ;
        p262.available_capacity_SET(-2.46761E38F) ;
        p262.video_status_SET((char)107) ;
        LoopBackDemoChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.relative_alt_GET() == 809020676);
            assert(pack.camera_id_GET() == (char)191);
            assert(pack.file_url_LEN(ph) == 111);
            assert(pack.file_url_TRY(ph).equals("waRJkedfkopgjArIueiaWplqogThqvijpfcdmrhomctgnlwapfDqzgzcsstCgowsqcjidlgwggarbFreWEbliwEajmssgdwxussvjdvdctufUfV"));
            assert(pack.time_boot_ms_GET() == 2086716734L);
            assert(pack.lon_GET() == 321302723);
            assert(pack.lat_GET() == 749289387);
            assert(pack.time_utc_GET() == 3946867244258004759L);
            assert(pack.capture_result_GET() == (byte) - 9);
            assert(pack.alt_GET() == -354141031);
            assert(pack.image_index_GET() == -878195327);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-8.593494E37F, -1.6644492E38F, -2.3945107E38F, -1.142982E38F}));
        });
        DemoDevice.CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.q_SET(new float[] {-8.593494E37F, -1.6644492E38F, -2.3945107E38F, -1.142982E38F}, 0) ;
        p263.capture_result_SET((byte) - 9) ;
        p263.alt_SET(-354141031) ;
        p263.camera_id_SET((char)191) ;
        p263.file_url_SET("waRJkedfkopgjArIueiaWplqogThqvijpfcdmrhomctgnlwapfDqzgzcsstCgowsqcjidlgwggarbFreWEbliwEajmssgdwxussvjdvdctufUfV", PH) ;
        p263.image_index_SET(-878195327) ;
        p263.relative_alt_SET(809020676) ;
        p263.time_boot_ms_SET(2086716734L) ;
        p263.lon_SET(321302723) ;
        p263.lat_SET(749289387) ;
        p263.time_utc_SET(3946867244258004759L) ;
        LoopBackDemoChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 694835269L);
            assert(pack.arming_time_utc_GET() == 3960533829930982114L);
            assert(pack.takeoff_time_utc_GET() == 7714542550628221348L);
            assert(pack.flight_uuid_GET() == 5107989249308377042L);
        });
        DemoDevice.FLIGHT_INFORMATION p264 = LoopBackDemoChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.flight_uuid_SET(5107989249308377042L) ;
        p264.takeoff_time_utc_SET(7714542550628221348L) ;
        p264.arming_time_utc_SET(3960533829930982114L) ;
        p264.time_boot_ms_SET(694835269L) ;
        LoopBackDemoChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -2.6355902E38F);
            assert(pack.roll_GET() == -3.3428315E38F);
            assert(pack.time_boot_ms_GET() == 969550634L);
            assert(pack.pitch_GET() == 5.580966E37F);
        });
        DemoDevice.MOUNT_ORIENTATION p265 = LoopBackDemoChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.roll_SET(-3.3428315E38F) ;
        p265.yaw_SET(-2.6355902E38F) ;
        p265.time_boot_ms_SET(969550634L) ;
        p265.pitch_SET(5.580966E37F) ;
        LoopBackDemoChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.sequence_GET() == (char)1176);
            assert(pack.first_message_offset_GET() == (char)43);
            assert(pack.target_system_GET() == (char)144);
            assert(pack.length_GET() == (char)223);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)245, (char)109, (char)58, (char)122, (char)179, (char)196, (char)116, (char)174, (char)248, (char)81, (char)187, (char)154, (char)190, (char)220, (char)56, (char)45, (char)136, (char)122, (char)94, (char)228, (char)245, (char)23, (char)6, (char)193, (char)4, (char)201, (char)22, (char)139, (char)241, (char)250, (char)111, (char)224, (char)90, (char)242, (char)103, (char)233, (char)33, (char)136, (char)0, (char)255, (char)109, (char)98, (char)25, (char)204, (char)75, (char)158, (char)134, (char)123, (char)76, (char)191, (char)187, (char)34, (char)151, (char)194, (char)152, (char)191, (char)225, (char)76, (char)235, (char)9, (char)172, (char)94, (char)199, (char)96, (char)43, (char)56, (char)173, (char)94, (char)240, (char)129, (char)85, (char)201, (char)117, (char)30, (char)89, (char)216, (char)177, (char)114, (char)0, (char)87, (char)253, (char)137, (char)196, (char)69, (char)75, (char)66, (char)7, (char)252, (char)9, (char)13, (char)12, (char)56, (char)219, (char)40, (char)24, (char)239, (char)161, (char)12, (char)107, (char)191, (char)155, (char)33, (char)15, (char)175, (char)238, (char)230, (char)156, (char)132, (char)85, (char)254, (char)191, (char)53, (char)52, (char)106, (char)171, (char)65, (char)43, (char)103, (char)29, (char)182, (char)247, (char)53, (char)56, (char)87, (char)243, (char)184, (char)232, (char)143, (char)37, (char)69, (char)29, (char)223, (char)150, (char)184, (char)79, (char)160, (char)152, (char)252, (char)109, (char)87, (char)183, (char)98, (char)157, (char)9, (char)125, (char)122, (char)165, (char)5, (char)2, (char)189, (char)27, (char)40, (char)211, (char)191, (char)115, (char)11, (char)18, (char)126, (char)15, (char)74, (char)11, (char)125, (char)222, (char)234, (char)91, (char)225, (char)21, (char)126, (char)3, (char)122, (char)12, (char)58, (char)190, (char)11, (char)99, (char)69, (char)210, (char)246, (char)130, (char)233, (char)104, (char)61, (char)17, (char)226, (char)71, (char)73, (char)78, (char)166, (char)100, (char)19, (char)135, (char)52, (char)204, (char)181, (char)90, (char)199, (char)137, (char)243, (char)18, (char)13, (char)204, (char)174, (char)50, (char)202, (char)201, (char)105, (char)132, (char)12, (char)70, (char)103, (char)79, (char)1, (char)30, (char)203, (char)134, (char)34, (char)16, (char)242, (char)246, (char)195, (char)122, (char)94, (char)233, (char)139, (char)127, (char)15, (char)94, (char)162, (char)172, (char)24, (char)145, (char)179, (char)15, (char)19, (char)19, (char)99, (char)182, (char)156, (char)24, (char)169, (char)227, (char)69, (char)91, (char)51, (char)248, (char)249, (char)85, (char)39, (char)107}));
            assert(pack.target_component_GET() == (char)147);
        });
        DemoDevice.LOGGING_DATA p266 = LoopBackDemoChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.data__SET(new char[] {(char)245, (char)109, (char)58, (char)122, (char)179, (char)196, (char)116, (char)174, (char)248, (char)81, (char)187, (char)154, (char)190, (char)220, (char)56, (char)45, (char)136, (char)122, (char)94, (char)228, (char)245, (char)23, (char)6, (char)193, (char)4, (char)201, (char)22, (char)139, (char)241, (char)250, (char)111, (char)224, (char)90, (char)242, (char)103, (char)233, (char)33, (char)136, (char)0, (char)255, (char)109, (char)98, (char)25, (char)204, (char)75, (char)158, (char)134, (char)123, (char)76, (char)191, (char)187, (char)34, (char)151, (char)194, (char)152, (char)191, (char)225, (char)76, (char)235, (char)9, (char)172, (char)94, (char)199, (char)96, (char)43, (char)56, (char)173, (char)94, (char)240, (char)129, (char)85, (char)201, (char)117, (char)30, (char)89, (char)216, (char)177, (char)114, (char)0, (char)87, (char)253, (char)137, (char)196, (char)69, (char)75, (char)66, (char)7, (char)252, (char)9, (char)13, (char)12, (char)56, (char)219, (char)40, (char)24, (char)239, (char)161, (char)12, (char)107, (char)191, (char)155, (char)33, (char)15, (char)175, (char)238, (char)230, (char)156, (char)132, (char)85, (char)254, (char)191, (char)53, (char)52, (char)106, (char)171, (char)65, (char)43, (char)103, (char)29, (char)182, (char)247, (char)53, (char)56, (char)87, (char)243, (char)184, (char)232, (char)143, (char)37, (char)69, (char)29, (char)223, (char)150, (char)184, (char)79, (char)160, (char)152, (char)252, (char)109, (char)87, (char)183, (char)98, (char)157, (char)9, (char)125, (char)122, (char)165, (char)5, (char)2, (char)189, (char)27, (char)40, (char)211, (char)191, (char)115, (char)11, (char)18, (char)126, (char)15, (char)74, (char)11, (char)125, (char)222, (char)234, (char)91, (char)225, (char)21, (char)126, (char)3, (char)122, (char)12, (char)58, (char)190, (char)11, (char)99, (char)69, (char)210, (char)246, (char)130, (char)233, (char)104, (char)61, (char)17, (char)226, (char)71, (char)73, (char)78, (char)166, (char)100, (char)19, (char)135, (char)52, (char)204, (char)181, (char)90, (char)199, (char)137, (char)243, (char)18, (char)13, (char)204, (char)174, (char)50, (char)202, (char)201, (char)105, (char)132, (char)12, (char)70, (char)103, (char)79, (char)1, (char)30, (char)203, (char)134, (char)34, (char)16, (char)242, (char)246, (char)195, (char)122, (char)94, (char)233, (char)139, (char)127, (char)15, (char)94, (char)162, (char)172, (char)24, (char)145, (char)179, (char)15, (char)19, (char)19, (char)99, (char)182, (char)156, (char)24, (char)169, (char)227, (char)69, (char)91, (char)51, (char)248, (char)249, (char)85, (char)39, (char)107}, 0) ;
        p266.length_SET((char)223) ;
        p266.sequence_SET((char)1176) ;
        p266.target_system_SET((char)144) ;
        p266.first_message_offset_SET((char)43) ;
        p266.target_component_SET((char)147) ;
        LoopBackDemoChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)192, (char)116, (char)215, (char)146, (char)30, (char)211, (char)6, (char)128, (char)147, (char)14, (char)154, (char)118, (char)13, (char)210, (char)76, (char)33, (char)192, (char)194, (char)222, (char)201, (char)234, (char)87, (char)134, (char)64, (char)170, (char)16, (char)92, (char)126, (char)105, (char)232, (char)132, (char)75, (char)69, (char)83, (char)37, (char)190, (char)64, (char)99, (char)100, (char)5, (char)99, (char)14, (char)68, (char)184, (char)221, (char)23, (char)36, (char)11, (char)31, (char)249, (char)173, (char)78, (char)44, (char)3, (char)32, (char)93, (char)49, (char)142, (char)71, (char)149, (char)12, (char)87, (char)162, (char)100, (char)148, (char)140, (char)119, (char)12, (char)133, (char)8, (char)214, (char)5, (char)230, (char)197, (char)130, (char)110, (char)42, (char)137, (char)134, (char)151, (char)244, (char)18, (char)62, (char)53, (char)218, (char)191, (char)149, (char)34, (char)56, (char)228, (char)208, (char)131, (char)137, (char)156, (char)189, (char)239, (char)238, (char)183, (char)143, (char)150, (char)121, (char)142, (char)41, (char)32, (char)54, (char)251, (char)94, (char)246, (char)5, (char)98, (char)51, (char)111, (char)41, (char)202, (char)78, (char)74, (char)36, (char)142, (char)212, (char)186, (char)241, (char)239, (char)82, (char)58, (char)129, (char)89, (char)132, (char)254, (char)65, (char)54, (char)17, (char)119, (char)163, (char)69, (char)199, (char)109, (char)93, (char)106, (char)217, (char)169, (char)229, (char)228, (char)253, (char)77, (char)97, (char)175, (char)193, (char)236, (char)192, (char)43, (char)197, (char)113, (char)123, (char)206, (char)69, (char)7, (char)14, (char)7, (char)195, (char)197, (char)42, (char)98, (char)48, (char)110, (char)76, (char)255, (char)84, (char)244, (char)38, (char)174, (char)237, (char)230, (char)9, (char)218, (char)36, (char)110, (char)52, (char)16, (char)19, (char)5, (char)0, (char)231, (char)222, (char)149, (char)208, (char)181, (char)136, (char)49, (char)72, (char)227, (char)213, (char)224, (char)244, (char)53, (char)15, (char)0, (char)59, (char)143, (char)15, (char)161, (char)15, (char)67, (char)190, (char)140, (char)3, (char)90, (char)210, (char)75, (char)83, (char)10, (char)168, (char)136, (char)245, (char)125, (char)191, (char)79, (char)44, (char)215, (char)50, (char)116, (char)23, (char)225, (char)180, (char)254, (char)127, (char)155, (char)83, (char)28, (char)135, (char)67, (char)0, (char)155, (char)235, (char)131, (char)115, (char)40, (char)232, (char)210, (char)253, (char)231, (char)111, (char)215, (char)157, (char)209, (char)3, (char)107, (char)206, (char)2, (char)95}));
            assert(pack.target_component_GET() == (char)126);
            assert(pack.target_system_GET() == (char)164);
            assert(pack.sequence_GET() == (char)28529);
            assert(pack.length_GET() == (char)30);
            assert(pack.first_message_offset_GET() == (char)145);
        });
        DemoDevice.LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.length_SET((char)30) ;
        p267.target_system_SET((char)164) ;
        p267.target_component_SET((char)126) ;
        p267.data__SET(new char[] {(char)192, (char)116, (char)215, (char)146, (char)30, (char)211, (char)6, (char)128, (char)147, (char)14, (char)154, (char)118, (char)13, (char)210, (char)76, (char)33, (char)192, (char)194, (char)222, (char)201, (char)234, (char)87, (char)134, (char)64, (char)170, (char)16, (char)92, (char)126, (char)105, (char)232, (char)132, (char)75, (char)69, (char)83, (char)37, (char)190, (char)64, (char)99, (char)100, (char)5, (char)99, (char)14, (char)68, (char)184, (char)221, (char)23, (char)36, (char)11, (char)31, (char)249, (char)173, (char)78, (char)44, (char)3, (char)32, (char)93, (char)49, (char)142, (char)71, (char)149, (char)12, (char)87, (char)162, (char)100, (char)148, (char)140, (char)119, (char)12, (char)133, (char)8, (char)214, (char)5, (char)230, (char)197, (char)130, (char)110, (char)42, (char)137, (char)134, (char)151, (char)244, (char)18, (char)62, (char)53, (char)218, (char)191, (char)149, (char)34, (char)56, (char)228, (char)208, (char)131, (char)137, (char)156, (char)189, (char)239, (char)238, (char)183, (char)143, (char)150, (char)121, (char)142, (char)41, (char)32, (char)54, (char)251, (char)94, (char)246, (char)5, (char)98, (char)51, (char)111, (char)41, (char)202, (char)78, (char)74, (char)36, (char)142, (char)212, (char)186, (char)241, (char)239, (char)82, (char)58, (char)129, (char)89, (char)132, (char)254, (char)65, (char)54, (char)17, (char)119, (char)163, (char)69, (char)199, (char)109, (char)93, (char)106, (char)217, (char)169, (char)229, (char)228, (char)253, (char)77, (char)97, (char)175, (char)193, (char)236, (char)192, (char)43, (char)197, (char)113, (char)123, (char)206, (char)69, (char)7, (char)14, (char)7, (char)195, (char)197, (char)42, (char)98, (char)48, (char)110, (char)76, (char)255, (char)84, (char)244, (char)38, (char)174, (char)237, (char)230, (char)9, (char)218, (char)36, (char)110, (char)52, (char)16, (char)19, (char)5, (char)0, (char)231, (char)222, (char)149, (char)208, (char)181, (char)136, (char)49, (char)72, (char)227, (char)213, (char)224, (char)244, (char)53, (char)15, (char)0, (char)59, (char)143, (char)15, (char)161, (char)15, (char)67, (char)190, (char)140, (char)3, (char)90, (char)210, (char)75, (char)83, (char)10, (char)168, (char)136, (char)245, (char)125, (char)191, (char)79, (char)44, (char)215, (char)50, (char)116, (char)23, (char)225, (char)180, (char)254, (char)127, (char)155, (char)83, (char)28, (char)135, (char)67, (char)0, (char)155, (char)235, (char)131, (char)115, (char)40, (char)232, (char)210, (char)253, (char)231, (char)111, (char)215, (char)157, (char)209, (char)3, (char)107, (char)206, (char)2, (char)95}, 0) ;
        p267.first_message_offset_SET((char)145) ;
        p267.sequence_SET((char)28529) ;
        LoopBackDemoChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)59);
            assert(pack.sequence_GET() == (char)3984);
            assert(pack.target_system_GET() == (char)174);
        });
        DemoDevice.LOGGING_ACK p268 = LoopBackDemoChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)174) ;
        p268.target_component_SET((char)59) ;
        p268.sequence_SET((char)3984) ;
        LoopBackDemoChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.camera_id_GET() == (char)108);
            assert(pack.rotation_GET() == (char)5808);
            assert(pack.status_GET() == (char)132);
            assert(pack.uri_LEN(ph) == 42);
            assert(pack.uri_TRY(ph).equals("iwamnemspkxnkqkexratjLKdnbvflkbhehHMxbtjku"));
            assert(pack.framerate_GET() == 2.9296697E38F);
            assert(pack.resolution_h_GET() == (char)52435);
            assert(pack.resolution_v_GET() == (char)26503);
            assert(pack.bitrate_GET() == 1494206123L);
        });
        DemoDevice.VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.resolution_h_SET((char)52435) ;
        p269.resolution_v_SET((char)26503) ;
        p269.camera_id_SET((char)108) ;
        p269.uri_SET("iwamnemspkxnkqkexratjLKdnbvflkbhehHMxbtjku", PH) ;
        p269.status_SET((char)132) ;
        p269.bitrate_SET(1494206123L) ;
        p269.rotation_SET((char)5808) ;
        p269.framerate_SET(2.9296697E38F) ;
        LoopBackDemoChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.resolution_v_GET() == (char)46966);
            assert(pack.framerate_GET() == -1.1646132E38F);
            assert(pack.uri_LEN(ph) == 64);
            assert(pack.uri_TRY(ph).equals("utjZxtztkzlhdpiccinvxrsgxlvpfcsptdqfdrvgflnrfDzsXmoppyequxbbsrbf"));
            assert(pack.camera_id_GET() == (char)146);
            assert(pack.resolution_h_GET() == (char)57285);
            assert(pack.rotation_GET() == (char)15766);
            assert(pack.target_component_GET() == (char)222);
            assert(pack.bitrate_GET() == 115215890L);
            assert(pack.target_system_GET() == (char)221);
        });
        DemoDevice.SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.target_system_SET((char)221) ;
        p270.uri_SET("utjZxtztkzlhdpiccinvxrsgxlvpfcsptdqfdrvgflnrfDzsXmoppyequxbbsrbf", PH) ;
        p270.framerate_SET(-1.1646132E38F) ;
        p270.resolution_v_SET((char)46966) ;
        p270.camera_id_SET((char)146) ;
        p270.rotation_SET((char)15766) ;
        p270.bitrate_SET(115215890L) ;
        p270.resolution_h_SET((char)57285) ;
        p270.target_component_SET((char)222) ;
        LoopBackDemoChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.ssid_LEN(ph) == 23);
            assert(pack.ssid_TRY(ph).equals("wJcnxWmqclsyebxyrtbbDyh"));
            assert(pack.password_LEN(ph) == 59);
            assert(pack.password_TRY(ph).equals("XzqsycmgtkVzbbmcuaresytjywweqhhjyYrxerxozqsnqvarxzezfscwvon"));
        });
        DemoDevice.WIFI_CONFIG_AP p299 = LoopBackDemoChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.password_SET("XzqsycmgtkVzbbmcuaresytjywweqhhjyYrxerxozqsnqvarxzezfscwvon", PH) ;
        p299.ssid_SET("wJcnxWmqclsyebxyrtbbDyh", PH) ;
        LoopBackDemoChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(pack.min_version_GET() == (char)51917);
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)61, (char)9, (char)99, (char)97, (char)233, (char)240, (char)174, (char)91}));
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)65, (char)247, (char)135, (char)240, (char)108, (char)168, (char)240, (char)41}));
            assert(pack.max_version_GET() == (char)22283);
            assert(pack.version_GET() == (char)9135);
        });
        DemoDevice.PROTOCOL_VERSION p300 = LoopBackDemoChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.min_version_SET((char)51917) ;
        p300.library_version_hash_SET(new char[] {(char)61, (char)9, (char)99, (char)97, (char)233, (char)240, (char)174, (char)91}, 0) ;
        p300.version_SET((char)9135) ;
        p300.spec_version_hash_SET(new char[] {(char)65, (char)247, (char)135, (char)240, (char)108, (char)168, (char)240, (char)41}, 0) ;
        p300.max_version_SET((char)22283) ;
        LoopBackDemoChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL);
            assert(pack.vendor_specific_status_code_GET() == (char)45791);
            assert(pack.uptime_sec_GET() == 3848190948L);
            assert(pack.sub_mode_GET() == (char)228);
            assert(pack.time_usec_GET() == 7505127976232320834L);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE);
        });
        DemoDevice.UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.sub_mode_SET((char)228) ;
        p310.uptime_sec_SET(3848190948L) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE) ;
        p310.time_usec_SET(7505127976232320834L) ;
        p310.vendor_specific_status_code_SET((char)45791) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL) ;
        LoopBackDemoChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.uptime_sec_GET() == 8494295L);
            assert(pack.sw_version_minor_GET() == (char)80);
            assert(pack.name_LEN(ph) == 33);
            assert(pack.name_TRY(ph).equals("zxPjyJpgyavqQasLyCqksZpflqmkehkYt"));
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)17, (char)121, (char)237, (char)149, (char)170, (char)4, (char)215, (char)239, (char)56, (char)250, (char)118, (char)172, (char)61, (char)31, (char)226, (char)5}));
            assert(pack.hw_version_major_GET() == (char)245);
            assert(pack.sw_vcs_commit_GET() == 3886698162L);
            assert(pack.hw_version_minor_GET() == (char)54);
            assert(pack.sw_version_major_GET() == (char)111);
            assert(pack.time_usec_GET() == 2619204594502235089L);
        });
        DemoDevice.UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.sw_vcs_commit_SET(3886698162L) ;
        p311.hw_version_minor_SET((char)54) ;
        p311.sw_version_minor_SET((char)80) ;
        p311.hw_unique_id_SET(new char[] {(char)17, (char)121, (char)237, (char)149, (char)170, (char)4, (char)215, (char)239, (char)56, (char)250, (char)118, (char)172, (char)61, (char)31, (char)226, (char)5}, 0) ;
        p311.hw_version_major_SET((char)245) ;
        p311.time_usec_SET(2619204594502235089L) ;
        p311.uptime_sec_SET(8494295L) ;
        p311.sw_version_major_SET((char)111) ;
        p311.name_SET("zxPjyJpgyavqQasLyCqksZpflqmkehkYt", PH) ;
        LoopBackDemoChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)94);
            assert(pack.target_component_GET() == (char)72);
            assert(pack.param_id_LEN(ph) == 2);
            assert(pack.param_id_TRY(ph).equals("ng"));
            assert(pack.param_index_GET() == (short)19360);
        });
        DemoDevice.PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_component_SET((char)72) ;
        p320.target_system_SET((char)94) ;
        p320.param_id_SET("ng", PH) ;
        p320.param_index_SET((short)19360) ;
        LoopBackDemoChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)150);
            assert(pack.target_system_GET() == (char)173);
        });
        DemoDevice.PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_component_SET((char)150) ;
        p321.target_system_SET((char)173) ;
        LoopBackDemoChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 1);
            assert(pack.param_id_TRY(ph).equals("d"));
            assert(pack.param_count_GET() == (char)36529);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8);
            assert(pack.param_index_GET() == (char)31981);
            assert(pack.param_value_LEN(ph) == 114);
            assert(pack.param_value_TRY(ph).equals("uwftbwmsbywyfkuzcytursqjftwthzdtxwowXuWltokghtdbbmbvopnkFmsbcdlijqguizwSmgOmzHEcvKlTlktybKaionxrrrpdohlamhzrbujduc"));
        });
        DemoDevice.PARAM_EXT_VALUE p322 = LoopBackDemoChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8) ;
        p322.param_id_SET("d", PH) ;
        p322.param_index_SET((char)31981) ;
        p322.param_count_SET((char)36529) ;
        p322.param_value_SET("uwftbwmsbywyfkuzcytursqjftwthzdtxwowXuWltokghtdbbmbvopnkFmsbcdlijqguizwSmgOmzHEcvKlTlktybKaionxrrrpdohlamhzrbujduc", PH) ;
        LoopBackDemoChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 11);
            assert(pack.param_value_TRY(ph).equals("ymgiehzspgq"));
            assert(pack.target_system_GET() == (char)244);
            assert(pack.target_component_GET() == (char)110);
            assert(pack.param_id_LEN(ph) == 2);
            assert(pack.param_id_TRY(ph).equals("tq"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32);
        });
        DemoDevice.PARAM_EXT_SET p323 = LoopBackDemoChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.param_id_SET("tq", PH) ;
        p323.target_component_SET((char)110) ;
        p323.param_value_SET("ymgiehzspgq", PH) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32) ;
        p323.target_system_SET((char)244) ;
        LoopBackDemoChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_ACCEPTED);
            assert(pack.param_id_LEN(ph) == 2);
            assert(pack.param_id_TRY(ph).equals("dy"));
            assert(pack.param_value_LEN(ph) == 19);
            assert(pack.param_value_TRY(ph).equals("emnbufqpuIMwibiryas"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
        });
        DemoDevice.PARAM_EXT_ACK p324 = LoopBackDemoChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_ACCEPTED) ;
        p324.param_value_SET("emnbufqpuIMwibiryas", PH) ;
        p324.param_id_SET("dy", PH) ;
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16) ;
        LoopBackDemoChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)43702, (char)32136, (char)24121, (char)29261, (char)14947, (char)49348, (char)27508, (char)29353, (char)17108, (char)31706, (char)7330, (char)32984, (char)3691, (char)1150, (char)54729, (char)548, (char)11371, (char)27144, (char)43205, (char)24966, (char)41280, (char)41980, (char)26648, (char)38677, (char)51569, (char)18935, (char)55876, (char)33185, (char)51321, (char)41995, (char)40529, (char)34917, (char)10742, (char)60801, (char)52180, (char)22167, (char)22576, (char)12654, (char)42611, (char)9496, (char)3227, (char)50516, (char)4441, (char)9470, (char)5310, (char)9609, (char)3083, (char)7709, (char)21203, (char)23565, (char)47624, (char)30014, (char)64092, (char)59920, (char)45709, (char)24634, (char)37472, (char)3921, (char)27595, (char)43702, (char)42040, (char)48352, (char)14731, (char)49588, (char)29372, (char)51742, (char)3663, (char)26023, (char)61420, (char)33681, (char)7818, (char)47945}));
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
            assert(pack.max_distance_GET() == (char)37920);
            assert(pack.increment_GET() == (char)156);
            assert(pack.min_distance_GET() == (char)65201);
            assert(pack.time_usec_GET() == 250841783241857287L);
        });
        DemoDevice.OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND) ;
        p330.time_usec_SET(250841783241857287L) ;
        p330.increment_SET((char)156) ;
        p330.max_distance_SET((char)37920) ;
        p330.distances_SET(new char[] {(char)43702, (char)32136, (char)24121, (char)29261, (char)14947, (char)49348, (char)27508, (char)29353, (char)17108, (char)31706, (char)7330, (char)32984, (char)3691, (char)1150, (char)54729, (char)548, (char)11371, (char)27144, (char)43205, (char)24966, (char)41280, (char)41980, (char)26648, (char)38677, (char)51569, (char)18935, (char)55876, (char)33185, (char)51321, (char)41995, (char)40529, (char)34917, (char)10742, (char)60801, (char)52180, (char)22167, (char)22576, (char)12654, (char)42611, (char)9496, (char)3227, (char)50516, (char)4441, (char)9470, (char)5310, (char)9609, (char)3083, (char)7709, (char)21203, (char)23565, (char)47624, (char)30014, (char)64092, (char)59920, (char)45709, (char)24634, (char)37472, (char)3921, (char)27595, (char)43702, (char)42040, (char)48352, (char)14731, (char)49588, (char)29372, (char)51742, (char)3663, (char)26023, (char)61420, (char)33681, (char)7818, (char)47945}, 0) ;
        p330.min_distance_SET((char)65201) ;
        LoopBackDemoChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
    }

}