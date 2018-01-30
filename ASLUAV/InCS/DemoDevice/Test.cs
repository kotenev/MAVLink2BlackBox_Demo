
using System;
using System.Diagnostics;
using System.Linq;
using org.unirail.BlackBox;
using Inside = org.unirail.BlackBox.Host.Pack.Meta.Field.Bounds.Inside;
namespace org.noname
{
    public class Test : DemoDevice
    {





        class TestChannelAdvanced : Channel.Advanced
        {
            public override bool CanRead { get { return true; } }
            public override bool CanWrite { get { return true; } }

            public override void failure(string reason)
            {
                base.failure(reason);
                Debug.Assert(false);
            }


            static Pack testing_pack;
            readonly  byte[]                        buf = new byte[4024];

            internal void send(Pack pack)
            {
                testing_pack = pack;
            }

            readonly Inside ph = new Inside();

            protected internal override Pack process(Pack pack, int id)
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
                        break;
                }
                switch(id)
                {
                }
                return null;
            }
            static readonly byte[] buff = new byte[1024];
            internal static void transmission(Channel src, Channel dst)
            {
                if(src is Channel.Advanced && !(dst is Channel.Advanced))
                {
                    for(int bytes; 0 < (bytes = src.Read(buff, 0, buff.Length));) ADV_TEST_CH.Write(buff, 0, bytes);
                    for(int bytes; 0 < (bytes = SMP_TEST_CH.Read(buff, 0, buff.Length));) dst.Write(buff, 0, bytes);
                }
                else if(!(src is Channel.Advanced) && dst is Channel.Advanced)
                {
                    for(int bytes; 0 < (bytes = src.Read(buff, 0, buff.Length));) SMP_TEST_CH.Write(buff, 0, bytes);
                    for(int bytes; 0 < (bytes = ADV_TEST_CH.Read(buff, 0, buff.Length));) dst.Write(buff, 0, bytes);
                }
                else
                    for(int bytes; 0 < (bytes = src.Read(buff, 0, buff.Length));) dst.Write(buff, 0, bytes);
                dst.process(null, Channel.PROCESS_RECEIVED);
            }
        }
        static readonly TestChannelAdvanced ADV_TEST_CH = new TestChannelAdvanced();

        public class TestChannelSimple : Channel
        {
            public override bool CanRead { get { return true; } }
            public override bool CanWrite { get { return true; } }

            public override void failure(String reason) { ADV_TEST_CH.failure(reason);}
            public void send(Pack pack) {ADV_TEST_CH.send(pack); }
            protected internal override Pack process(Pack pack, int id) { return ADV_TEST_CH.process(pack, id); }
        }
        static readonly TestChannelSimple SMP_TEST_CH = new TestChannelSimple();  //test channel with SimpleProtocol



        public static void Main(string[] args)
        {
            Inside PH = new Inside();
            LoopBackDemoChannel.instance.OnHEARTBEATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (MAV_TYPE)MAV_TYPE.MAV_TYPE_TRICOPTER);
                Debug.Assert(pack.custom_mode == (uint)3603909075U);
                Debug.Assert(pack.system_status == (MAV_STATE)MAV_STATE.MAV_STATE_STANDBY);
                Debug.Assert(pack.autopilot == (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_SMACCMPILOT);
                Debug.Assert(pack.mavlink_version == (byte)(byte)62);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);
            };
            DemoDevice.HEARTBEAT p0 = LoopBackDemoChannel.new_HEARTBEAT();
            PH.setPack(p0);
            p0.autopilot = (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_SMACCMPILOT;
            p0.type = (MAV_TYPE)MAV_TYPE.MAV_TYPE_TRICOPTER;
            p0.custom_mode = (uint)3603909075U;
            p0.mavlink_version = (byte)(byte)62;
            p0.system_status = (MAV_STATE)MAV_STATE.MAV_STATE_STANDBY;
            p0.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
            LoopBackDemoChannel.instance.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)60734);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)55911);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)58);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)54987);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)48707);
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)45244);
                Debug.Assert(pack.current_battery == (short)(short) -19651);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)26012);
                Debug.Assert(pack.load == (ushort)(ushort)17838);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)25327);
            };
            DemoDevice.SYS_STATUS p1 = LoopBackDemoChannel.new_SYS_STATUS();
            PH.setPack(p1);
            p1.battery_remaining = (sbyte)(sbyte)58;
            p1.current_battery = (short)(short) -19651;
            p1.voltage_battery = (ushort)(ushort)25327;
            p1.errors_count2 = (ushort)(ushort)60734;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2;
            p1.load = (ushort)(ushort)17838;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2;
            p1.errors_count1 = (ushort)(ushort)54987;
            p1.drop_rate_comm = (ushort)(ushort)45244;
            p1.errors_count4 = (ushort)(ushort)55911;
            p1.errors_count3 = (ushort)(ushort)26012;
            p1.errors_comm = (ushort)(ushort)48707;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2;
            LoopBackDemoChannel.instance.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_unix_usec == (ulong)3537070855264767788L);
                Debug.Assert(pack.time_boot_ms == (uint)3650199312U);
            };
            DemoDevice.SYSTEM_TIME p2 = LoopBackDemoChannel.new_SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)3537070855264767788L;
            p2.time_boot_ms = (uint)3650199312U;
            LoopBackDemoChannel.instance.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)1.3614257E38F);
                Debug.Assert(pack.z == (float)1.795267E38F);
                Debug.Assert(pack.y == (float)3.4138394E37F);
                Debug.Assert(pack.afy == (float) -4.673325E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)17121);
                Debug.Assert(pack.vx == (float) -3.3080578E38F);
                Debug.Assert(pack.afz == (float) -2.3755813E38F);
                Debug.Assert(pack.x == (float)2.0814962E37F);
                Debug.Assert(pack.vz == (float)7.2236284E37F);
                Debug.Assert(pack.vy == (float)2.2810863E38F);
                Debug.Assert(pack.yaw_rate == (float)1.6146614E38F);
                Debug.Assert(pack.afx == (float) -4.919248E37F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.time_boot_ms == (uint)3472807014U);
            };
            DemoDevice.POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.y = (float)3.4138394E37F;
            p3.type_mask = (ushort)(ushort)17121;
            p3.afz = (float) -2.3755813E38F;
            p3.yaw = (float)1.3614257E38F;
            p3.vx = (float) -3.3080578E38F;
            p3.time_boot_ms = (uint)3472807014U;
            p3.z = (float)1.795267E38F;
            p3.afy = (float) -4.673325E37F;
            p3.vy = (float)2.2810863E38F;
            p3.vz = (float)7.2236284E37F;
            p3.afx = (float) -4.919248E37F;
            p3.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p3.x = (float)2.0814962E37F;
            p3.yaw_rate = (float)1.6146614E38F;
            LoopBackDemoChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (uint)1541716418U);
                Debug.Assert(pack.target_component == (byte)(byte)232);
                Debug.Assert(pack.time_usec == (ulong)7001558460147931192L);
                Debug.Assert(pack.target_system == (byte)(byte)218);
            };
            DemoDevice.PING p4 = LoopBackDemoChannel.new_PING();
            PH.setPack(p4);
            p4.seq = (uint)1541716418U;
            p4.time_usec = (ulong)7001558460147931192L;
            p4.target_component = (byte)(byte)232;
            p4.target_system = (byte)(byte)218;
            LoopBackDemoChannel.instance.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.version == (byte)(byte)216);
                Debug.Assert(pack.control_request == (byte)(byte)170);
                Debug.Assert(pack.target_system == (byte)(byte)124);
                Debug.Assert(pack.passkey_LEN(ph) == 11);
                Debug.Assert(pack.passkey_TRY(ph).Equals("cocnjdgxxfO"));
            };
            DemoDevice.CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.control_request = (byte)(byte)170;
            p5.version = (byte)(byte)216;
            p5.passkey_SET("cocnjdgxxfO", PH) ;
            p5.target_system = (byte)(byte)124;
            LoopBackDemoChannel.instance.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gcs_system_id == (byte)(byte)252);
                Debug.Assert(pack.ack == (byte)(byte)43);
                Debug.Assert(pack.control_request == (byte)(byte)212);
            };
            DemoDevice.CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.ack = (byte)(byte)43;
            p6.control_request = (byte)(byte)212;
            p6.gcs_system_id = (byte)(byte)252;
            LoopBackDemoChannel.instance.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 25);
                Debug.Assert(pack.key_TRY(ph).Equals("sddhftfcirtslljwriLtDepou"));
            };
            DemoDevice.AUTH_KEY p7 = LoopBackDemoChannel.new_AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("sddhftfcirtslljwriLtDepou", PH) ;
            LoopBackDemoChannel.instance.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)239);
                Debug.Assert(pack.base_mode == (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_DISARMED);
                Debug.Assert(pack.custom_mode == (uint)2635299313U);
            };
            DemoDevice.SET_MODE p11 = LoopBackDemoChannel.new_SET_MODE();
            PH.setPack(p11);
            p11.target_system = (byte)(byte)239;
            p11.custom_mode = (uint)2635299313U;
            p11.base_mode = (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_DISARMED;
            LoopBackDemoChannel.instance.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 9);
                Debug.Assert(pack.param_id_TRY(ph).Equals("zwyxzlbTe"));
                Debug.Assert(pack.target_component == (byte)(byte)218);
                Debug.Assert(pack.target_system == (byte)(byte)49);
                Debug.Assert(pack.param_index == (short)(short)24761);
            };
            DemoDevice.PARAM_REQUEST_READ p20 = LoopBackDemoChannel.new_PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.target_component = (byte)(byte)218;
            p20.param_id_SET("zwyxzlbTe", PH) ;
            p20.target_system = (byte)(byte)49;
            p20.param_index = (short)(short)24761;
            LoopBackDemoChannel.instance.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)154);
                Debug.Assert(pack.target_system == (byte)(byte)211);
            };
            DemoDevice.PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.new_PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_component = (byte)(byte)154;
            p21.target_system = (byte)(byte)211;
            LoopBackDemoChannel.instance.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 8);
                Debug.Assert(pack.param_id_TRY(ph).Equals("kzjkluip"));
                Debug.Assert(pack.param_count == (ushort)(ushort)30081);
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64);
                Debug.Assert(pack.param_value == (float) -4.2618956E37F);
                Debug.Assert(pack.param_index == (ushort)(ushort)57362);
            };
            DemoDevice.PARAM_VALUE p22 = LoopBackDemoChannel.new_PARAM_VALUE();
            PH.setPack(p22);
            p22.param_index = (ushort)(ushort)57362;
            p22.param_count = (ushort)(ushort)30081;
            p22.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64;
            p22.param_id_SET("kzjkluip", PH) ;
            p22.param_value = (float) -4.2618956E37F;
            LoopBackDemoChannel.instance.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64);
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("gxflz"));
                Debug.Assert(pack.target_system == (byte)(byte)50);
                Debug.Assert(pack.param_value == (float) -2.8343164E38F);
                Debug.Assert(pack.target_component == (byte)(byte)53);
            };
            DemoDevice.PARAM_SET p23 = LoopBackDemoChannel.new_PARAM_SET();
            PH.setPack(p23);
            p23.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64;
            p23.param_id_SET("gxflz", PH) ;
            p23.target_system = (byte)(byte)50;
            p23.target_component = (byte)(byte)53;
            p23.param_value = (float) -2.8343164E38F;
            LoopBackDemoChannel.instance.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vel == (ushort)(ushort)34348);
                Debug.Assert(pack.epv == (ushort)(ushort)58850);
                Debug.Assert(pack.eph == (ushort)(ushort)63723);
                Debug.Assert(pack.time_usec == (ulong)3927507158481731521L);
                Debug.Assert(pack.lat == (int) -569796754);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)1349963632U);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)210142076U);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)3255054105U);
                Debug.Assert(pack.cog == (ushort)(ushort)34053);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int)275979432);
                Debug.Assert(pack.alt == (int)1034358221);
                Debug.Assert(pack.lon == (int)1821994666);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT);
                Debug.Assert(pack.satellites_visible == (byte)(byte)170);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)187167107U);
            };
            DemoDevice.GPS_RAW_INT p24 = LoopBackDemoChannel.new_GPS_RAW_INT();
            PH.setPack(p24);
            p24.eph = (ushort)(ushort)63723;
            p24.cog = (ushort)(ushort)34053;
            p24.epv = (ushort)(ushort)58850;
            p24.hdg_acc_SET((uint)210142076U, PH) ;
            p24.time_usec = (ulong)3927507158481731521L;
            p24.h_acc_SET((uint)187167107U, PH) ;
            p24.vel = (ushort)(ushort)34348;
            p24.lon = (int)1821994666;
            p24.lat = (int) -569796754;
            p24.alt = (int)1034358221;
            p24.alt_ellipsoid_SET((int)275979432, PH) ;
            p24.v_acc_SET((uint)3255054105U, PH) ;
            p24.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT;
            p24.satellites_visible = (byte)(byte)170;
            p24.vel_acc_SET((uint)1349963632U, PH) ;
            LoopBackDemoChannel.instance.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)255, (byte)144, (byte)45, (byte)130, (byte)169, (byte)24, (byte)44, (byte)113, (byte)92, (byte)51, (byte)203, (byte)70, (byte)11, (byte)144, (byte)22, (byte)31, (byte)219, (byte)165, (byte)63, (byte)152}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)247);
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)14, (byte)56, (byte)208, (byte)14, (byte)120, (byte)186, (byte)211, (byte)244, (byte)114, (byte)144, (byte)207, (byte)216, (byte)34, (byte)79, (byte)218, (byte)98, (byte)53, (byte)129, (byte)156, (byte)3}));
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)0, (byte)155, (byte)187, (byte)78, (byte)28, (byte)79, (byte)62, (byte)168, (byte)123, (byte)97, (byte)135, (byte)67, (byte)170, (byte)209, (byte)40, (byte)59, (byte)39, (byte)87, (byte)138, (byte)137}));
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)28, (byte)8, (byte)169, (byte)250, (byte)41, (byte)206, (byte)239, (byte)111, (byte)27, (byte)237, (byte)234, (byte)133, (byte)212, (byte)232, (byte)89, (byte)94, (byte)117, (byte)40, (byte)234, (byte)38}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)136, (byte)227, (byte)212, (byte)217, (byte)122, (byte)20, (byte)80, (byte)43, (byte)124, (byte)62, (byte)148, (byte)69, (byte)205, (byte)222, (byte)82, (byte)67, (byte)213, (byte)80, (byte)175, (byte)103}));
            };
            DemoDevice.GPS_STATUS p25 = LoopBackDemoChannel.new_GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_azimuth_SET(new byte[] {(byte)136, (byte)227, (byte)212, (byte)217, (byte)122, (byte)20, (byte)80, (byte)43, (byte)124, (byte)62, (byte)148, (byte)69, (byte)205, (byte)222, (byte)82, (byte)67, (byte)213, (byte)80, (byte)175, (byte)103}, 0) ;
            p25.satellite_used_SET(new byte[] {(byte)28, (byte)8, (byte)169, (byte)250, (byte)41, (byte)206, (byte)239, (byte)111, (byte)27, (byte)237, (byte)234, (byte)133, (byte)212, (byte)232, (byte)89, (byte)94, (byte)117, (byte)40, (byte)234, (byte)38}, 0) ;
            p25.satellite_prn_SET(new byte[] {(byte)14, (byte)56, (byte)208, (byte)14, (byte)120, (byte)186, (byte)211, (byte)244, (byte)114, (byte)144, (byte)207, (byte)216, (byte)34, (byte)79, (byte)218, (byte)98, (byte)53, (byte)129, (byte)156, (byte)3}, 0) ;
            p25.satellites_visible = (byte)(byte)247;
            p25.satellite_snr_SET(new byte[] {(byte)255, (byte)144, (byte)45, (byte)130, (byte)169, (byte)24, (byte)44, (byte)113, (byte)92, (byte)51, (byte)203, (byte)70, (byte)11, (byte)144, (byte)22, (byte)31, (byte)219, (byte)165, (byte)63, (byte)152}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)0, (byte)155, (byte)187, (byte)78, (byte)28, (byte)79, (byte)62, (byte)168, (byte)123, (byte)97, (byte)135, (byte)67, (byte)170, (byte)209, (byte)40, (byte)59, (byte)39, (byte)87, (byte)138, (byte)137}, 0) ;
            LoopBackDemoChannel.instance.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (short)(short) -30115);
                Debug.Assert(pack.xmag == (short)(short)25327);
                Debug.Assert(pack.xacc == (short)(short)5573);
                Debug.Assert(pack.time_boot_ms == (uint)1189003233U);
                Debug.Assert(pack.zacc == (short)(short) -24556);
                Debug.Assert(pack.ygyro == (short)(short)7220);
                Debug.Assert(pack.zgyro == (short)(short) -24599);
                Debug.Assert(pack.xgyro == (short)(short)11085);
                Debug.Assert(pack.zmag == (short)(short) -2634);
                Debug.Assert(pack.ymag == (short)(short) -3559);
            };
            DemoDevice.SCALED_IMU p26 = LoopBackDemoChannel.new_SCALED_IMU();
            PH.setPack(p26);
            p26.yacc = (short)(short) -30115;
            p26.zacc = (short)(short) -24556;
            p26.xacc = (short)(short)5573;
            p26.ygyro = (short)(short)7220;
            p26.xgyro = (short)(short)11085;
            p26.zmag = (short)(short) -2634;
            p26.zgyro = (short)(short) -24599;
            p26.xmag = (short)(short)25327;
            p26.time_boot_ms = (uint)1189003233U;
            p26.ymag = (short)(short) -3559;
            LoopBackDemoChannel.instance.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xmag == (short)(short)5407);
                Debug.Assert(pack.ygyro == (short)(short) -19825);
                Debug.Assert(pack.zgyro == (short)(short) -12463);
                Debug.Assert(pack.zacc == (short)(short)17436);
                Debug.Assert(pack.time_usec == (ulong)8603602437944954505L);
                Debug.Assert(pack.xacc == (short)(short) -26956);
                Debug.Assert(pack.ymag == (short)(short) -20682);
                Debug.Assert(pack.zmag == (short)(short)25696);
                Debug.Assert(pack.xgyro == (short)(short)20888);
                Debug.Assert(pack.yacc == (short)(short)25759);
            };
            DemoDevice.RAW_IMU p27 = LoopBackDemoChannel.new_RAW_IMU();
            PH.setPack(p27);
            p27.xmag = (short)(short)5407;
            p27.zmag = (short)(short)25696;
            p27.zacc = (short)(short)17436;
            p27.xgyro = (short)(short)20888;
            p27.ygyro = (short)(short) -19825;
            p27.xacc = (short)(short) -26956;
            p27.yacc = (short)(short)25759;
            p27.ymag = (short)(short) -20682;
            p27.zgyro = (short)(short) -12463;
            p27.time_usec = (ulong)8603602437944954505L;
            LoopBackDemoChannel.instance.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)2823152224951225305L);
                Debug.Assert(pack.press_diff1 == (short)(short)6013);
                Debug.Assert(pack.temperature == (short)(short)5288);
                Debug.Assert(pack.press_abs == (short)(short)13205);
                Debug.Assert(pack.press_diff2 == (short)(short)6666);
            };
            DemoDevice.RAW_PRESSURE p28 = LoopBackDemoChannel.new_RAW_PRESSURE();
            PH.setPack(p28);
            p28.temperature = (short)(short)5288;
            p28.press_diff2 = (short)(short)6666;
            p28.press_diff1 = (short)(short)6013;
            p28.time_usec = (ulong)2823152224951225305L;
            p28.press_abs = (short)(short)13205;
            LoopBackDemoChannel.instance.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)408558741U);
                Debug.Assert(pack.temperature == (short)(short) -24058);
                Debug.Assert(pack.press_abs == (float) -1.854905E38F);
                Debug.Assert(pack.press_diff == (float)3.186559E38F);
            };
            DemoDevice.SCALED_PRESSURE p29 = LoopBackDemoChannel.new_SCALED_PRESSURE();
            PH.setPack(p29);
            p29.time_boot_ms = (uint)408558741U;
            p29.temperature = (short)(short) -24058;
            p29.press_diff = (float)3.186559E38F;
            p29.press_abs = (float) -1.854905E38F;
            LoopBackDemoChannel.instance.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitchspeed == (float)1.0901795E38F);
                Debug.Assert(pack.roll == (float)2.096791E38F);
                Debug.Assert(pack.rollspeed == (float)2.3043456E38F);
                Debug.Assert(pack.yawspeed == (float)2.2453782E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3351998049U);
                Debug.Assert(pack.yaw == (float)3.026769E38F);
                Debug.Assert(pack.pitch == (float) -2.143158E38F);
            };
            DemoDevice.ATTITUDE p30 = LoopBackDemoChannel.new_ATTITUDE();
            PH.setPack(p30);
            p30.pitchspeed = (float)1.0901795E38F;
            p30.yawspeed = (float)2.2453782E38F;
            p30.time_boot_ms = (uint)3351998049U;
            p30.yaw = (float)3.026769E38F;
            p30.pitch = (float) -2.143158E38F;
            p30.roll = (float)2.096791E38F;
            p30.rollspeed = (float)2.3043456E38F;
            LoopBackDemoChannel.instance.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q2 == (float)2.3487596E38F);
                Debug.Assert(pack.rollspeed == (float)1.6789705E38F);
                Debug.Assert(pack.q3 == (float)3.2360128E38F);
                Debug.Assert(pack.q1 == (float)3.959804E37F);
                Debug.Assert(pack.time_boot_ms == (uint)1593918644U);
                Debug.Assert(pack.q4 == (float)1.4620015E38F);
                Debug.Assert(pack.yawspeed == (float)1.3529217E38F);
                Debug.Assert(pack.pitchspeed == (float) -1.435835E38F);
            };
            DemoDevice.ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.rollspeed = (float)1.6789705E38F;
            p31.q2 = (float)2.3487596E38F;
            p31.pitchspeed = (float) -1.435835E38F;
            p31.q4 = (float)1.4620015E38F;
            p31.time_boot_ms = (uint)1593918644U;
            p31.q3 = (float)3.2360128E38F;
            p31.q1 = (float)3.959804E37F;
            p31.yawspeed = (float)1.3529217E38F;
            LoopBackDemoChannel.instance.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float) -1.35097E38F);
                Debug.Assert(pack.z == (float)9.365197E37F);
                Debug.Assert(pack.time_boot_ms == (uint)974950911U);
                Debug.Assert(pack.y == (float)3.2730897E38F);
                Debug.Assert(pack.vz == (float)1.3007959E38F);
                Debug.Assert(pack.vx == (float)1.8324875E38F);
                Debug.Assert(pack.x == (float) -2.8704277E38F);
            };
            DemoDevice.LOCAL_POSITION_NED p32 = LoopBackDemoChannel.new_LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.z = (float)9.365197E37F;
            p32.vy = (float) -1.35097E38F;
            p32.vz = (float)1.3007959E38F;
            p32.y = (float)3.2730897E38F;
            p32.vx = (float)1.8324875E38F;
            p32.x = (float) -2.8704277E38F;
            p32.time_boot_ms = (uint)974950911U;
            LoopBackDemoChannel.instance.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -1134468808);
                Debug.Assert(pack.time_boot_ms == (uint)3237734696U);
                Debug.Assert(pack.hdg == (ushort)(ushort)8732);
                Debug.Assert(pack.alt == (int) -30379192);
                Debug.Assert(pack.vx == (short)(short)14919);
                Debug.Assert(pack.vz == (short)(short) -26170);
                Debug.Assert(pack.lon == (int) -424580733);
                Debug.Assert(pack.relative_alt == (int)1368406062);
                Debug.Assert(pack.vy == (short)(short)2770);
            };
            DemoDevice.GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.lon = (int) -424580733;
            p33.lat = (int) -1134468808;
            p33.alt = (int) -30379192;
            p33.vz = (short)(short) -26170;
            p33.vy = (short)(short)2770;
            p33.hdg = (ushort)(ushort)8732;
            p33.relative_alt = (int)1368406062;
            p33.vx = (short)(short)14919;
            p33.time_boot_ms = (uint)3237734696U;
            LoopBackDemoChannel.instance.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)364947559U);
                Debug.Assert(pack.chan8_scaled == (short)(short) -26254);
                Debug.Assert(pack.chan5_scaled == (short)(short) -9859);
                Debug.Assert(pack.chan6_scaled == (short)(short) -15728);
                Debug.Assert(pack.port == (byte)(byte)166);
                Debug.Assert(pack.chan7_scaled == (short)(short)5498);
                Debug.Assert(pack.chan1_scaled == (short)(short) -22802);
                Debug.Assert(pack.chan2_scaled == (short)(short)20403);
                Debug.Assert(pack.chan4_scaled == (short)(short)19908);
                Debug.Assert(pack.rssi == (byte)(byte)212);
                Debug.Assert(pack.chan3_scaled == (short)(short) -16506);
            };
            DemoDevice.RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.new_RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.chan3_scaled = (short)(short) -16506;
            p34.time_boot_ms = (uint)364947559U;
            p34.chan6_scaled = (short)(short) -15728;
            p34.chan2_scaled = (short)(short)20403;
            p34.chan5_scaled = (short)(short) -9859;
            p34.chan7_scaled = (short)(short)5498;
            p34.chan4_scaled = (short)(short)19908;
            p34.rssi = (byte)(byte)212;
            p34.port = (byte)(byte)166;
            p34.chan8_scaled = (short)(short) -26254;
            p34.chan1_scaled = (short)(short) -22802;
            LoopBackDemoChannel.instance.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)44507);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)50348);
                Debug.Assert(pack.rssi == (byte)(byte)33);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)32122);
                Debug.Assert(pack.time_boot_ms == (uint)1443568369U);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)61980);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)6089);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)28119);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)25554);
                Debug.Assert(pack.port == (byte)(byte)188);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)35925);
            };
            DemoDevice.RC_CHANNELS_RAW p35 = LoopBackDemoChannel.new_RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.chan2_raw = (ushort)(ushort)6089;
            p35.rssi = (byte)(byte)33;
            p35.time_boot_ms = (uint)1443568369U;
            p35.chan8_raw = (ushort)(ushort)28119;
            p35.port = (byte)(byte)188;
            p35.chan7_raw = (ushort)(ushort)35925;
            p35.chan6_raw = (ushort)(ushort)61980;
            p35.chan3_raw = (ushort)(ushort)50348;
            p35.chan5_raw = (ushort)(ushort)44507;
            p35.chan4_raw = (ushort)(ushort)25554;
            p35.chan1_raw = (ushort)(ushort)32122;
            LoopBackDemoChannel.instance.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)48363);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)33946);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)37625);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)61770);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)21975);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)44716);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)4935);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)15321);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)22425);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)14617);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)55605);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)57929);
                Debug.Assert(pack.time_usec == (uint)2074626524U);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)57706);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)58241);
                Debug.Assert(pack.port == (byte)(byte)132);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)48807);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)62841);
            };
            DemoDevice.SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.new_SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo15_raw_SET((ushort)(ushort)33946, PH) ;
            p36.servo12_raw_SET((ushort)(ushort)58241, PH) ;
            p36.servo13_raw_SET((ushort)(ushort)4935, PH) ;
            p36.port = (byte)(byte)132;
            p36.servo10_raw_SET((ushort)(ushort)14617, PH) ;
            p36.servo16_raw_SET((ushort)(ushort)57929, PH) ;
            p36.servo9_raw_SET((ushort)(ushort)55605, PH) ;
            p36.servo5_raw = (ushort)(ushort)62841;
            p36.servo11_raw_SET((ushort)(ushort)15321, PH) ;
            p36.servo2_raw = (ushort)(ushort)44716;
            p36.servo4_raw = (ushort)(ushort)22425;
            p36.servo6_raw = (ushort)(ushort)21975;
            p36.servo3_raw = (ushort)(ushort)48363;
            p36.servo14_raw_SET((ushort)(ushort)57706, PH) ;
            p36.servo8_raw = (ushort)(ushort)61770;
            p36.servo7_raw = (ushort)(ushort)48807;
            p36.time_usec = (uint)2074626524U;
            p36.servo1_raw = (ushort)(ushort)37625;
            LoopBackDemoChannel.instance.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.end_index == (short)(short)13773);
                Debug.Assert(pack.target_system == (byte)(byte)51);
                Debug.Assert(pack.start_index == (short)(short) -31752);
                Debug.Assert(pack.target_component == (byte)(byte)17);
            };
            DemoDevice.MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.new_MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.target_component = (byte)(byte)17;
            p37.target_system = (byte)(byte)51;
            p37.start_index = (short)(short) -31752;
            p37.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p37.end_index = (short)(short)13773;
            LoopBackDemoChannel.instance.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_index == (short)(short) -32521);
                Debug.Assert(pack.target_system == (byte)(byte)28);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.end_index == (short)(short)32619);
                Debug.Assert(pack.target_component == (byte)(byte)197);
            };
            DemoDevice.MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.new_MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.target_system = (byte)(byte)28;
            p38.target_component = (byte)(byte)197;
            p38.end_index = (short)(short)32619;
            p38.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p38.start_index = (short)(short) -32521;
            LoopBackDemoChannel.instance.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT);
                Debug.Assert(pack.autocontinue == (byte)(byte)28);
                Debug.Assert(pack.target_system == (byte)(byte)142);
                Debug.Assert(pack.x == (float)1.934951E38F);
                Debug.Assert(pack.current == (byte)(byte)148);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT);
                Debug.Assert(pack.y == (float) -2.6023922E38F);
                Debug.Assert(pack.param2 == (float)7.791914E37F);
                Debug.Assert(pack.param1 == (float) -7.4000387E37F);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.seq == (ushort)(ushort)54949);
                Debug.Assert(pack.target_component == (byte)(byte)207);
                Debug.Assert(pack.z == (float) -2.1256128E38F);
                Debug.Assert(pack.param4 == (float)1.1815127E38F);
                Debug.Assert(pack.param3 == (float) -1.029902E38F);
            };
            DemoDevice.MISSION_ITEM p39 = LoopBackDemoChannel.new_MISSION_ITEM();
            PH.setPack(p39);
            p39.command = (MAV_CMD)MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT;
            p39.param4 = (float)1.1815127E38F;
            p39.current = (byte)(byte)148;
            p39.x = (float)1.934951E38F;
            p39.param2 = (float)7.791914E37F;
            p39.autocontinue = (byte)(byte)28;
            p39.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p39.param1 = (float) -7.4000387E37F;
            p39.target_system = (byte)(byte)142;
            p39.z = (float) -2.1256128E38F;
            p39.param3 = (float) -1.029902E38F;
            p39.seq = (ushort)(ushort)54949;
            p39.y = (float) -2.6023922E38F;
            p39.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p39.target_component = (byte)(byte)207;
            LoopBackDemoChannel.instance.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)120);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_system == (byte)(byte)200);
                Debug.Assert(pack.seq == (ushort)(ushort)55036);
            };
            DemoDevice.MISSION_REQUEST p40 = LoopBackDemoChannel.new_MISSION_REQUEST();
            PH.setPack(p40);
            p40.seq = (ushort)(ushort)55036;
            p40.target_component = (byte)(byte)120;
            p40.target_system = (byte)(byte)200;
            p40.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            LoopBackDemoChannel.instance.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)64);
                Debug.Assert(pack.target_system == (byte)(byte)205);
                Debug.Assert(pack.seq == (ushort)(ushort)51884);
            };
            DemoDevice.MISSION_SET_CURRENT p41 = LoopBackDemoChannel.new_MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_system = (byte)(byte)205;
            p41.seq = (ushort)(ushort)51884;
            p41.target_component = (byte)(byte)64;
            LoopBackDemoChannel.instance.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)53602);
            };
            DemoDevice.MISSION_CURRENT p42 = LoopBackDemoChannel.new_MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)53602;
            LoopBackDemoChannel.instance.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)253);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)186);
            };
            DemoDevice.MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.new_MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p43.target_system = (byte)(byte)253;
            p43.target_component = (byte)(byte)186;
            LoopBackDemoChannel.instance.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)82);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)83);
                Debug.Assert(pack.count == (ushort)(ushort)46353);
            };
            DemoDevice.MISSION_COUNT p44 = LoopBackDemoChannel.new_MISSION_COUNT();
            PH.setPack(p44);
            p44.target_system = (byte)(byte)82;
            p44.count = (ushort)(ushort)46353;
            p44.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p44.target_component = (byte)(byte)83;
            LoopBackDemoChannel.instance.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)191);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_component == (byte)(byte)107);
            };
            DemoDevice.MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.new_MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_component = (byte)(byte)107;
            p45.target_system = (byte)(byte)191;
            p45.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            LoopBackDemoChannel.instance.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)48936);
            };
            DemoDevice.MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.new_MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)48936;
            LoopBackDemoChannel.instance.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_system == (byte)(byte)73);
                Debug.Assert(pack.type == (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_INVALID);
                Debug.Assert(pack.target_component == (byte)(byte)26);
            };
            DemoDevice.MISSION_ACK p47 = LoopBackDemoChannel.new_MISSION_ACK();
            PH.setPack(p47);
            p47.target_system = (byte)(byte)73;
            p47.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p47.type = (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_INVALID;
            p47.target_component = (byte)(byte)26;
            LoopBackDemoChannel.instance.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int) -1490652743);
                Debug.Assert(pack.latitude == (int)420153935);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)3433596982974561030L);
                Debug.Assert(pack.target_system == (byte)(byte)57);
                Debug.Assert(pack.longitude == (int) -646271603);
            };
            DemoDevice.SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.new_SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.altitude = (int) -1490652743;
            p48.longitude = (int) -646271603;
            p48.latitude = (int)420153935;
            p48.target_system = (byte)(byte)57;
            p48.time_usec_SET((ulong)3433596982974561030L, PH) ;
            LoopBackDemoChannel.instance.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int)81857138);
                Debug.Assert(pack.latitude == (int) -1231153244);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)2362054784715632189L);
                Debug.Assert(pack.longitude == (int)559301983);
            };
            DemoDevice.GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.new_GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.time_usec_SET((ulong)2362054784715632189L, PH) ;
            p49.altitude = (int)81857138;
            p49.latitude = (int) -1231153244;
            p49.longitude = (int)559301983;
            LoopBackDemoChannel.instance.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.scale == (float)1.839664E38F);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)167);
                Debug.Assert(pack.param_value0 == (float)1.3979048E38F);
                Debug.Assert(pack.param_value_min == (float)1.4575345E38F);
                Debug.Assert(pack.target_component == (byte)(byte)109);
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("hVpdy"));
                Debug.Assert(pack.target_system == (byte)(byte)45);
                Debug.Assert(pack.param_value_max == (float)7.2167336E36F);
                Debug.Assert(pack.param_index == (short)(short)21080);
            };
            DemoDevice.PARAM_MAP_RC p50 = LoopBackDemoChannel.new_PARAM_MAP_RC();
            PH.setPack(p50);
            p50.param_value_max = (float)7.2167336E36F;
            p50.param_id_SET("hVpdy", PH) ;
            p50.param_index = (short)(short)21080;
            p50.parameter_rc_channel_index = (byte)(byte)167;
            p50.scale = (float)1.839664E38F;
            p50.param_value_min = (float)1.4575345E38F;
            p50.target_component = (byte)(byte)109;
            p50.param_value0 = (float)1.3979048E38F;
            p50.target_system = (byte)(byte)45;
            LoopBackDemoChannel.instance.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)104);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.seq == (ushort)(ushort)30256);
                Debug.Assert(pack.target_component == (byte)(byte)101);
            };
            DemoDevice.MISSION_REQUEST_INT p51 = LoopBackDemoChannel.new_MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.seq = (ushort)(ushort)30256;
            p51.target_component = (byte)(byte)101;
            p51.target_system = (byte)(byte)104;
            p51.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            LoopBackDemoChannel.instance.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p2y == (float) -2.5780878E38F);
                Debug.Assert(pack.p2z == (float)2.0186946E38F);
                Debug.Assert(pack.target_component == (byte)(byte)157);
                Debug.Assert(pack.p1z == (float)2.7759378E38F);
                Debug.Assert(pack.p1x == (float)3.2927892E38F);
                Debug.Assert(pack.p1y == (float) -1.5854879E38F);
                Debug.Assert(pack.target_system == (byte)(byte)23);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_NED);
                Debug.Assert(pack.p2x == (float)1.5126123E38F);
            };
            DemoDevice.SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.new_SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p1z = (float)2.7759378E38F;
            p54.target_system = (byte)(byte)23;
            p54.p1y = (float) -1.5854879E38F;
            p54.p2z = (float)2.0186946E38F;
            p54.target_component = (byte)(byte)157;
            p54.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_NED;
            p54.p2x = (float)1.5126123E38F;
            p54.p1x = (float)3.2927892E38F;
            p54.p2y = (float) -2.5780878E38F;
            LoopBackDemoChannel.instance.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p1z == (float)5.197312E37F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED);
                Debug.Assert(pack.p2z == (float)2.7361412E37F);
                Debug.Assert(pack.p2x == (float)3.044367E38F);
                Debug.Assert(pack.p1y == (float) -2.6290224E38F);
                Debug.Assert(pack.p2y == (float) -1.7421329E37F);
                Debug.Assert(pack.p1x == (float) -2.8607718E38F);
            };
            DemoDevice.SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.new_SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p2z = (float)2.7361412E37F;
            p55.p1z = (float)5.197312E37F;
            p55.p1y = (float) -2.6290224E38F;
            p55.p2y = (float) -1.7421329E37F;
            p55.p1x = (float) -2.8607718E38F;
            p55.p2x = (float)3.044367E38F;
            p55.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED;
            LoopBackDemoChannel.instance.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rollspeed == (float)1.894525E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-3.17215E38F, -6.360755E37F, 8.0424424E37F, 2.212381E38F, -3.2283193E38F, -7.079594E37F, -2.1783229E38F, 1.7930474E37F, -9.333939E37F}));
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.4071158E38F, -2.1227872E38F, 1.9662109E38F, -3.124623E38F}));
                Debug.Assert(pack.pitchspeed == (float)7.9588013E37F);
                Debug.Assert(pack.yawspeed == (float)2.1403936E38F);
                Debug.Assert(pack.time_usec == (ulong)5668028262060986157L);
            };
            DemoDevice.ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.rollspeed = (float)1.894525E38F;
            p61.time_usec = (ulong)5668028262060986157L;
            p61.q_SET(new float[] {1.4071158E38F, -2.1227872E38F, 1.9662109E38F, -3.124623E38F}, 0) ;
            p61.pitchspeed = (float)7.9588013E37F;
            p61.covariance_SET(new float[] {-3.17215E38F, -6.360755E37F, 8.0424424E37F, 2.212381E38F, -3.2283193E38F, -7.079594E37F, -2.1783229E38F, 1.7930474E37F, -9.333939E37F}, 0) ;
            p61.yawspeed = (float)2.1403936E38F;
            LoopBackDemoChannel.instance.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.nav_bearing == (short)(short) -4758);
                Debug.Assert(pack.xtrack_error == (float) -7.249761E35F);
                Debug.Assert(pack.aspd_error == (float) -2.846515E38F);
                Debug.Assert(pack.target_bearing == (short)(short)6151);
                Debug.Assert(pack.nav_roll == (float) -2.407355E38F);
                Debug.Assert(pack.alt_error == (float)7.0744223E37F);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)21623);
                Debug.Assert(pack.nav_pitch == (float) -3.1642762E37F);
            };
            DemoDevice.NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.new_NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.nav_roll = (float) -2.407355E38F;
            p62.xtrack_error = (float) -7.249761E35F;
            p62.alt_error = (float)7.0744223E37F;
            p62.aspd_error = (float) -2.846515E38F;
            p62.nav_bearing = (short)(short) -4758;
            p62.wp_dist = (ushort)(ushort)21623;
            p62.target_bearing = (short)(short)6151;
            p62.nav_pitch = (float) -3.1642762E37F;
            LoopBackDemoChannel.instance.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vz == (float) -7.647684E37F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-2.5112025E38F, -2.3905665E38F, 3.173454E38F, 1.1188978E38F, -3.5553472E37F, 2.7089323E37F, -1.7317949E38F, -1.9138871E38F, 8.1807305E36F, -1.1352022E37F, -3.3755692E38F, 1.7468907E38F, 1.5031614E38F, 3.295751E38F, -2.1587051E38F, -4.882638E37F, -6.410685E37F, -2.3528481E38F, 2.9968757E38F, 3.1775579E38F, 2.542135E38F, -1.8925182E37F, 2.7905004E38F, 2.3659228E38F, -6.638107E37F, 2.5553921E38F, -2.9549183E38F, -1.8790515E38F, 1.9959794E38F, 6.1282783E37F, 9.447179E37F, -2.1269433E38F, 3.3908282E38F, 1.651545E38F, 2.3384675E38F, 3.0744079E38F}));
                Debug.Assert(pack.alt == (int) -735798613);
                Debug.Assert(pack.lat == (int)1108511612);
                Debug.Assert(pack.relative_alt == (int)405894836);
                Debug.Assert(pack.time_usec == (ulong)6856516711638689141L);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
                Debug.Assert(pack.vy == (float)2.1408227E38F);
                Debug.Assert(pack.vx == (float)9.793736E37F);
                Debug.Assert(pack.lon == (int)308025273);
            };
            DemoDevice.GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.alt = (int) -735798613;
            p63.lat = (int)1108511612;
            p63.vz = (float) -7.647684E37F;
            p63.vx = (float)9.793736E37F;
            p63.time_usec = (ulong)6856516711638689141L;
            p63.lon = (int)308025273;
            p63.relative_alt = (int)405894836;
            p63.vy = (float)2.1408227E38F;
            p63.covariance_SET(new float[] {-2.5112025E38F, -2.3905665E38F, 3.173454E38F, 1.1188978E38F, -3.5553472E37F, 2.7089323E37F, -1.7317949E38F, -1.9138871E38F, 8.1807305E36F, -1.1352022E37F, -3.3755692E38F, 1.7468907E38F, 1.5031614E38F, 3.295751E38F, -2.1587051E38F, -4.882638E37F, -6.410685E37F, -2.3528481E38F, 2.9968757E38F, 3.1775579E38F, 2.542135E38F, -1.8925182E37F, 2.7905004E38F, 2.3659228E38F, -6.638107E37F, 2.5553921E38F, -2.9549183E38F, -1.8790515E38F, 1.9959794E38F, 6.1282783E37F, 9.447179E37F, -2.1269433E38F, 3.3908282E38F, 1.651545E38F, 2.3384675E38F, 3.0744079E38F}, 0) ;
            p63.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS;
            LoopBackDemoChannel.instance.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
                Debug.Assert(pack.ax == (float) -1.423331E38F);
                Debug.Assert(pack.y == (float)1.0780999E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {1.7588883E38F, 1.1239683E38F, -9.963652E37F, 1.2874599E38F, 2.4306383E38F, 2.76008E37F, 1.4880568E38F, -2.98857E38F, 2.5836559E38F, -2.3541535E38F, 2.0230314E38F, 2.8414812E38F, 3.3732598E37F, -6.6366595E37F, -2.0605987E38F, -4.2244033E37F, 2.2842938E38F, -1.3849046E38F, -2.1168688E38F, -1.2175324E38F, 2.5958113E38F, 3.3001304E38F, -2.2935511E38F, -1.2385925E38F, 3.0604008E38F, 2.1379175E38F, -6.883437E36F, 3.190333E38F, 1.5138222E38F, 2.3926475E38F, -2.7883794E38F, -2.503355E38F, -2.642797E36F, 1.8940592E38F, -2.4929059E38F, -2.6287031E38F, 1.0852078E38F, 6.9259244E36F, 6.1585133E37F, -2.0819881E38F, 2.4783774E38F, -4.058352E37F, 8.961148E37F, -1.437612E38F, -3.2625256E38F}));
                Debug.Assert(pack.x == (float)7.761855E37F);
                Debug.Assert(pack.vy == (float) -1.5706114E38F);
                Debug.Assert(pack.vz == (float) -1.8376678E38F);
                Debug.Assert(pack.ay == (float)3.1362743E38F);
                Debug.Assert(pack.az == (float) -1.0203418E38F);
                Debug.Assert(pack.z == (float) -1.9896586E37F);
                Debug.Assert(pack.time_usec == (ulong)3089489415782268554L);
                Debug.Assert(pack.vx == (float)3.396126E38F);
            };
            DemoDevice.LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.z = (float) -1.9896586E37F;
            p64.x = (float)7.761855E37F;
            p64.y = (float)1.0780999E38F;
            p64.time_usec = (ulong)3089489415782268554L;
            p64.az = (float) -1.0203418E38F;
            p64.vx = (float)3.396126E38F;
            p64.vz = (float) -1.8376678E38F;
            p64.vy = (float) -1.5706114E38F;
            p64.ax = (float) -1.423331E38F;
            p64.ay = (float)3.1362743E38F;
            p64.covariance_SET(new float[] {1.7588883E38F, 1.1239683E38F, -9.963652E37F, 1.2874599E38F, 2.4306383E38F, 2.76008E37F, 1.4880568E38F, -2.98857E38F, 2.5836559E38F, -2.3541535E38F, 2.0230314E38F, 2.8414812E38F, 3.3732598E37F, -6.6366595E37F, -2.0605987E38F, -4.2244033E37F, 2.2842938E38F, -1.3849046E38F, -2.1168688E38F, -1.2175324E38F, 2.5958113E38F, 3.3001304E38F, -2.2935511E38F, -1.2385925E38F, 3.0604008E38F, 2.1379175E38F, -6.883437E36F, 3.190333E38F, 1.5138222E38F, 2.3926475E38F, -2.7883794E38F, -2.503355E38F, -2.642797E36F, 1.8940592E38F, -2.4929059E38F, -2.6287031E38F, 1.0852078E38F, 6.9259244E36F, 6.1585133E37F, -2.0819881E38F, 2.4783774E38F, -4.058352E37F, 8.961148E37F, -1.437612E38F, -3.2625256E38F}, 0) ;
            p64.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO;
            LoopBackDemoChannel.instance.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)44042);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)19073);
                Debug.Assert(pack.rssi == (byte)(byte)132);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)31741);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)48131);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)22477);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)47075);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)47211);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)47257);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)29708);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)59359);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)41062);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)4702);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)43789);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)47943);
                Debug.Assert(pack.time_boot_ms == (uint)2675017226U);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)47866);
                Debug.Assert(pack.chancount == (byte)(byte)163);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)2921);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)63049);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)52430);
            };
            DemoDevice.RC_CHANNELS p65 = LoopBackDemoChannel.new_RC_CHANNELS();
            PH.setPack(p65);
            p65.chan2_raw = (ushort)(ushort)47211;
            p65.chan4_raw = (ushort)(ushort)31741;
            p65.rssi = (byte)(byte)132;
            p65.chan9_raw = (ushort)(ushort)47866;
            p65.chan15_raw = (ushort)(ushort)22477;
            p65.chan3_raw = (ushort)(ushort)41062;
            p65.chan14_raw = (ushort)(ushort)19073;
            p65.chan13_raw = (ushort)(ushort)47075;
            p65.time_boot_ms = (uint)2675017226U;
            p65.chan11_raw = (ushort)(ushort)47257;
            p65.chan18_raw = (ushort)(ushort)47943;
            p65.chan1_raw = (ushort)(ushort)4702;
            p65.chan12_raw = (ushort)(ushort)48131;
            p65.chan5_raw = (ushort)(ushort)52430;
            p65.chan10_raw = (ushort)(ushort)59359;
            p65.chancount = (byte)(byte)163;
            p65.chan16_raw = (ushort)(ushort)63049;
            p65.chan7_raw = (ushort)(ushort)29708;
            p65.chan8_raw = (ushort)(ushort)43789;
            p65.chan17_raw = (ushort)(ushort)2921;
            p65.chan6_raw = (ushort)(ushort)44042;
            LoopBackDemoChannel.instance.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.req_stream_id == (byte)(byte)203);
                Debug.Assert(pack.target_component == (byte)(byte)206);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)23116);
                Debug.Assert(pack.start_stop == (byte)(byte)202);
                Debug.Assert(pack.target_system == (byte)(byte)28);
            };
            DemoDevice.REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.new_REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.target_component = (byte)(byte)206;
            p66.req_message_rate = (ushort)(ushort)23116;
            p66.target_system = (byte)(byte)28;
            p66.start_stop = (byte)(byte)202;
            p66.req_stream_id = (byte)(byte)203;
            LoopBackDemoChannel.instance.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.on_off == (byte)(byte)110);
                Debug.Assert(pack.stream_id == (byte)(byte)69);
                Debug.Assert(pack.message_rate == (ushort)(ushort)33947);
            };
            DemoDevice.DATA_STREAM p67 = LoopBackDemoChannel.new_DATA_STREAM();
            PH.setPack(p67);
            p67.message_rate = (ushort)(ushort)33947;
            p67.on_off = (byte)(byte)110;
            p67.stream_id = (byte)(byte)69;
            LoopBackDemoChannel.instance.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.buttons == (ushort)(ushort)60160);
                Debug.Assert(pack.z == (short)(short) -2975);
                Debug.Assert(pack.target == (byte)(byte)199);
                Debug.Assert(pack.r == (short)(short)32510);
                Debug.Assert(pack.x == (short)(short) -14303);
                Debug.Assert(pack.y == (short)(short)9721);
            };
            DemoDevice.MANUAL_CONTROL p69 = LoopBackDemoChannel.new_MANUAL_CONTROL();
            PH.setPack(p69);
            p69.x = (short)(short) -14303;
            p69.r = (short)(short)32510;
            p69.y = (short)(short)9721;
            p69.buttons = (ushort)(ushort)60160;
            p69.target = (byte)(byte)199;
            p69.z = (short)(short) -2975;
            LoopBackDemoChannel.instance.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)28956);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)13578);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)30991);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)45714);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)8707);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)13952);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)19951);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)34132);
                Debug.Assert(pack.target_component == (byte)(byte)223);
                Debug.Assert(pack.target_system == (byte)(byte)44);
            };
            DemoDevice.RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.new_RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.target_component = (byte)(byte)223;
            p70.chan1_raw = (ushort)(ushort)45714;
            p70.chan8_raw = (ushort)(ushort)34132;
            p70.chan3_raw = (ushort)(ushort)13952;
            p70.chan5_raw = (ushort)(ushort)13578;
            p70.chan2_raw = (ushort)(ushort)30991;
            p70.chan7_raw = (ushort)(ushort)28956;
            p70.chan4_raw = (ushort)(ushort)8707;
            p70.target_system = (byte)(byte)44;
            p70.chan6_raw = (ushort)(ushort)19951;
            LoopBackDemoChannel.instance.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)62499);
                Debug.Assert(pack.z == (float)3.366683E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)122);
                Debug.Assert(pack.param2 == (float)1.9356098E38F);
                Debug.Assert(pack.param1 == (float)2.3855935E38F);
                Debug.Assert(pack.y == (int)301836325);
                Debug.Assert(pack.param3 == (float)2.3437665E38F);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.current == (byte)(byte)77);
                Debug.Assert(pack.x == (int)1455257739);
                Debug.Assert(pack.target_component == (byte)(byte)223);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM);
                Debug.Assert(pack.target_system == (byte)(byte)164);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED);
                Debug.Assert(pack.param4 == (float) -5.350248E37F);
            };
            DemoDevice.MISSION_ITEM_INT p73 = LoopBackDemoChannel.new_MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.y = (int)301836325;
            p73.target_component = (byte)(byte)223;
            p73.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p73.current = (byte)(byte)77;
            p73.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p73.target_system = (byte)(byte)164;
            p73.autocontinue = (byte)(byte)122;
            p73.command = (MAV_CMD)MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM;
            p73.param3 = (float)2.3437665E38F;
            p73.x = (int)1455257739;
            p73.param1 = (float)2.3855935E38F;
            p73.param4 = (float) -5.350248E37F;
            p73.z = (float)3.366683E38F;
            p73.seq = (ushort)(ushort)62499;
            p73.param2 = (float)1.9356098E38F;
            LoopBackDemoChannel.instance.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.heading == (short)(short)2334);
                Debug.Assert(pack.climb == (float) -1.2069316E38F);
                Debug.Assert(pack.airspeed == (float) -2.1957503E38F);
                Debug.Assert(pack.throttle == (ushort)(ushort)51569);
                Debug.Assert(pack.groundspeed == (float)2.9345836E38F);
                Debug.Assert(pack.alt == (float)1.9703087E38F);
            };
            DemoDevice.VFR_HUD p74 = LoopBackDemoChannel.new_VFR_HUD();
            PH.setPack(p74);
            p74.climb = (float) -1.2069316E38F;
            p74.airspeed = (float) -2.1957503E38F;
            p74.throttle = (ushort)(ushort)51569;
            p74.alt = (float)1.9703087E38F;
            p74.heading = (short)(short)2334;
            p74.groundspeed = (float)2.9345836E38F;
            LoopBackDemoChannel.instance.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)181);
                Debug.Assert(pack.z == (float)4.882446E37F);
                Debug.Assert(pack.param4 == (float) -1.2723995E38F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD);
                Debug.Assert(pack.target_component == (byte)(byte)48);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.autocontinue == (byte)(byte)25);
                Debug.Assert(pack.param3 == (float)8.146541E37F);
                Debug.Assert(pack.param2 == (float) -7.5261856E37F);
                Debug.Assert(pack.x == (int) -847521050);
                Debug.Assert(pack.current == (byte)(byte)56);
                Debug.Assert(pack.param1 == (float)2.64223E38F);
                Debug.Assert(pack.y == (int) -460211619);
            };
            DemoDevice.COMMAND_INT p75 = LoopBackDemoChannel.new_COMMAND_INT();
            PH.setPack(p75);
            p75.target_system = (byte)(byte)181;
            p75.param4 = (float) -1.2723995E38F;
            p75.param2 = (float) -7.5261856E37F;
            p75.x = (int) -847521050;
            p75.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION;
            p75.autocontinue = (byte)(byte)25;
            p75.z = (float)4.882446E37F;
            p75.param1 = (float)2.64223E38F;
            p75.param3 = (float)8.146541E37F;
            p75.y = (int) -460211619;
            p75.current = (byte)(byte)56;
            p75.command = (MAV_CMD)MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD;
            p75.target_component = (byte)(byte)48;
            LoopBackDemoChannel.instance.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param1 == (float)1.6380603E38F);
                Debug.Assert(pack.target_system == (byte)(byte)186);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_CONDITION_LAST);
                Debug.Assert(pack.confirmation == (byte)(byte)76);
                Debug.Assert(pack.param5 == (float) -2.284615E38F);
                Debug.Assert(pack.param4 == (float)1.247727E38F);
                Debug.Assert(pack.param7 == (float)2.5371324E37F);
                Debug.Assert(pack.param2 == (float) -1.4460412E38F);
                Debug.Assert(pack.target_component == (byte)(byte)244);
                Debug.Assert(pack.param3 == (float)2.0177562E38F);
                Debug.Assert(pack.param6 == (float)2.1465128E38F);
            };
            DemoDevice.COMMAND_LONG p76 = LoopBackDemoChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.confirmation = (byte)(byte)76;
            p76.param1 = (float)1.6380603E38F;
            p76.target_component = (byte)(byte)244;
            p76.param7 = (float)2.5371324E37F;
            p76.target_system = (byte)(byte)186;
            p76.param2 = (float) -1.4460412E38F;
            p76.param5 = (float) -2.284615E38F;
            p76.param4 = (float)1.247727E38F;
            p76.param3 = (float)2.0177562E38F;
            p76.param6 = (float)2.1465128E38F;
            p76.command = (MAV_CMD)MAV_CMD.MAV_CMD_CONDITION_LAST;
            LoopBackDemoChannel.instance.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.result == (MAV_RESULT)MAV_RESULT.MAV_RESULT_FAILED);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)253);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)255);
                Debug.Assert(pack.result_param2_TRY(ph) == (int) -1478145875);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_REPEAT_RELAY);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)123);
            };
            DemoDevice.COMMAND_ACK p77 = LoopBackDemoChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.target_system_SET((byte)(byte)123, PH) ;
            p77.target_component_SET((byte)(byte)255, PH) ;
            p77.result_param2_SET((int) -1478145875, PH) ;
            p77.progress_SET((byte)(byte)253, PH) ;
            p77.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_REPEAT_RELAY;
            p77.result = (MAV_RESULT)MAV_RESULT.MAV_RESULT_FAILED;
            LoopBackDemoChannel.instance.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)4157495674U);
                Debug.Assert(pack.pitch == (float) -1.8933155E38F);
                Debug.Assert(pack.roll == (float) -1.9146512E38F);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)47);
                Debug.Assert(pack.mode_switch == (byte)(byte)26);
                Debug.Assert(pack.yaw == (float)2.1316608E38F);
                Debug.Assert(pack.thrust == (float)2.096405E38F);
            };
            DemoDevice.MANUAL_SETPOINT p81 = LoopBackDemoChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.yaw = (float)2.1316608E38F;
            p81.thrust = (float)2.096405E38F;
            p81.manual_override_switch = (byte)(byte)47;
            p81.roll = (float) -1.9146512E38F;
            p81.pitch = (float) -1.8933155E38F;
            p81.time_boot_ms = (uint)4157495674U;
            p81.mode_switch = (byte)(byte)26;
            LoopBackDemoChannel.instance.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.3378981E38F, -6.2107096E37F, -4.617557E37F, -3.3015491E38F}));
                Debug.Assert(pack.type_mask == (byte)(byte)104);
                Debug.Assert(pack.target_component == (byte)(byte)135);
                Debug.Assert(pack.time_boot_ms == (uint)4241623487U);
                Debug.Assert(pack.body_yaw_rate == (float)9.126644E37F);
                Debug.Assert(pack.thrust == (float) -2.2725156E38F);
                Debug.Assert(pack.target_system == (byte)(byte)53);
                Debug.Assert(pack.body_roll_rate == (float)1.321971E38F);
                Debug.Assert(pack.body_pitch_rate == (float)6.778767E37F);
            };
            DemoDevice.SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.body_yaw_rate = (float)9.126644E37F;
            p82.target_system = (byte)(byte)53;
            p82.body_roll_rate = (float)1.321971E38F;
            p82.thrust = (float) -2.2725156E38F;
            p82.body_pitch_rate = (float)6.778767E37F;
            p82.time_boot_ms = (uint)4241623487U;
            p82.target_component = (byte)(byte)135;
            p82.q_SET(new float[] {-1.3378981E38F, -6.2107096E37F, -4.617557E37F, -3.3015491E38F}, 0) ;
            p82.type_mask = (byte)(byte)104;
            LoopBackDemoChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type_mask == (byte)(byte)180);
                Debug.Assert(pack.body_yaw_rate == (float)1.5943761E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1273679683U);
                Debug.Assert(pack.body_roll_rate == (float)5.6178786E37F);
                Debug.Assert(pack.body_pitch_rate == (float)3.3299467E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-3.1617707E38F, 2.7882256E37F, -1.743577E38F, -1.7605347E38F}));
                Debug.Assert(pack.thrust == (float)2.4080311E38F);
            };
            DemoDevice.ATTITUDE_TARGET p83 = LoopBackDemoChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.body_roll_rate = (float)5.6178786E37F;
            p83.time_boot_ms = (uint)1273679683U;
            p83.body_yaw_rate = (float)1.5943761E38F;
            p83.body_pitch_rate = (float)3.3299467E38F;
            p83.type_mask = (byte)(byte)180;
            p83.thrust = (float)2.4080311E38F;
            p83.q_SET(new float[] {-3.1617707E38F, 2.7882256E37F, -1.743577E38F, -1.7605347E38F}, 0) ;
            LoopBackDemoChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -2.2255338E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.type_mask == (ushort)(ushort)59115);
                Debug.Assert(pack.target_system == (byte)(byte)118);
                Debug.Assert(pack.target_component == (byte)(byte)227);
                Debug.Assert(pack.afz == (float)2.397687E38F);
                Debug.Assert(pack.vx == (float)2.974304E38F);
                Debug.Assert(pack.vz == (float) -1.3694029E37F);
                Debug.Assert(pack.z == (float)1.8174165E38F);
                Debug.Assert(pack.yaw_rate == (float)1.2255369E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2408573157U);
                Debug.Assert(pack.afy == (float)9.51235E37F);
                Debug.Assert(pack.vy == (float) -2.361457E38F);
                Debug.Assert(pack.x == (float) -2.3682031E38F);
                Debug.Assert(pack.yaw == (float) -1.7794113E38F);
                Debug.Assert(pack.afx == (float) -5.837945E37F);
            };
            DemoDevice.SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.type_mask = (ushort)(ushort)59115;
            p84.target_system = (byte)(byte)118;
            p84.vz = (float) -1.3694029E37F;
            p84.vy = (float) -2.361457E38F;
            p84.afz = (float)2.397687E38F;
            p84.afy = (float)9.51235E37F;
            p84.y = (float) -2.2255338E38F;
            p84.afx = (float) -5.837945E37F;
            p84.yaw_rate = (float)1.2255369E38F;
            p84.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p84.x = (float) -2.3682031E38F;
            p84.target_component = (byte)(byte)227;
            p84.time_boot_ms = (uint)2408573157U;
            p84.yaw = (float) -1.7794113E38F;
            p84.vx = (float)2.974304E38F;
            p84.z = (float)1.8174165E38F;
            LoopBackDemoChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vz == (float)1.6486798E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)51221);
                Debug.Assert(pack.target_component == (byte)(byte)189);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.yaw == (float) -1.7308086E38F);
                Debug.Assert(pack.afy == (float) -4.1749492E37F);
                Debug.Assert(pack.afx == (float) -1.393919E38F);
                Debug.Assert(pack.vy == (float) -3.3487846E38F);
                Debug.Assert(pack.afz == (float) -9.507603E37F);
                Debug.Assert(pack.time_boot_ms == (uint)4112780184U);
                Debug.Assert(pack.yaw_rate == (float)1.4536992E38F);
                Debug.Assert(pack.lon_int == (int)1840709021);
                Debug.Assert(pack.alt == (float)3.0085525E38F);
                Debug.Assert(pack.target_system == (byte)(byte)64);
                Debug.Assert(pack.lat_int == (int) -811164241);
                Debug.Assert(pack.vx == (float)2.8728788E38F);
            };
            DemoDevice.SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.afy = (float) -4.1749492E37F;
            p86.yaw_rate = (float)1.4536992E38F;
            p86.lat_int = (int) -811164241;
            p86.time_boot_ms = (uint)4112780184U;
            p86.vy = (float) -3.3487846E38F;
            p86.lon_int = (int)1840709021;
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            p86.target_component = (byte)(byte)189;
            p86.afx = (float) -1.393919E38F;
            p86.yaw = (float) -1.7308086E38F;
            p86.afz = (float) -9.507603E37F;
            p86.alt = (float)3.0085525E38F;
            p86.target_system = (byte)(byte)64;
            p86.type_mask = (ushort)(ushort)51221;
            p86.vz = (float)1.6486798E38F;
            p86.vx = (float)2.8728788E38F;
            LoopBackDemoChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (float) -1.5551671E38F);
                Debug.Assert(pack.time_boot_ms == (uint)714010331U);
                Debug.Assert(pack.yaw_rate == (float)3.2109983E38F);
                Debug.Assert(pack.afy == (float) -2.6705112E38F);
                Debug.Assert(pack.yaw == (float)2.151334E38F);
                Debug.Assert(pack.lat_int == (int)95286652);
                Debug.Assert(pack.vx == (float)6.083032E37F);
                Debug.Assert(pack.vy == (float)2.4167882E38F);
                Debug.Assert(pack.afx == (float)9.704456E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)38502);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.lon_int == (int)149147086);
                Debug.Assert(pack.vz == (float)3.3959477E38F);
                Debug.Assert(pack.afz == (float) -1.6410032E38F);
            };
            DemoDevice.POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.lat_int = (int)95286652;
            p87.yaw_rate = (float)3.2109983E38F;
            p87.afz = (float) -1.6410032E38F;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            p87.afy = (float) -2.6705112E38F;
            p87.afx = (float)9.704456E37F;
            p87.vx = (float)6.083032E37F;
            p87.vy = (float)2.4167882E38F;
            p87.vz = (float)3.3959477E38F;
            p87.type_mask = (ushort)(ushort)38502;
            p87.lon_int = (int)149147086;
            p87.time_boot_ms = (uint)714010331U;
            p87.alt = (float) -1.5551671E38F;
            p87.yaw = (float)2.151334E38F;
            LoopBackDemoChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)1.2869451E38F);
                Debug.Assert(pack.x == (float) -3.1265458E38F);
                Debug.Assert(pack.yaw == (float)3.5428236E36F);
                Debug.Assert(pack.pitch == (float)2.2374345E38F);
                Debug.Assert(pack.time_boot_ms == (uint)728765485U);
                Debug.Assert(pack.z == (float) -3.1012522E38F);
                Debug.Assert(pack.roll == (float) -8.841811E37F);
            };
            DemoDevice.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.pitch = (float)2.2374345E38F;
            p89.yaw = (float)3.5428236E36F;
            p89.z = (float) -3.1012522E38F;
            p89.roll = (float) -8.841811E37F;
            p89.x = (float) -3.1265458E38F;
            p89.time_boot_ms = (uint)728765485U;
            p89.y = (float)1.2869451E38F;
            LoopBackDemoChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (short)(short)18878);
                Debug.Assert(pack.vz == (short)(short) -3307);
                Debug.Assert(pack.roll == (float) -6.4062006E37F);
                Debug.Assert(pack.lon == (int)8388821);
                Debug.Assert(pack.pitchspeed == (float) -1.6585603E38F);
                Debug.Assert(pack.pitch == (float) -2.0965233E38F);
                Debug.Assert(pack.yacc == (short)(short) -12447);
                Debug.Assert(pack.alt == (int)647837795);
                Debug.Assert(pack.yaw == (float) -2.0570523E38F);
                Debug.Assert(pack.vx == (short)(short) -27270);
                Debug.Assert(pack.time_usec == (ulong)1626802932590939334L);
                Debug.Assert(pack.zacc == (short)(short)7938);
                Debug.Assert(pack.rollspeed == (float)1.9649787E38F);
                Debug.Assert(pack.lat == (int)682327096);
                Debug.Assert(pack.yawspeed == (float)3.3123615E38F);
                Debug.Assert(pack.vy == (short)(short)17271);
            };
            DemoDevice.HIL_STATE p90 = LoopBackDemoChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.vx = (short)(short) -27270;
            p90.vy = (short)(short)17271;
            p90.yawspeed = (float)3.3123615E38F;
            p90.zacc = (short)(short)7938;
            p90.pitch = (float) -2.0965233E38F;
            p90.rollspeed = (float)1.9649787E38F;
            p90.lon = (int)8388821;
            p90.vz = (short)(short) -3307;
            p90.xacc = (short)(short)18878;
            p90.yacc = (short)(short) -12447;
            p90.time_usec = (ulong)1626802932590939334L;
            p90.alt = (int)647837795;
            p90.lat = (int)682327096;
            p90.roll = (float) -6.4062006E37F;
            p90.yaw = (float) -2.0570523E38F;
            p90.pitchspeed = (float) -1.6585603E38F;
            LoopBackDemoChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch_elevator == (float) -1.5743209E38F);
                Debug.Assert(pack.yaw_rudder == (float) -2.0658155E38F);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_MANUAL_DISARMED);
                Debug.Assert(pack.time_usec == (ulong)2464326277751183578L);
                Debug.Assert(pack.nav_mode == (byte)(byte)127);
                Debug.Assert(pack.aux4 == (float) -1.841652E38F);
                Debug.Assert(pack.roll_ailerons == (float) -6.5121503E37F);
                Debug.Assert(pack.aux3 == (float) -2.5420152E37F);
                Debug.Assert(pack.aux2 == (float)9.804994E37F);
                Debug.Assert(pack.aux1 == (float)2.565841E38F);
                Debug.Assert(pack.throttle == (float) -3.425985E37F);
            };
            DemoDevice.HIL_CONTROLS p91 = LoopBackDemoChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.aux3 = (float) -2.5420152E37F;
            p91.roll_ailerons = (float) -6.5121503E37F;
            p91.throttle = (float) -3.425985E37F;
            p91.nav_mode = (byte)(byte)127;
            p91.pitch_elevator = (float) -1.5743209E38F;
            p91.aux2 = (float)9.804994E37F;
            p91.time_usec = (ulong)2464326277751183578L;
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_MANUAL_DISARMED;
            p91.yaw_rudder = (float) -2.0658155E38F;
            p91.aux4 = (float) -1.841652E38F;
            p91.aux1 = (float)2.565841E38F;
            LoopBackDemoChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)18372);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)27841);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)27869);
                Debug.Assert(pack.time_usec == (ulong)8054683714741735496L);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)23452);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)15003);
                Debug.Assert(pack.rssi == (byte)(byte)129);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)31758);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)19926);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)6053);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)24956);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)7298);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)40023);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)34061);
            };
            DemoDevice.HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan2_raw = (ushort)(ushort)19926;
            p92.chan8_raw = (ushort)(ushort)7298;
            p92.time_usec = (ulong)8054683714741735496L;
            p92.chan1_raw = (ushort)(ushort)40023;
            p92.chan9_raw = (ushort)(ushort)27869;
            p92.chan3_raw = (ushort)(ushort)15003;
            p92.chan11_raw = (ushort)(ushort)6053;
            p92.chan7_raw = (ushort)(ushort)27841;
            p92.chan12_raw = (ushort)(ushort)34061;
            p92.chan4_raw = (ushort)(ushort)24956;
            p92.chan10_raw = (ushort)(ushort)18372;
            p92.rssi = (byte)(byte)129;
            p92.chan5_raw = (ushort)(ushort)23452;
            p92.chan6_raw = (ushort)(ushort)31758;
            LoopBackDemoChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.controls.SequenceEqual(new float[] {1.5628337E38F, -5.531069E37F, 1.55937E38F, -3.1229286E38F, 8.32012E37F, 2.8620866E37F, -9.339905E36F, -3.8977183E37F, -2.531163E38F, 2.2256716E38F, -2.3457157E37F, -2.933683E38F, 4.2783365E37F, -7.098057E37F, -2.2172189E38F, 1.638627E38F}));
                Debug.Assert(pack.flags == (ulong)3763491450234572918L);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_MANUAL_DISARMED);
                Debug.Assert(pack.time_usec == (ulong)779998379554166114L);
            };
            DemoDevice.HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_MANUAL_DISARMED;
            p93.controls_SET(new float[] {1.5628337E38F, -5.531069E37F, 1.55937E38F, -3.1229286E38F, 8.32012E37F, 2.8620866E37F, -9.339905E36F, -3.8977183E37F, -2.531163E38F, 2.2256716E38F, -2.3457157E37F, -2.933683E38F, 4.2783365E37F, -7.098057E37F, -2.2172189E38F, 1.638627E38F}, 0) ;
            p93.flags = (ulong)3763491450234572918L;
            p93.time_usec = (ulong)779998379554166114L;
            LoopBackDemoChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float) -2.8682355E38F);
                Debug.Assert(pack.time_usec == (ulong)8378356299126734359L);
                Debug.Assert(pack.flow_comp_m_y == (float) -3.043919E38F);
                Debug.Assert(pack.flow_x == (short)(short)4744);
                Debug.Assert(pack.ground_distance == (float)2.1698878E38F);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float) -2.2694846E38F);
                Debug.Assert(pack.quality == (byte)(byte)40);
                Debug.Assert(pack.flow_y == (short)(short) -28770);
                Debug.Assert(pack.flow_comp_m_x == (float) -2.5528976E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)238);
            };
            DemoDevice.OPTICAL_FLOW p100 = LoopBackDemoChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.flow_comp_m_y = (float) -3.043919E38F;
            p100.flow_x = (short)(short)4744;
            p100.sensor_id = (byte)(byte)238;
            p100.flow_comp_m_x = (float) -2.5528976E38F;
            p100.time_usec = (ulong)8378356299126734359L;
            p100.ground_distance = (float)2.1698878E38F;
            p100.quality = (byte)(byte)40;
            p100.flow_rate_x_SET((float) -2.8682355E38F, PH) ;
            p100.flow_rate_y_SET((float) -2.2694846E38F, PH) ;
            p100.flow_y = (short)(short) -28770;
            LoopBackDemoChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float) -4.3977553E37F);
                Debug.Assert(pack.usec == (ulong)3838728097738163806L);
                Debug.Assert(pack.yaw == (float) -1.719163E38F);
                Debug.Assert(pack.x == (float)3.7319084E37F);
                Debug.Assert(pack.z == (float)2.8564405E38F);
                Debug.Assert(pack.pitch == (float)2.872279E38F);
                Debug.Assert(pack.y == (float) -4.8920954E37F);
            };
            DemoDevice.GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.yaw = (float) -1.719163E38F;
            p101.y = (float) -4.8920954E37F;
            p101.x = (float)3.7319084E37F;
            p101.roll = (float) -4.3977553E37F;
            p101.pitch = (float)2.872279E38F;
            p101.z = (float)2.8564405E38F;
            p101.usec = (ulong)3838728097738163806L;
            LoopBackDemoChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)3.1265026E38F);
                Debug.Assert(pack.roll == (float) -4.5956685E37F);
                Debug.Assert(pack.x == (float)9.313341E37F);
                Debug.Assert(pack.y == (float)3.2563751E38F);
                Debug.Assert(pack.pitch == (float)1.5563618E38F);
                Debug.Assert(pack.usec == (ulong)6975762197389745705L);
                Debug.Assert(pack.z == (float)9.622043E37F);
            };
            DemoDevice.VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.x = (float)9.313341E37F;
            p102.z = (float)9.622043E37F;
            p102.pitch = (float)1.5563618E38F;
            p102.yaw = (float)3.1265026E38F;
            p102.usec = (ulong)6975762197389745705L;
            p102.y = (float)3.2563751E38F;
            p102.roll = (float) -4.5956685E37F;
            LoopBackDemoChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.usec == (ulong)6129836010886920237L);
                Debug.Assert(pack.x == (float) -2.481914E38F);
                Debug.Assert(pack.y == (float)4.7925544E37F);
                Debug.Assert(pack.z == (float)4.7588466E37F);
            };
            DemoDevice.VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.z = (float)4.7588466E37F;
            p103.usec = (ulong)6129836010886920237L;
            p103.x = (float) -2.481914E38F;
            p103.y = (float)4.7925544E37F;
            LoopBackDemoChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)3.8095796E37F);
                Debug.Assert(pack.y == (float)2.9276504E38F);
                Debug.Assert(pack.pitch == (float) -3.7846434E36F);
                Debug.Assert(pack.x == (float) -1.6965057E38F);
                Debug.Assert(pack.roll == (float) -2.7017733E38F);
                Debug.Assert(pack.yaw == (float)1.4074393E38F);
                Debug.Assert(pack.usec == (ulong)4427458892850049478L);
            };
            DemoDevice.VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.usec = (ulong)4427458892850049478L;
            p104.roll = (float) -2.7017733E38F;
            p104.x = (float) -1.6965057E38F;
            p104.pitch = (float) -3.7846434E36F;
            p104.y = (float)2.9276504E38F;
            p104.z = (float)3.8095796E37F;
            p104.yaw = (float)1.4074393E38F;
            LoopBackDemoChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fields_updated == (ushort)(ushort)15590);
                Debug.Assert(pack.time_usec == (ulong)7342466664769503016L);
                Debug.Assert(pack.ymag == (float) -3.5613908E37F);
                Debug.Assert(pack.xacc == (float)3.2347458E38F);
                Debug.Assert(pack.abs_pressure == (float)1.5785488E38F);
                Debug.Assert(pack.xmag == (float) -3.1683182E38F);
                Debug.Assert(pack.zacc == (float) -3.3284763E38F);
                Debug.Assert(pack.ygyro == (float) -2.8718044E38F);
                Debug.Assert(pack.pressure_alt == (float)1.8605111E38F);
                Debug.Assert(pack.zgyro == (float) -1.4793879E38F);
                Debug.Assert(pack.xgyro == (float) -3.0712994E38F);
                Debug.Assert(pack.diff_pressure == (float)2.4612177E38F);
                Debug.Assert(pack.yacc == (float) -1.1712258E38F);
                Debug.Assert(pack.zmag == (float)1.2512563E37F);
                Debug.Assert(pack.temperature == (float)8.659894E37F);
            };
            DemoDevice.HIGHRES_IMU p105 = LoopBackDemoChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.zmag = (float)1.2512563E37F;
            p105.yacc = (float) -1.1712258E38F;
            p105.ymag = (float) -3.5613908E37F;
            p105.xacc = (float)3.2347458E38F;
            p105.time_usec = (ulong)7342466664769503016L;
            p105.xgyro = (float) -3.0712994E38F;
            p105.fields_updated = (ushort)(ushort)15590;
            p105.pressure_alt = (float)1.8605111E38F;
            p105.xmag = (float) -3.1683182E38F;
            p105.ygyro = (float) -2.8718044E38F;
            p105.abs_pressure = (float)1.5785488E38F;
            p105.zacc = (float) -3.3284763E38F;
            p105.diff_pressure = (float)2.4612177E38F;
            p105.zgyro = (float) -1.4793879E38F;
            p105.temperature = (float)8.659894E37F;
            LoopBackDemoChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integration_time_us == (uint)1208005154U);
                Debug.Assert(pack.time_usec == (ulong)790685331022866010L);
                Debug.Assert(pack.integrated_ygyro == (float) -2.6216692E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)4199373868U);
                Debug.Assert(pack.quality == (byte)(byte)227);
                Debug.Assert(pack.temperature == (short)(short) -31471);
                Debug.Assert(pack.integrated_y == (float) -1.8143056E38F);
                Debug.Assert(pack.integrated_xgyro == (float) -1.1575539E38F);
                Debug.Assert(pack.integrated_x == (float) -2.5118967E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)36);
                Debug.Assert(pack.distance == (float)8.161241E37F);
                Debug.Assert(pack.integrated_zgyro == (float) -1.5811817E38F);
            };
            DemoDevice.OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.temperature = (short)(short) -31471;
            p106.sensor_id = (byte)(byte)36;
            p106.distance = (float)8.161241E37F;
            p106.quality = (byte)(byte)227;
            p106.integration_time_us = (uint)1208005154U;
            p106.time_usec = (ulong)790685331022866010L;
            p106.integrated_ygyro = (float) -2.6216692E38F;
            p106.integrated_xgyro = (float) -1.1575539E38F;
            p106.time_delta_distance_us = (uint)4199373868U;
            p106.integrated_y = (float) -1.8143056E38F;
            p106.integrated_zgyro = (float) -1.5811817E38F;
            p106.integrated_x = (float) -2.5118967E38F;
            LoopBackDemoChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (float) -3.2416978E38F);
                Debug.Assert(pack.time_usec == (ulong)5157215469350987890L);
                Debug.Assert(pack.pressure_alt == (float)1.8459905E38F);
                Debug.Assert(pack.ymag == (float)2.4580024E37F);
                Debug.Assert(pack.abs_pressure == (float)8.101928E36F);
                Debug.Assert(pack.temperature == (float)1.5431314E37F);
                Debug.Assert(pack.xmag == (float)1.0354076E38F);
                Debug.Assert(pack.zmag == (float) -2.4418177E38F);
                Debug.Assert(pack.ygyro == (float) -1.8354062E38F);
                Debug.Assert(pack.yacc == (float)1.849909E38F);
                Debug.Assert(pack.fields_updated == (uint)61092962U);
                Debug.Assert(pack.zgyro == (float) -1.5802778E38F);
                Debug.Assert(pack.diff_pressure == (float)2.366711E38F);
                Debug.Assert(pack.zacc == (float) -2.8377535E38F);
                Debug.Assert(pack.xgyro == (float) -1.8676844E38F);
            };
            DemoDevice.HIL_SENSOR p107 = LoopBackDemoChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.xacc = (float) -3.2416978E38F;
            p107.ymag = (float)2.4580024E37F;
            p107.abs_pressure = (float)8.101928E36F;
            p107.diff_pressure = (float)2.366711E38F;
            p107.xgyro = (float) -1.8676844E38F;
            p107.zacc = (float) -2.8377535E38F;
            p107.temperature = (float)1.5431314E37F;
            p107.fields_updated = (uint)61092962U;
            p107.zmag = (float) -2.4418177E38F;
            p107.zgyro = (float) -1.5802778E38F;
            p107.yacc = (float)1.849909E38F;
            p107.time_usec = (ulong)5157215469350987890L;
            p107.pressure_alt = (float)1.8459905E38F;
            p107.ygyro = (float) -1.8354062E38F;
            p107.xmag = (float)1.0354076E38F;
            LoopBackDemoChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (float)2.5637534E38F);
                Debug.Assert(pack.std_dev_vert == (float) -2.598915E38F);
                Debug.Assert(pack.lon == (float) -1.2809693E38F);
                Debug.Assert(pack.ve == (float)3.3265955E38F);
                Debug.Assert(pack.xgyro == (float)1.7968467E38F);
                Debug.Assert(pack.alt == (float)2.7322706E38F);
                Debug.Assert(pack.pitch == (float)1.0734313E38F);
                Debug.Assert(pack.q1 == (float) -1.5163217E38F);
                Debug.Assert(pack.roll == (float)1.9579E38F);
                Debug.Assert(pack.zgyro == (float)2.8831709E38F);
                Debug.Assert(pack.q2 == (float)1.8961207E38F);
                Debug.Assert(pack.vn == (float)2.9819213E38F);
                Debug.Assert(pack.vd == (float)2.160256E38F);
                Debug.Assert(pack.q4 == (float) -1.7741977E38F);
                Debug.Assert(pack.ygyro == (float)3.2805445E38F);
                Debug.Assert(pack.zacc == (float) -5.690783E36F);
                Debug.Assert(pack.yaw == (float)8.4684917E36F);
                Debug.Assert(pack.std_dev_horz == (float) -1.0070271E38F);
                Debug.Assert(pack.lat == (float)5.78012E37F);
                Debug.Assert(pack.yacc == (float) -1.8558662E38F);
                Debug.Assert(pack.q3 == (float)3.0640855E38F);
            };
            DemoDevice.SIM_STATE p108 = LoopBackDemoChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q3 = (float)3.0640855E38F;
            p108.std_dev_vert = (float) -2.598915E38F;
            p108.lat = (float)5.78012E37F;
            p108.lon = (float) -1.2809693E38F;
            p108.std_dev_horz = (float) -1.0070271E38F;
            p108.roll = (float)1.9579E38F;
            p108.xgyro = (float)1.7968467E38F;
            p108.pitch = (float)1.0734313E38F;
            p108.yaw = (float)8.4684917E36F;
            p108.alt = (float)2.7322706E38F;
            p108.zgyro = (float)2.8831709E38F;
            p108.ve = (float)3.3265955E38F;
            p108.zacc = (float) -5.690783E36F;
            p108.q2 = (float)1.8961207E38F;
            p108.xacc = (float)2.5637534E38F;
            p108.q4 = (float) -1.7741977E38F;
            p108.yacc = (float) -1.8558662E38F;
            p108.vn = (float)2.9819213E38F;
            p108.vd = (float)2.160256E38F;
            p108.ygyro = (float)3.2805445E38F;
            p108.q1 = (float) -1.5163217E38F;
            LoopBackDemoChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.noise == (byte)(byte)15);
                Debug.Assert(pack.txbuf == (byte)(byte)14);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)26637);
                Debug.Assert(pack.remnoise == (byte)(byte)157);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)18142);
                Debug.Assert(pack.remrssi == (byte)(byte)230);
                Debug.Assert(pack.rssi == (byte)(byte)49);
            };
            DemoDevice.RADIO_STATUS p109 = LoopBackDemoChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.remrssi = (byte)(byte)230;
            p109.fixed_ = (ushort)(ushort)26637;
            p109.noise = (byte)(byte)15;
            p109.rxerrors = (ushort)(ushort)18142;
            p109.rssi = (byte)(byte)49;
            p109.remnoise = (byte)(byte)157;
            p109.txbuf = (byte)(byte)14;
            LoopBackDemoChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)235);
                Debug.Assert(pack.target_component == (byte)(byte)4);
                Debug.Assert(pack.target_network == (byte)(byte)45);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)8, (byte)188, (byte)29, (byte)166, (byte)38, (byte)88, (byte)99, (byte)183, (byte)108, (byte)212, (byte)175, (byte)154, (byte)208, (byte)129, (byte)111, (byte)36, (byte)195, (byte)88, (byte)118, (byte)160, (byte)103, (byte)31, (byte)72, (byte)144, (byte)113, (byte)87, (byte)155, (byte)58, (byte)211, (byte)174, (byte)112, (byte)203, (byte)167, (byte)213, (byte)179, (byte)237, (byte)23, (byte)53, (byte)69, (byte)158, (byte)47, (byte)194, (byte)59, (byte)171, (byte)251, (byte)26, (byte)226, (byte)74, (byte)29, (byte)199, (byte)45, (byte)93, (byte)123, (byte)200, (byte)7, (byte)86, (byte)127, (byte)137, (byte)217, (byte)93, (byte)222, (byte)62, (byte)101, (byte)103, (byte)205, (byte)154, (byte)86, (byte)85, (byte)94, (byte)224, (byte)11, (byte)238, (byte)94, (byte)161, (byte)108, (byte)108, (byte)211, (byte)35, (byte)227, (byte)149, (byte)179, (byte)163, (byte)144, (byte)116, (byte)132, (byte)243, (byte)220, (byte)221, (byte)220, (byte)33, (byte)46, (byte)49, (byte)10, (byte)122, (byte)222, (byte)89, (byte)5, (byte)61, (byte)218, (byte)189, (byte)134, (byte)233, (byte)27, (byte)210, (byte)132, (byte)222, (byte)146, (byte)121, (byte)158, (byte)139, (byte)236, (byte)170, (byte)32, (byte)97, (byte)76, (byte)120, (byte)24, (byte)74, (byte)42, (byte)15, (byte)213, (byte)116, (byte)179, (byte)202, (byte)80, (byte)119, (byte)64, (byte)163, (byte)245, (byte)178, (byte)237, (byte)112, (byte)8, (byte)102, (byte)152, (byte)66, (byte)242, (byte)141, (byte)136, (byte)254, (byte)228, (byte)100, (byte)33, (byte)72, (byte)205, (byte)11, (byte)21, (byte)190, (byte)174, (byte)26, (byte)37, (byte)154, (byte)50, (byte)103, (byte)229, (byte)240, (byte)73, (byte)219, (byte)208, (byte)210, (byte)99, (byte)246, (byte)82, (byte)141, (byte)50, (byte)247, (byte)247, (byte)157, (byte)15, (byte)240, (byte)140, (byte)212, (byte)253, (byte)19, (byte)119, (byte)136, (byte)171, (byte)69, (byte)120, (byte)240, (byte)17, (byte)0, (byte)94, (byte)230, (byte)13, (byte)144, (byte)196, (byte)40, (byte)179, (byte)254, (byte)6, (byte)117, (byte)172, (byte)150, (byte)204, (byte)131, (byte)247, (byte)136, (byte)242, (byte)138, (byte)255, (byte)123, (byte)73, (byte)39, (byte)58, (byte)19, (byte)15, (byte)0, (byte)219, (byte)207, (byte)36, (byte)100, (byte)216, (byte)60, (byte)183, (byte)83, (byte)187, (byte)235, (byte)157, (byte)155, (byte)16, (byte)77, (byte)12, (byte)83, (byte)177, (byte)194, (byte)151, (byte)122, (byte)165, (byte)64, (byte)146, (byte)181, (byte)92, (byte)37, (byte)176, (byte)57, (byte)198, (byte)83, (byte)111, (byte)4, (byte)74, (byte)189, (byte)13, (byte)135, (byte)102, (byte)30, (byte)79, (byte)225, (byte)220, (byte)233, (byte)115}));
            };
            DemoDevice.FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)45;
            p110.payload_SET(new byte[] {(byte)8, (byte)188, (byte)29, (byte)166, (byte)38, (byte)88, (byte)99, (byte)183, (byte)108, (byte)212, (byte)175, (byte)154, (byte)208, (byte)129, (byte)111, (byte)36, (byte)195, (byte)88, (byte)118, (byte)160, (byte)103, (byte)31, (byte)72, (byte)144, (byte)113, (byte)87, (byte)155, (byte)58, (byte)211, (byte)174, (byte)112, (byte)203, (byte)167, (byte)213, (byte)179, (byte)237, (byte)23, (byte)53, (byte)69, (byte)158, (byte)47, (byte)194, (byte)59, (byte)171, (byte)251, (byte)26, (byte)226, (byte)74, (byte)29, (byte)199, (byte)45, (byte)93, (byte)123, (byte)200, (byte)7, (byte)86, (byte)127, (byte)137, (byte)217, (byte)93, (byte)222, (byte)62, (byte)101, (byte)103, (byte)205, (byte)154, (byte)86, (byte)85, (byte)94, (byte)224, (byte)11, (byte)238, (byte)94, (byte)161, (byte)108, (byte)108, (byte)211, (byte)35, (byte)227, (byte)149, (byte)179, (byte)163, (byte)144, (byte)116, (byte)132, (byte)243, (byte)220, (byte)221, (byte)220, (byte)33, (byte)46, (byte)49, (byte)10, (byte)122, (byte)222, (byte)89, (byte)5, (byte)61, (byte)218, (byte)189, (byte)134, (byte)233, (byte)27, (byte)210, (byte)132, (byte)222, (byte)146, (byte)121, (byte)158, (byte)139, (byte)236, (byte)170, (byte)32, (byte)97, (byte)76, (byte)120, (byte)24, (byte)74, (byte)42, (byte)15, (byte)213, (byte)116, (byte)179, (byte)202, (byte)80, (byte)119, (byte)64, (byte)163, (byte)245, (byte)178, (byte)237, (byte)112, (byte)8, (byte)102, (byte)152, (byte)66, (byte)242, (byte)141, (byte)136, (byte)254, (byte)228, (byte)100, (byte)33, (byte)72, (byte)205, (byte)11, (byte)21, (byte)190, (byte)174, (byte)26, (byte)37, (byte)154, (byte)50, (byte)103, (byte)229, (byte)240, (byte)73, (byte)219, (byte)208, (byte)210, (byte)99, (byte)246, (byte)82, (byte)141, (byte)50, (byte)247, (byte)247, (byte)157, (byte)15, (byte)240, (byte)140, (byte)212, (byte)253, (byte)19, (byte)119, (byte)136, (byte)171, (byte)69, (byte)120, (byte)240, (byte)17, (byte)0, (byte)94, (byte)230, (byte)13, (byte)144, (byte)196, (byte)40, (byte)179, (byte)254, (byte)6, (byte)117, (byte)172, (byte)150, (byte)204, (byte)131, (byte)247, (byte)136, (byte)242, (byte)138, (byte)255, (byte)123, (byte)73, (byte)39, (byte)58, (byte)19, (byte)15, (byte)0, (byte)219, (byte)207, (byte)36, (byte)100, (byte)216, (byte)60, (byte)183, (byte)83, (byte)187, (byte)235, (byte)157, (byte)155, (byte)16, (byte)77, (byte)12, (byte)83, (byte)177, (byte)194, (byte)151, (byte)122, (byte)165, (byte)64, (byte)146, (byte)181, (byte)92, (byte)37, (byte)176, (byte)57, (byte)198, (byte)83, (byte)111, (byte)4, (byte)74, (byte)189, (byte)13, (byte)135, (byte)102, (byte)30, (byte)79, (byte)225, (byte)220, (byte)233, (byte)115}, 0) ;
            p110.target_component = (byte)(byte)4;
            p110.target_system = (byte)(byte)235;
            LoopBackDemoChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ts1 == (long)7585205769243105853L);
                Debug.Assert(pack.tc1 == (long)4642414507837391999L);
            };
            DemoDevice.TIMESYNC p111 = LoopBackDemoChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long)4642414507837391999L;
            p111.ts1 = (long)7585205769243105853L;
            LoopBackDemoChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (uint)900581352U);
                Debug.Assert(pack.time_usec == (ulong)184670180764067339L);
            };
            DemoDevice.CAMERA_TRIGGER p112 = LoopBackDemoChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.seq = (uint)900581352U;
            p112.time_usec = (ulong)184670180764067339L;
            LoopBackDemoChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.epv == (ushort)(ushort)5148);
                Debug.Assert(pack.vd == (short)(short) -16770);
                Debug.Assert(pack.vel == (ushort)(ushort)14803);
                Debug.Assert(pack.vn == (short)(short)4212);
                Debug.Assert(pack.fix_type == (byte)(byte)204);
                Debug.Assert(pack.alt == (int)56777913);
                Debug.Assert(pack.ve == (short)(short) -4145);
                Debug.Assert(pack.lat == (int) -474780922);
                Debug.Assert(pack.time_usec == (ulong)1843678409301541919L);
                Debug.Assert(pack.lon == (int)1441414969);
                Debug.Assert(pack.eph == (ushort)(ushort)52858);
                Debug.Assert(pack.cog == (ushort)(ushort)49245);
                Debug.Assert(pack.satellites_visible == (byte)(byte)70);
            };
            DemoDevice.HIL_GPS p113 = LoopBackDemoChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.vd = (short)(short) -16770;
            p113.fix_type = (byte)(byte)204;
            p113.time_usec = (ulong)1843678409301541919L;
            p113.satellites_visible = (byte)(byte)70;
            p113.lon = (int)1441414969;
            p113.eph = (ushort)(ushort)52858;
            p113.cog = (ushort)(ushort)49245;
            p113.vel = (ushort)(ushort)14803;
            p113.vn = (short)(short)4212;
            p113.ve = (short)(short) -4145;
            p113.lat = (int) -474780922;
            p113.epv = (ushort)(ushort)5148;
            p113.alt = (int)56777913;
            LoopBackDemoChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_xgyro == (float)1.3242342E38F);
                Debug.Assert(pack.integrated_x == (float) -9.21402E37F);
                Debug.Assert(pack.temperature == (short)(short)21220);
                Debug.Assert(pack.integrated_zgyro == (float)2.0613652E38F);
                Debug.Assert(pack.integration_time_us == (uint)21740855U);
                Debug.Assert(pack.time_delta_distance_us == (uint)700182707U);
                Debug.Assert(pack.integrated_y == (float) -3.369624E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)222);
                Debug.Assert(pack.quality == (byte)(byte)0);
                Debug.Assert(pack.integrated_ygyro == (float)1.9937534E38F);
                Debug.Assert(pack.distance == (float) -2.0826682E38F);
                Debug.Assert(pack.time_usec == (ulong)8624205925830117429L);
            };
            DemoDevice.HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.integration_time_us = (uint)21740855U;
            p114.integrated_ygyro = (float)1.9937534E38F;
            p114.quality = (byte)(byte)0;
            p114.integrated_xgyro = (float)1.3242342E38F;
            p114.time_usec = (ulong)8624205925830117429L;
            p114.integrated_y = (float) -3.369624E38F;
            p114.integrated_x = (float) -9.21402E37F;
            p114.time_delta_distance_us = (uint)700182707U;
            p114.integrated_zgyro = (float)2.0613652E38F;
            p114.temperature = (short)(short)21220;
            p114.distance = (float) -2.0826682E38F;
            p114.sensor_id = (byte)(byte)222;
            LoopBackDemoChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zacc == (short)(short) -11740);
                Debug.Assert(pack.lon == (int) -542075324);
                Debug.Assert(pack.alt == (int) -1417204124);
                Debug.Assert(pack.lat == (int) -26368492);
                Debug.Assert(pack.rollspeed == (float)1.5027665E38F);
                Debug.Assert(pack.time_usec == (ulong)7087872096054669277L);
                Debug.Assert(pack.yacc == (short)(short) -21068);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)38291);
                Debug.Assert(pack.vy == (short)(short)31920);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {4.6119934E36F, 2.0170647E38F, -3.0312501E38F, 1.5755664E38F}));
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)43643);
                Debug.Assert(pack.vz == (short)(short)6635);
                Debug.Assert(pack.xacc == (short)(short) -30489);
                Debug.Assert(pack.pitchspeed == (float) -1.91386E38F);
                Debug.Assert(pack.yawspeed == (float) -2.9278192E38F);
                Debug.Assert(pack.vx == (short)(short) -22821);
            };
            DemoDevice.HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.lat = (int) -26368492;
            p115.attitude_quaternion_SET(new float[] {4.6119934E36F, 2.0170647E38F, -3.0312501E38F, 1.5755664E38F}, 0) ;
            p115.vy = (short)(short)31920;
            p115.lon = (int) -542075324;
            p115.xacc = (short)(short) -30489;
            p115.alt = (int) -1417204124;
            p115.yacc = (short)(short) -21068;
            p115.rollspeed = (float)1.5027665E38F;
            p115.time_usec = (ulong)7087872096054669277L;
            p115.true_airspeed = (ushort)(ushort)43643;
            p115.vz = (short)(short)6635;
            p115.vx = (short)(short) -22821;
            p115.pitchspeed = (float) -1.91386E38F;
            p115.yawspeed = (float) -2.9278192E38F;
            p115.zacc = (short)(short) -11740;
            p115.ind_airspeed = (ushort)(ushort)38291;
            LoopBackDemoChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xmag == (short)(short)3096);
                Debug.Assert(pack.zgyro == (short)(short)28884);
                Debug.Assert(pack.ygyro == (short)(short)23008);
                Debug.Assert(pack.zacc == (short)(short)11274);
                Debug.Assert(pack.zmag == (short)(short) -6172);
                Debug.Assert(pack.time_boot_ms == (uint)2985589800U);
                Debug.Assert(pack.xacc == (short)(short)19213);
                Debug.Assert(pack.ymag == (short)(short)22989);
                Debug.Assert(pack.yacc == (short)(short)1075);
                Debug.Assert(pack.xgyro == (short)(short)30524);
            };
            DemoDevice.SCALED_IMU2 p116 = LoopBackDemoChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.xacc = (short)(short)19213;
            p116.zacc = (short)(short)11274;
            p116.xgyro = (short)(short)30524;
            p116.ygyro = (short)(short)23008;
            p116.time_boot_ms = (uint)2985589800U;
            p116.zmag = (short)(short) -6172;
            p116.zgyro = (short)(short)28884;
            p116.yacc = (short)(short)1075;
            p116.xmag = (short)(short)3096;
            p116.ymag = (short)(short)22989;
            LoopBackDemoChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)142);
                Debug.Assert(pack.target_component == (byte)(byte)195);
                Debug.Assert(pack.end == (ushort)(ushort)45390);
                Debug.Assert(pack.start == (ushort)(ushort)8374);
            };
            DemoDevice.LOG_REQUEST_LIST p117 = LoopBackDemoChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.start = (ushort)(ushort)8374;
            p117.target_component = (byte)(byte)195;
            p117.end = (ushort)(ushort)45390;
            p117.target_system = (byte)(byte)142;
            LoopBackDemoChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.num_logs == (ushort)(ushort)7114);
                Debug.Assert(pack.time_utc == (uint)3021148782U);
                Debug.Assert(pack.id == (ushort)(ushort)42311);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)8582);
                Debug.Assert(pack.size == (uint)1704754133U);
            };
            DemoDevice.LOG_ENTRY p118 = LoopBackDemoChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.last_log_num = (ushort)(ushort)8582;
            p118.size = (uint)1704754133U;
            p118.num_logs = (ushort)(ushort)7114;
            p118.time_utc = (uint)3021148782U;
            p118.id = (ushort)(ushort)42311;
            LoopBackDemoChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)38);
                Debug.Assert(pack.id == (ushort)(ushort)2377);
                Debug.Assert(pack.ofs == (uint)2891761153U);
                Debug.Assert(pack.count == (uint)372410999U);
                Debug.Assert(pack.target_system == (byte)(byte)130);
            };
            DemoDevice.LOG_REQUEST_DATA p119 = LoopBackDemoChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_component = (byte)(byte)38;
            p119.id = (ushort)(ushort)2377;
            p119.count = (uint)372410999U;
            p119.target_system = (byte)(byte)130;
            p119.ofs = (uint)2891761153U;
            LoopBackDemoChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)154, (byte)62, (byte)228, (byte)155, (byte)131, (byte)40, (byte)106, (byte)66, (byte)207, (byte)78, (byte)59, (byte)14, (byte)207, (byte)35, (byte)233, (byte)57, (byte)227, (byte)163, (byte)189, (byte)90, (byte)96, (byte)30, (byte)37, (byte)199, (byte)67, (byte)28, (byte)78, (byte)153, (byte)229, (byte)143, (byte)250, (byte)30, (byte)235, (byte)204, (byte)127, (byte)81, (byte)242, (byte)85, (byte)61, (byte)126, (byte)40, (byte)228, (byte)191, (byte)161, (byte)138, (byte)61, (byte)128, (byte)0, (byte)230, (byte)141, (byte)37, (byte)250, (byte)42, (byte)5, (byte)194, (byte)12, (byte)227, (byte)135, (byte)74, (byte)217, (byte)183, (byte)37, (byte)69, (byte)142, (byte)51, (byte)180, (byte)23, (byte)157, (byte)15, (byte)230, (byte)230, (byte)150, (byte)155, (byte)246, (byte)167, (byte)90, (byte)219, (byte)116, (byte)237, (byte)128, (byte)49, (byte)18, (byte)160, (byte)110, (byte)241, (byte)17, (byte)195, (byte)129, (byte)113, (byte)248}));
                Debug.Assert(pack.ofs == (uint)3368092449U);
                Debug.Assert(pack.id == (ushort)(ushort)53543);
                Debug.Assert(pack.count == (byte)(byte)1);
            };
            DemoDevice.LOG_DATA p120 = LoopBackDemoChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.count = (byte)(byte)1;
            p120.data__SET(new byte[] {(byte)154, (byte)62, (byte)228, (byte)155, (byte)131, (byte)40, (byte)106, (byte)66, (byte)207, (byte)78, (byte)59, (byte)14, (byte)207, (byte)35, (byte)233, (byte)57, (byte)227, (byte)163, (byte)189, (byte)90, (byte)96, (byte)30, (byte)37, (byte)199, (byte)67, (byte)28, (byte)78, (byte)153, (byte)229, (byte)143, (byte)250, (byte)30, (byte)235, (byte)204, (byte)127, (byte)81, (byte)242, (byte)85, (byte)61, (byte)126, (byte)40, (byte)228, (byte)191, (byte)161, (byte)138, (byte)61, (byte)128, (byte)0, (byte)230, (byte)141, (byte)37, (byte)250, (byte)42, (byte)5, (byte)194, (byte)12, (byte)227, (byte)135, (byte)74, (byte)217, (byte)183, (byte)37, (byte)69, (byte)142, (byte)51, (byte)180, (byte)23, (byte)157, (byte)15, (byte)230, (byte)230, (byte)150, (byte)155, (byte)246, (byte)167, (byte)90, (byte)219, (byte)116, (byte)237, (byte)128, (byte)49, (byte)18, (byte)160, (byte)110, (byte)241, (byte)17, (byte)195, (byte)129, (byte)113, (byte)248}, 0) ;
            p120.id = (ushort)(ushort)53543;
            p120.ofs = (uint)3368092449U;
            LoopBackDemoChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)105);
                Debug.Assert(pack.target_system == (byte)(byte)155);
            };
            DemoDevice.LOG_ERASE p121 = LoopBackDemoChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_component = (byte)(byte)105;
            p121.target_system = (byte)(byte)155;
            LoopBackDemoChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)170);
                Debug.Assert(pack.target_system == (byte)(byte)212);
            };
            DemoDevice.LOG_REQUEST_END p122 = LoopBackDemoChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)212;
            p122.target_component = (byte)(byte)170;
            LoopBackDemoChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)95);
                Debug.Assert(pack.len == (byte)(byte)27);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)255, (byte)165, (byte)169, (byte)21, (byte)66, (byte)214, (byte)61, (byte)1, (byte)229, (byte)11, (byte)225, (byte)20, (byte)179, (byte)157, (byte)125, (byte)60, (byte)225, (byte)237, (byte)88, (byte)103, (byte)237, (byte)220, (byte)199, (byte)244, (byte)23, (byte)80, (byte)43, (byte)125, (byte)234, (byte)58, (byte)54, (byte)72, (byte)124, (byte)57, (byte)140, (byte)83, (byte)102, (byte)161, (byte)165, (byte)127, (byte)135, (byte)87, (byte)251, (byte)252, (byte)233, (byte)57, (byte)62, (byte)243, (byte)35, (byte)160, (byte)240, (byte)106, (byte)31, (byte)188, (byte)105, (byte)191, (byte)81, (byte)192, (byte)244, (byte)127, (byte)151, (byte)91, (byte)132, (byte)7, (byte)218, (byte)87, (byte)140, (byte)41, (byte)57, (byte)201, (byte)86, (byte)139, (byte)129, (byte)155, (byte)127, (byte)52, (byte)16, (byte)5, (byte)37, (byte)115, (byte)226, (byte)114, (byte)255, (byte)148, (byte)152, (byte)48, (byte)206, (byte)149, (byte)109, (byte)9, (byte)182, (byte)216, (byte)75, (byte)22, (byte)224, (byte)96, (byte)123, (byte)241, (byte)15, (byte)57, (byte)125, (byte)18, (byte)231, (byte)137, (byte)222, (byte)112, (byte)186, (byte)35, (byte)189, (byte)199}));
                Debug.Assert(pack.target_component == (byte)(byte)216);
            };
            DemoDevice.GPS_INJECT_DATA p123 = LoopBackDemoChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.len = (byte)(byte)27;
            p123.target_system = (byte)(byte)95;
            p123.target_component = (byte)(byte)216;
            p123.data__SET(new byte[] {(byte)255, (byte)165, (byte)169, (byte)21, (byte)66, (byte)214, (byte)61, (byte)1, (byte)229, (byte)11, (byte)225, (byte)20, (byte)179, (byte)157, (byte)125, (byte)60, (byte)225, (byte)237, (byte)88, (byte)103, (byte)237, (byte)220, (byte)199, (byte)244, (byte)23, (byte)80, (byte)43, (byte)125, (byte)234, (byte)58, (byte)54, (byte)72, (byte)124, (byte)57, (byte)140, (byte)83, (byte)102, (byte)161, (byte)165, (byte)127, (byte)135, (byte)87, (byte)251, (byte)252, (byte)233, (byte)57, (byte)62, (byte)243, (byte)35, (byte)160, (byte)240, (byte)106, (byte)31, (byte)188, (byte)105, (byte)191, (byte)81, (byte)192, (byte)244, (byte)127, (byte)151, (byte)91, (byte)132, (byte)7, (byte)218, (byte)87, (byte)140, (byte)41, (byte)57, (byte)201, (byte)86, (byte)139, (byte)129, (byte)155, (byte)127, (byte)52, (byte)16, (byte)5, (byte)37, (byte)115, (byte)226, (byte)114, (byte)255, (byte)148, (byte)152, (byte)48, (byte)206, (byte)149, (byte)109, (byte)9, (byte)182, (byte)216, (byte)75, (byte)22, (byte)224, (byte)96, (byte)123, (byte)241, (byte)15, (byte)57, (byte)125, (byte)18, (byte)231, (byte)137, (byte)222, (byte)112, (byte)186, (byte)35, (byte)189, (byte)199}, 0) ;
            LoopBackDemoChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
                Debug.Assert(pack.eph == (ushort)(ushort)8331);
                Debug.Assert(pack.dgps_age == (uint)1623955092U);
                Debug.Assert(pack.dgps_numch == (byte)(byte)79);
                Debug.Assert(pack.satellites_visible == (byte)(byte)198);
                Debug.Assert(pack.vel == (ushort)(ushort)32203);
                Debug.Assert(pack.time_usec == (ulong)3902433599966325636L);
                Debug.Assert(pack.cog == (ushort)(ushort)30969);
                Debug.Assert(pack.lon == (int) -1145134288);
                Debug.Assert(pack.epv == (ushort)(ushort)30941);
                Debug.Assert(pack.lat == (int) -537119043);
                Debug.Assert(pack.alt == (int)477552403);
            };
            DemoDevice.GPS2_RAW p124 = LoopBackDemoChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.lon = (int) -1145134288;
            p124.satellites_visible = (byte)(byte)198;
            p124.alt = (int)477552403;
            p124.dgps_age = (uint)1623955092U;
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p124.time_usec = (ulong)3902433599966325636L;
            p124.vel = (ushort)(ushort)32203;
            p124.epv = (ushort)(ushort)30941;
            p124.cog = (ushort)(ushort)30969;
            p124.lat = (int) -537119043;
            p124.eph = (ushort)(ushort)8331;
            p124.dgps_numch = (byte)(byte)79;
            LoopBackDemoChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Vservo == (ushort)(ushort)47120);
                Debug.Assert(pack.flags == (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT);
                Debug.Assert(pack.Vcc == (ushort)(ushort)17675);
            };
            DemoDevice.POWER_STATUS p125 = LoopBackDemoChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT;
            p125.Vservo = (ushort)(ushort)47120;
            p125.Vcc = (ushort)(ushort)17675;
            LoopBackDemoChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baudrate == (uint)987945073U);
                Debug.Assert(pack.timeout == (ushort)(ushort)1418);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)251, (byte)156, (byte)197, (byte)188, (byte)55, (byte)217, (byte)65, (byte)219, (byte)231, (byte)228, (byte)217, (byte)128, (byte)222, (byte)67, (byte)173, (byte)27, (byte)249, (byte)90, (byte)113, (byte)233, (byte)77, (byte)197, (byte)43, (byte)156, (byte)248, (byte)147, (byte)137, (byte)1, (byte)17, (byte)165, (byte)109, (byte)3, (byte)17, (byte)80, (byte)158, (byte)9, (byte)176, (byte)180, (byte)161, (byte)24, (byte)199, (byte)157, (byte)35, (byte)190, (byte)186, (byte)38, (byte)70, (byte)135, (byte)157, (byte)112, (byte)67, (byte)175, (byte)43, (byte)108, (byte)131, (byte)170, (byte)77, (byte)79, (byte)140, (byte)44, (byte)39, (byte)37, (byte)53, (byte)226, (byte)176, (byte)195, (byte)153, (byte)9, (byte)248, (byte)252}));
                Debug.Assert(pack.count == (byte)(byte)181);
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND);
                Debug.Assert(pack.device == (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2);
            };
            DemoDevice.SERIAL_CONTROL p126 = LoopBackDemoChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2;
            p126.count = (byte)(byte)181;
            p126.baudrate = (uint)987945073U;
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND;
            p126.data__SET(new byte[] {(byte)251, (byte)156, (byte)197, (byte)188, (byte)55, (byte)217, (byte)65, (byte)219, (byte)231, (byte)228, (byte)217, (byte)128, (byte)222, (byte)67, (byte)173, (byte)27, (byte)249, (byte)90, (byte)113, (byte)233, (byte)77, (byte)197, (byte)43, (byte)156, (byte)248, (byte)147, (byte)137, (byte)1, (byte)17, (byte)165, (byte)109, (byte)3, (byte)17, (byte)80, (byte)158, (byte)9, (byte)176, (byte)180, (byte)161, (byte)24, (byte)199, (byte)157, (byte)35, (byte)190, (byte)186, (byte)38, (byte)70, (byte)135, (byte)157, (byte)112, (byte)67, (byte)175, (byte)43, (byte)108, (byte)131, (byte)170, (byte)77, (byte)79, (byte)140, (byte)44, (byte)39, (byte)37, (byte)53, (byte)226, (byte)176, (byte)195, (byte)153, (byte)9, (byte)248, (byte)252}, 0) ;
            p126.timeout = (ushort)(ushort)1418;
            LoopBackDemoChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baseline_c_mm == (int)19763089);
                Debug.Assert(pack.accuracy == (uint)1782338731U);
                Debug.Assert(pack.baseline_b_mm == (int) -1776257300);
                Debug.Assert(pack.time_last_baseline_ms == (uint)84257992U);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)174);
                Debug.Assert(pack.tow == (uint)425867947U);
                Debug.Assert(pack.rtk_health == (byte)(byte)162);
                Debug.Assert(pack.baseline_a_mm == (int)1349677314);
                Debug.Assert(pack.nsats == (byte)(byte)118);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)66);
                Debug.Assert(pack.wn == (ushort)(ushort)58562);
                Debug.Assert(pack.iar_num_hypotheses == (int)901343679);
                Debug.Assert(pack.rtk_rate == (byte)(byte)45);
            };
            DemoDevice.GPS_RTK p127 = LoopBackDemoChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.wn = (ushort)(ushort)58562;
            p127.nsats = (byte)(byte)118;
            p127.tow = (uint)425867947U;
            p127.rtk_rate = (byte)(byte)45;
            p127.rtk_health = (byte)(byte)162;
            p127.baseline_c_mm = (int)19763089;
            p127.iar_num_hypotheses = (int)901343679;
            p127.baseline_coords_type = (byte)(byte)66;
            p127.time_last_baseline_ms = (uint)84257992U;
            p127.baseline_b_mm = (int) -1776257300;
            p127.baseline_a_mm = (int)1349677314;
            p127.rtk_receiver_id = (byte)(byte)174;
            p127.accuracy = (uint)1782338731U;
            LoopBackDemoChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)58);
                Debug.Assert(pack.baseline_c_mm == (int) -675993674);
                Debug.Assert(pack.iar_num_hypotheses == (int)59547866);
                Debug.Assert(pack.accuracy == (uint)2095417397U);
                Debug.Assert(pack.time_last_baseline_ms == (uint)94833927U);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)189);
                Debug.Assert(pack.nsats == (byte)(byte)231);
                Debug.Assert(pack.tow == (uint)3961107386U);
                Debug.Assert(pack.wn == (ushort)(ushort)21385);
                Debug.Assert(pack.rtk_health == (byte)(byte)30);
                Debug.Assert(pack.rtk_rate == (byte)(byte)49);
                Debug.Assert(pack.baseline_a_mm == (int)1018502002);
                Debug.Assert(pack.baseline_b_mm == (int)1722029296);
            };
            DemoDevice.GPS2_RTK p128 = LoopBackDemoChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.accuracy = (uint)2095417397U;
            p128.rtk_health = (byte)(byte)30;
            p128.tow = (uint)3961107386U;
            p128.rtk_receiver_id = (byte)(byte)189;
            p128.rtk_rate = (byte)(byte)49;
            p128.baseline_c_mm = (int) -675993674;
            p128.nsats = (byte)(byte)231;
            p128.baseline_coords_type = (byte)(byte)58;
            p128.baseline_b_mm = (int)1722029296;
            p128.baseline_a_mm = (int)1018502002;
            p128.time_last_baseline_ms = (uint)94833927U;
            p128.iar_num_hypotheses = (int)59547866;
            p128.wn = (ushort)(ushort)21385;
            LoopBackDemoChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zgyro == (short)(short)19720);
                Debug.Assert(pack.yacc == (short)(short) -6550);
                Debug.Assert(pack.ymag == (short)(short)1551);
                Debug.Assert(pack.xmag == (short)(short) -13448);
                Debug.Assert(pack.zacc == (short)(short) -7183);
                Debug.Assert(pack.zmag == (short)(short) -26213);
                Debug.Assert(pack.xacc == (short)(short) -28190);
                Debug.Assert(pack.xgyro == (short)(short) -9085);
                Debug.Assert(pack.ygyro == (short)(short) -31865);
                Debug.Assert(pack.time_boot_ms == (uint)3359193780U);
            };
            DemoDevice.SCALED_IMU3 p129 = LoopBackDemoChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.xacc = (short)(short) -28190;
            p129.zgyro = (short)(short)19720;
            p129.xgyro = (short)(short) -9085;
            p129.zmag = (short)(short) -26213;
            p129.ygyro = (short)(short) -31865;
            p129.yacc = (short)(short) -6550;
            p129.zacc = (short)(short) -7183;
            p129.time_boot_ms = (uint)3359193780U;
            p129.xmag = (short)(short) -13448;
            p129.ymag = (short)(short)1551;
            LoopBackDemoChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.jpg_quality == (byte)(byte)96);
                Debug.Assert(pack.payload == (byte)(byte)207);
                Debug.Assert(pack.type == (byte)(byte)229);
                Debug.Assert(pack.height == (ushort)(ushort)55727);
                Debug.Assert(pack.width == (ushort)(ushort)39036);
                Debug.Assert(pack.size == (uint)2258788672U);
                Debug.Assert(pack.packets == (ushort)(ushort)51365);
            };
            DemoDevice.DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.packets = (ushort)(ushort)51365;
            p130.type = (byte)(byte)229;
            p130.size = (uint)2258788672U;
            p130.width = (ushort)(ushort)39036;
            p130.height = (ushort)(ushort)55727;
            p130.payload = (byte)(byte)207;
            p130.jpg_quality = (byte)(byte)96;
            LoopBackDemoChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)54, (byte)225, (byte)24, (byte)127, (byte)81, (byte)90, (byte)240, (byte)232, (byte)163, (byte)230, (byte)64, (byte)87, (byte)191, (byte)202, (byte)84, (byte)189, (byte)190, (byte)73, (byte)110, (byte)122, (byte)76, (byte)214, (byte)227, (byte)34, (byte)210, (byte)61, (byte)29, (byte)74, (byte)50, (byte)115, (byte)211, (byte)219, (byte)113, (byte)29, (byte)87, (byte)236, (byte)47, (byte)37, (byte)175, (byte)8, (byte)143, (byte)58, (byte)108, (byte)89, (byte)51, (byte)193, (byte)33, (byte)117, (byte)164, (byte)67, (byte)254, (byte)77, (byte)109, (byte)63, (byte)244, (byte)45, (byte)187, (byte)7, (byte)229, (byte)78, (byte)226, (byte)132, (byte)56, (byte)243, (byte)42, (byte)180, (byte)79, (byte)178, (byte)106, (byte)112, (byte)184, (byte)183, (byte)245, (byte)51, (byte)96, (byte)232, (byte)193, (byte)254, (byte)188, (byte)41, (byte)214, (byte)91, (byte)100, (byte)34, (byte)40, (byte)24, (byte)96, (byte)180, (byte)247, (byte)234, (byte)221, (byte)193, (byte)146, (byte)186, (byte)221, (byte)112, (byte)16, (byte)143, (byte)176, (byte)233, (byte)151, (byte)73, (byte)241, (byte)85, (byte)76, (byte)196, (byte)96, (byte)186, (byte)176, (byte)163, (byte)54, (byte)187, (byte)28, (byte)176, (byte)166, (byte)61, (byte)184, (byte)47, (byte)113, (byte)121, (byte)143, (byte)32, (byte)22, (byte)131, (byte)241, (byte)251, (byte)28, (byte)95, (byte)179, (byte)135, (byte)26, (byte)254, (byte)116, (byte)145, (byte)248, (byte)161, (byte)209, (byte)254, (byte)229, (byte)19, (byte)10, (byte)109, (byte)188, (byte)40, (byte)87, (byte)79, (byte)52, (byte)145, (byte)203, (byte)107, (byte)223, (byte)215, (byte)119, (byte)239, (byte)135, (byte)154, (byte)24, (byte)81, (byte)95, (byte)18, (byte)147, (byte)88, (byte)29, (byte)242, (byte)129, (byte)159, (byte)96, (byte)6, (byte)60, (byte)232, (byte)119, (byte)0, (byte)102, (byte)133, (byte)6, (byte)94, (byte)140, (byte)35, (byte)31, (byte)198, (byte)155, (byte)10, (byte)242, (byte)133, (byte)66, (byte)236, (byte)174, (byte)99, (byte)20, (byte)53, (byte)115, (byte)133, (byte)96, (byte)130, (byte)57, (byte)71, (byte)77, (byte)147, (byte)121, (byte)84, (byte)156, (byte)153, (byte)2, (byte)123, (byte)127, (byte)102, (byte)212, (byte)48, (byte)166, (byte)141, (byte)134, (byte)87, (byte)147, (byte)133, (byte)172, (byte)24, (byte)112, (byte)189, (byte)55, (byte)245, (byte)91, (byte)57, (byte)200, (byte)165, (byte)238, (byte)20, (byte)97, (byte)243, (byte)130, (byte)41, (byte)242, (byte)57, (byte)145, (byte)33, (byte)246, (byte)22, (byte)137, (byte)12, (byte)208, (byte)200, (byte)144, (byte)231, (byte)157, (byte)131, (byte)127, (byte)28, (byte)100, (byte)155, (byte)189, (byte)210, (byte)39, (byte)169, (byte)221}));
                Debug.Assert(pack.seqnr == (ushort)(ushort)52808);
            };
            DemoDevice.ENCAPSULATED_DATA p131 = LoopBackDemoChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.data__SET(new byte[] {(byte)54, (byte)225, (byte)24, (byte)127, (byte)81, (byte)90, (byte)240, (byte)232, (byte)163, (byte)230, (byte)64, (byte)87, (byte)191, (byte)202, (byte)84, (byte)189, (byte)190, (byte)73, (byte)110, (byte)122, (byte)76, (byte)214, (byte)227, (byte)34, (byte)210, (byte)61, (byte)29, (byte)74, (byte)50, (byte)115, (byte)211, (byte)219, (byte)113, (byte)29, (byte)87, (byte)236, (byte)47, (byte)37, (byte)175, (byte)8, (byte)143, (byte)58, (byte)108, (byte)89, (byte)51, (byte)193, (byte)33, (byte)117, (byte)164, (byte)67, (byte)254, (byte)77, (byte)109, (byte)63, (byte)244, (byte)45, (byte)187, (byte)7, (byte)229, (byte)78, (byte)226, (byte)132, (byte)56, (byte)243, (byte)42, (byte)180, (byte)79, (byte)178, (byte)106, (byte)112, (byte)184, (byte)183, (byte)245, (byte)51, (byte)96, (byte)232, (byte)193, (byte)254, (byte)188, (byte)41, (byte)214, (byte)91, (byte)100, (byte)34, (byte)40, (byte)24, (byte)96, (byte)180, (byte)247, (byte)234, (byte)221, (byte)193, (byte)146, (byte)186, (byte)221, (byte)112, (byte)16, (byte)143, (byte)176, (byte)233, (byte)151, (byte)73, (byte)241, (byte)85, (byte)76, (byte)196, (byte)96, (byte)186, (byte)176, (byte)163, (byte)54, (byte)187, (byte)28, (byte)176, (byte)166, (byte)61, (byte)184, (byte)47, (byte)113, (byte)121, (byte)143, (byte)32, (byte)22, (byte)131, (byte)241, (byte)251, (byte)28, (byte)95, (byte)179, (byte)135, (byte)26, (byte)254, (byte)116, (byte)145, (byte)248, (byte)161, (byte)209, (byte)254, (byte)229, (byte)19, (byte)10, (byte)109, (byte)188, (byte)40, (byte)87, (byte)79, (byte)52, (byte)145, (byte)203, (byte)107, (byte)223, (byte)215, (byte)119, (byte)239, (byte)135, (byte)154, (byte)24, (byte)81, (byte)95, (byte)18, (byte)147, (byte)88, (byte)29, (byte)242, (byte)129, (byte)159, (byte)96, (byte)6, (byte)60, (byte)232, (byte)119, (byte)0, (byte)102, (byte)133, (byte)6, (byte)94, (byte)140, (byte)35, (byte)31, (byte)198, (byte)155, (byte)10, (byte)242, (byte)133, (byte)66, (byte)236, (byte)174, (byte)99, (byte)20, (byte)53, (byte)115, (byte)133, (byte)96, (byte)130, (byte)57, (byte)71, (byte)77, (byte)147, (byte)121, (byte)84, (byte)156, (byte)153, (byte)2, (byte)123, (byte)127, (byte)102, (byte)212, (byte)48, (byte)166, (byte)141, (byte)134, (byte)87, (byte)147, (byte)133, (byte)172, (byte)24, (byte)112, (byte)189, (byte)55, (byte)245, (byte)91, (byte)57, (byte)200, (byte)165, (byte)238, (byte)20, (byte)97, (byte)243, (byte)130, (byte)41, (byte)242, (byte)57, (byte)145, (byte)33, (byte)246, (byte)22, (byte)137, (byte)12, (byte)208, (byte)200, (byte)144, (byte)231, (byte)157, (byte)131, (byte)127, (byte)28, (byte)100, (byte)155, (byte)189, (byte)210, (byte)39, (byte)169, (byte)221}, 0) ;
            p131.seqnr = (ushort)(ushort)52808;
            LoopBackDemoChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.covariance == (byte)(byte)235);
                Debug.Assert(pack.current_distance == (ushort)(ushort)43745);
                Debug.Assert(pack.id == (byte)(byte)199);
                Debug.Assert(pack.orientation == (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_45);
                Debug.Assert(pack.min_distance == (ushort)(ushort)60292);
                Debug.Assert(pack.type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
                Debug.Assert(pack.max_distance == (ushort)(ushort)47090);
                Debug.Assert(pack.time_boot_ms == (uint)1811276989U);
            };
            DemoDevice.DISTANCE_SENSOR p132 = LoopBackDemoChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER;
            p132.covariance = (byte)(byte)235;
            p132.current_distance = (ushort)(ushort)43745;
            p132.min_distance = (ushort)(ushort)60292;
            p132.max_distance = (ushort)(ushort)47090;
            p132.id = (byte)(byte)199;
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_45;
            p132.time_boot_ms = (uint)1811276989U;
            LoopBackDemoChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)11369);
                Debug.Assert(pack.lon == (int) -1619709380);
                Debug.Assert(pack.lat == (int)1513060101);
                Debug.Assert(pack.mask == (ulong)9008508952503174282L);
            };
            DemoDevice.TERRAIN_REQUEST p133 = LoopBackDemoChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.grid_spacing = (ushort)(ushort)11369;
            p133.lon = (int) -1619709380;
            p133.lat = (int)1513060101;
            p133.mask = (ulong)9008508952503174282L;
            LoopBackDemoChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)13757);
                Debug.Assert(pack.lat == (int)660092457);
                Debug.Assert(pack.gridbit == (byte)(byte)151);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short)1636, (short)26677, (short)12538, (short)19476, (short) -16579, (short) -20224, (short) -21856, (short)17299, (short)30170, (short) -6529, (short) -2641, (short)26934, (short)21111, (short) -24976, (short) -20551, (short)15042}));
                Debug.Assert(pack.lon == (int) -1941872176);
            };
            DemoDevice.TERRAIN_DATA p134 = LoopBackDemoChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lon = (int) -1941872176;
            p134.lat = (int)660092457;
            p134.data__SET(new short[] {(short)1636, (short)26677, (short)12538, (short)19476, (short) -16579, (short) -20224, (short) -21856, (short)17299, (short)30170, (short) -6529, (short) -2641, (short)26934, (short)21111, (short) -24976, (short) -20551, (short)15042}, 0) ;
            p134.grid_spacing = (ushort)(ushort)13757;
            p134.gridbit = (byte)(byte)151;
            LoopBackDemoChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)954616152);
                Debug.Assert(pack.lat == (int)743877286);
            };
            DemoDevice.TERRAIN_CHECK p135 = LoopBackDemoChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)743877286;
            p135.lon = (int)954616152;
            LoopBackDemoChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.terrain_height == (float) -2.6903365E38F);
                Debug.Assert(pack.current_height == (float)2.754348E37F);
                Debug.Assert(pack.loaded == (ushort)(ushort)1560);
                Debug.Assert(pack.lon == (int) -1799869557);
                Debug.Assert(pack.lat == (int) -1072172739);
                Debug.Assert(pack.pending == (ushort)(ushort)34988);
                Debug.Assert(pack.spacing == (ushort)(ushort)28404);
            };
            DemoDevice.TERRAIN_REPORT p136 = LoopBackDemoChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.current_height = (float)2.754348E37F;
            p136.loaded = (ushort)(ushort)1560;
            p136.lat = (int) -1072172739;
            p136.pending = (ushort)(ushort)34988;
            p136.spacing = (ushort)(ushort)28404;
            p136.lon = (int) -1799869557;
            p136.terrain_height = (float) -2.6903365E38F;
            LoopBackDemoChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float) -1.193613E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1526840721U);
                Debug.Assert(pack.press_diff == (float) -2.6843816E38F);
                Debug.Assert(pack.temperature == (short)(short) -7360);
            };
            DemoDevice.SCALED_PRESSURE2 p137 = LoopBackDemoChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)1526840721U;
            p137.temperature = (short)(short) -7360;
            p137.press_diff = (float) -2.6843816E38F;
            p137.press_abs = (float) -1.193613E38F;
            LoopBackDemoChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -3.2785414E38F);
                Debug.Assert(pack.x == (float) -2.9324442E38F);
                Debug.Assert(pack.y == (float) -1.4784211E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-3.0691904E38F, -2.4840246E38F, -2.0151797E38F, -2.0496725E36F}));
                Debug.Assert(pack.time_usec == (ulong)8056178410494874850L);
            };
            DemoDevice.ATT_POS_MOCAP p138 = LoopBackDemoChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.time_usec = (ulong)8056178410494874850L;
            p138.x = (float) -2.9324442E38F;
            p138.y = (float) -1.4784211E38F;
            p138.q_SET(new float[] {-3.0691904E38F, -2.4840246E38F, -2.0151797E38F, -2.0496725E36F}, 0) ;
            p138.z = (float) -3.2785414E38F;
            LoopBackDemoChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)8516846373541981877L);
                Debug.Assert(pack.target_system == (byte)(byte)92);
                Debug.Assert(pack.target_component == (byte)(byte)158);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-1.4637573E37F, -6.7650537E37F, 5.243764E37F, 8.63016E37F, 2.6402085E38F, 2.8956482E38F, 3.185743E38F, -2.168142E38F}));
                Debug.Assert(pack.group_mlx == (byte)(byte)57);
            };
            DemoDevice.SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.target_component = (byte)(byte)158;
            p139.controls_SET(new float[] {-1.4637573E37F, -6.7650537E37F, 5.243764E37F, 8.63016E37F, 2.6402085E38F, 2.8956482E38F, 3.185743E38F, -2.168142E38F}, 0) ;
            p139.target_system = (byte)(byte)92;
            p139.group_mlx = (byte)(byte)57;
            p139.time_usec = (ulong)8516846373541981877L;
            LoopBackDemoChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.group_mlx == (byte)(byte)243);
                Debug.Assert(pack.time_usec == (ulong)6294394195681222079L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-3.1214933E38F, 1.9741498E38F, 1.9779203E38F, -1.3806482E38F, -1.5330327E37F, -3.292995E38F, 1.6493389E38F, 9.790506E37F}));
            };
            DemoDevice.ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.group_mlx = (byte)(byte)243;
            p140.time_usec = (ulong)6294394195681222079L;
            p140.controls_SET(new float[] {-3.1214933E38F, 1.9741498E38F, 1.9779203E38F, -1.3806482E38F, -1.5330327E37F, -3.292995E38F, 1.6493389E38F, 9.790506E37F}, 0) ;
            LoopBackDemoChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)7375798383043787752L);
                Debug.Assert(pack.altitude_local == (float)6.043575E37F);
                Debug.Assert(pack.altitude_monotonic == (float)2.1657742E38F);
                Debug.Assert(pack.altitude_amsl == (float)4.086348E37F);
                Debug.Assert(pack.altitude_relative == (float)1.0402788E38F);
                Debug.Assert(pack.bottom_clearance == (float)2.1662396E38F);
                Debug.Assert(pack.altitude_terrain == (float) -5.6444247E37F);
            };
            DemoDevice.ALTITUDE p141 = LoopBackDemoChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.altitude_local = (float)6.043575E37F;
            p141.bottom_clearance = (float)2.1662396E38F;
            p141.altitude_monotonic = (float)2.1657742E38F;
            p141.altitude_terrain = (float) -5.6444247E37F;
            p141.time_usec = (ulong)7375798383043787752L;
            p141.altitude_relative = (float)1.0402788E38F;
            p141.altitude_amsl = (float)4.086348E37F;
            LoopBackDemoChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.transfer_type == (byte)(byte)116);
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)100, (byte)129, (byte)181, (byte)36, (byte)30, (byte)143, (byte)203, (byte)2, (byte)245, (byte)205, (byte)49, (byte)49, (byte)197, (byte)4, (byte)169, (byte)221, (byte)250, (byte)30, (byte)38, (byte)140, (byte)19, (byte)217, (byte)112, (byte)68, (byte)28, (byte)147, (byte)167, (byte)163, (byte)222, (byte)248, (byte)152, (byte)204, (byte)177, (byte)22, (byte)22, (byte)135, (byte)9, (byte)156, (byte)61, (byte)255, (byte)94, (byte)167, (byte)28, (byte)137, (byte)73, (byte)145, (byte)248, (byte)85, (byte)230, (byte)139, (byte)174, (byte)38, (byte)40, (byte)56, (byte)210, (byte)235, (byte)209, (byte)188, (byte)113, (byte)67, (byte)62, (byte)241, (byte)235, (byte)214, (byte)45, (byte)201, (byte)3, (byte)244, (byte)38, (byte)123, (byte)8, (byte)71, (byte)108, (byte)241, (byte)97, (byte)14, (byte)47, (byte)154, (byte)179, (byte)160, (byte)16, (byte)181, (byte)81, (byte)78, (byte)154, (byte)200, (byte)21, (byte)182, (byte)42, (byte)1, (byte)74, (byte)175, (byte)143, (byte)228, (byte)8, (byte)251, (byte)2, (byte)220, (byte)75, (byte)253, (byte)186, (byte)78, (byte)127, (byte)174, (byte)210, (byte)143, (byte)114, (byte)172, (byte)9, (byte)116, (byte)10, (byte)202, (byte)75, (byte)22, (byte)111, (byte)96, (byte)206, (byte)106, (byte)118, (byte)194}));
                Debug.Assert(pack.uri_type == (byte)(byte)51);
                Debug.Assert(pack.request_id == (byte)(byte)115);
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)176, (byte)110, (byte)173, (byte)209, (byte)242, (byte)88, (byte)165, (byte)254, (byte)193, (byte)51, (byte)99, (byte)243, (byte)183, (byte)210, (byte)180, (byte)232, (byte)55, (byte)227, (byte)70, (byte)80, (byte)222, (byte)21, (byte)64, (byte)65, (byte)116, (byte)176, (byte)182, (byte)100, (byte)161, (byte)75, (byte)184, (byte)28, (byte)93, (byte)116, (byte)139, (byte)210, (byte)1, (byte)8, (byte)112, (byte)245, (byte)161, (byte)78, (byte)207, (byte)183, (byte)31, (byte)8, (byte)60, (byte)36, (byte)222, (byte)138, (byte)58, (byte)112, (byte)60, (byte)90, (byte)232, (byte)19, (byte)107, (byte)99, (byte)218, (byte)205, (byte)171, (byte)244, (byte)160, (byte)149, (byte)15, (byte)41, (byte)18, (byte)57, (byte)175, (byte)18, (byte)61, (byte)121, (byte)185, (byte)37, (byte)90, (byte)75, (byte)192, (byte)144, (byte)6, (byte)149, (byte)172, (byte)11, (byte)255, (byte)213, (byte)34, (byte)169, (byte)78, (byte)21, (byte)82, (byte)108, (byte)181, (byte)42, (byte)69, (byte)40, (byte)0, (byte)210, (byte)110, (byte)177, (byte)89, (byte)55, (byte)209, (byte)102, (byte)199, (byte)10, (byte)63, (byte)159, (byte)214, (byte)34, (byte)12, (byte)172, (byte)94, (byte)247, (byte)143, (byte)207, (byte)234, (byte)219, (byte)53, (byte)2, (byte)84, (byte)143}));
            };
            DemoDevice.RESOURCE_REQUEST p142 = LoopBackDemoChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.storage_SET(new byte[] {(byte)100, (byte)129, (byte)181, (byte)36, (byte)30, (byte)143, (byte)203, (byte)2, (byte)245, (byte)205, (byte)49, (byte)49, (byte)197, (byte)4, (byte)169, (byte)221, (byte)250, (byte)30, (byte)38, (byte)140, (byte)19, (byte)217, (byte)112, (byte)68, (byte)28, (byte)147, (byte)167, (byte)163, (byte)222, (byte)248, (byte)152, (byte)204, (byte)177, (byte)22, (byte)22, (byte)135, (byte)9, (byte)156, (byte)61, (byte)255, (byte)94, (byte)167, (byte)28, (byte)137, (byte)73, (byte)145, (byte)248, (byte)85, (byte)230, (byte)139, (byte)174, (byte)38, (byte)40, (byte)56, (byte)210, (byte)235, (byte)209, (byte)188, (byte)113, (byte)67, (byte)62, (byte)241, (byte)235, (byte)214, (byte)45, (byte)201, (byte)3, (byte)244, (byte)38, (byte)123, (byte)8, (byte)71, (byte)108, (byte)241, (byte)97, (byte)14, (byte)47, (byte)154, (byte)179, (byte)160, (byte)16, (byte)181, (byte)81, (byte)78, (byte)154, (byte)200, (byte)21, (byte)182, (byte)42, (byte)1, (byte)74, (byte)175, (byte)143, (byte)228, (byte)8, (byte)251, (byte)2, (byte)220, (byte)75, (byte)253, (byte)186, (byte)78, (byte)127, (byte)174, (byte)210, (byte)143, (byte)114, (byte)172, (byte)9, (byte)116, (byte)10, (byte)202, (byte)75, (byte)22, (byte)111, (byte)96, (byte)206, (byte)106, (byte)118, (byte)194}, 0) ;
            p142.transfer_type = (byte)(byte)116;
            p142.uri_type = (byte)(byte)51;
            p142.request_id = (byte)(byte)115;
            p142.uri_SET(new byte[] {(byte)176, (byte)110, (byte)173, (byte)209, (byte)242, (byte)88, (byte)165, (byte)254, (byte)193, (byte)51, (byte)99, (byte)243, (byte)183, (byte)210, (byte)180, (byte)232, (byte)55, (byte)227, (byte)70, (byte)80, (byte)222, (byte)21, (byte)64, (byte)65, (byte)116, (byte)176, (byte)182, (byte)100, (byte)161, (byte)75, (byte)184, (byte)28, (byte)93, (byte)116, (byte)139, (byte)210, (byte)1, (byte)8, (byte)112, (byte)245, (byte)161, (byte)78, (byte)207, (byte)183, (byte)31, (byte)8, (byte)60, (byte)36, (byte)222, (byte)138, (byte)58, (byte)112, (byte)60, (byte)90, (byte)232, (byte)19, (byte)107, (byte)99, (byte)218, (byte)205, (byte)171, (byte)244, (byte)160, (byte)149, (byte)15, (byte)41, (byte)18, (byte)57, (byte)175, (byte)18, (byte)61, (byte)121, (byte)185, (byte)37, (byte)90, (byte)75, (byte)192, (byte)144, (byte)6, (byte)149, (byte)172, (byte)11, (byte)255, (byte)213, (byte)34, (byte)169, (byte)78, (byte)21, (byte)82, (byte)108, (byte)181, (byte)42, (byte)69, (byte)40, (byte)0, (byte)210, (byte)110, (byte)177, (byte)89, (byte)55, (byte)209, (byte)102, (byte)199, (byte)10, (byte)63, (byte)159, (byte)214, (byte)34, (byte)12, (byte)172, (byte)94, (byte)247, (byte)143, (byte)207, (byte)234, (byte)219, (byte)53, (byte)2, (byte)84, (byte)143}, 0) ;
            LoopBackDemoChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short)10735);
                Debug.Assert(pack.time_boot_ms == (uint)1606911008U);
                Debug.Assert(pack.press_diff == (float) -2.461409E38F);
                Debug.Assert(pack.press_abs == (float)2.677859E37F);
            };
            DemoDevice.SCALED_PRESSURE3 p143 = LoopBackDemoChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.press_abs = (float)2.677859E37F;
            p143.temperature = (short)(short)10735;
            p143.time_boot_ms = (uint)1606911008U;
            p143.press_diff = (float) -2.461409E38F;
            LoopBackDemoChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.custom_state == (ulong)1887259408582749325L);
                Debug.Assert(pack.lon == (int)862002668);
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {3.203855E37F, -2.610386E38F, -6.9709666E36F}));
                Debug.Assert(pack.est_capabilities == (byte)(byte)209);
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {-6.033802E37F, -2.6477002E38F, -2.7584637E38F, 2.2805939E38F}));
                Debug.Assert(pack.timestamp == (ulong)4774034852413424141L);
                Debug.Assert(pack.vel.SequenceEqual(new float[] {-2.1755418E38F, 1.2511535E36F, 2.9847205E38F}));
                Debug.Assert(pack.rates.SequenceEqual(new float[] {-1.700944E38F, -2.852048E38F, -1.3909638E38F}));
                Debug.Assert(pack.alt == (float)8.625326E37F);
                Debug.Assert(pack.lat == (int) -1859032751);
                Debug.Assert(pack.acc.SequenceEqual(new float[] {2.9486287E38F, 1.1965657E38F, 2.0411752E37F}));
            };
            DemoDevice.FOLLOW_TARGET p144 = LoopBackDemoChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.lat = (int) -1859032751;
            p144.attitude_q_SET(new float[] {-6.033802E37F, -2.6477002E38F, -2.7584637E38F, 2.2805939E38F}, 0) ;
            p144.alt = (float)8.625326E37F;
            p144.custom_state = (ulong)1887259408582749325L;
            p144.vel_SET(new float[] {-2.1755418E38F, 1.2511535E36F, 2.9847205E38F}, 0) ;
            p144.est_capabilities = (byte)(byte)209;
            p144.timestamp = (ulong)4774034852413424141L;
            p144.acc_SET(new float[] {2.9486287E38F, 1.1965657E38F, 2.0411752E37F}, 0) ;
            p144.position_cov_SET(new float[] {3.203855E37F, -2.610386E38F, -6.9709666E36F}, 0) ;
            p144.lon = (int)862002668;
            p144.rates_SET(new float[] {-1.700944E38F, -2.852048E38F, -1.3909638E38F}, 0) ;
            LoopBackDemoChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {-9.881928E37F, -1.9373814E38F, 5.273312E36F, 1.3996469E38F}));
                Debug.Assert(pack.roll_rate == (float) -1.2380246E37F);
                Debug.Assert(pack.z_pos == (float)3.0646085E37F);
                Debug.Assert(pack.x_vel == (float)2.1586097E37F);
                Debug.Assert(pack.z_vel == (float)5.532926E37F);
                Debug.Assert(pack.y_vel == (float) -3.1448053E38F);
                Debug.Assert(pack.x_pos == (float) -1.0926226E38F);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {1.3583592E37F, -2.0824392E38F, 1.2110819E38F}));
                Debug.Assert(pack.z_acc == (float) -1.109426E38F);
                Debug.Assert(pack.pitch_rate == (float) -1.0339101E38F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {2.6169251E38F, -3.3553683E38F, 2.7966518E38F}));
                Debug.Assert(pack.x_acc == (float) -7.241061E37F);
                Debug.Assert(pack.y_acc == (float)1.3526264E38F);
                Debug.Assert(pack.yaw_rate == (float) -3.074758E38F);
                Debug.Assert(pack.y_pos == (float)1.0271076E38F);
                Debug.Assert(pack.time_usec == (ulong)2529814811641076507L);
                Debug.Assert(pack.airspeed == (float)8.764753E37F);
            };
            DemoDevice.CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.pitch_rate = (float) -1.0339101E38F;
            p146.y_vel = (float) -3.1448053E38F;
            p146.z_acc = (float) -1.109426E38F;
            p146.x_vel = (float)2.1586097E37F;
            p146.y_acc = (float)1.3526264E38F;
            p146.x_acc = (float) -7.241061E37F;
            p146.z_pos = (float)3.0646085E37F;
            p146.vel_variance_SET(new float[] {2.6169251E38F, -3.3553683E38F, 2.7966518E38F}, 0) ;
            p146.time_usec = (ulong)2529814811641076507L;
            p146.y_pos = (float)1.0271076E38F;
            p146.z_vel = (float)5.532926E37F;
            p146.q_SET(new float[] {-9.881928E37F, -1.9373814E38F, 5.273312E36F, 1.3996469E38F}, 0) ;
            p146.airspeed = (float)8.764753E37F;
            p146.yaw_rate = (float) -3.074758E38F;
            p146.pos_variance_SET(new float[] {1.3583592E37F, -2.0824392E38F, 1.2110819E38F}, 0) ;
            p146.roll_rate = (float) -1.2380246E37F;
            p146.x_pos = (float) -1.0926226E38F;
            LoopBackDemoChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)10903, (ushort)19444, (ushort)62653, (ushort)41469, (ushort)6555, (ushort)10697, (ushort)55871, (ushort)15713, (ushort)24612, (ushort)47161}));
                Debug.Assert(pack.current_consumed == (int) -400364503);
                Debug.Assert(pack.type == (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION);
                Debug.Assert(pack.current_battery == (short)(short)16398);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 8);
                Debug.Assert(pack.id == (byte)(byte)95);
                Debug.Assert(pack.energy_consumed == (int) -1008280813);
                Debug.Assert(pack.temperature == (short)(short) -29495);
                Debug.Assert(pack.battery_function == (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
            };
            DemoDevice.BATTERY_STATUS p147 = LoopBackDemoChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.temperature = (short)(short) -29495;
            p147.battery_remaining = (sbyte)(sbyte) - 8;
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD;
            p147.current_consumed = (int) -400364503;
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION;
            p147.voltages_SET(new ushort[] {(ushort)10903, (ushort)19444, (ushort)62653, (ushort)41469, (ushort)6555, (ushort)10697, (ushort)55871, (ushort)15713, (ushort)24612, (ushort)47161}, 0) ;
            p147.id = (byte)(byte)95;
            p147.energy_consumed = (int) -1008280813;
            p147.current_battery = (short)(short)16398;
            LoopBackDemoChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.os_sw_version == (uint)1779720289U);
                Debug.Assert(pack.board_version == (uint)4216322548U);
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED);
                Debug.Assert(pack.product_id == (ushort)(ushort)1594);
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)201, (byte)155, (byte)18, (byte)41, (byte)94, (byte)252, (byte)169, (byte)0, (byte)199, (byte)89, (byte)28, (byte)84, (byte)6, (byte)201, (byte)58, (byte)124, (byte)77, (byte)232}));
                Debug.Assert(pack.uid == (ulong)4625654449857489883L);
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)65, (byte)107, (byte)16, (byte)91, (byte)175, (byte)12, (byte)57, (byte)249}));
                Debug.Assert(pack.vendor_id == (ushort)(ushort)27573);
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)138, (byte)255, (byte)13, (byte)192, (byte)198, (byte)26, (byte)12, (byte)200}));
                Debug.Assert(pack.middleware_sw_version == (uint)3961483574U);
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)179, (byte)191, (byte)111, (byte)127, (byte)126, (byte)190, (byte)252, (byte)57}));
                Debug.Assert(pack.flight_sw_version == (uint)3598723751U);
            };
            DemoDevice.AUTOPILOT_VERSION p148 = LoopBackDemoChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.product_id = (ushort)(ushort)1594;
            p148.uid2_SET(new byte[] {(byte)201, (byte)155, (byte)18, (byte)41, (byte)94, (byte)252, (byte)169, (byte)0, (byte)199, (byte)89, (byte)28, (byte)84, (byte)6, (byte)201, (byte)58, (byte)124, (byte)77, (byte)232}, 0, PH) ;
            p148.vendor_id = (ushort)(ushort)27573;
            p148.middleware_sw_version = (uint)3961483574U;
            p148.os_sw_version = (uint)1779720289U;
            p148.flight_custom_version_SET(new byte[] {(byte)138, (byte)255, (byte)13, (byte)192, (byte)198, (byte)26, (byte)12, (byte)200}, 0) ;
            p148.board_version = (uint)4216322548U;
            p148.flight_sw_version = (uint)3598723751U;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED;
            p148.uid = (ulong)4625654449857489883L;
            p148.os_custom_version_SET(new byte[] {(byte)65, (byte)107, (byte)16, (byte)91, (byte)175, (byte)12, (byte)57, (byte)249}, 0) ;
            p148.middleware_custom_version_SET(new byte[] {(byte)179, (byte)191, (byte)111, (byte)127, (byte)126, (byte)190, (byte)252, (byte)57}, 0) ;
            LoopBackDemoChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL);
                Debug.Assert(pack.angle_y == (float)1.0734773E38F);
                Debug.Assert(pack.distance == (float) -2.8643204E38F);
                Debug.Assert(pack.y_TRY(ph) == (float) -3.2564212E38F);
                Debug.Assert(pack.size_y == (float) -2.5482531E38F);
                Debug.Assert(pack.target_num == (byte)(byte)35);
                Debug.Assert(pack.time_usec == (ulong)6584890496629409843L);
                Debug.Assert(pack.size_x == (float) -1.1891441E38F);
                Debug.Assert(pack.z_TRY(ph) == (float) -1.7207923E38F);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)221);
                Debug.Assert(pack.x_TRY(ph) == (float)1.2451811E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.angle_x == (float)2.8447966E38F);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {1.5760135E38F, 2.4225799E38F, 9.193675E37F, 3.0514837E38F}));
            };
            DemoDevice.LANDING_TARGET p149 = LoopBackDemoChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.angle_y = (float)1.0734773E38F;
            p149.z_SET((float) -1.7207923E38F, PH) ;
            p149.y_SET((float) -3.2564212E38F, PH) ;
            p149.time_usec = (ulong)6584890496629409843L;
            p149.q_SET(new float[] {1.5760135E38F, 2.4225799E38F, 9.193675E37F, 3.0514837E38F}, 0, PH) ;
            p149.size_x = (float) -1.1891441E38F;
            p149.size_y = (float) -2.5482531E38F;
            p149.angle_x = (float)2.8447966E38F;
            p149.target_num = (byte)(byte)35;
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL;
            p149.x_SET((float)1.2451811E38F, PH) ;
            p149.position_valid_SET((byte)(byte)221, PH) ;
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p149.distance = (float) -2.8643204E38F;
            LoopBackDemoChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSENS_POWERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.adc121_cs2_amp == (float)4.293824E37F);
                Debug.Assert(pack.adc121_vspb_volt == (float)2.2980027E38F);
                Debug.Assert(pack.adc121_cs1_amp == (float) -1.0893725E38F);
                Debug.Assert(pack.adc121_cspb_amp == (float) -1.9759067E38F);
            };
            DemoDevice.SENS_POWER p201 = LoopBackDemoChannel.new_SENS_POWER();
            PH.setPack(p201);
            p201.adc121_vspb_volt = (float)2.2980027E38F;
            p201.adc121_cspb_amp = (float) -1.9759067E38F;
            p201.adc121_cs2_amp = (float)4.293824E37F;
            p201.adc121_cs1_amp = (float) -1.0893725E38F;
            LoopBackDemoChannel.instance.send(p201);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSENS_MPPTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mppt1_amp == (float)7.040614E37F);
                Debug.Assert(pack.mppt2_volt == (float)5.558868E37F);
                Debug.Assert(pack.mppt3_pwm == (ushort)(ushort)37354);
                Debug.Assert(pack.mppt3_amp == (float)2.705676E38F);
                Debug.Assert(pack.mppt1_pwm == (ushort)(ushort)46468);
                Debug.Assert(pack.mppt_timestamp == (ulong)6888071060612006179L);
                Debug.Assert(pack.mppt3_volt == (float) -3.274357E38F);
                Debug.Assert(pack.mppt3_status == (byte)(byte)11);
                Debug.Assert(pack.mppt2_status == (byte)(byte)87);
                Debug.Assert(pack.mppt1_volt == (float) -1.0890307E38F);
                Debug.Assert(pack.mppt2_amp == (float) -3.0912798E38F);
                Debug.Assert(pack.mppt1_status == (byte)(byte)55);
                Debug.Assert(pack.mppt2_pwm == (ushort)(ushort)50628);
            };
            DemoDevice.SENS_MPPT p202 = LoopBackDemoChannel.new_SENS_MPPT();
            PH.setPack(p202);
            p202.mppt1_amp = (float)7.040614E37F;
            p202.mppt1_pwm = (ushort)(ushort)46468;
            p202.mppt2_pwm = (ushort)(ushort)50628;
            p202.mppt3_status = (byte)(byte)11;
            p202.mppt2_status = (byte)(byte)87;
            p202.mppt3_amp = (float)2.705676E38F;
            p202.mppt_timestamp = (ulong)6888071060612006179L;
            p202.mppt3_volt = (float) -3.274357E38F;
            p202.mppt2_amp = (float) -3.0912798E38F;
            p202.mppt2_volt = (float)5.558868E37F;
            p202.mppt1_volt = (float) -1.0890307E38F;
            p202.mppt1_status = (byte)(byte)55;
            p202.mppt3_pwm = (ushort)(ushort)37354;
            LoopBackDemoChannel.instance.send(p202);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnASLCTRL_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uElev == (float) -1.9801954E38F);
                Debug.Assert(pack.pRef == (float) -1.5010112E38F);
                Debug.Assert(pack.timestamp == (ulong)2679860131297186779L);
                Debug.Assert(pack.YawAngleRef == (float) -1.2258426E38F);
                Debug.Assert(pack.PitchAngleRef == (float)2.1959006E38F);
                Debug.Assert(pack.q == (float) -2.23182E38F);
                Debug.Assert(pack.uThrot2 == (float)9.098807E37F);
                Debug.Assert(pack.uRud == (float)5.960087E36F);
                Debug.Assert(pack.p == (float)2.8509632E38F);
                Debug.Assert(pack.uAil == (float) -1.5560672E38F);
                Debug.Assert(pack.PitchAngle == (float)8.065219E37F);
                Debug.Assert(pack.AirspeedRef == (float) -2.7210298E36F);
                Debug.Assert(pack.SpoilersEngaged == (byte)(byte)196);
                Debug.Assert(pack.rRef == (float)3.2050105E38F);
                Debug.Assert(pack.YawAngle == (float)2.3678673E38F);
                Debug.Assert(pack.r == (float) -2.2882744E38F);
                Debug.Assert(pack.uThrot == (float)2.8668245E38F);
                Debug.Assert(pack.aslctrl_mode == (byte)(byte)187);
                Debug.Assert(pack.RollAngle == (float)3.1828926E38F);
                Debug.Assert(pack.hRef_t == (float)1.5587387E38F);
                Debug.Assert(pack.hRef == (float)1.0100406E38F);
                Debug.Assert(pack.nZ == (float) -2.0152846E38F);
                Debug.Assert(pack.qRef == (float) -1.106643E38F);
                Debug.Assert(pack.RollAngleRef == (float)2.0113307E38F);
                Debug.Assert(pack.h == (float) -2.3720335E38F);
            };
            DemoDevice.ASLCTRL_DATA p203 = LoopBackDemoChannel.new_ASLCTRL_DATA();
            PH.setPack(p203);
            p203.PitchAngle = (float)8.065219E37F;
            p203.qRef = (float) -1.106643E38F;
            p203.p = (float)2.8509632E38F;
            p203.RollAngleRef = (float)2.0113307E38F;
            p203.uAil = (float) -1.5560672E38F;
            p203.nZ = (float) -2.0152846E38F;
            p203.uElev = (float) -1.9801954E38F;
            p203.q = (float) -2.23182E38F;
            p203.uRud = (float)5.960087E36F;
            p203.h = (float) -2.3720335E38F;
            p203.PitchAngleRef = (float)2.1959006E38F;
            p203.rRef = (float)3.2050105E38F;
            p203.timestamp = (ulong)2679860131297186779L;
            p203.SpoilersEngaged = (byte)(byte)196;
            p203.hRef_t = (float)1.5587387E38F;
            p203.r = (float) -2.2882744E38F;
            p203.aslctrl_mode = (byte)(byte)187;
            p203.hRef = (float)1.0100406E38F;
            p203.pRef = (float) -1.5010112E38F;
            p203.AirspeedRef = (float) -2.7210298E36F;
            p203.YawAngle = (float)2.3678673E38F;
            p203.RollAngle = (float)3.1828926E38F;
            p203.YawAngleRef = (float) -1.2258426E38F;
            p203.uThrot2 = (float)9.098807E37F;
            p203.uThrot = (float)2.8668245E38F;
            LoopBackDemoChannel.instance.send(p203);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnASLCTRL_DEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.f_5 == (float) -9.324041E37F);
                Debug.Assert(pack.f_2 == (float)8.0360286E37F);
                Debug.Assert(pack.f_7 == (float) -8.0177024E37F);
                Debug.Assert(pack.f_4 == (float)2.1621627E37F);
                Debug.Assert(pack.i32_1 == (uint)3571931396U);
                Debug.Assert(pack.f_1 == (float)2.5130486E38F);
                Debug.Assert(pack.f_3 == (float)7.923249E37F);
                Debug.Assert(pack.i8_2 == (byte)(byte)117);
                Debug.Assert(pack.i8_1 == (byte)(byte)19);
                Debug.Assert(pack.f_6 == (float) -2.4301907E38F);
                Debug.Assert(pack.f_8 == (float)2.127535E38F);
            };
            DemoDevice.ASLCTRL_DEBUG p204 = LoopBackDemoChannel.new_ASLCTRL_DEBUG();
            PH.setPack(p204);
            p204.i8_2 = (byte)(byte)117;
            p204.f_4 = (float)2.1621627E37F;
            p204.f_8 = (float)2.127535E38F;
            p204.f_5 = (float) -9.324041E37F;
            p204.f_3 = (float)7.923249E37F;
            p204.f_7 = (float) -8.0177024E37F;
            p204.i32_1 = (uint)3571931396U;
            p204.f_1 = (float)2.5130486E38F;
            p204.i8_1 = (byte)(byte)19;
            p204.f_6 = (float) -2.4301907E38F;
            p204.f_2 = (float)8.0360286E37F;
            LoopBackDemoChannel.instance.send(p204);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnASLUAV_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Motor_rpm == (float) -1.1873387E37F);
                Debug.Assert(pack.LED_status == (byte)(byte)65);
                Debug.Assert(pack.SATCOM_status == (byte)(byte)72);
                Debug.Assert(pack.Servo_status.SequenceEqual(new byte[] {(byte)59, (byte)36, (byte)11, (byte)203, (byte)118, (byte)12, (byte)208, (byte)108}));
            };
            DemoDevice.ASLUAV_STATUS p205 = LoopBackDemoChannel.new_ASLUAV_STATUS();
            PH.setPack(p205);
            p205.LED_status = (byte)(byte)65;
            p205.SATCOM_status = (byte)(byte)72;
            p205.Motor_rpm = (float) -1.1873387E37F;
            p205.Servo_status_SET(new byte[] {(byte)59, (byte)36, (byte)11, (byte)203, (byte)118, (byte)12, (byte)208, (byte)108}, 0) ;
            LoopBackDemoChannel.instance.send(p205);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnEKF_EXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Airspeed == (float) -3.308704E37F);
                Debug.Assert(pack.Windspeed == (float) -2.7128417E38F);
                Debug.Assert(pack.WindDir == (float) -2.324936E38F);
                Debug.Assert(pack.WindZ == (float)1.0888312E38F);
                Debug.Assert(pack.beta == (float)2.2785151E38F);
                Debug.Assert(pack.alpha == (float)3.316369E38F);
                Debug.Assert(pack.timestamp == (ulong)1180879391244327197L);
            };
            DemoDevice.EKF_EXT p206 = LoopBackDemoChannel.new_EKF_EXT();
            PH.setPack(p206);
            p206.WindZ = (float)1.0888312E38F;
            p206.WindDir = (float) -2.324936E38F;
            p206.Airspeed = (float) -3.308704E37F;
            p206.timestamp = (ulong)1180879391244327197L;
            p206.Windspeed = (float) -2.7128417E38F;
            p206.beta = (float)2.2785151E38F;
            p206.alpha = (float)3.316369E38F;
            LoopBackDemoChannel.instance.send(p206);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnASL_OBCTRLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.obctrl_status == (byte)(byte)90);
                Debug.Assert(pack.timestamp == (ulong)5847764930958088198L);
                Debug.Assert(pack.uAilR == (float) -1.5014626E38F);
                Debug.Assert(pack.uThrot == (float)2.027049E38F);
                Debug.Assert(pack.uAilL == (float) -3.1406835E38F);
                Debug.Assert(pack.uRud == (float)2.9225745E38F);
                Debug.Assert(pack.uElev == (float)2.1534297E38F);
                Debug.Assert(pack.uThrot2 == (float)1.0506807E38F);
            };
            DemoDevice.ASL_OBCTRL p207 = LoopBackDemoChannel.new_ASL_OBCTRL();
            PH.setPack(p207);
            p207.obctrl_status = (byte)(byte)90;
            p207.uAilL = (float) -3.1406835E38F;
            p207.uThrot = (float)2.027049E38F;
            p207.uThrot2 = (float)1.0506807E38F;
            p207.uAilR = (float) -1.5014626E38F;
            p207.uElev = (float)2.1534297E38F;
            p207.timestamp = (ulong)5847764930958088198L;
            p207.uRud = (float)2.9225745E38F;
            LoopBackDemoChannel.instance.send(p207);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSENS_ATMOSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.TempAmbient == (float) -2.5596295E38F);
                Debug.Assert(pack.Humidity == (float)2.3206316E38F);
            };
            DemoDevice.SENS_ATMOS p208 = LoopBackDemoChannel.new_SENS_ATMOS();
            PH.setPack(p208);
            p208.Humidity = (float)2.3206316E38F;
            p208.TempAmbient = (float) -2.5596295E38F;
            LoopBackDemoChannel.instance.send(p208);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSENS_BATMONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.cellvoltage5 == (ushort)(ushort)29856);
                Debug.Assert(pack.cellvoltage4 == (ushort)(ushort)12202);
                Debug.Assert(pack.temperature == (float) -1.1570699E38F);
                Debug.Assert(pack.hostfetcontrol == (ushort)(ushort)55458);
                Debug.Assert(pack.cellvoltage6 == (ushort)(ushort)16402);
                Debug.Assert(pack.batterystatus == (ushort)(ushort)31862);
                Debug.Assert(pack.voltage == (ushort)(ushort)14663);
                Debug.Assert(pack.cellvoltage1 == (ushort)(ushort)21128);
                Debug.Assert(pack.serialnumber == (ushort)(ushort)7036);
                Debug.Assert(pack.cellvoltage3 == (ushort)(ushort)11037);
                Debug.Assert(pack.SoC == (byte)(byte)2);
                Debug.Assert(pack.current == (short)(short)3071);
                Debug.Assert(pack.cellvoltage2 == (ushort)(ushort)17017);
            };
            DemoDevice.SENS_BATMON p209 = LoopBackDemoChannel.new_SENS_BATMON();
            PH.setPack(p209);
            p209.cellvoltage4 = (ushort)(ushort)12202;
            p209.cellvoltage5 = (ushort)(ushort)29856;
            p209.temperature = (float) -1.1570699E38F;
            p209.cellvoltage1 = (ushort)(ushort)21128;
            p209.cellvoltage3 = (ushort)(ushort)11037;
            p209.cellvoltage6 = (ushort)(ushort)16402;
            p209.hostfetcontrol = (ushort)(ushort)55458;
            p209.current = (short)(short)3071;
            p209.cellvoltage2 = (ushort)(ushort)17017;
            p209.serialnumber = (ushort)(ushort)7036;
            p209.batterystatus = (ushort)(ushort)31862;
            p209.SoC = (byte)(byte)2;
            p209.voltage = (ushort)(ushort)14663;
            LoopBackDemoChannel.instance.send(p209);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFW_SOARING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.DistToSoarPoint == (float)3.2269604E38F);
                Debug.Assert(pack.xLon == (float) -2.0674523E38F);
                Debug.Assert(pack.DebugVar1 == (float)2.503843E38F);
                Debug.Assert(pack.z2_DeltaRoll == (float) -1.8745908E38F);
                Debug.Assert(pack.VarW == (float) -1.3868592E38F);
                Debug.Assert(pack.ControlMode == (byte)(byte)188);
                Debug.Assert(pack.VarLon == (float)6.282394E37F);
                Debug.Assert(pack.VarR == (float)3.1864513E38F);
                Debug.Assert(pack.timestampModeChanged == (ulong)3385952586855883766L);
                Debug.Assert(pack.xLat == (float)3.130398E38F);
                Debug.Assert(pack.ThermalGSNorth == (float) -1.0088282E38F);
                Debug.Assert(pack.TSE_dot == (float)1.7719958E38F);
                Debug.Assert(pack.z1_exp == (float)7.0394717E37F);
                Debug.Assert(pack.z1_LocalUpdraftSpeed == (float)3.6061833E37F);
                Debug.Assert(pack.z2_exp == (float) -1.4103639E38F);
                Debug.Assert(pack.valid == (byte)(byte)79);
                Debug.Assert(pack.xW == (float)8.571463E36F);
                Debug.Assert(pack.LoiterDirection == (float) -1.1623052E38F);
                Debug.Assert(pack.timestamp == (ulong)4973263769875521408L);
                Debug.Assert(pack.xR == (float)2.2992665E38F);
                Debug.Assert(pack.DebugVar2 == (float) -2.9960524E38F);
                Debug.Assert(pack.vSinkExp == (float) -5.5121885E37F);
                Debug.Assert(pack.ThermalGSEast == (float) -1.4727513E38F);
                Debug.Assert(pack.VarLat == (float)1.1791321E37F);
                Debug.Assert(pack.LoiterRadius == (float)2.532764E38F);
            };
            DemoDevice.FW_SOARING_DATA p210 = LoopBackDemoChannel.new_FW_SOARING_DATA();
            PH.setPack(p210);
            p210.ThermalGSEast = (float) -1.4727513E38F;
            p210.xLon = (float) -2.0674523E38F;
            p210.TSE_dot = (float)1.7719958E38F;
            p210.DebugVar1 = (float)2.503843E38F;
            p210.ControlMode = (byte)(byte)188;
            p210.VarLat = (float)1.1791321E37F;
            p210.DebugVar2 = (float) -2.9960524E38F;
            p210.VarLon = (float)6.282394E37F;
            p210.timestamp = (ulong)4973263769875521408L;
            p210.DistToSoarPoint = (float)3.2269604E38F;
            p210.xW = (float)8.571463E36F;
            p210.xR = (float)2.2992665E38F;
            p210.LoiterDirection = (float) -1.1623052E38F;
            p210.xLat = (float)3.130398E38F;
            p210.LoiterRadius = (float)2.532764E38F;
            p210.timestampModeChanged = (ulong)3385952586855883766L;
            p210.ThermalGSNorth = (float) -1.0088282E38F;
            p210.VarW = (float) -1.3868592E38F;
            p210.z1_LocalUpdraftSpeed = (float)3.6061833E37F;
            p210.vSinkExp = (float) -5.5121885E37F;
            p210.VarR = (float)3.1864513E38F;
            p210.z2_DeltaRoll = (float) -1.8745908E38F;
            p210.z2_exp = (float) -1.4103639E38F;
            p210.z1_exp = (float)7.0394717E37F;
            p210.valid = (byte)(byte)79;
            LoopBackDemoChannel.instance.send(p210);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSENSORPOD_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.recording_nodes_count == (byte)(byte)151);
                Debug.Assert(pack.timestamp == (ulong)7663049397293381486L);
                Debug.Assert(pack.free_space == (ushort)(ushort)21582);
                Debug.Assert(pack.visensor_rate_4 == (byte)(byte)96);
                Debug.Assert(pack.visensor_rate_1 == (byte)(byte)26);
                Debug.Assert(pack.cpu_temp == (byte)(byte)17);
                Debug.Assert(pack.visensor_rate_2 == (byte)(byte)76);
                Debug.Assert(pack.visensor_rate_3 == (byte)(byte)115);
            };
            DemoDevice.SENSORPOD_STATUS p211 = LoopBackDemoChannel.new_SENSORPOD_STATUS();
            PH.setPack(p211);
            p211.cpu_temp = (byte)(byte)17;
            p211.recording_nodes_count = (byte)(byte)151;
            p211.free_space = (ushort)(ushort)21582;
            p211.timestamp = (ulong)7663049397293381486L;
            p211.visensor_rate_4 = (byte)(byte)96;
            p211.visensor_rate_3 = (byte)(byte)115;
            p211.visensor_rate_1 = (byte)(byte)26;
            p211.visensor_rate_2 = (byte)(byte)76;
            LoopBackDemoChannel.instance.send(p211);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSENS_POWER_BOARDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pwr_brd_servo_3_amp == (float)2.9247938E38F);
                Debug.Assert(pack.pwr_brd_servo_2_amp == (float) -2.0317447E38F);
                Debug.Assert(pack.pwr_brd_servo_1_amp == (float) -2.7007738E38F);
                Debug.Assert(pack.pwr_brd_mot_l_amp == (float) -1.5635635E38F);
                Debug.Assert(pack.pwr_brd_mot_r_amp == (float) -1.1038882E38F);
                Debug.Assert(pack.pwr_brd_system_volt == (float) -8.784015E37F);
                Debug.Assert(pack.timestamp == (ulong)685570234682738376L);
                Debug.Assert(pack.pwr_brd_led_status == (byte)(byte)161);
                Debug.Assert(pack.pwr_brd_servo_4_amp == (float)1.0548175E38F);
                Debug.Assert(pack.pwr_brd_servo_volt == (float)2.4783768E38F);
                Debug.Assert(pack.pwr_brd_status == (byte)(byte)196);
                Debug.Assert(pack.pwr_brd_aux_amp == (float)3.0565721E38F);
            };
            DemoDevice.SENS_POWER_BOARD p212 = LoopBackDemoChannel.new_SENS_POWER_BOARD();
            PH.setPack(p212);
            p212.pwr_brd_servo_volt = (float)2.4783768E38F;
            p212.pwr_brd_servo_1_amp = (float) -2.7007738E38F;
            p212.pwr_brd_mot_r_amp = (float) -1.1038882E38F;
            p212.pwr_brd_system_volt = (float) -8.784015E37F;
            p212.pwr_brd_servo_2_amp = (float) -2.0317447E38F;
            p212.pwr_brd_status = (byte)(byte)196;
            p212.pwr_brd_servo_3_amp = (float)2.9247938E38F;
            p212.pwr_brd_aux_amp = (float)3.0565721E38F;
            p212.pwr_brd_led_status = (byte)(byte)161;
            p212.pwr_brd_mot_l_amp = (float) -1.5635635E38F;
            p212.timestamp = (ulong)685570234682738376L;
            p212.pwr_brd_servo_4_amp = (float)1.0548175E38F;
            LoopBackDemoChannel.instance.send(p212);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pos_vert_ratio == (float)1.2414537E38F);
                Debug.Assert(pack.vel_ratio == (float) -2.093183E38F);
                Debug.Assert(pack.pos_vert_accuracy == (float)8.033756E37F);
                Debug.Assert(pack.pos_horiz_ratio == (float) -2.7558006E38F);
                Debug.Assert(pack.mag_ratio == (float)8.09964E37F);
                Debug.Assert(pack.pos_horiz_accuracy == (float)2.759323E38F);
                Debug.Assert(pack.time_usec == (ulong)1695693320700867312L);
                Debug.Assert(pack.hagl_ratio == (float) -2.006369E38F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS);
                Debug.Assert(pack.tas_ratio == (float) -2.6689641E38F);
            };
            DemoDevice.ESTIMATOR_STATUS p230 = LoopBackDemoChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.pos_vert_ratio = (float)1.2414537E38F;
            p230.pos_vert_accuracy = (float)8.033756E37F;
            p230.mag_ratio = (float)8.09964E37F;
            p230.hagl_ratio = (float) -2.006369E38F;
            p230.tas_ratio = (float) -2.6689641E38F;
            p230.pos_horiz_ratio = (float) -2.7558006E38F;
            p230.time_usec = (ulong)1695693320700867312L;
            p230.vel_ratio = (float) -2.093183E38F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS;
            p230.pos_horiz_accuracy = (float)2.759323E38F;
            LoopBackDemoChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.var_horiz == (float) -2.0003068E38F);
                Debug.Assert(pack.time_usec == (ulong)137077455402101429L);
                Debug.Assert(pack.wind_alt == (float) -8.74002E37F);
                Debug.Assert(pack.var_vert == (float) -2.2681539E38F);
                Debug.Assert(pack.wind_y == (float) -1.6281112E38F);
                Debug.Assert(pack.wind_x == (float) -3.0835526E38F);
                Debug.Assert(pack.vert_accuracy == (float)3.2924736E38F);
                Debug.Assert(pack.horiz_accuracy == (float)2.174439E38F);
                Debug.Assert(pack.wind_z == (float) -1.0778543E38F);
            };
            DemoDevice.WIND_COV p231 = LoopBackDemoChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.var_horiz = (float) -2.0003068E38F;
            p231.wind_alt = (float) -8.74002E37F;
            p231.wind_z = (float) -1.0778543E38F;
            p231.wind_x = (float) -3.0835526E38F;
            p231.var_vert = (float) -2.2681539E38F;
            p231.horiz_accuracy = (float)2.174439E38F;
            p231.wind_y = (float) -1.6281112E38F;
            p231.time_usec = (ulong)137077455402101429L;
            p231.vert_accuracy = (float)3.2924736E38F;
            LoopBackDemoChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY);
                Debug.Assert(pack.vert_accuracy == (float) -1.0434966E38F);
                Debug.Assert(pack.lon == (int) -1347394141);
                Debug.Assert(pack.fix_type == (byte)(byte)217);
                Debug.Assert(pack.vdop == (float)2.1437339E38F);
                Debug.Assert(pack.time_week == (ushort)(ushort)45122);
                Debug.Assert(pack.lat == (int) -2037219419);
                Debug.Assert(pack.vn == (float) -1.8627979E37F);
                Debug.Assert(pack.gps_id == (byte)(byte)229);
                Debug.Assert(pack.alt == (float)2.1718786E38F);
                Debug.Assert(pack.vd == (float)2.1694524E38F);
                Debug.Assert(pack.hdop == (float)2.3278093E38F);
                Debug.Assert(pack.speed_accuracy == (float)3.3833747E38F);
                Debug.Assert(pack.time_usec == (ulong)7184953726769508641L);
                Debug.Assert(pack.satellites_visible == (byte)(byte)84);
                Debug.Assert(pack.horiz_accuracy == (float)3.9122867E37F);
                Debug.Assert(pack.time_week_ms == (uint)1600079956U);
                Debug.Assert(pack.ve == (float) -1.8847068E38F);
            };
            DemoDevice.GPS_INPUT p232 = LoopBackDemoChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.lon = (int) -1347394141;
            p232.fix_type = (byte)(byte)217;
            p232.vn = (float) -1.8627979E37F;
            p232.gps_id = (byte)(byte)229;
            p232.ve = (float) -1.8847068E38F;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY;
            p232.hdop = (float)2.3278093E38F;
            p232.vd = (float)2.1694524E38F;
            p232.vdop = (float)2.1437339E38F;
            p232.speed_accuracy = (float)3.3833747E38F;
            p232.time_week = (ushort)(ushort)45122;
            p232.vert_accuracy = (float) -1.0434966E38F;
            p232.time_week_ms = (uint)1600079956U;
            p232.alt = (float)2.1718786E38F;
            p232.horiz_accuracy = (float)3.9122867E37F;
            p232.time_usec = (ulong)7184953726769508641L;
            p232.satellites_visible = (byte)(byte)84;
            p232.lat = (int) -2037219419;
            LoopBackDemoChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (byte)(byte)41);
                Debug.Assert(pack.len == (byte)(byte)219);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)206, (byte)219, (byte)121, (byte)10, (byte)61, (byte)74, (byte)235, (byte)58, (byte)45, (byte)198, (byte)36, (byte)45, (byte)67, (byte)243, (byte)201, (byte)88, (byte)125, (byte)166, (byte)228, (byte)39, (byte)191, (byte)200, (byte)195, (byte)218, (byte)206, (byte)126, (byte)55, (byte)41, (byte)167, (byte)67, (byte)122, (byte)201, (byte)92, (byte)48, (byte)122, (byte)131, (byte)17, (byte)138, (byte)184, (byte)64, (byte)233, (byte)99, (byte)220, (byte)95, (byte)30, (byte)11, (byte)43, (byte)89, (byte)122, (byte)182, (byte)130, (byte)190, (byte)84, (byte)249, (byte)177, (byte)212, (byte)38, (byte)153, (byte)69, (byte)221, (byte)223, (byte)6, (byte)111, (byte)237, (byte)101, (byte)254, (byte)117, (byte)21, (byte)13, (byte)178, (byte)16, (byte)2, (byte)193, (byte)214, (byte)0, (byte)117, (byte)99, (byte)74, (byte)61, (byte)42, (byte)120, (byte)101, (byte)235, (byte)96, (byte)19, (byte)41, (byte)4, (byte)254, (byte)140, (byte)105, (byte)81, (byte)63, (byte)79, (byte)61, (byte)103, (byte)28, (byte)248, (byte)73, (byte)94, (byte)124, (byte)126, (byte)85, (byte)223, (byte)6, (byte)216, (byte)176, (byte)88, (byte)18, (byte)133, (byte)170, (byte)147, (byte)185, (byte)112, (byte)134, (byte)39, (byte)163, (byte)61, (byte)254, (byte)85, (byte)118, (byte)222, (byte)161, (byte)98, (byte)71, (byte)237, (byte)131, (byte)32, (byte)60, (byte)139, (byte)143, (byte)211, (byte)108, (byte)192, (byte)183, (byte)221, (byte)163, (byte)147, (byte)204, (byte)74, (byte)111, (byte)202, (byte)217, (byte)135, (byte)237, (byte)107, (byte)38, (byte)85, (byte)2, (byte)19, (byte)160, (byte)107, (byte)202, (byte)166, (byte)130, (byte)140, (byte)220, (byte)96, (byte)148, (byte)147, (byte)235, (byte)105, (byte)225, (byte)209, (byte)199, (byte)131, (byte)44, (byte)80, (byte)59, (byte)228, (byte)112, (byte)54, (byte)77, (byte)162, (byte)94, (byte)11, (byte)55, (byte)207, (byte)225, (byte)156, (byte)2}));
            };
            DemoDevice.GPS_RTCM_DATA p233 = LoopBackDemoChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.len = (byte)(byte)219;
            p233.data__SET(new byte[] {(byte)206, (byte)219, (byte)121, (byte)10, (byte)61, (byte)74, (byte)235, (byte)58, (byte)45, (byte)198, (byte)36, (byte)45, (byte)67, (byte)243, (byte)201, (byte)88, (byte)125, (byte)166, (byte)228, (byte)39, (byte)191, (byte)200, (byte)195, (byte)218, (byte)206, (byte)126, (byte)55, (byte)41, (byte)167, (byte)67, (byte)122, (byte)201, (byte)92, (byte)48, (byte)122, (byte)131, (byte)17, (byte)138, (byte)184, (byte)64, (byte)233, (byte)99, (byte)220, (byte)95, (byte)30, (byte)11, (byte)43, (byte)89, (byte)122, (byte)182, (byte)130, (byte)190, (byte)84, (byte)249, (byte)177, (byte)212, (byte)38, (byte)153, (byte)69, (byte)221, (byte)223, (byte)6, (byte)111, (byte)237, (byte)101, (byte)254, (byte)117, (byte)21, (byte)13, (byte)178, (byte)16, (byte)2, (byte)193, (byte)214, (byte)0, (byte)117, (byte)99, (byte)74, (byte)61, (byte)42, (byte)120, (byte)101, (byte)235, (byte)96, (byte)19, (byte)41, (byte)4, (byte)254, (byte)140, (byte)105, (byte)81, (byte)63, (byte)79, (byte)61, (byte)103, (byte)28, (byte)248, (byte)73, (byte)94, (byte)124, (byte)126, (byte)85, (byte)223, (byte)6, (byte)216, (byte)176, (byte)88, (byte)18, (byte)133, (byte)170, (byte)147, (byte)185, (byte)112, (byte)134, (byte)39, (byte)163, (byte)61, (byte)254, (byte)85, (byte)118, (byte)222, (byte)161, (byte)98, (byte)71, (byte)237, (byte)131, (byte)32, (byte)60, (byte)139, (byte)143, (byte)211, (byte)108, (byte)192, (byte)183, (byte)221, (byte)163, (byte)147, (byte)204, (byte)74, (byte)111, (byte)202, (byte)217, (byte)135, (byte)237, (byte)107, (byte)38, (byte)85, (byte)2, (byte)19, (byte)160, (byte)107, (byte)202, (byte)166, (byte)130, (byte)140, (byte)220, (byte)96, (byte)148, (byte)147, (byte)235, (byte)105, (byte)225, (byte)209, (byte)199, (byte)131, (byte)44, (byte)80, (byte)59, (byte)228, (byte)112, (byte)54, (byte)77, (byte)162, (byte)94, (byte)11, (byte)55, (byte)207, (byte)225, (byte)156, (byte)2}, 0) ;
            p233.flags = (byte)(byte)41;
            LoopBackDemoChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.custom_mode == (uint)3632531296U);
                Debug.Assert(pack.latitude == (int)355348889);
                Debug.Assert(pack.battery_remaining == (byte)(byte)194);
                Debug.Assert(pack.throttle == (sbyte)(sbyte)44);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte) - 7);
                Debug.Assert(pack.failsafe == (byte)(byte)99);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)203);
                Debug.Assert(pack.wp_num == (byte)(byte)253);
                Debug.Assert(pack.longitude == (int) -1772833602);
                Debug.Assert(pack.gps_nsat == (byte)(byte)119);
                Debug.Assert(pack.altitude_sp == (short)(short) -20913);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)65353);
                Debug.Assert(pack.pitch == (short)(short)27968);
                Debug.Assert(pack.heading_sp == (short)(short) -4236);
                Debug.Assert(pack.heading == (ushort)(ushort)13494);
                Debug.Assert(pack.airspeed == (byte)(byte)206);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
                Debug.Assert(pack.temperature == (sbyte)(sbyte) - 33);
                Debug.Assert(pack.roll == (short)(short)11695);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte) - 93);
                Debug.Assert(pack.groundspeed == (byte)(byte)147);
                Debug.Assert(pack.altitude_amsl == (short)(short) -23373);
                Debug.Assert(pack.gps_fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
            };
            DemoDevice.HIGH_LATENCY p234 = LoopBackDemoChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.failsafe = (byte)(byte)99;
            p234.climb_rate = (sbyte)(sbyte) - 7;
            p234.custom_mode = (uint)3632531296U;
            p234.gps_nsat = (byte)(byte)119;
            p234.longitude = (int) -1772833602;
            p234.airspeed_sp = (byte)(byte)203;
            p234.wp_num = (byte)(byte)253;
            p234.wp_distance = (ushort)(ushort)65353;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF;
            p234.temperature_air = (sbyte)(sbyte) - 93;
            p234.heading_sp = (short)(short) -4236;
            p234.roll = (short)(short)11695;
            p234.throttle = (sbyte)(sbyte)44;
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED;
            p234.latitude = (int)355348889;
            p234.pitch = (short)(short)27968;
            p234.battery_remaining = (byte)(byte)194;
            p234.altitude_sp = (short)(short) -20913;
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p234.heading = (ushort)(ushort)13494;
            p234.altitude_amsl = (short)(short) -23373;
            p234.temperature = (sbyte)(sbyte) - 33;
            p234.groundspeed = (byte)(byte)147;
            p234.airspeed = (byte)(byte)206;
            LoopBackDemoChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.clipping_0 == (uint)437729644U);
                Debug.Assert(pack.clipping_1 == (uint)4267201662U);
                Debug.Assert(pack.vibration_y == (float)2.4867194E38F);
                Debug.Assert(pack.vibration_z == (float) -3.2099457E38F);
                Debug.Assert(pack.clipping_2 == (uint)2085016860U);
                Debug.Assert(pack.time_usec == (ulong)5988897472876586025L);
                Debug.Assert(pack.vibration_x == (float)1.7741421E38F);
            };
            DemoDevice.VIBRATION p241 = LoopBackDemoChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.clipping_1 = (uint)4267201662U;
            p241.vibration_z = (float) -3.2099457E38F;
            p241.time_usec = (ulong)5988897472876586025L;
            p241.clipping_2 = (uint)2085016860U;
            p241.clipping_0 = (uint)437729644U;
            p241.vibration_x = (float)1.7741421E38F;
            p241.vibration_y = (float)2.4867194E38F;
            LoopBackDemoChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.approach_z == (float) -2.6703417E38F);
                Debug.Assert(pack.altitude == (int)1144052387);
                Debug.Assert(pack.y == (float) -1.5923722E37F);
                Debug.Assert(pack.longitude == (int) -1979346079);
                Debug.Assert(pack.approach_x == (float)3.0256631E38F);
                Debug.Assert(pack.z == (float)1.7441828E38F);
                Debug.Assert(pack.x == (float) -2.0737953E38F);
                Debug.Assert(pack.latitude == (int) -1955197601);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.6399423E35F, -2.9453886E38F, -2.1476356E38F, 2.880545E38F}));
                Debug.Assert(pack.approach_y == (float) -4.5837916E37F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)3801211118824938620L);
            };
            DemoDevice.HOME_POSITION p242 = LoopBackDemoChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.x = (float) -2.0737953E38F;
            p242.approach_y = (float) -4.5837916E37F;
            p242.approach_z = (float) -2.6703417E38F;
            p242.altitude = (int)1144052387;
            p242.q_SET(new float[] {1.6399423E35F, -2.9453886E38F, -2.1476356E38F, 2.880545E38F}, 0) ;
            p242.approach_x = (float)3.0256631E38F;
            p242.y = (float) -1.5923722E37F;
            p242.latitude = (int) -1955197601;
            p242.time_usec_SET((ulong)3801211118824938620L, PH) ;
            p242.z = (float)1.7441828E38F;
            p242.longitude = (int) -1979346079;
            LoopBackDemoChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)28090268274213867L);
                Debug.Assert(pack.target_system == (byte)(byte)107);
                Debug.Assert(pack.altitude == (int) -815144496);
                Debug.Assert(pack.z == (float) -6.3206215E37F);
                Debug.Assert(pack.longitude == (int) -1563116801);
                Debug.Assert(pack.latitude == (int)1906826688);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.3023572E37F, 4.7103386E37F, 1.5884274E38F, 2.0703404E37F}));
                Debug.Assert(pack.approach_y == (float) -1.5844403E37F);
                Debug.Assert(pack.approach_z == (float)7.101922E37F);
                Debug.Assert(pack.y == (float)2.0457453E38F);
                Debug.Assert(pack.x == (float) -1.5538254E38F);
                Debug.Assert(pack.approach_x == (float) -1.9746177E38F);
            };
            DemoDevice.SET_HOME_POSITION p243 = LoopBackDemoChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)107;
            p243.y = (float)2.0457453E38F;
            p243.latitude = (int)1906826688;
            p243.approach_x = (float) -1.9746177E38F;
            p243.x = (float) -1.5538254E38F;
            p243.longitude = (int) -1563116801;
            p243.q_SET(new float[] {-2.3023572E37F, 4.7103386E37F, 1.5884274E38F, 2.0703404E37F}, 0) ;
            p243.time_usec_SET((ulong)28090268274213867L, PH) ;
            p243.approach_z = (float)7.101922E37F;
            p243.approach_y = (float) -1.5844403E37F;
            p243.altitude = (int) -815144496;
            p243.z = (float) -6.3206215E37F;
            LoopBackDemoChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_id == (ushort)(ushort)17449);
                Debug.Assert(pack.interval_us == (int)189093996);
            };
            DemoDevice.MESSAGE_INTERVAL p244 = LoopBackDemoChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.interval_us = (int)189093996;
            p244.message_id = (ushort)(ushort)17449;
            LoopBackDemoChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vtol_state == (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
            };
            DemoDevice.EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING;
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC;
            LoopBackDemoChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)8888);
                Debug.Assert(pack.lon == (int) -1369581028);
                Debug.Assert(pack.altitude == (int)815996777);
                Debug.Assert(pack.heading == (ushort)(ushort)19888);
                Debug.Assert(pack.flags == (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK);
                Debug.Assert(pack.callsign_LEN(ph) == 4);
                Debug.Assert(pack.callsign_TRY(ph).Equals("vlmc"));
                Debug.Assert(pack.altitude_type == (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
                Debug.Assert(pack.lat == (int) -1141432013);
                Debug.Assert(pack.tslc == (byte)(byte)174);
                Debug.Assert(pack.emitter_type == (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_GLIDER);
                Debug.Assert(pack.squawk == (ushort)(ushort)5473);
                Debug.Assert(pack.ICAO_address == (uint)2231314461U);
                Debug.Assert(pack.ver_velocity == (short)(short) -31442);
            };
            DemoDevice.ADSB_VEHICLE p246 = LoopBackDemoChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK;
            p246.squawk = (ushort)(ushort)5473;
            p246.altitude = (int)815996777;
            p246.ver_velocity = (short)(short) -31442;
            p246.ICAO_address = (uint)2231314461U;
            p246.heading = (ushort)(ushort)19888;
            p246.lat = (int) -1141432013;
            p246.callsign_SET("vlmc", PH) ;
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC;
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_GLIDER;
            p246.hor_velocity = (ushort)(ushort)8888;
            p246.lon = (int) -1369581028;
            p246.tslc = (byte)(byte)174;
            LoopBackDemoChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_to_minimum_delta == (float) -1.2996396E38F);
                Debug.Assert(pack.altitude_minimum_delta == (float)2.25204E38F);
                Debug.Assert(pack.threat_level == (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
                Debug.Assert(pack.id == (uint)1656197120U);
                Debug.Assert(pack.horizontal_minimum_delta == (float) -5.22562E37F);
                Debug.Assert(pack.src_ == (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
                Debug.Assert(pack.action == (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER);
            };
            DemoDevice.COLLISION p247 = LoopBackDemoChannel.new_COLLISION();
            PH.setPack(p247);
            p247.horizontal_minimum_delta = (float) -5.22562E37F;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH;
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER;
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.altitude_minimum_delta = (float)2.25204E38F;
            p247.id = (uint)1656197120U;
            p247.time_to_minimum_delta = (float) -1.2996396E38F;
            LoopBackDemoChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_network == (byte)(byte)40);
                Debug.Assert(pack.target_system == (byte)(byte)121);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)199, (byte)4, (byte)73, (byte)15, (byte)235, (byte)211, (byte)152, (byte)211, (byte)15, (byte)137, (byte)227, (byte)127, (byte)170, (byte)186, (byte)28, (byte)182, (byte)96, (byte)8, (byte)145, (byte)166, (byte)231, (byte)23, (byte)4, (byte)89, (byte)129, (byte)124, (byte)141, (byte)235, (byte)124, (byte)141, (byte)95, (byte)235, (byte)58, (byte)182, (byte)11, (byte)80, (byte)127, (byte)252, (byte)216, (byte)185, (byte)215, (byte)121, (byte)132, (byte)179, (byte)126, (byte)159, (byte)56, (byte)133, (byte)27, (byte)98, (byte)175, (byte)169, (byte)100, (byte)179, (byte)168, (byte)32, (byte)16, (byte)135, (byte)66, (byte)18, (byte)162, (byte)215, (byte)26, (byte)124, (byte)6, (byte)26, (byte)169, (byte)70, (byte)109, (byte)241, (byte)250, (byte)11, (byte)211, (byte)246, (byte)41, (byte)247, (byte)91, (byte)41, (byte)6, (byte)134, (byte)165, (byte)9, (byte)157, (byte)26, (byte)21, (byte)78, (byte)185, (byte)175, (byte)164, (byte)228, (byte)31, (byte)56, (byte)100, (byte)99, (byte)173, (byte)225, (byte)214, (byte)45, (byte)130, (byte)107, (byte)154, (byte)110, (byte)111, (byte)233, (byte)242, (byte)80, (byte)137, (byte)190, (byte)16, (byte)199, (byte)120, (byte)229, (byte)72, (byte)215, (byte)19, (byte)74, (byte)179, (byte)152, (byte)161, (byte)19, (byte)140, (byte)235, (byte)43, (byte)176, (byte)181, (byte)219, (byte)119, (byte)141, (byte)155, (byte)187, (byte)30, (byte)48, (byte)9, (byte)200, (byte)107, (byte)8, (byte)35, (byte)59, (byte)217, (byte)111, (byte)255, (byte)170, (byte)193, (byte)61, (byte)242, (byte)136, (byte)241, (byte)199, (byte)205, (byte)111, (byte)77, (byte)178, (byte)101, (byte)47, (byte)24, (byte)155, (byte)175, (byte)44, (byte)227, (byte)37, (byte)6, (byte)233, (byte)242, (byte)254, (byte)102, (byte)43, (byte)231, (byte)152, (byte)224, (byte)254, (byte)244, (byte)28, (byte)159, (byte)136, (byte)31, (byte)133, (byte)148, (byte)175, (byte)110, (byte)59, (byte)54, (byte)226, (byte)225, (byte)142, (byte)113, (byte)43, (byte)194, (byte)202, (byte)10, (byte)33, (byte)144, (byte)196, (byte)55, (byte)237, (byte)23, (byte)203, (byte)106, (byte)146, (byte)236, (byte)216, (byte)8, (byte)190, (byte)122, (byte)105, (byte)187, (byte)132, (byte)103, (byte)128, (byte)155, (byte)44, (byte)28, (byte)61, (byte)151, (byte)186, (byte)69, (byte)14, (byte)147, (byte)25, (byte)76, (byte)111, (byte)16, (byte)68, (byte)13, (byte)165, (byte)191, (byte)96, (byte)192, (byte)167, (byte)70, (byte)56, (byte)4, (byte)28, (byte)84, (byte)95, (byte)104, (byte)197, (byte)218, (byte)22, (byte)48, (byte)79, (byte)164, (byte)209, (byte)114, (byte)95, (byte)171, (byte)157, (byte)244, (byte)44, (byte)151}));
                Debug.Assert(pack.message_type == (ushort)(ushort)36175);
                Debug.Assert(pack.target_component == (byte)(byte)25);
            };
            DemoDevice.V2_EXTENSION p248 = LoopBackDemoChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_component = (byte)(byte)25;
            p248.target_system = (byte)(byte)121;
            p248.target_network = (byte)(byte)40;
            p248.payload_SET(new byte[] {(byte)199, (byte)4, (byte)73, (byte)15, (byte)235, (byte)211, (byte)152, (byte)211, (byte)15, (byte)137, (byte)227, (byte)127, (byte)170, (byte)186, (byte)28, (byte)182, (byte)96, (byte)8, (byte)145, (byte)166, (byte)231, (byte)23, (byte)4, (byte)89, (byte)129, (byte)124, (byte)141, (byte)235, (byte)124, (byte)141, (byte)95, (byte)235, (byte)58, (byte)182, (byte)11, (byte)80, (byte)127, (byte)252, (byte)216, (byte)185, (byte)215, (byte)121, (byte)132, (byte)179, (byte)126, (byte)159, (byte)56, (byte)133, (byte)27, (byte)98, (byte)175, (byte)169, (byte)100, (byte)179, (byte)168, (byte)32, (byte)16, (byte)135, (byte)66, (byte)18, (byte)162, (byte)215, (byte)26, (byte)124, (byte)6, (byte)26, (byte)169, (byte)70, (byte)109, (byte)241, (byte)250, (byte)11, (byte)211, (byte)246, (byte)41, (byte)247, (byte)91, (byte)41, (byte)6, (byte)134, (byte)165, (byte)9, (byte)157, (byte)26, (byte)21, (byte)78, (byte)185, (byte)175, (byte)164, (byte)228, (byte)31, (byte)56, (byte)100, (byte)99, (byte)173, (byte)225, (byte)214, (byte)45, (byte)130, (byte)107, (byte)154, (byte)110, (byte)111, (byte)233, (byte)242, (byte)80, (byte)137, (byte)190, (byte)16, (byte)199, (byte)120, (byte)229, (byte)72, (byte)215, (byte)19, (byte)74, (byte)179, (byte)152, (byte)161, (byte)19, (byte)140, (byte)235, (byte)43, (byte)176, (byte)181, (byte)219, (byte)119, (byte)141, (byte)155, (byte)187, (byte)30, (byte)48, (byte)9, (byte)200, (byte)107, (byte)8, (byte)35, (byte)59, (byte)217, (byte)111, (byte)255, (byte)170, (byte)193, (byte)61, (byte)242, (byte)136, (byte)241, (byte)199, (byte)205, (byte)111, (byte)77, (byte)178, (byte)101, (byte)47, (byte)24, (byte)155, (byte)175, (byte)44, (byte)227, (byte)37, (byte)6, (byte)233, (byte)242, (byte)254, (byte)102, (byte)43, (byte)231, (byte)152, (byte)224, (byte)254, (byte)244, (byte)28, (byte)159, (byte)136, (byte)31, (byte)133, (byte)148, (byte)175, (byte)110, (byte)59, (byte)54, (byte)226, (byte)225, (byte)142, (byte)113, (byte)43, (byte)194, (byte)202, (byte)10, (byte)33, (byte)144, (byte)196, (byte)55, (byte)237, (byte)23, (byte)203, (byte)106, (byte)146, (byte)236, (byte)216, (byte)8, (byte)190, (byte)122, (byte)105, (byte)187, (byte)132, (byte)103, (byte)128, (byte)155, (byte)44, (byte)28, (byte)61, (byte)151, (byte)186, (byte)69, (byte)14, (byte)147, (byte)25, (byte)76, (byte)111, (byte)16, (byte)68, (byte)13, (byte)165, (byte)191, (byte)96, (byte)192, (byte)167, (byte)70, (byte)56, (byte)4, (byte)28, (byte)84, (byte)95, (byte)104, (byte)197, (byte)218, (byte)22, (byte)48, (byte)79, (byte)164, (byte)209, (byte)114, (byte)95, (byte)171, (byte)157, (byte)244, (byte)44, (byte)151}, 0) ;
            p248.message_type = (ushort)(ushort)36175;
            LoopBackDemoChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ver == (byte)(byte)77);
                Debug.Assert(pack.address == (ushort)(ushort)13827);
                Debug.Assert(pack.type == (byte)(byte)35);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte)0, (sbyte)76, (sbyte)1, (sbyte)92, (sbyte) - 114, (sbyte)30, (sbyte) - 15, (sbyte)61, (sbyte)118, (sbyte) - 24, (sbyte) - 58, (sbyte) - 5, (sbyte) - 127, (sbyte) - 38, (sbyte) - 29, (sbyte)42, (sbyte) - 93, (sbyte)88, (sbyte) - 1, (sbyte) - 18, (sbyte) - 20, (sbyte)97, (sbyte) - 8, (sbyte)29, (sbyte) - 95, (sbyte) - 43, (sbyte) - 80, (sbyte)44, (sbyte) - 74, (sbyte)50, (sbyte) - 18, (sbyte)78}));
            };
            DemoDevice.MEMORY_VECT p249 = LoopBackDemoChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.value_SET(new sbyte[] {(sbyte)0, (sbyte)76, (sbyte)1, (sbyte)92, (sbyte) - 114, (sbyte)30, (sbyte) - 15, (sbyte)61, (sbyte)118, (sbyte) - 24, (sbyte) - 58, (sbyte) - 5, (sbyte) - 127, (sbyte) - 38, (sbyte) - 29, (sbyte)42, (sbyte) - 93, (sbyte)88, (sbyte) - 1, (sbyte) - 18, (sbyte) - 20, (sbyte)97, (sbyte) - 8, (sbyte)29, (sbyte) - 95, (sbyte) - 43, (sbyte) - 80, (sbyte)44, (sbyte) - 74, (sbyte)50, (sbyte) - 18, (sbyte)78}, 0) ;
            p249.type = (byte)(byte)35;
            p249.ver = (byte)(byte)77;
            p249.address = (ushort)(ushort)13827;
            LoopBackDemoChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -7.9432305E37F);
                Debug.Assert(pack.x == (float) -2.7888504E38F);
                Debug.Assert(pack.name_LEN(ph) == 6);
                Debug.Assert(pack.name_TRY(ph).Equals("vbvpJi"));
                Debug.Assert(pack.time_usec == (ulong)5324619289022684939L);
                Debug.Assert(pack.y == (float)3.2088052E38F);
            };
            DemoDevice.DEBUG_VECT p250 = LoopBackDemoChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.y = (float)3.2088052E38F;
            p250.z = (float) -7.9432305E37F;
            p250.time_usec = (ulong)5324619289022684939L;
            p250.name_SET("vbvpJi", PH) ;
            p250.x = (float) -2.7888504E38F;
            LoopBackDemoChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 2);
                Debug.Assert(pack.name_TRY(ph).Equals("ti"));
                Debug.Assert(pack.value == (float)1.191345E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2863000045U);
            };
            DemoDevice.NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)2863000045U;
            p251.value = (float)1.191345E38F;
            p251.name_SET("ti", PH) ;
            LoopBackDemoChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3626177099U);
                Debug.Assert(pack.value == (int)258217945);
                Debug.Assert(pack.name_LEN(ph) == 6);
                Debug.Assert(pack.name_TRY(ph).Equals("ukimfs"));
            };
            DemoDevice.NAMED_VALUE_INT p252 = LoopBackDemoChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.name_SET("ukimfs", PH) ;
            p252.value = (int)258217945;
            p252.time_boot_ms = (uint)3626177099U;
            LoopBackDemoChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.text_LEN(ph) == 43);
                Debug.Assert(pack.text_TRY(ph).Equals("jsdcialearrnmjbbwoixwTolwgkHdQXdrkWfoatdcry"));
                Debug.Assert(pack.severity == (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_DEBUG);
            };
            DemoDevice.STATUSTEXT p253 = LoopBackDemoChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.text_SET("jsdcialearrnmjbbwoixwTolwgkHdQXdrkWfoatdcry", PH) ;
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_DEBUG;
            LoopBackDemoChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ind == (byte)(byte)147);
                Debug.Assert(pack.time_boot_ms == (uint)3938885244U);
                Debug.Assert(pack.value == (float) -5.895687E37F);
            };
            DemoDevice.DEBUG p254 = LoopBackDemoChannel.new_DEBUG();
            PH.setPack(p254);
            p254.value = (float) -5.895687E37F;
            p254.ind = (byte)(byte)147;
            p254.time_boot_ms = (uint)3938885244U;
            LoopBackDemoChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)133);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)137, (byte)164, (byte)187, (byte)79, (byte)105, (byte)140, (byte)14, (byte)150, (byte)254, (byte)217, (byte)96, (byte)141, (byte)175, (byte)233, (byte)94, (byte)253, (byte)130, (byte)159, (byte)47, (byte)107, (byte)52, (byte)11, (byte)116, (byte)13, (byte)223, (byte)93, (byte)185, (byte)67, (byte)233, (byte)71, (byte)88, (byte)9}));
                Debug.Assert(pack.target_component == (byte)(byte)185);
                Debug.Assert(pack.initial_timestamp == (ulong)5713235369640448721L);
            };
            DemoDevice.SETUP_SIGNING p256 = LoopBackDemoChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_component = (byte)(byte)185;
            p256.initial_timestamp = (ulong)5713235369640448721L;
            p256.secret_key_SET(new byte[] {(byte)137, (byte)164, (byte)187, (byte)79, (byte)105, (byte)140, (byte)14, (byte)150, (byte)254, (byte)217, (byte)96, (byte)141, (byte)175, (byte)233, (byte)94, (byte)253, (byte)130, (byte)159, (byte)47, (byte)107, (byte)52, (byte)11, (byte)116, (byte)13, (byte)223, (byte)93, (byte)185, (byte)67, (byte)233, (byte)71, (byte)88, (byte)9}, 0) ;
            p256.target_system = (byte)(byte)133;
            LoopBackDemoChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3908845129U);
                Debug.Assert(pack.last_change_ms == (uint)1907734437U);
                Debug.Assert(pack.state == (byte)(byte)129);
            };
            DemoDevice.BUTTON_CHANGE p257 = LoopBackDemoChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)3908845129U;
            p257.last_change_ms = (uint)1907734437U;
            p257.state = (byte)(byte)129;
            LoopBackDemoChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tune_LEN(ph) == 26);
                Debug.Assert(pack.tune_TRY(ph).Equals("mbyhbiInlpugmwdaffxsvegwvh"));
                Debug.Assert(pack.target_system == (byte)(byte)20);
                Debug.Assert(pack.target_component == (byte)(byte)222);
            };
            DemoDevice.PLAY_TUNE p258 = LoopBackDemoChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)20;
            p258.tune_SET("mbyhbiInlpugmwdaffxsvegwvh", PH) ;
            p258.target_component = (byte)(byte)222;
            LoopBackDemoChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.resolution_h == (ushort)(ushort)21220);
                Debug.Assert(pack.focal_length == (float)5.5167835E37F);
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)62, (byte)225, (byte)129, (byte)79, (byte)236, (byte)247, (byte)87, (byte)62, (byte)119, (byte)148, (byte)17, (byte)53, (byte)230, (byte)225, (byte)96, (byte)7, (byte)7, (byte)55, (byte)138, (byte)98, (byte)131, (byte)206, (byte)31, (byte)15, (byte)115, (byte)198, (byte)114, (byte)116, (byte)207, (byte)79, (byte)35, (byte)52}));
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 134);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("msCjZuxeixvcOmiwtuJrtijufszTylsqphaosUkxdbhidGlviezubzdrwvephugtuytbtzpumbngdqbscazxpdAVecVFuegDcdoisnaxrulbtibomsjgpjzFdagtsfqxDbofki"));
                Debug.Assert(pack.time_boot_ms == (uint)3032572708U);
                Debug.Assert(pack.firmware_version == (uint)2710433380U);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)64904);
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO);
                Debug.Assert(pack.lens_id == (byte)(byte)79);
                Debug.Assert(pack.sensor_size_v == (float)2.694488E38F);
                Debug.Assert(pack.sensor_size_h == (float) -2.3920662E38F);
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)159, (byte)188, (byte)71, (byte)191, (byte)134, (byte)176, (byte)59, (byte)165, (byte)175, (byte)104, (byte)189, (byte)138, (byte)120, (byte)97, (byte)34, (byte)209, (byte)213, (byte)44, (byte)178, (byte)95, (byte)63, (byte)5, (byte)147, (byte)132, (byte)115, (byte)20, (byte)24, (byte)83, (byte)251, (byte)213, (byte)215, (byte)162}));
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)31508);
            };
            DemoDevice.CAMERA_INFORMATION p259 = LoopBackDemoChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.sensor_size_v = (float)2.694488E38F;
            p259.cam_definition_version = (ushort)(ushort)31508;
            p259.resolution_h = (ushort)(ushort)21220;
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO;
            p259.vendor_name_SET(new byte[] {(byte)159, (byte)188, (byte)71, (byte)191, (byte)134, (byte)176, (byte)59, (byte)165, (byte)175, (byte)104, (byte)189, (byte)138, (byte)120, (byte)97, (byte)34, (byte)209, (byte)213, (byte)44, (byte)178, (byte)95, (byte)63, (byte)5, (byte)147, (byte)132, (byte)115, (byte)20, (byte)24, (byte)83, (byte)251, (byte)213, (byte)215, (byte)162}, 0) ;
            p259.firmware_version = (uint)2710433380U;
            p259.model_name_SET(new byte[] {(byte)62, (byte)225, (byte)129, (byte)79, (byte)236, (byte)247, (byte)87, (byte)62, (byte)119, (byte)148, (byte)17, (byte)53, (byte)230, (byte)225, (byte)96, (byte)7, (byte)7, (byte)55, (byte)138, (byte)98, (byte)131, (byte)206, (byte)31, (byte)15, (byte)115, (byte)198, (byte)114, (byte)116, (byte)207, (byte)79, (byte)35, (byte)52}, 0) ;
            p259.resolution_v = (ushort)(ushort)64904;
            p259.lens_id = (byte)(byte)79;
            p259.time_boot_ms = (uint)3032572708U;
            p259.focal_length = (float)5.5167835E37F;
            p259.cam_definition_uri_SET("msCjZuxeixvcOmiwtuJrtijufszTylsqphaosUkxdbhidGlviezubzdrwvephugtuytbtzpumbngdqbscazxpdAVecVFuegDcdoisnaxrulbtibomsjgpjzFdagtsfqxDbofki", PH) ;
            p259.sensor_size_h = (float) -2.3920662E38F;
            LoopBackDemoChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode_id == (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE);
                Debug.Assert(pack.time_boot_ms == (uint)2477245491U);
            };
            DemoDevice.CAMERA_SETTINGS p260 = LoopBackDemoChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE;
            p260.time_boot_ms = (uint)2477245491U;
            LoopBackDemoChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.status == (byte)(byte)196);
                Debug.Assert(pack.available_capacity == (float) -1.7705945E38F);
                Debug.Assert(pack.total_capacity == (float)2.926153E38F);
                Debug.Assert(pack.read_speed == (float) -2.1388093E38F);
                Debug.Assert(pack.storage_id == (byte)(byte)123);
                Debug.Assert(pack.used_capacity == (float)2.5295432E38F);
                Debug.Assert(pack.write_speed == (float)1.7778915E38F);
                Debug.Assert(pack.storage_count == (byte)(byte)49);
                Debug.Assert(pack.time_boot_ms == (uint)1367326537U);
            };
            DemoDevice.STORAGE_INFORMATION p261 = LoopBackDemoChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.write_speed = (float)1.7778915E38F;
            p261.time_boot_ms = (uint)1367326537U;
            p261.available_capacity = (float) -1.7705945E38F;
            p261.read_speed = (float) -2.1388093E38F;
            p261.used_capacity = (float)2.5295432E38F;
            p261.total_capacity = (float)2.926153E38F;
            p261.storage_count = (byte)(byte)49;
            p261.storage_id = (byte)(byte)123;
            p261.status = (byte)(byte)196;
            LoopBackDemoChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.video_status == (byte)(byte)153);
                Debug.Assert(pack.image_interval == (float) -3.952667E37F);
                Debug.Assert(pack.available_capacity == (float)6.4394663E37F);
                Debug.Assert(pack.time_boot_ms == (uint)4236322847U);
                Debug.Assert(pack.recording_time_ms == (uint)1236169423U);
                Debug.Assert(pack.image_status == (byte)(byte)111);
            };
            DemoDevice.CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.image_interval = (float) -3.952667E37F;
            p262.available_capacity = (float)6.4394663E37F;
            p262.time_boot_ms = (uint)4236322847U;
            p262.image_status = (byte)(byte)111;
            p262.recording_time_ms = (uint)1236169423U;
            p262.video_status = (byte)(byte)153;
            LoopBackDemoChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2303311285U);
                Debug.Assert(pack.time_utc == (ulong)8731627133774516999L);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.3487979E38F, -3.859775E37F, -1.1836934E38F, -2.1544184E38F}));
                Debug.Assert(pack.image_index == (int)514999311);
                Debug.Assert(pack.lat == (int) -1645261040);
                Debug.Assert(pack.file_url_LEN(ph) == 167);
                Debug.Assert(pack.file_url_TRY(ph).Equals("cpvXwxwlmxDauvMrjnnkxtcvEaemavucyWlcmaMketnnixebwasBlnehvhixyposceckcdttcjpnzzrnavemNLjgcsqlzkaguweDdqyBapoldhbkfsAbheeglxztqjlicNjkbNFCbMwalybyxnmhqizvcsdvbcrkMttvdca"));
                Debug.Assert(pack.relative_alt == (int)2032641858);
                Debug.Assert(pack.camera_id == (byte)(byte)149);
                Debug.Assert(pack.capture_result == (sbyte)(sbyte)126);
                Debug.Assert(pack.alt == (int) -612642202);
                Debug.Assert(pack.lon == (int) -965767246);
            };
            DemoDevice.CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.file_url_SET("cpvXwxwlmxDauvMrjnnkxtcvEaemavucyWlcmaMketnnixebwasBlnehvhixyposceckcdttcjpnzzrnavemNLjgcsqlzkaguweDdqyBapoldhbkfsAbheeglxztqjlicNjkbNFCbMwalybyxnmhqizvcsdvbcrkMttvdca", PH) ;
            p263.camera_id = (byte)(byte)149;
            p263.capture_result = (sbyte)(sbyte)126;
            p263.q_SET(new float[] {1.3487979E38F, -3.859775E37F, -1.1836934E38F, -2.1544184E38F}, 0) ;
            p263.lon = (int) -965767246;
            p263.time_utc = (ulong)8731627133774516999L;
            p263.image_index = (int)514999311;
            p263.relative_alt = (int)2032641858;
            p263.time_boot_ms = (uint)2303311285U;
            p263.alt = (int) -612642202;
            p263.lat = (int) -1645261040;
            LoopBackDemoChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flight_uuid == (ulong)4576907232502446515L);
                Debug.Assert(pack.arming_time_utc == (ulong)8751778258072937615L);
                Debug.Assert(pack.takeoff_time_utc == (ulong)30374733324786731L);
                Debug.Assert(pack.time_boot_ms == (uint)1987315688U);
            };
            DemoDevice.FLIGHT_INFORMATION p264 = LoopBackDemoChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.arming_time_utc = (ulong)8751778258072937615L;
            p264.time_boot_ms = (uint)1987315688U;
            p264.takeoff_time_utc = (ulong)30374733324786731L;
            p264.flight_uuid = (ulong)4576907232502446515L;
            LoopBackDemoChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float) -3.289186E38F);
                Debug.Assert(pack.yaw == (float) -5.557393E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3880381771U);
                Debug.Assert(pack.roll == (float)2.9196894E38F);
            };
            DemoDevice.MOUNT_ORIENTATION p265 = LoopBackDemoChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.roll = (float)2.9196894E38F;
            p265.pitch = (float) -3.289186E38F;
            p265.time_boot_ms = (uint)3880381771U;
            p265.yaw = (float) -5.557393E37F;
            LoopBackDemoChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)231);
                Debug.Assert(pack.first_message_offset == (byte)(byte)170);
                Debug.Assert(pack.length == (byte)(byte)61);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)173, (byte)110, (byte)205, (byte)195, (byte)34, (byte)177, (byte)160, (byte)195, (byte)170, (byte)24, (byte)251, (byte)226, (byte)172, (byte)64, (byte)67, (byte)115, (byte)14, (byte)1, (byte)194, (byte)206, (byte)172, (byte)167, (byte)108, (byte)251, (byte)3, (byte)209, (byte)247, (byte)81, (byte)23, (byte)29, (byte)210, (byte)183, (byte)237, (byte)149, (byte)156, (byte)203, (byte)88, (byte)226, (byte)17, (byte)65, (byte)69, (byte)218, (byte)187, (byte)210, (byte)213, (byte)229, (byte)22, (byte)70, (byte)48, (byte)168, (byte)181, (byte)169, (byte)33, (byte)221, (byte)107, (byte)49, (byte)201, (byte)227, (byte)57, (byte)6, (byte)189, (byte)15, (byte)111, (byte)51, (byte)17, (byte)39, (byte)92, (byte)56, (byte)37, (byte)32, (byte)175, (byte)145, (byte)142, (byte)148, (byte)76, (byte)5, (byte)15, (byte)157, (byte)153, (byte)158, (byte)155, (byte)191, (byte)56, (byte)192, (byte)162, (byte)212, (byte)40, (byte)162, (byte)136, (byte)187, (byte)83, (byte)215, (byte)19, (byte)10, (byte)163, (byte)139, (byte)172, (byte)254, (byte)244, (byte)113, (byte)167, (byte)6, (byte)163, (byte)24, (byte)191, (byte)231, (byte)136, (byte)245, (byte)160, (byte)48, (byte)91, (byte)101, (byte)102, (byte)215, (byte)102, (byte)178, (byte)207, (byte)83, (byte)82, (byte)146, (byte)236, (byte)91, (byte)77, (byte)119, (byte)74, (byte)96, (byte)234, (byte)77, (byte)216, (byte)166, (byte)18, (byte)224, (byte)193, (byte)118, (byte)204, (byte)38, (byte)166, (byte)118, (byte)117, (byte)0, (byte)172, (byte)160, (byte)87, (byte)72, (byte)89, (byte)114, (byte)178, (byte)192, (byte)3, (byte)48, (byte)75, (byte)244, (byte)253, (byte)25, (byte)213, (byte)195, (byte)80, (byte)235, (byte)35, (byte)177, (byte)136, (byte)97, (byte)93, (byte)191, (byte)165, (byte)203, (byte)160, (byte)12, (byte)133, (byte)82, (byte)249, (byte)194, (byte)196, (byte)209, (byte)84, (byte)196, (byte)177, (byte)210, (byte)40, (byte)242, (byte)98, (byte)32, (byte)29, (byte)171, (byte)241, (byte)211, (byte)61, (byte)62, (byte)214, (byte)79, (byte)116, (byte)18, (byte)161, (byte)173, (byte)46, (byte)178, (byte)193, (byte)215, (byte)45, (byte)135, (byte)93, (byte)83, (byte)156, (byte)70, (byte)32, (byte)218, (byte)145, (byte)44, (byte)196, (byte)84, (byte)69, (byte)162, (byte)116, (byte)108, (byte)53, (byte)187, (byte)140, (byte)137, (byte)7, (byte)57, (byte)103, (byte)47, (byte)225, (byte)14, (byte)127, (byte)9, (byte)206, (byte)234, (byte)27, (byte)124, (byte)38, (byte)165, (byte)163, (byte)46, (byte)133, (byte)168, (byte)2, (byte)99, (byte)237, (byte)146, (byte)44, (byte)172, (byte)220, (byte)100, (byte)250, (byte)35, (byte)34, (byte)31, (byte)173}));
                Debug.Assert(pack.target_component == (byte)(byte)136);
                Debug.Assert(pack.sequence == (ushort)(ushort)38671);
            };
            DemoDevice.LOGGING_DATA p266 = LoopBackDemoChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_component = (byte)(byte)136;
            p266.sequence = (ushort)(ushort)38671;
            p266.target_system = (byte)(byte)231;
            p266.length = (byte)(byte)61;
            p266.first_message_offset = (byte)(byte)170;
            p266.data__SET(new byte[] {(byte)173, (byte)110, (byte)205, (byte)195, (byte)34, (byte)177, (byte)160, (byte)195, (byte)170, (byte)24, (byte)251, (byte)226, (byte)172, (byte)64, (byte)67, (byte)115, (byte)14, (byte)1, (byte)194, (byte)206, (byte)172, (byte)167, (byte)108, (byte)251, (byte)3, (byte)209, (byte)247, (byte)81, (byte)23, (byte)29, (byte)210, (byte)183, (byte)237, (byte)149, (byte)156, (byte)203, (byte)88, (byte)226, (byte)17, (byte)65, (byte)69, (byte)218, (byte)187, (byte)210, (byte)213, (byte)229, (byte)22, (byte)70, (byte)48, (byte)168, (byte)181, (byte)169, (byte)33, (byte)221, (byte)107, (byte)49, (byte)201, (byte)227, (byte)57, (byte)6, (byte)189, (byte)15, (byte)111, (byte)51, (byte)17, (byte)39, (byte)92, (byte)56, (byte)37, (byte)32, (byte)175, (byte)145, (byte)142, (byte)148, (byte)76, (byte)5, (byte)15, (byte)157, (byte)153, (byte)158, (byte)155, (byte)191, (byte)56, (byte)192, (byte)162, (byte)212, (byte)40, (byte)162, (byte)136, (byte)187, (byte)83, (byte)215, (byte)19, (byte)10, (byte)163, (byte)139, (byte)172, (byte)254, (byte)244, (byte)113, (byte)167, (byte)6, (byte)163, (byte)24, (byte)191, (byte)231, (byte)136, (byte)245, (byte)160, (byte)48, (byte)91, (byte)101, (byte)102, (byte)215, (byte)102, (byte)178, (byte)207, (byte)83, (byte)82, (byte)146, (byte)236, (byte)91, (byte)77, (byte)119, (byte)74, (byte)96, (byte)234, (byte)77, (byte)216, (byte)166, (byte)18, (byte)224, (byte)193, (byte)118, (byte)204, (byte)38, (byte)166, (byte)118, (byte)117, (byte)0, (byte)172, (byte)160, (byte)87, (byte)72, (byte)89, (byte)114, (byte)178, (byte)192, (byte)3, (byte)48, (byte)75, (byte)244, (byte)253, (byte)25, (byte)213, (byte)195, (byte)80, (byte)235, (byte)35, (byte)177, (byte)136, (byte)97, (byte)93, (byte)191, (byte)165, (byte)203, (byte)160, (byte)12, (byte)133, (byte)82, (byte)249, (byte)194, (byte)196, (byte)209, (byte)84, (byte)196, (byte)177, (byte)210, (byte)40, (byte)242, (byte)98, (byte)32, (byte)29, (byte)171, (byte)241, (byte)211, (byte)61, (byte)62, (byte)214, (byte)79, (byte)116, (byte)18, (byte)161, (byte)173, (byte)46, (byte)178, (byte)193, (byte)215, (byte)45, (byte)135, (byte)93, (byte)83, (byte)156, (byte)70, (byte)32, (byte)218, (byte)145, (byte)44, (byte)196, (byte)84, (byte)69, (byte)162, (byte)116, (byte)108, (byte)53, (byte)187, (byte)140, (byte)137, (byte)7, (byte)57, (byte)103, (byte)47, (byte)225, (byte)14, (byte)127, (byte)9, (byte)206, (byte)234, (byte)27, (byte)124, (byte)38, (byte)165, (byte)163, (byte)46, (byte)133, (byte)168, (byte)2, (byte)99, (byte)237, (byte)146, (byte)44, (byte)172, (byte)220, (byte)100, (byte)250, (byte)35, (byte)34, (byte)31, (byte)173}, 0) ;
            LoopBackDemoChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.first_message_offset == (byte)(byte)97);
                Debug.Assert(pack.target_component == (byte)(byte)157);
                Debug.Assert(pack.target_system == (byte)(byte)206);
                Debug.Assert(pack.length == (byte)(byte)128);
                Debug.Assert(pack.sequence == (ushort)(ushort)10893);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)139, (byte)109, (byte)149, (byte)119, (byte)164, (byte)39, (byte)43, (byte)142, (byte)80, (byte)214, (byte)143, (byte)6, (byte)125, (byte)12, (byte)91, (byte)223, (byte)227, (byte)128, (byte)22, (byte)157, (byte)16, (byte)51, (byte)113, (byte)169, (byte)177, (byte)236, (byte)116, (byte)111, (byte)182, (byte)157, (byte)230, (byte)26, (byte)135, (byte)208, (byte)4, (byte)141, (byte)40, (byte)209, (byte)181, (byte)142, (byte)54, (byte)75, (byte)244, (byte)188, (byte)147, (byte)109, (byte)80, (byte)135, (byte)196, (byte)186, (byte)41, (byte)179, (byte)159, (byte)140, (byte)183, (byte)201, (byte)104, (byte)184, (byte)81, (byte)10, (byte)199, (byte)100, (byte)103, (byte)204, (byte)237, (byte)130, (byte)165, (byte)68, (byte)127, (byte)138, (byte)92, (byte)236, (byte)29, (byte)130, (byte)141, (byte)70, (byte)253, (byte)116, (byte)236, (byte)96, (byte)6, (byte)177, (byte)203, (byte)215, (byte)7, (byte)157, (byte)207, (byte)103, (byte)75, (byte)95, (byte)187, (byte)75, (byte)98, (byte)152, (byte)69, (byte)217, (byte)217, (byte)181, (byte)230, (byte)212, (byte)172, (byte)55, (byte)60, (byte)105, (byte)17, (byte)194, (byte)205, (byte)39, (byte)170, (byte)98, (byte)125, (byte)245, (byte)241, (byte)72, (byte)142, (byte)203, (byte)49, (byte)7, (byte)36, (byte)179, (byte)51, (byte)85, (byte)212, (byte)164, (byte)172, (byte)203, (byte)145, (byte)98, (byte)15, (byte)114, (byte)114, (byte)47, (byte)141, (byte)139, (byte)115, (byte)158, (byte)211, (byte)202, (byte)70, (byte)116, (byte)146, (byte)72, (byte)157, (byte)232, (byte)212, (byte)97, (byte)58, (byte)85, (byte)15, (byte)168, (byte)155, (byte)168, (byte)8, (byte)88, (byte)160, (byte)148, (byte)162, (byte)133, (byte)146, (byte)221, (byte)40, (byte)58, (byte)211, (byte)197, (byte)179, (byte)203, (byte)138, (byte)71, (byte)9, (byte)26, (byte)163, (byte)46, (byte)127, (byte)200, (byte)24, (byte)23, (byte)34, (byte)61, (byte)69, (byte)233, (byte)87, (byte)151, (byte)72, (byte)117, (byte)236, (byte)18, (byte)48, (byte)64, (byte)16, (byte)161, (byte)188, (byte)16, (byte)133, (byte)169, (byte)163, (byte)85, (byte)224, (byte)195, (byte)214, (byte)85, (byte)75, (byte)126, (byte)224, (byte)73, (byte)13, (byte)61, (byte)156, (byte)16, (byte)88, (byte)232, (byte)10, (byte)171, (byte)78, (byte)73, (byte)11, (byte)225, (byte)236, (byte)187, (byte)75, (byte)106, (byte)128, (byte)32, (byte)27, (byte)78, (byte)1, (byte)21, (byte)55, (byte)250, (byte)24, (byte)93, (byte)238, (byte)77, (byte)96, (byte)4, (byte)79, (byte)117, (byte)38, (byte)193, (byte)151, (byte)35, (byte)66, (byte)11, (byte)204, (byte)115, (byte)181, (byte)217, (byte)27, (byte)98, (byte)140}));
            };
            DemoDevice.LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_system = (byte)(byte)206;
            p267.length = (byte)(byte)128;
            p267.target_component = (byte)(byte)157;
            p267.data__SET(new byte[] {(byte)139, (byte)109, (byte)149, (byte)119, (byte)164, (byte)39, (byte)43, (byte)142, (byte)80, (byte)214, (byte)143, (byte)6, (byte)125, (byte)12, (byte)91, (byte)223, (byte)227, (byte)128, (byte)22, (byte)157, (byte)16, (byte)51, (byte)113, (byte)169, (byte)177, (byte)236, (byte)116, (byte)111, (byte)182, (byte)157, (byte)230, (byte)26, (byte)135, (byte)208, (byte)4, (byte)141, (byte)40, (byte)209, (byte)181, (byte)142, (byte)54, (byte)75, (byte)244, (byte)188, (byte)147, (byte)109, (byte)80, (byte)135, (byte)196, (byte)186, (byte)41, (byte)179, (byte)159, (byte)140, (byte)183, (byte)201, (byte)104, (byte)184, (byte)81, (byte)10, (byte)199, (byte)100, (byte)103, (byte)204, (byte)237, (byte)130, (byte)165, (byte)68, (byte)127, (byte)138, (byte)92, (byte)236, (byte)29, (byte)130, (byte)141, (byte)70, (byte)253, (byte)116, (byte)236, (byte)96, (byte)6, (byte)177, (byte)203, (byte)215, (byte)7, (byte)157, (byte)207, (byte)103, (byte)75, (byte)95, (byte)187, (byte)75, (byte)98, (byte)152, (byte)69, (byte)217, (byte)217, (byte)181, (byte)230, (byte)212, (byte)172, (byte)55, (byte)60, (byte)105, (byte)17, (byte)194, (byte)205, (byte)39, (byte)170, (byte)98, (byte)125, (byte)245, (byte)241, (byte)72, (byte)142, (byte)203, (byte)49, (byte)7, (byte)36, (byte)179, (byte)51, (byte)85, (byte)212, (byte)164, (byte)172, (byte)203, (byte)145, (byte)98, (byte)15, (byte)114, (byte)114, (byte)47, (byte)141, (byte)139, (byte)115, (byte)158, (byte)211, (byte)202, (byte)70, (byte)116, (byte)146, (byte)72, (byte)157, (byte)232, (byte)212, (byte)97, (byte)58, (byte)85, (byte)15, (byte)168, (byte)155, (byte)168, (byte)8, (byte)88, (byte)160, (byte)148, (byte)162, (byte)133, (byte)146, (byte)221, (byte)40, (byte)58, (byte)211, (byte)197, (byte)179, (byte)203, (byte)138, (byte)71, (byte)9, (byte)26, (byte)163, (byte)46, (byte)127, (byte)200, (byte)24, (byte)23, (byte)34, (byte)61, (byte)69, (byte)233, (byte)87, (byte)151, (byte)72, (byte)117, (byte)236, (byte)18, (byte)48, (byte)64, (byte)16, (byte)161, (byte)188, (byte)16, (byte)133, (byte)169, (byte)163, (byte)85, (byte)224, (byte)195, (byte)214, (byte)85, (byte)75, (byte)126, (byte)224, (byte)73, (byte)13, (byte)61, (byte)156, (byte)16, (byte)88, (byte)232, (byte)10, (byte)171, (byte)78, (byte)73, (byte)11, (byte)225, (byte)236, (byte)187, (byte)75, (byte)106, (byte)128, (byte)32, (byte)27, (byte)78, (byte)1, (byte)21, (byte)55, (byte)250, (byte)24, (byte)93, (byte)238, (byte)77, (byte)96, (byte)4, (byte)79, (byte)117, (byte)38, (byte)193, (byte)151, (byte)35, (byte)66, (byte)11, (byte)204, (byte)115, (byte)181, (byte)217, (byte)27, (byte)98, (byte)140}, 0) ;
            p267.first_message_offset = (byte)(byte)97;
            p267.sequence = (ushort)(ushort)10893;
            LoopBackDemoChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)250);
                Debug.Assert(pack.sequence == (ushort)(ushort)17273);
                Debug.Assert(pack.target_system == (byte)(byte)215);
            };
            DemoDevice.LOGGING_ACK p268 = LoopBackDemoChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_component = (byte)(byte)250;
            p268.target_system = (byte)(byte)215;
            p268.sequence = (ushort)(ushort)17273;
            LoopBackDemoChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uri_LEN(ph) == 144);
                Debug.Assert(pack.uri_TRY(ph).Equals("eofuvskuqyuynbanmtyhgezqairnqlrOcpeRdqoqcmlmYuknejxfItylpxBpuwreuimtuppygryxmdcwWaVlldwmyPedyklknNcVStxlyfriwfcrvtztiofcqywlvuZEusfxsqtyxseunngk"));
                Debug.Assert(pack.status == (byte)(byte)0);
                Debug.Assert(pack.rotation == (ushort)(ushort)34552);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)58542);
                Debug.Assert(pack.bitrate == (uint)3146716497U);
                Debug.Assert(pack.camera_id == (byte)(byte)0);
                Debug.Assert(pack.framerate == (float)3.0166498E38F);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)37636);
            };
            DemoDevice.VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.status = (byte)(byte)0;
            p269.rotation = (ushort)(ushort)34552;
            p269.bitrate = (uint)3146716497U;
            p269.resolution_h = (ushort)(ushort)58542;
            p269.resolution_v = (ushort)(ushort)37636;
            p269.camera_id = (byte)(byte)0;
            p269.uri_SET("eofuvskuqyuynbanmtyhgezqairnqlrOcpeRdqoqcmlmYuknejxfItylpxBpuwreuimtuppygryxmdcwWaVlldwmyPedyklknNcVStxlyfriwfcrvtztiofcqywlvuZEusfxsqtyxseunngk", PH) ;
            p269.framerate = (float)3.0166498E38F;
            LoopBackDemoChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.framerate == (float)2.1264642E37F);
                Debug.Assert(pack.camera_id == (byte)(byte)221);
                Debug.Assert(pack.target_component == (byte)(byte)25);
                Debug.Assert(pack.bitrate == (uint)3433328091U);
                Debug.Assert(pack.uri_LEN(ph) == 201);
                Debug.Assert(pack.uri_TRY(ph).Equals("QZjgvchhuitxuzzvhijxFmcwervrPggcwzdgRgmczudbovtdaEqpcduaroqlsjczzrjwzerqbxyxmazdhenNylksuZyTygpCkjwVqsutpidmolwaocMkcoxujuyufaRkagswoctftrkxwqkFweMgmoxOyylrenxszzwdNsuzqdhjduamStruhqGrltpjmtobjoxqocYyd"));
                Debug.Assert(pack.target_system == (byte)(byte)114);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)4336);
                Debug.Assert(pack.rotation == (ushort)(ushort)18629);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)59272);
            };
            DemoDevice.SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.resolution_v = (ushort)(ushort)4336;
            p270.camera_id = (byte)(byte)221;
            p270.target_component = (byte)(byte)25;
            p270.framerate = (float)2.1264642E37F;
            p270.uri_SET("QZjgvchhuitxuzzvhijxFmcwervrPggcwzdgRgmczudbovtdaEqpcduaroqlsjczzrjwzerqbxyxmazdhenNylksuZyTygpCkjwVqsutpidmolwaocMkcoxujuyufaRkagswoctftrkxwqkFweMgmoxOyylrenxszzwdNsuzqdhjduamStruhqGrltpjmtobjoxqocYyd", PH) ;
            p270.target_system = (byte)(byte)114;
            p270.bitrate = (uint)3433328091U;
            p270.resolution_h = (ushort)(ushort)59272;
            p270.rotation = (ushort)(ushort)18629;
            LoopBackDemoChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ssid_LEN(ph) == 4);
                Debug.Assert(pack.ssid_TRY(ph).Equals("zifc"));
                Debug.Assert(pack.password_LEN(ph) == 33);
                Debug.Assert(pack.password_TRY(ph).Equals("tkdibzaqgfsfgwieapxjjwprHlrcnvngw"));
            };
            DemoDevice.WIFI_CONFIG_AP p299 = LoopBackDemoChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.password_SET("tkdibzaqgfsfgwieapxjjwprHlrcnvngw", PH) ;
            p299.ssid_SET("zifc", PH) ;
            LoopBackDemoChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.min_version == (ushort)(ushort)44111);
                Debug.Assert(pack.max_version == (ushort)(ushort)28066);
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)17, (byte)181, (byte)245, (byte)194, (byte)116, (byte)194, (byte)179, (byte)53}));
                Debug.Assert(pack.version == (ushort)(ushort)28216);
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)103, (byte)222, (byte)248, (byte)25, (byte)201, (byte)8, (byte)68, (byte)134}));
            };
            DemoDevice.PROTOCOL_VERSION p300 = LoopBackDemoChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.spec_version_hash_SET(new byte[] {(byte)103, (byte)222, (byte)248, (byte)25, (byte)201, (byte)8, (byte)68, (byte)134}, 0) ;
            p300.version = (ushort)(ushort)28216;
            p300.max_version = (ushort)(ushort)28066;
            p300.min_version = (ushort)(ushort)44111;
            p300.library_version_hash_SET(new byte[] {(byte)17, (byte)181, (byte)245, (byte)194, (byte)116, (byte)194, (byte)179, (byte)53}, 0) ;
            LoopBackDemoChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uptime_sec == (uint)3620512241U);
                Debug.Assert(pack.time_usec == (ulong)7847549210189856031L);
                Debug.Assert(pack.health == (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK);
                Debug.Assert(pack.mode == (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)39040);
                Debug.Assert(pack.sub_mode == (byte)(byte)192);
            };
            DemoDevice.UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.sub_mode = (byte)(byte)192;
            p310.vendor_specific_status_code = (ushort)(ushort)39040;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE;
            p310.time_usec = (ulong)7847549210189856031L;
            p310.uptime_sec = (uint)3620512241U;
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK;
            LoopBackDemoChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sw_version_minor == (byte)(byte)232);
                Debug.Assert(pack.hw_version_minor == (byte)(byte)111);
                Debug.Assert(pack.time_usec == (ulong)7010148102506495123L);
                Debug.Assert(pack.hw_version_major == (byte)(byte)199);
                Debug.Assert(pack.sw_vcs_commit == (uint)3437813951U);
                Debug.Assert(pack.sw_version_major == (byte)(byte)6);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)211, (byte)4, (byte)120, (byte)213, (byte)89, (byte)239, (byte)230, (byte)178, (byte)233, (byte)201, (byte)152, (byte)191, (byte)233, (byte)38, (byte)94, (byte)21}));
                Debug.Assert(pack.name_LEN(ph) == 34);
                Debug.Assert(pack.name_TRY(ph).Equals("kgqizhlntdBHVafqscyddgynugxIyvdhof"));
                Debug.Assert(pack.uptime_sec == (uint)1528281394U);
            };
            DemoDevice.UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.sw_vcs_commit = (uint)3437813951U;
            p311.hw_version_major = (byte)(byte)199;
            p311.hw_unique_id_SET(new byte[] {(byte)211, (byte)4, (byte)120, (byte)213, (byte)89, (byte)239, (byte)230, (byte)178, (byte)233, (byte)201, (byte)152, (byte)191, (byte)233, (byte)38, (byte)94, (byte)21}, 0) ;
            p311.sw_version_major = (byte)(byte)6;
            p311.sw_version_minor = (byte)(byte)232;
            p311.name_SET("kgqizhlntdBHVafqscyddgynugxIyvdhof", PH) ;
            p311.time_usec = (ulong)7010148102506495123L;
            p311.hw_version_minor = (byte)(byte)111;
            p311.uptime_sec = (uint)1528281394U;
            LoopBackDemoChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 14);
                Debug.Assert(pack.param_id_TRY(ph).Equals("zllnotrfnkqkae"));
                Debug.Assert(pack.param_index == (short)(short)3430);
                Debug.Assert(pack.target_system == (byte)(byte)51);
                Debug.Assert(pack.target_component == (byte)(byte)62);
            };
            DemoDevice.PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.param_id_SET("zllnotrfnkqkae", PH) ;
            p320.target_system = (byte)(byte)51;
            p320.param_index = (short)(short)3430;
            p320.target_component = (byte)(byte)62;
            LoopBackDemoChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)121);
                Debug.Assert(pack.target_system == (byte)(byte)153);
            };
            DemoDevice.PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)153;
            p321.target_component = (byte)(byte)121;
            LoopBackDemoChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
                Debug.Assert(pack.param_count == (ushort)(ushort)40778);
                Debug.Assert(pack.param_value_LEN(ph) == 77);
                Debug.Assert(pack.param_value_TRY(ph).Equals("ygibhrpOnowkdeDrqzyqqodbmfvejbnXmHdregyypfXbqxozekdegcquklawwlhlvvbdhXdrcudqx"));
                Debug.Assert(pack.param_index == (ushort)(ushort)11243);
                Debug.Assert(pack.param_id_LEN(ph) == 3);
                Debug.Assert(pack.param_id_TRY(ph).Equals("zRk"));
            };
            DemoDevice.PARAM_EXT_VALUE p322 = LoopBackDemoChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_index = (ushort)(ushort)11243;
            p322.param_value_SET("ygibhrpOnowkdeDrqzyqqodbmfvejbnXmHdregyypfXbqxozekdegcquklawwlhlvvbdhXdrcudqx", PH) ;
            p322.param_count = (ushort)(ushort)40778;
            p322.param_id_SET("zRk", PH) ;
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64;
            LoopBackDemoChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32);
                Debug.Assert(pack.param_value_LEN(ph) == 73);
                Debug.Assert(pack.param_value_TRY(ph).Equals("weeqzixevkfsDPAeryjabyqicwlgedrEjLhgnRgvbAcYklxtovvuhsfzdwxglkxZpcudlZJac"));
                Debug.Assert(pack.target_system == (byte)(byte)251);
                Debug.Assert(pack.param_id_LEN(ph) == 7);
                Debug.Assert(pack.param_id_TRY(ph).Equals("swlNqgn"));
                Debug.Assert(pack.target_component == (byte)(byte)66);
            };
            DemoDevice.PARAM_EXT_SET p323 = LoopBackDemoChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.param_id_SET("swlNqgn", PH) ;
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32;
            p323.param_value_SET("weeqzixevkfsDPAeryjabyqicwlgedrEjLhgnRgvbAcYklxtovvuhsfzdwxglkxZpcudlZJac", PH) ;
            p323.target_system = (byte)(byte)251;
            p323.target_component = (byte)(byte)66;
            LoopBackDemoChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_result == (PARAM_ACK)PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED);
                Debug.Assert(pack.param_value_LEN(ph) == 121);
                Debug.Assert(pack.param_value_TRY(ph).Equals("YjsuulhdamNvutazhidocySlewbrhddhvsqUewqpbbxnzqmdbfyxmpaafApzwgLeydiyefkcmpvomhvlfeXbngcgsscbeordbsnrTzzmorcZdliqiqzbWxcpf"));
                Debug.Assert(pack.param_id_LEN(ph) == 10);
                Debug.Assert(pack.param_id_TRY(ph).Equals("rlkkazmmso"));
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
            };
            DemoDevice.PARAM_EXT_ACK p324 = LoopBackDemoChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED;
            p324.param_id_SET("rlkkazmmso", PH) ;
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64;
            p324.param_value_SET("YjsuulhdamNvutazhidocySlewbrhddhvsqUewqpbbxnzqmdbfyxmpaafApzwgLeydiyefkcmpvomhvlfeXbngcgsscbeordbsnrTzzmorcZdliqiqzbWxcpf", PH) ;
            LoopBackDemoChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.min_distance == (ushort)(ushort)60258);
                Debug.Assert(pack.max_distance == (ushort)(ushort)55444);
                Debug.Assert(pack.increment == (byte)(byte)57);
                Debug.Assert(pack.sensor_type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)53807, (ushort)16592, (ushort)6072, (ushort)780, (ushort)10060, (ushort)19392, (ushort)4650, (ushort)3727, (ushort)54808, (ushort)26999, (ushort)42802, (ushort)33924, (ushort)6773, (ushort)21786, (ushort)241, (ushort)46494, (ushort)37134, (ushort)63162, (ushort)32236, (ushort)8163, (ushort)58858, (ushort)15253, (ushort)47982, (ushort)14883, (ushort)28166, (ushort)65212, (ushort)55596, (ushort)33651, (ushort)3998, (ushort)34130, (ushort)26966, (ushort)2798, (ushort)37022, (ushort)7650, (ushort)35062, (ushort)39292, (ushort)64323, (ushort)50425, (ushort)4335, (ushort)3691, (ushort)55615, (ushort)33113, (ushort)37939, (ushort)35932, (ushort)62545, (ushort)25713, (ushort)57298, (ushort)11096, (ushort)64893, (ushort)61370, (ushort)65077, (ushort)41861, (ushort)64436, (ushort)2906, (ushort)38252, (ushort)5911, (ushort)26765, (ushort)63947, (ushort)16046, (ushort)11320, (ushort)13231, (ushort)14554, (ushort)59677, (ushort)29595, (ushort)60820, (ushort)62595, (ushort)10469, (ushort)55722, (ushort)57593, (ushort)1115, (ushort)37363, (ushort)45526}));
                Debug.Assert(pack.time_usec == (ulong)3599099452100597175L);
            };
            DemoDevice.OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.max_distance = (ushort)(ushort)55444;
            p330.increment = (byte)(byte)57;
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            p330.min_distance = (ushort)(ushort)60258;
            p330.distances_SET(new ushort[] {(ushort)53807, (ushort)16592, (ushort)6072, (ushort)780, (ushort)10060, (ushort)19392, (ushort)4650, (ushort)3727, (ushort)54808, (ushort)26999, (ushort)42802, (ushort)33924, (ushort)6773, (ushort)21786, (ushort)241, (ushort)46494, (ushort)37134, (ushort)63162, (ushort)32236, (ushort)8163, (ushort)58858, (ushort)15253, (ushort)47982, (ushort)14883, (ushort)28166, (ushort)65212, (ushort)55596, (ushort)33651, (ushort)3998, (ushort)34130, (ushort)26966, (ushort)2798, (ushort)37022, (ushort)7650, (ushort)35062, (ushort)39292, (ushort)64323, (ushort)50425, (ushort)4335, (ushort)3691, (ushort)55615, (ushort)33113, (ushort)37939, (ushort)35932, (ushort)62545, (ushort)25713, (ushort)57298, (ushort)11096, (ushort)64893, (ushort)61370, (ushort)65077, (ushort)41861, (ushort)64436, (ushort)2906, (ushort)38252, (ushort)5911, (ushort)26765, (ushort)63947, (ushort)16046, (ushort)11320, (ushort)13231, (ushort)14554, (ushort)59677, (ushort)29595, (ushort)60820, (ushort)62595, (ushort)10469, (ushort)55722, (ushort)57593, (ushort)1115, (ushort)37363, (ushort)45526}, 0) ;
            p330.time_usec = (ulong)3599099452100597175L;
            LoopBackDemoChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
        }
    }
}