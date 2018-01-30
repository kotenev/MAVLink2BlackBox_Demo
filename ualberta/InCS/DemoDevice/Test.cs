
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
                Debug.Assert(pack.system_status == (MAV_STATE)MAV_STATE.MAV_STATE_UNINIT);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
                Debug.Assert(pack.autopilot == (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_INVALID);
                Debug.Assert(pack.custom_mode == (uint)1010938462U);
                Debug.Assert(pack.type == (MAV_TYPE)MAV_TYPE.MAV_TYPE_ONBOARD_CONTROLLER);
                Debug.Assert(pack.mavlink_version == (byte)(byte)159);
            };
            DemoDevice.HEARTBEAT p0 = LoopBackDemoChannel.new_HEARTBEAT();
            PH.setPack(p0);
            p0.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            p0.custom_mode = (uint)1010938462U;
            p0.system_status = (MAV_STATE)MAV_STATE.MAV_STATE_UNINIT;
            p0.mavlink_version = (byte)(byte)159;
            p0.autopilot = (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_INVALID;
            p0.type = (MAV_TYPE)MAV_TYPE.MAV_TYPE_ONBOARD_CONTROLLER;
            LoopBackDemoChannel.instance.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)44778);
                Debug.Assert(pack.current_battery == (short)(short)4825);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)15764);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)121);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)28866);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)43006);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)61427);
                Debug.Assert(pack.load == (ushort)(ushort)37927);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)57415);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)783);
            };
            DemoDevice.SYS_STATUS p1 = LoopBackDemoChannel.new_SYS_STATUS();
            PH.setPack(p1);
            p1.current_battery = (short)(short)4825;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
            p1.errors_count4 = (ushort)(ushort)61427;
            p1.load = (ushort)(ushort)37927;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;
            p1.voltage_battery = (ushort)(ushort)783;
            p1.errors_count2 = (ushort)(ushort)28866;
            p1.drop_rate_comm = (ushort)(ushort)44778;
            p1.errors_count3 = (ushort)(ushort)57415;
            p1.battery_remaining = (sbyte)(sbyte)121;
            p1.errors_comm = (ushort)(ushort)15764;
            p1.errors_count1 = (ushort)(ushort)43006;
            LoopBackDemoChannel.instance.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2982380291U);
                Debug.Assert(pack.time_unix_usec == (ulong)8097438179807741412L);
            };
            DemoDevice.SYSTEM_TIME p2 = LoopBackDemoChannel.new_SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)8097438179807741412L;
            p2.time_boot_ms = (uint)2982380291U;
            LoopBackDemoChannel.instance.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)2.4446451E38F);
                Debug.Assert(pack.y == (float) -1.0873084E38F);
                Debug.Assert(pack.afx == (float) -2.0074718E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2020586277U);
                Debug.Assert(pack.yaw == (float)1.3019444E38F);
                Debug.Assert(pack.vx == (float)2.2801943E38F);
                Debug.Assert(pack.vy == (float) -8.3430744E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)65376);
                Debug.Assert(pack.vz == (float) -2.36664E37F);
                Debug.Assert(pack.yaw_rate == (float) -9.862355E37F);
                Debug.Assert(pack.x == (float) -1.686023E38F);
                Debug.Assert(pack.afz == (float)2.5146326E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.afy == (float)1.9924378E38F);
            };
            DemoDevice.POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.yaw = (float)1.3019444E38F;
            p3.afz = (float)2.5146326E38F;
            p3.vz = (float) -2.36664E37F;
            p3.yaw_rate = (float) -9.862355E37F;
            p3.vy = (float) -8.3430744E37F;
            p3.y = (float) -1.0873084E38F;
            p3.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p3.afx = (float) -2.0074718E38F;
            p3.x = (float) -1.686023E38F;
            p3.afy = (float)1.9924378E38F;
            p3.z = (float)2.4446451E38F;
            p3.vx = (float)2.2801943E38F;
            p3.type_mask = (ushort)(ushort)65376;
            p3.time_boot_ms = (uint)2020586277U;
            LoopBackDemoChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)134);
                Debug.Assert(pack.seq == (uint)3940800834U);
                Debug.Assert(pack.target_component == (byte)(byte)248);
                Debug.Assert(pack.time_usec == (ulong)7918632871302316821L);
            };
            DemoDevice.PING p4 = LoopBackDemoChannel.new_PING();
            PH.setPack(p4);
            p4.time_usec = (ulong)7918632871302316821L;
            p4.target_system = (byte)(byte)134;
            p4.seq = (uint)3940800834U;
            p4.target_component = (byte)(byte)248;
            LoopBackDemoChannel.instance.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.version == (byte)(byte)166);
                Debug.Assert(pack.control_request == (byte)(byte)224);
                Debug.Assert(pack.target_system == (byte)(byte)176);
                Debug.Assert(pack.passkey_LEN(ph) == 5);
                Debug.Assert(pack.passkey_TRY(ph).Equals("upLds"));
            };
            DemoDevice.CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.version = (byte)(byte)166;
            p5.passkey_SET("upLds", PH) ;
            p5.control_request = (byte)(byte)224;
            p5.target_system = (byte)(byte)176;
            LoopBackDemoChannel.instance.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gcs_system_id == (byte)(byte)22);
                Debug.Assert(pack.ack == (byte)(byte)33);
                Debug.Assert(pack.control_request == (byte)(byte)68);
            };
            DemoDevice.CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.ack = (byte)(byte)33;
            p6.control_request = (byte)(byte)68;
            p6.gcs_system_id = (byte)(byte)22;
            LoopBackDemoChannel.instance.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 21);
                Debug.Assert(pack.key_TRY(ph).Equals("ZbTgpizHkqvJusSpduxqi"));
            };
            DemoDevice.AUTH_KEY p7 = LoopBackDemoChannel.new_AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("ZbTgpizHkqvJusSpduxqi", PH) ;
            LoopBackDemoChannel.instance.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.base_mode == (MAV_MODE)MAV_MODE.MAV_MODE_TEST_DISARMED);
                Debug.Assert(pack.target_system == (byte)(byte)16);
                Debug.Assert(pack.custom_mode == (uint)2804531612U);
            };
            DemoDevice.SET_MODE p11 = LoopBackDemoChannel.new_SET_MODE();
            PH.setPack(p11);
            p11.custom_mode = (uint)2804531612U;
            p11.target_system = (byte)(byte)16;
            p11.base_mode = (MAV_MODE)MAV_MODE.MAV_MODE_TEST_DISARMED;
            LoopBackDemoChannel.instance.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)101);
                Debug.Assert(pack.target_component == (byte)(byte)105);
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("emcFf"));
                Debug.Assert(pack.param_index == (short)(short) -26427);
            };
            DemoDevice.PARAM_REQUEST_READ p20 = LoopBackDemoChannel.new_PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.target_component = (byte)(byte)105;
            p20.param_index = (short)(short) -26427;
            p20.target_system = (byte)(byte)101;
            p20.param_id_SET("emcFf", PH) ;
            LoopBackDemoChannel.instance.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)255);
                Debug.Assert(pack.target_system == (byte)(byte)80);
            };
            DemoDevice.PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.new_PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_component = (byte)(byte)255;
            p21.target_system = (byte)(byte)80;
            LoopBackDemoChannel.instance.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value == (float) -8.467433E37F);
                Debug.Assert(pack.param_id_LEN(ph) == 1);
                Debug.Assert(pack.param_id_TRY(ph).Equals("b"));
                Debug.Assert(pack.param_count == (ushort)(ushort)52077);
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16);
                Debug.Assert(pack.param_index == (ushort)(ushort)3109);
            };
            DemoDevice.PARAM_VALUE p22 = LoopBackDemoChannel.new_PARAM_VALUE();
            PH.setPack(p22);
            p22.param_index = (ushort)(ushort)3109;
            p22.param_count = (ushort)(ushort)52077;
            p22.param_id_SET("b", PH) ;
            p22.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16;
            p22.param_value = (float) -8.467433E37F;
            LoopBackDemoChannel.instance.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 16);
                Debug.Assert(pack.param_id_TRY(ph).Equals("ftkttmkywwzkovat"));
                Debug.Assert(pack.target_system == (byte)(byte)137);
                Debug.Assert(pack.param_value == (float)2.508137E38F);
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16);
                Debug.Assert(pack.target_component == (byte)(byte)95);
            };
            DemoDevice.PARAM_SET p23 = LoopBackDemoChannel.new_PARAM_SET();
            PH.setPack(p23);
            p23.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16;
            p23.param_id_SET("ftkttmkywwzkovat", PH) ;
            p23.param_value = (float)2.508137E38F;
            p23.target_component = (byte)(byte)95;
            p23.target_system = (byte)(byte)137;
            LoopBackDemoChannel.instance.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
                Debug.Assert(pack.time_usec == (ulong)5270509760624435408L);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)2206621860U);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int) -200013449);
                Debug.Assert(pack.vel == (ushort)(ushort)12588);
                Debug.Assert(pack.lon == (int) -2758537);
                Debug.Assert(pack.lat == (int)2080960305);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)114556866U);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)617286350U);
                Debug.Assert(pack.satellites_visible == (byte)(byte)188);
                Debug.Assert(pack.eph == (ushort)(ushort)10606);
                Debug.Assert(pack.alt == (int)169848594);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)2891563791U);
                Debug.Assert(pack.epv == (ushort)(ushort)56313);
                Debug.Assert(pack.cog == (ushort)(ushort)2886);
            };
            DemoDevice.GPS_RAW_INT p24 = LoopBackDemoChannel.new_GPS_RAW_INT();
            PH.setPack(p24);
            p24.eph = (ushort)(ushort)10606;
            p24.time_usec = (ulong)5270509760624435408L;
            p24.satellites_visible = (byte)(byte)188;
            p24.alt_ellipsoid_SET((int) -200013449, PH) ;
            p24.vel_acc_SET((uint)2891563791U, PH) ;
            p24.cog = (ushort)(ushort)2886;
            p24.lon = (int) -2758537;
            p24.hdg_acc_SET((uint)617286350U, PH) ;
            p24.epv = (ushort)(ushort)56313;
            p24.v_acc_SET((uint)2206621860U, PH) ;
            p24.alt = (int)169848594;
            p24.h_acc_SET((uint)114556866U, PH) ;
            p24.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p24.vel = (ushort)(ushort)12588;
            p24.lat = (int)2080960305;
            LoopBackDemoChannel.instance.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellites_visible == (byte)(byte)227);
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)248, (byte)238, (byte)96, (byte)24, (byte)212, (byte)168, (byte)29, (byte)158, (byte)77, (byte)41, (byte)220, (byte)24, (byte)26, (byte)39, (byte)212, (byte)152, (byte)47, (byte)220, (byte)186, (byte)105}));
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)199, (byte)198, (byte)14, (byte)187, (byte)222, (byte)55, (byte)11, (byte)27, (byte)118, (byte)204, (byte)252, (byte)210, (byte)222, (byte)191, (byte)95, (byte)138, (byte)152, (byte)65, (byte)250, (byte)249}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)109, (byte)240, (byte)133, (byte)163, (byte)17, (byte)224, (byte)105, (byte)198, (byte)133, (byte)218, (byte)77, (byte)101, (byte)194, (byte)30, (byte)45, (byte)252, (byte)138, (byte)7, (byte)70, (byte)5}));
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)165, (byte)77, (byte)223, (byte)88, (byte)225, (byte)53, (byte)85, (byte)103, (byte)145, (byte)147, (byte)43, (byte)159, (byte)39, (byte)215, (byte)233, (byte)194, (byte)7, (byte)206, (byte)28, (byte)91}));
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)111, (byte)15, (byte)137, (byte)5, (byte)149, (byte)179, (byte)19, (byte)56, (byte)160, (byte)192, (byte)188, (byte)106, (byte)54, (byte)170, (byte)201, (byte)19, (byte)94, (byte)160, (byte)115, (byte)72}));
            };
            DemoDevice.GPS_STATUS p25 = LoopBackDemoChannel.new_GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_azimuth_SET(new byte[] {(byte)109, (byte)240, (byte)133, (byte)163, (byte)17, (byte)224, (byte)105, (byte)198, (byte)133, (byte)218, (byte)77, (byte)101, (byte)194, (byte)30, (byte)45, (byte)252, (byte)138, (byte)7, (byte)70, (byte)5}, 0) ;
            p25.satellites_visible = (byte)(byte)227;
            p25.satellite_snr_SET(new byte[] {(byte)165, (byte)77, (byte)223, (byte)88, (byte)225, (byte)53, (byte)85, (byte)103, (byte)145, (byte)147, (byte)43, (byte)159, (byte)39, (byte)215, (byte)233, (byte)194, (byte)7, (byte)206, (byte)28, (byte)91}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)199, (byte)198, (byte)14, (byte)187, (byte)222, (byte)55, (byte)11, (byte)27, (byte)118, (byte)204, (byte)252, (byte)210, (byte)222, (byte)191, (byte)95, (byte)138, (byte)152, (byte)65, (byte)250, (byte)249}, 0) ;
            p25.satellite_used_SET(new byte[] {(byte)248, (byte)238, (byte)96, (byte)24, (byte)212, (byte)168, (byte)29, (byte)158, (byte)77, (byte)41, (byte)220, (byte)24, (byte)26, (byte)39, (byte)212, (byte)152, (byte)47, (byte)220, (byte)186, (byte)105}, 0) ;
            p25.satellite_prn_SET(new byte[] {(byte)111, (byte)15, (byte)137, (byte)5, (byte)149, (byte)179, (byte)19, (byte)56, (byte)160, (byte)192, (byte)188, (byte)106, (byte)54, (byte)170, (byte)201, (byte)19, (byte)94, (byte)160, (byte)115, (byte)72}, 0) ;
            LoopBackDemoChannel.instance.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xmag == (short)(short) -29739);
                Debug.Assert(pack.zgyro == (short)(short)27168);
                Debug.Assert(pack.xgyro == (short)(short) -3130);
                Debug.Assert(pack.yacc == (short)(short)18401);
                Debug.Assert(pack.ygyro == (short)(short)2005);
                Debug.Assert(pack.zmag == (short)(short)28552);
                Debug.Assert(pack.xacc == (short)(short)1129);
                Debug.Assert(pack.zacc == (short)(short) -3290);
                Debug.Assert(pack.time_boot_ms == (uint)2427448339U);
                Debug.Assert(pack.ymag == (short)(short)21038);
            };
            DemoDevice.SCALED_IMU p26 = LoopBackDemoChannel.new_SCALED_IMU();
            PH.setPack(p26);
            p26.zgyro = (short)(short)27168;
            p26.xacc = (short)(short)1129;
            p26.time_boot_ms = (uint)2427448339U;
            p26.xmag = (short)(short) -29739;
            p26.xgyro = (short)(short) -3130;
            p26.ygyro = (short)(short)2005;
            p26.ymag = (short)(short)21038;
            p26.zacc = (short)(short) -3290;
            p26.yacc = (short)(short)18401;
            p26.zmag = (short)(short)28552;
            LoopBackDemoChannel.instance.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (short)(short)23740);
                Debug.Assert(pack.ymag == (short)(short)1540);
                Debug.Assert(pack.xmag == (short)(short) -27948);
                Debug.Assert(pack.zacc == (short)(short) -23536);
                Debug.Assert(pack.zmag == (short)(short)14798);
                Debug.Assert(pack.xgyro == (short)(short) -26608);
                Debug.Assert(pack.time_usec == (ulong)8936112260356481198L);
                Debug.Assert(pack.zgyro == (short)(short)31718);
                Debug.Assert(pack.yacc == (short)(short) -31143);
                Debug.Assert(pack.xacc == (short)(short)18679);
            };
            DemoDevice.RAW_IMU p27 = LoopBackDemoChannel.new_RAW_IMU();
            PH.setPack(p27);
            p27.ymag = (short)(short)1540;
            p27.time_usec = (ulong)8936112260356481198L;
            p27.xgyro = (short)(short) -26608;
            p27.yacc = (short)(short) -31143;
            p27.zacc = (short)(short) -23536;
            p27.ygyro = (short)(short)23740;
            p27.zgyro = (short)(short)31718;
            p27.xmag = (short)(short) -27948;
            p27.xacc = (short)(short)18679;
            p27.zmag = (short)(short)14798;
            LoopBackDemoChannel.instance.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)5503514250558570492L);
                Debug.Assert(pack.press_diff2 == (short)(short) -461);
                Debug.Assert(pack.temperature == (short)(short)16064);
                Debug.Assert(pack.press_diff1 == (short)(short) -21603);
                Debug.Assert(pack.press_abs == (short)(short) -4205);
            };
            DemoDevice.RAW_PRESSURE p28 = LoopBackDemoChannel.new_RAW_PRESSURE();
            PH.setPack(p28);
            p28.press_abs = (short)(short) -4205;
            p28.time_usec = (ulong)5503514250558570492L;
            p28.press_diff2 = (short)(short) -461;
            p28.press_diff1 = (short)(short) -21603;
            p28.temperature = (short)(short)16064;
            LoopBackDemoChannel.instance.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -19590);
                Debug.Assert(pack.press_diff == (float)2.116366E38F);
                Debug.Assert(pack.time_boot_ms == (uint)4531707U);
                Debug.Assert(pack.press_abs == (float)1.297155E38F);
            };
            DemoDevice.SCALED_PRESSURE p29 = LoopBackDemoChannel.new_SCALED_PRESSURE();
            PH.setPack(p29);
            p29.press_diff = (float)2.116366E38F;
            p29.press_abs = (float)1.297155E38F;
            p29.time_boot_ms = (uint)4531707U;
            p29.temperature = (short)(short) -19590;
            LoopBackDemoChannel.instance.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float) -8.4639895E37F);
                Debug.Assert(pack.rollspeed == (float) -1.0355466E38F);
                Debug.Assert(pack.yawspeed == (float) -1.0078468E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2740420925U);
                Debug.Assert(pack.roll == (float)1.9744463E38F);
                Debug.Assert(pack.pitchspeed == (float)3.0105623E38F);
                Debug.Assert(pack.pitch == (float) -2.5949812E38F);
            };
            DemoDevice.ATTITUDE p30 = LoopBackDemoChannel.new_ATTITUDE();
            PH.setPack(p30);
            p30.pitchspeed = (float)3.0105623E38F;
            p30.roll = (float)1.9744463E38F;
            p30.pitch = (float) -2.5949812E38F;
            p30.yaw = (float) -8.4639895E37F;
            p30.time_boot_ms = (uint)2740420925U;
            p30.yawspeed = (float) -1.0078468E38F;
            p30.rollspeed = (float) -1.0355466E38F;
            LoopBackDemoChannel.instance.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitchspeed == (float)1.0530174E36F);
                Debug.Assert(pack.time_boot_ms == (uint)760154742U);
                Debug.Assert(pack.q4 == (float)2.8889005E38F);
                Debug.Assert(pack.q2 == (float) -2.9154169E38F);
                Debug.Assert(pack.q1 == (float) -1.2970456E38F);
                Debug.Assert(pack.rollspeed == (float) -1.8690362E38F);
                Debug.Assert(pack.q3 == (float) -6.173298E37F);
                Debug.Assert(pack.yawspeed == (float)1.9280925E38F);
            };
            DemoDevice.ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.time_boot_ms = (uint)760154742U;
            p31.rollspeed = (float) -1.8690362E38F;
            p31.yawspeed = (float)1.9280925E38F;
            p31.pitchspeed = (float)1.0530174E36F;
            p31.q3 = (float) -6.173298E37F;
            p31.q1 = (float) -1.2970456E38F;
            p31.q4 = (float)2.8889005E38F;
            p31.q2 = (float) -2.9154169E38F;
            LoopBackDemoChannel.instance.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -2.486305E38F);
                Debug.Assert(pack.z == (float)5.105967E37F);
                Debug.Assert(pack.vz == (float)4.290866E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3874991545U);
                Debug.Assert(pack.y == (float)3.1546073E38F);
                Debug.Assert(pack.vy == (float)1.0658092E38F);
                Debug.Assert(pack.vx == (float) -2.9180313E38F);
            };
            DemoDevice.LOCAL_POSITION_NED p32 = LoopBackDemoChannel.new_LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.vz = (float)4.290866E37F;
            p32.time_boot_ms = (uint)3874991545U;
            p32.x = (float) -2.486305E38F;
            p32.vy = (float)1.0658092E38F;
            p32.vx = (float) -2.9180313E38F;
            p32.y = (float)3.1546073E38F;
            p32.z = (float)5.105967E37F;
            LoopBackDemoChannel.instance.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hdg == (ushort)(ushort)36482);
                Debug.Assert(pack.vy == (short)(short)428);
                Debug.Assert(pack.alt == (int)1973047865);
                Debug.Assert(pack.vx == (short)(short) -21483);
                Debug.Assert(pack.lat == (int) -89762094);
                Debug.Assert(pack.vz == (short)(short)27506);
                Debug.Assert(pack.time_boot_ms == (uint)4283465828U);
                Debug.Assert(pack.lon == (int)578610660);
                Debug.Assert(pack.relative_alt == (int) -820313340);
            };
            DemoDevice.GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.hdg = (ushort)(ushort)36482;
            p33.vz = (short)(short)27506;
            p33.lon = (int)578610660;
            p33.lat = (int) -89762094;
            p33.vy = (short)(short)428;
            p33.time_boot_ms = (uint)4283465828U;
            p33.vx = (short)(short) -21483;
            p33.relative_alt = (int) -820313340;
            p33.alt = (int)1973047865;
            LoopBackDemoChannel.instance.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan5_scaled == (short)(short) -32560);
                Debug.Assert(pack.chan4_scaled == (short)(short) -21346);
                Debug.Assert(pack.chan8_scaled == (short)(short) -32601);
                Debug.Assert(pack.chan1_scaled == (short)(short)3329);
                Debug.Assert(pack.time_boot_ms == (uint)1035035314U);
                Debug.Assert(pack.rssi == (byte)(byte)78);
                Debug.Assert(pack.chan3_scaled == (short)(short) -17510);
                Debug.Assert(pack.port == (byte)(byte)203);
                Debug.Assert(pack.chan7_scaled == (short)(short) -9053);
                Debug.Assert(pack.chan2_scaled == (short)(short)29907);
                Debug.Assert(pack.chan6_scaled == (short)(short) -27016);
            };
            DemoDevice.RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.new_RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.chan7_scaled = (short)(short) -9053;
            p34.rssi = (byte)(byte)78;
            p34.chan5_scaled = (short)(short) -32560;
            p34.port = (byte)(byte)203;
            p34.chan3_scaled = (short)(short) -17510;
            p34.chan1_scaled = (short)(short)3329;
            p34.chan4_scaled = (short)(short) -21346;
            p34.chan6_scaled = (short)(short) -27016;
            p34.time_boot_ms = (uint)1035035314U;
            p34.chan8_scaled = (short)(short) -32601;
            p34.chan2_scaled = (short)(short)29907;
            LoopBackDemoChannel.instance.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)59986);
                Debug.Assert(pack.rssi == (byte)(byte)159);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)13265);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)35049);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)51329);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)62888);
                Debug.Assert(pack.time_boot_ms == (uint)2283255885U);
                Debug.Assert(pack.port == (byte)(byte)104);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)32996);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)51875);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)19792);
            };
            DemoDevice.RC_CHANNELS_RAW p35 = LoopBackDemoChannel.new_RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.chan4_raw = (ushort)(ushort)51329;
            p35.chan5_raw = (ushort)(ushort)62888;
            p35.chan8_raw = (ushort)(ushort)51875;
            p35.chan1_raw = (ushort)(ushort)59986;
            p35.chan7_raw = (ushort)(ushort)32996;
            p35.chan6_raw = (ushort)(ushort)35049;
            p35.chan2_raw = (ushort)(ushort)19792;
            p35.port = (byte)(byte)104;
            p35.chan3_raw = (ushort)(ushort)13265;
            p35.rssi = (byte)(byte)159;
            p35.time_boot_ms = (uint)2283255885U;
            LoopBackDemoChannel.instance.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)57746);
                Debug.Assert(pack.port == (byte)(byte)8);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)35775);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)20493);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)50877);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)1788);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)29614);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)50215);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)55969);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)63415);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)40335);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)44216);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)44373);
                Debug.Assert(pack.time_usec == (uint)928597387U);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)23845);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)51500);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)34848);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)49751);
            };
            DemoDevice.SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.new_SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo8_raw = (ushort)(ushort)44373;
            p36.servo2_raw = (ushort)(ushort)57746;
            p36.servo4_raw = (ushort)(ushort)1788;
            p36.servo10_raw_SET((ushort)(ushort)23845, PH) ;
            p36.servo6_raw = (ushort)(ushort)29614;
            p36.servo11_raw_SET((ushort)(ushort)40335, PH) ;
            p36.servo13_raw_SET((ushort)(ushort)55969, PH) ;
            p36.time_usec = (uint)928597387U;
            p36.servo5_raw = (ushort)(ushort)49751;
            p36.servo16_raw_SET((ushort)(ushort)20493, PH) ;
            p36.servo15_raw_SET((ushort)(ushort)50877, PH) ;
            p36.servo3_raw = (ushort)(ushort)34848;
            p36.port = (byte)(byte)8;
            p36.servo7_raw = (ushort)(ushort)63415;
            p36.servo1_raw = (ushort)(ushort)51500;
            p36.servo14_raw_SET((ushort)(ushort)50215, PH) ;
            p36.servo12_raw_SET((ushort)(ushort)35775, PH) ;
            p36.servo9_raw_SET((ushort)(ushort)44216, PH) ;
            LoopBackDemoChannel.instance.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.end_index == (short)(short)29159);
                Debug.Assert(pack.target_component == (byte)(byte)148);
                Debug.Assert(pack.start_index == (short)(short) -10069);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_system == (byte)(byte)192);
            };
            DemoDevice.MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.new_MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.target_component = (byte)(byte)148;
            p37.end_index = (short)(short)29159;
            p37.target_system = (byte)(byte)192;
            p37.start_index = (short)(short) -10069;
            p37.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            LoopBackDemoChannel.instance.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_index == (short)(short) -22851);
                Debug.Assert(pack.end_index == (short)(short) -21149);
                Debug.Assert(pack.target_system == (byte)(byte)210);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_component == (byte)(byte)12);
            };
            DemoDevice.MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.new_MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.start_index = (short)(short) -22851;
            p38.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p38.target_system = (byte)(byte)210;
            p38.end_index = (short)(short) -21149;
            p38.target_component = (byte)(byte)12;
            LoopBackDemoChannel.instance.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param4 == (float) -9.598555E37F);
                Debug.Assert(pack.z == (float) -1.8408467E38F);
                Debug.Assert(pack.target_component == (byte)(byte)157);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_MISSION_START);
                Debug.Assert(pack.seq == (ushort)(ushort)35938);
                Debug.Assert(pack.param1 == (float)1.910828E38F);
                Debug.Assert(pack.x == (float) -5.039837E37F);
                Debug.Assert(pack.param3 == (float) -2.5967871E38F);
                Debug.Assert(pack.target_system == (byte)(byte)168);
                Debug.Assert(pack.current == (byte)(byte)243);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.y == (float)3.0510086E38F);
                Debug.Assert(pack.param2 == (float) -9.311323E37F);
                Debug.Assert(pack.autocontinue == (byte)(byte)194);
            };
            DemoDevice.MISSION_ITEM p39 = LoopBackDemoChannel.new_MISSION_ITEM();
            PH.setPack(p39);
            p39.seq = (ushort)(ushort)35938;
            p39.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p39.y = (float)3.0510086E38F;
            p39.target_component = (byte)(byte)157;
            p39.current = (byte)(byte)243;
            p39.param1 = (float)1.910828E38F;
            p39.autocontinue = (byte)(byte)194;
            p39.target_system = (byte)(byte)168;
            p39.param3 = (float) -2.5967871E38F;
            p39.param4 = (float) -9.598555E37F;
            p39.x = (float) -5.039837E37F;
            p39.param2 = (float) -9.311323E37F;
            p39.command = (MAV_CMD)MAV_CMD.MAV_CMD_MISSION_START;
            p39.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p39.z = (float) -1.8408467E38F;
            LoopBackDemoChannel.instance.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)84);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)210);
                Debug.Assert(pack.seq == (ushort)(ushort)41406);
            };
            DemoDevice.MISSION_REQUEST p40 = LoopBackDemoChannel.new_MISSION_REQUEST();
            PH.setPack(p40);
            p40.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p40.target_component = (byte)(byte)210;
            p40.seq = (ushort)(ushort)41406;
            p40.target_system = (byte)(byte)84;
            LoopBackDemoChannel.instance.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)48);
                Debug.Assert(pack.target_system == (byte)(byte)224);
                Debug.Assert(pack.seq == (ushort)(ushort)45123);
            };
            DemoDevice.MISSION_SET_CURRENT p41 = LoopBackDemoChannel.new_MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_component = (byte)(byte)48;
            p41.target_system = (byte)(byte)224;
            p41.seq = (ushort)(ushort)45123;
            LoopBackDemoChannel.instance.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)8815);
            };
            DemoDevice.MISSION_CURRENT p42 = LoopBackDemoChannel.new_MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)8815;
            LoopBackDemoChannel.instance.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_component == (byte)(byte)134);
                Debug.Assert(pack.target_system == (byte)(byte)213);
            };
            DemoDevice.MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.new_MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p43.target_component = (byte)(byte)134;
            p43.target_system = (byte)(byte)213;
            LoopBackDemoChannel.instance.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)167);
                Debug.Assert(pack.count == (ushort)(ushort)50008);
                Debug.Assert(pack.target_system == (byte)(byte)89);
            };
            DemoDevice.MISSION_COUNT p44 = LoopBackDemoChannel.new_MISSION_COUNT();
            PH.setPack(p44);
            p44.target_component = (byte)(byte)167;
            p44.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p44.target_system = (byte)(byte)89;
            p44.count = (ushort)(ushort)50008;
            LoopBackDemoChannel.instance.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)154);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_system == (byte)(byte)41);
            };
            DemoDevice.MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.new_MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_system = (byte)(byte)41;
            p45.target_component = (byte)(byte)154;
            p45.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            LoopBackDemoChannel.instance.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)23325);
            };
            DemoDevice.MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.new_MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)23325;
            LoopBackDemoChannel.instance.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)168);
                Debug.Assert(pack.type == (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM7);
                Debug.Assert(pack.target_system == (byte)(byte)47);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            };
            DemoDevice.MISSION_ACK p47 = LoopBackDemoChannel.new_MISSION_ACK();
            PH.setPack(p47);
            p47.target_component = (byte)(byte)168;
            p47.target_system = (byte)(byte)47;
            p47.type = (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM7;
            p47.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            LoopBackDemoChannel.instance.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.longitude == (int) -1067538403);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)102048465298499519L);
                Debug.Assert(pack.altitude == (int)1383696764);
                Debug.Assert(pack.target_system == (byte)(byte)123);
                Debug.Assert(pack.latitude == (int) -1947961582);
            };
            DemoDevice.SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.new_SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.latitude = (int) -1947961582;
            p48.target_system = (byte)(byte)123;
            p48.time_usec_SET((ulong)102048465298499519L, PH) ;
            p48.longitude = (int) -1067538403;
            p48.altitude = (int)1383696764;
            LoopBackDemoChannel.instance.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.latitude == (int)2046278159);
                Debug.Assert(pack.longitude == (int) -2145116929);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)4485907259477607467L);
                Debug.Assert(pack.altitude == (int) -294963748);
            };
            DemoDevice.GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.new_GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.altitude = (int) -294963748;
            p49.longitude = (int) -2145116929;
            p49.latitude = (int)2046278159;
            p49.time_usec_SET((ulong)4485907259477607467L, PH) ;
            LoopBackDemoChannel.instance.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_min == (float)2.8884182E38F);
                Debug.Assert(pack.param_value_max == (float) -3.072354E38F);
                Debug.Assert(pack.target_system == (byte)(byte)222);
                Debug.Assert(pack.scale == (float) -3.3033129E38F);
                Debug.Assert(pack.param_index == (short)(short)18115);
                Debug.Assert(pack.param_value0 == (float) -2.402549E38F);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)179);
                Debug.Assert(pack.param_id_LEN(ph) == 9);
                Debug.Assert(pack.param_id_TRY(ph).Equals("loXuasdqR"));
                Debug.Assert(pack.target_component == (byte)(byte)194);
            };
            DemoDevice.PARAM_MAP_RC p50 = LoopBackDemoChannel.new_PARAM_MAP_RC();
            PH.setPack(p50);
            p50.parameter_rc_channel_index = (byte)(byte)179;
            p50.param_value_max = (float) -3.072354E38F;
            p50.target_system = (byte)(byte)222;
            p50.scale = (float) -3.3033129E38F;
            p50.param_id_SET("loXuasdqR", PH) ;
            p50.target_component = (byte)(byte)194;
            p50.param_index = (short)(short)18115;
            p50.param_value_min = (float)2.8884182E38F;
            p50.param_value0 = (float) -2.402549E38F;
            LoopBackDemoChannel.instance.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.seq == (ushort)(ushort)29054);
                Debug.Assert(pack.target_component == (byte)(byte)219);
                Debug.Assert(pack.target_system == (byte)(byte)36);
            };
            DemoDevice.MISSION_REQUEST_INT p51 = LoopBackDemoChannel.new_MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.target_component = (byte)(byte)219;
            p51.seq = (ushort)(ushort)29054;
            p51.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p51.target_system = (byte)(byte)36;
            LoopBackDemoChannel.instance.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.target_component == (byte)(byte)250);
                Debug.Assert(pack.p2z == (float)2.7590527E38F);
                Debug.Assert(pack.p1z == (float) -2.7593441E38F);
                Debug.Assert(pack.p2y == (float) -1.4116376E38F);
                Debug.Assert(pack.target_system == (byte)(byte)209);
                Debug.Assert(pack.p2x == (float) -2.1307014E37F);
                Debug.Assert(pack.p1y == (float)1.6578209E37F);
                Debug.Assert(pack.p1x == (float)1.3661652E38F);
            };
            DemoDevice.SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.new_SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p1x = (float)1.3661652E38F;
            p54.p2x = (float) -2.1307014E37F;
            p54.target_system = (byte)(byte)209;
            p54.p1z = (float) -2.7593441E38F;
            p54.p2y = (float) -1.4116376E38F;
            p54.p1y = (float)1.6578209E37F;
            p54.target_component = (byte)(byte)250;
            p54.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p54.p2z = (float)2.7590527E38F;
            LoopBackDemoChannel.instance.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p1y == (float)1.7816827E37F);
                Debug.Assert(pack.p2z == (float)2.7977402E38F);
                Debug.Assert(pack.p1x == (float)2.8598544E38F);
                Debug.Assert(pack.p2x == (float) -3.2215697E38F);
                Debug.Assert(pack.p1z == (float)2.6679072E38F);
                Debug.Assert(pack.p2y == (float)1.1191697E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU);
            };
            DemoDevice.SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.new_SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p1y = (float)1.7816827E37F;
            p55.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p55.p1x = (float)2.8598544E38F;
            p55.p1z = (float)2.6679072E38F;
            p55.p2x = (float) -3.2215697E38F;
            p55.p2z = (float)2.7977402E38F;
            p55.p2y = (float)1.1191697E38F;
            LoopBackDemoChannel.instance.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rollspeed == (float)9.638241E37F);
                Debug.Assert(pack.pitchspeed == (float) -2.6967003E38F);
                Debug.Assert(pack.time_usec == (ulong)3316292829855624990L);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {1.2796258E38F, 2.1602726E38F, 7.047289E37F, -2.2411562E38F, 2.147176E38F, 3.0994297E38F, -2.8700109E38F, -1.5342568E38F, 1.9033975E38F}));
                Debug.Assert(pack.q.SequenceEqual(new float[] {-3.3463197E38F, -8.46327E37F, 1.23846E38F, 3.1061555E38F}));
                Debug.Assert(pack.yawspeed == (float)2.790283E36F);
            };
            DemoDevice.ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.pitchspeed = (float) -2.6967003E38F;
            p61.time_usec = (ulong)3316292829855624990L;
            p61.yawspeed = (float)2.790283E36F;
            p61.covariance_SET(new float[] {1.2796258E38F, 2.1602726E38F, 7.047289E37F, -2.2411562E38F, 2.147176E38F, 3.0994297E38F, -2.8700109E38F, -1.5342568E38F, 1.9033975E38F}, 0) ;
            p61.rollspeed = (float)9.638241E37F;
            p61.q_SET(new float[] {-3.3463197E38F, -8.46327E37F, 1.23846E38F, 3.1061555E38F}, 0) ;
            LoopBackDemoChannel.instance.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.nav_roll == (float)1.8665934E38F);
                Debug.Assert(pack.alt_error == (float)2.7730735E38F);
                Debug.Assert(pack.xtrack_error == (float) -2.4519052E38F);
                Debug.Assert(pack.nav_pitch == (float)3.059755E38F);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)21256);
                Debug.Assert(pack.aspd_error == (float) -1.5541791E38F);
                Debug.Assert(pack.nav_bearing == (short)(short)741);
                Debug.Assert(pack.target_bearing == (short)(short) -15643);
            };
            DemoDevice.NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.new_NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.nav_roll = (float)1.8665934E38F;
            p62.aspd_error = (float) -1.5541791E38F;
            p62.nav_bearing = (short)(short)741;
            p62.xtrack_error = (float) -2.4519052E38F;
            p62.alt_error = (float)2.7730735E38F;
            p62.target_bearing = (short)(short) -15643;
            p62.nav_pitch = (float)3.059755E38F;
            p62.wp_dist = (ushort)(ushort)21256;
            LoopBackDemoChannel.instance.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (float)3.6226333E37F);
                Debug.Assert(pack.relative_alt == (int) -882015550);
                Debug.Assert(pack.lon == (int) -746827639);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {1.8322553E38F, -2.6602035E38F, 1.5781573E38F, 1.5274445E38F, -3.3315693E38F, 3.3162953E38F, -2.6621918E38F, 2.9670129E38F, -2.2166288E38F, -8.341628E37F, -9.418368E37F, -3.2752453E38F, 9.450478E37F, -2.989918E38F, 1.500865E38F, 1.0543385E38F, -1.0836265E38F, 2.129382E38F, 8.955431E37F, 2.9781417E38F, -2.0135374E38F, -5.7930334E37F, -2.257882E38F, 3.6114235E37F, -2.252897E38F, -1.3808841E38F, -2.1672586E38F, -2.9775222E38F, 2.3870883E38F, -2.0011844E38F, 2.035135E38F, 3.20716E38F, -1.4221675E38F, 4.315421E36F, 3.022475E38F, 1.9616842E38F}));
                Debug.Assert(pack.vy == (float)9.245753E37F);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE);
                Debug.Assert(pack.vz == (float)8.2263025E37F);
                Debug.Assert(pack.alt == (int) -1763698672);
                Debug.Assert(pack.lat == (int) -1657172942);
                Debug.Assert(pack.time_usec == (ulong)3134734095655337638L);
            };
            DemoDevice.GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.relative_alt = (int) -882015550;
            p63.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE;
            p63.alt = (int) -1763698672;
            p63.vz = (float)8.2263025E37F;
            p63.vx = (float)3.6226333E37F;
            p63.time_usec = (ulong)3134734095655337638L;
            p63.vy = (float)9.245753E37F;
            p63.lon = (int) -746827639;
            p63.covariance_SET(new float[] {1.8322553E38F, -2.6602035E38F, 1.5781573E38F, 1.5274445E38F, -3.3315693E38F, 3.3162953E38F, -2.6621918E38F, 2.9670129E38F, -2.2166288E38F, -8.341628E37F, -9.418368E37F, -3.2752453E38F, 9.450478E37F, -2.989918E38F, 1.500865E38F, 1.0543385E38F, -1.0836265E38F, 2.129382E38F, 8.955431E37F, 2.9781417E38F, -2.0135374E38F, -5.7930334E37F, -2.257882E38F, 3.6114235E37F, -2.252897E38F, -1.3808841E38F, -2.1672586E38F, -2.9775222E38F, 2.3870883E38F, -2.0011844E38F, 2.035135E38F, 3.20716E38F, -1.4221675E38F, 4.315421E36F, 3.022475E38F, 1.9616842E38F}, 0) ;
            p63.lat = (int) -1657172942;
            LoopBackDemoChannel.instance.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float) -1.1390734E38F);
                Debug.Assert(pack.ay == (float)2.0297295E38F);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
                Debug.Assert(pack.y == (float)2.3104188E38F);
                Debug.Assert(pack.z == (float) -6.9994413E37F);
                Debug.Assert(pack.ax == (float) -1.0874008E38F);
                Debug.Assert(pack.vx == (float)8.592389E37F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-2.7378886E38F, 2.7676526E38F, 1.0728763E37F, -1.9035386E38F, 2.8370876E38F, -1.9148185E38F, 5.057983E37F, 3.39058E38F, 1.1279668E36F, -2.9616109E38F, 4.372675E37F, 4.5913843E37F, 3.3314655E38F, 8.1656596E37F, -1.6536488E38F, -3.1876776E38F, -5.6580976E37F, 2.852299E38F, 2.3867332E38F, 1.6715024E38F, 3.3870667E38F, 3.0811643E38F, 1.1285999E38F, -6.47601E37F, 2.7078124E38F, 2.0146465E38F, 3.298385E38F, -2.1745597E38F, -2.1855122E38F, 1.7354098E38F, 1.0072241E38F, -1.5094897E38F, 1.2895211E38F, 3.1352318E38F, 7.3757074E37F, 2.510635E38F, -1.9524038E38F, -1.4096622E38F, -3.0126903E38F, -6.592176E37F, -1.0411788E38F, 3.0824853E38F, 1.1600005E38F, -2.7547135E38F, 2.9076923E38F}));
                Debug.Assert(pack.vz == (float)2.637261E38F);
                Debug.Assert(pack.az == (float) -1.9692682E38F);
                Debug.Assert(pack.time_usec == (ulong)7380849526849486865L);
                Debug.Assert(pack.x == (float)1.3725263E38F);
            };
            DemoDevice.LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.vz = (float)2.637261E38F;
            p64.az = (float) -1.9692682E38F;
            p64.covariance_SET(new float[] {-2.7378886E38F, 2.7676526E38F, 1.0728763E37F, -1.9035386E38F, 2.8370876E38F, -1.9148185E38F, 5.057983E37F, 3.39058E38F, 1.1279668E36F, -2.9616109E38F, 4.372675E37F, 4.5913843E37F, 3.3314655E38F, 8.1656596E37F, -1.6536488E38F, -3.1876776E38F, -5.6580976E37F, 2.852299E38F, 2.3867332E38F, 1.6715024E38F, 3.3870667E38F, 3.0811643E38F, 1.1285999E38F, -6.47601E37F, 2.7078124E38F, 2.0146465E38F, 3.298385E38F, -2.1745597E38F, -2.1855122E38F, 1.7354098E38F, 1.0072241E38F, -1.5094897E38F, 1.2895211E38F, 3.1352318E38F, 7.3757074E37F, 2.510635E38F, -1.9524038E38F, -1.4096622E38F, -3.0126903E38F, -6.592176E37F, -1.0411788E38F, 3.0824853E38F, 1.1600005E38F, -2.7547135E38F, 2.9076923E38F}, 0) ;
            p64.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS;
            p64.ax = (float) -1.0874008E38F;
            p64.vy = (float) -1.1390734E38F;
            p64.vx = (float)8.592389E37F;
            p64.time_usec = (ulong)7380849526849486865L;
            p64.x = (float)1.3725263E38F;
            p64.z = (float) -6.9994413E37F;
            p64.y = (float)2.3104188E38F;
            p64.ay = (float)2.0297295E38F;
            LoopBackDemoChannel.instance.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)37638);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)18677);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)52769);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)33226);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)57694);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)13073);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)26517);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)26324);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)18178);
                Debug.Assert(pack.rssi == (byte)(byte)57);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)20286);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)61810);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)40467);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)31790);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)59380);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)65404);
                Debug.Assert(pack.time_boot_ms == (uint)572726900U);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)13132);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)56852);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)64777);
                Debug.Assert(pack.chancount == (byte)(byte)144);
            };
            DemoDevice.RC_CHANNELS p65 = LoopBackDemoChannel.new_RC_CHANNELS();
            PH.setPack(p65);
            p65.time_boot_ms = (uint)572726900U;
            p65.chan1_raw = (ushort)(ushort)31790;
            p65.chan13_raw = (ushort)(ushort)59380;
            p65.chan4_raw = (ushort)(ushort)65404;
            p65.chan16_raw = (ushort)(ushort)13132;
            p65.chan2_raw = (ushort)(ushort)40467;
            p65.chan11_raw = (ushort)(ushort)26324;
            p65.chan12_raw = (ushort)(ushort)18178;
            p65.chan7_raw = (ushort)(ushort)33226;
            p65.chan17_raw = (ushort)(ushort)57694;
            p65.chan15_raw = (ushort)(ushort)61810;
            p65.rssi = (byte)(byte)57;
            p65.chan14_raw = (ushort)(ushort)26517;
            p65.chan18_raw = (ushort)(ushort)13073;
            p65.chan8_raw = (ushort)(ushort)56852;
            p65.chan5_raw = (ushort)(ushort)20286;
            p65.chan9_raw = (ushort)(ushort)52769;
            p65.chan6_raw = (ushort)(ushort)64777;
            p65.chan10_raw = (ushort)(ushort)37638;
            p65.chancount = (byte)(byte)144;
            p65.chan3_raw = (ushort)(ushort)18677;
            LoopBackDemoChannel.instance.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.req_stream_id == (byte)(byte)15);
                Debug.Assert(pack.target_system == (byte)(byte)75);
                Debug.Assert(pack.start_stop == (byte)(byte)156);
                Debug.Assert(pack.target_component == (byte)(byte)197);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)57923);
            };
            DemoDevice.REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.new_REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.req_message_rate = (ushort)(ushort)57923;
            p66.start_stop = (byte)(byte)156;
            p66.target_component = (byte)(byte)197;
            p66.target_system = (byte)(byte)75;
            p66.req_stream_id = (byte)(byte)15;
            LoopBackDemoChannel.instance.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_rate == (ushort)(ushort)55324);
                Debug.Assert(pack.stream_id == (byte)(byte)178);
                Debug.Assert(pack.on_off == (byte)(byte)40);
            };
            DemoDevice.DATA_STREAM p67 = LoopBackDemoChannel.new_DATA_STREAM();
            PH.setPack(p67);
            p67.message_rate = (ushort)(ushort)55324;
            p67.on_off = (byte)(byte)40;
            p67.stream_id = (byte)(byte)178;
            LoopBackDemoChannel.instance.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target == (byte)(byte)155);
                Debug.Assert(pack.r == (short)(short) -31106);
                Debug.Assert(pack.z == (short)(short) -27599);
                Debug.Assert(pack.x == (short)(short)29970);
                Debug.Assert(pack.y == (short)(short) -15373);
                Debug.Assert(pack.buttons == (ushort)(ushort)8741);
            };
            DemoDevice.MANUAL_CONTROL p69 = LoopBackDemoChannel.new_MANUAL_CONTROL();
            PH.setPack(p69);
            p69.z = (short)(short) -27599;
            p69.x = (short)(short)29970;
            p69.target = (byte)(byte)155;
            p69.buttons = (ushort)(ushort)8741;
            p69.y = (short)(short) -15373;
            p69.r = (short)(short) -31106;
            LoopBackDemoChannel.instance.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)39217);
                Debug.Assert(pack.target_component == (byte)(byte)137);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)41220);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)55154);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)20983);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)22668);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)20456);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)16268);
                Debug.Assert(pack.target_system == (byte)(byte)88);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)20643);
            };
            DemoDevice.RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.new_RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan1_raw = (ushort)(ushort)41220;
            p70.chan4_raw = (ushort)(ushort)22668;
            p70.chan7_raw = (ushort)(ushort)20983;
            p70.chan2_raw = (ushort)(ushort)16268;
            p70.chan8_raw = (ushort)(ushort)20456;
            p70.chan5_raw = (ushort)(ushort)55154;
            p70.chan6_raw = (ushort)(ushort)39217;
            p70.target_component = (byte)(byte)137;
            p70.target_system = (byte)(byte)88;
            p70.chan3_raw = (ushort)(ushort)20643;
            LoopBackDemoChannel.instance.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)3);
                Debug.Assert(pack.autocontinue == (byte)(byte)168);
                Debug.Assert(pack.x == (int) -865122547);
                Debug.Assert(pack.y == (int)373180349);
                Debug.Assert(pack.target_system == (byte)(byte)13);
                Debug.Assert(pack.param1 == (float) -3.2742557E38F);
                Debug.Assert(pack.param4 == (float)3.756827E37F);
                Debug.Assert(pack.param2 == (float)1.3289583E38F);
                Debug.Assert(pack.z == (float) -3.1918722E38F);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.current == (byte)(byte)141);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.param3 == (float)7.248508E37F);
                Debug.Assert(pack.seq == (ushort)(ushort)37038);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE);
            };
            DemoDevice.MISSION_ITEM_INT p73 = LoopBackDemoChannel.new_MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.param2 = (float)1.3289583E38F;
            p73.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p73.seq = (ushort)(ushort)37038;
            p73.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION;
            p73.target_system = (byte)(byte)13;
            p73.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE;
            p73.x = (int) -865122547;
            p73.param4 = (float)3.756827E37F;
            p73.current = (byte)(byte)141;
            p73.target_component = (byte)(byte)3;
            p73.param3 = (float)7.248508E37F;
            p73.autocontinue = (byte)(byte)168;
            p73.y = (int)373180349;
            p73.z = (float) -3.1918722E38F;
            p73.param1 = (float) -3.2742557E38F;
            LoopBackDemoChannel.instance.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.heading == (short)(short) -17138);
                Debug.Assert(pack.climb == (float) -1.7863463E38F);
                Debug.Assert(pack.throttle == (ushort)(ushort)37891);
                Debug.Assert(pack.alt == (float)3.1829273E38F);
                Debug.Assert(pack.airspeed == (float)1.9528815E38F);
                Debug.Assert(pack.groundspeed == (float) -1.94848E38F);
            };
            DemoDevice.VFR_HUD p74 = LoopBackDemoChannel.new_VFR_HUD();
            PH.setPack(p74);
            p74.climb = (float) -1.7863463E38F;
            p74.groundspeed = (float) -1.94848E38F;
            p74.heading = (short)(short) -17138;
            p74.alt = (float)3.1829273E38F;
            p74.airspeed = (float)1.9528815E38F;
            p74.throttle = (ushort)(ushort)37891;
            LoopBackDemoChannel.instance.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_NAV_LOITER_TIME);
                Debug.Assert(pack.autocontinue == (byte)(byte)237);
                Debug.Assert(pack.target_system == (byte)(byte)68);
                Debug.Assert(pack.z == (float)2.2784711E38F);
                Debug.Assert(pack.target_component == (byte)(byte)212);
                Debug.Assert(pack.current == (byte)(byte)50);
                Debug.Assert(pack.y == (int) -764065097);
                Debug.Assert(pack.x == (int) -44287621);
                Debug.Assert(pack.param3 == (float) -2.5854254E36F);
                Debug.Assert(pack.param4 == (float) -3.4580553E37F);
                Debug.Assert(pack.param2 == (float) -1.1768185E38F);
                Debug.Assert(pack.param1 == (float)2.6873982E38F);
            };
            DemoDevice.COMMAND_INT p75 = LoopBackDemoChannel.new_COMMAND_INT();
            PH.setPack(p75);
            p75.y = (int) -764065097;
            p75.x = (int) -44287621;
            p75.current = (byte)(byte)50;
            p75.param4 = (float) -3.4580553E37F;
            p75.z = (float)2.2784711E38F;
            p75.target_component = (byte)(byte)212;
            p75.param1 = (float)2.6873982E38F;
            p75.param2 = (float) -1.1768185E38F;
            p75.autocontinue = (byte)(byte)237;
            p75.command = (MAV_CMD)MAV_CMD.MAV_CMD_NAV_LOITER_TIME;
            p75.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p75.target_system = (byte)(byte)68;
            p75.param3 = (float) -2.5854254E36F;
            LoopBackDemoChannel.instance.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE);
                Debug.Assert(pack.param7 == (float)2.7121774E38F);
                Debug.Assert(pack.param4 == (float)2.7836404E38F);
                Debug.Assert(pack.param6 == (float) -2.506677E37F);
                Debug.Assert(pack.param5 == (float)1.123894E38F);
                Debug.Assert(pack.param2 == (float) -9.417694E37F);
                Debug.Assert(pack.param1 == (float) -3.0111097E38F);
                Debug.Assert(pack.target_component == (byte)(byte)201);
                Debug.Assert(pack.target_system == (byte)(byte)95);
                Debug.Assert(pack.confirmation == (byte)(byte)116);
                Debug.Assert(pack.param3 == (float) -7.9295E36F);
            };
            DemoDevice.COMMAND_LONG p76 = LoopBackDemoChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.param6 = (float) -2.506677E37F;
            p76.param4 = (float)2.7836404E38F;
            p76.param7 = (float)2.7121774E38F;
            p76.param3 = (float) -7.9295E36F;
            p76.confirmation = (byte)(byte)116;
            p76.param1 = (float) -3.0111097E38F;
            p76.command = (MAV_CMD)MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE;
            p76.target_component = (byte)(byte)201;
            p76.param5 = (float)1.123894E38F;
            p76.param2 = (float) -9.417694E37F;
            p76.target_system = (byte)(byte)95;
            LoopBackDemoChannel.instance.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)249);
                Debug.Assert(pack.result == (MAV_RESULT)MAV_RESULT.MAV_RESULT_IN_PROGRESS);
                Debug.Assert(pack.result_param2_TRY(ph) == (int) -1768988950);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_SET_HOME);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)153);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)201);
            };
            DemoDevice.COMMAND_ACK p77 = LoopBackDemoChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_SET_HOME;
            p77.progress_SET((byte)(byte)201, PH) ;
            p77.target_component_SET((byte)(byte)153, PH) ;
            p77.result = (MAV_RESULT)MAV_RESULT.MAV_RESULT_IN_PROGRESS;
            p77.result_param2_SET((int) -1768988950, PH) ;
            p77.target_system_SET((byte)(byte)249, PH) ;
            LoopBackDemoChannel.instance.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float)1.3439531E38F);
                Debug.Assert(pack.mode_switch == (byte)(byte)88);
                Debug.Assert(pack.yaw == (float)7.811822E37F);
                Debug.Assert(pack.time_boot_ms == (uint)1664078585U);
                Debug.Assert(pack.thrust == (float) -1.0130572E38F);
                Debug.Assert(pack.pitch == (float)1.0972495E38F);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)47);
            };
            DemoDevice.MANUAL_SETPOINT p81 = LoopBackDemoChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.mode_switch = (byte)(byte)88;
            p81.thrust = (float) -1.0130572E38F;
            p81.manual_override_switch = (byte)(byte)47;
            p81.yaw = (float)7.811822E37F;
            p81.roll = (float)1.3439531E38F;
            p81.pitch = (float)1.0972495E38F;
            p81.time_boot_ms = (uint)1664078585U;
            LoopBackDemoChannel.instance.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type_mask == (byte)(byte)15);
                Debug.Assert(pack.target_component == (byte)(byte)93);
                Debug.Assert(pack.thrust == (float)1.5832548E38F);
                Debug.Assert(pack.body_pitch_rate == (float)1.9289172E38F);
                Debug.Assert(pack.body_yaw_rate == (float)1.1692016E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1575554951U);
                Debug.Assert(pack.target_system == (byte)(byte)9);
                Debug.Assert(pack.body_roll_rate == (float)1.9492663E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-6.961762E37F, 1.6853453E38F, -1.2377408E38F, -1.5687441E38F}));
            };
            DemoDevice.SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.body_roll_rate = (float)1.9492663E38F;
            p82.target_system = (byte)(byte)9;
            p82.thrust = (float)1.5832548E38F;
            p82.time_boot_ms = (uint)1575554951U;
            p82.type_mask = (byte)(byte)15;
            p82.body_yaw_rate = (float)1.1692016E38F;
            p82.q_SET(new float[] {-6.961762E37F, 1.6853453E38F, -1.2377408E38F, -1.5687441E38F}, 0) ;
            p82.body_pitch_rate = (float)1.9289172E38F;
            p82.target_component = (byte)(byte)93;
            LoopBackDemoChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.body_roll_rate == (float) -2.5139489E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-4.3328597E37F, -1.867656E38F, -6.9536456E37F, -2.720891E38F}));
                Debug.Assert(pack.body_yaw_rate == (float)1.6258923E38F);
                Debug.Assert(pack.thrust == (float)8.28734E37F);
                Debug.Assert(pack.type_mask == (byte)(byte)250);
                Debug.Assert(pack.body_pitch_rate == (float)1.2897818E37F);
                Debug.Assert(pack.time_boot_ms == (uint)4184677085U);
            };
            DemoDevice.ATTITUDE_TARGET p83 = LoopBackDemoChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.body_yaw_rate = (float)1.6258923E38F;
            p83.time_boot_ms = (uint)4184677085U;
            p83.body_roll_rate = (float) -2.5139489E38F;
            p83.thrust = (float)8.28734E37F;
            p83.body_pitch_rate = (float)1.2897818E37F;
            p83.q_SET(new float[] {-4.3328597E37F, -1.867656E38F, -6.9536456E37F, -2.720891E38F}, 0) ;
            p83.type_mask = (byte)(byte)250;
            LoopBackDemoChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float)2.465471E37F);
                Debug.Assert(pack.target_component == (byte)(byte)34);
                Debug.Assert(pack.vz == (float) -1.1095037E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)40280);
                Debug.Assert(pack.afy == (float) -1.0539896E38F);
                Debug.Assert(pack.vx == (float) -1.3260904E38F);
                Debug.Assert(pack.yaw == (float) -9.652511E37F);
                Debug.Assert(pack.z == (float) -5.610029E37F);
                Debug.Assert(pack.time_boot_ms == (uint)1153820070U);
                Debug.Assert(pack.target_system == (byte)(byte)96);
                Debug.Assert(pack.x == (float)2.0576827E38F);
                Debug.Assert(pack.afz == (float) -2.6471622E37F);
                Debug.Assert(pack.afx == (float) -1.0104406E38F);
                Debug.Assert(pack.y == (float)2.7812625E38F);
                Debug.Assert(pack.yaw_rate == (float)2.028178E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL);
            };
            DemoDevice.SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.z = (float) -5.610029E37F;
            p84.x = (float)2.0576827E38F;
            p84.afz = (float) -2.6471622E37F;
            p84.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            p84.vx = (float) -1.3260904E38F;
            p84.target_system = (byte)(byte)96;
            p84.target_component = (byte)(byte)34;
            p84.afx = (float) -1.0104406E38F;
            p84.yaw_rate = (float)2.028178E38F;
            p84.vy = (float)2.465471E37F;
            p84.afy = (float) -1.0539896E38F;
            p84.vz = (float) -1.1095037E38F;
            p84.type_mask = (ushort)(ushort)40280;
            p84.yaw = (float) -9.652511E37F;
            p84.y = (float)2.7812625E38F;
            p84.time_boot_ms = (uint)1153820070U;
            LoopBackDemoChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type_mask == (ushort)(ushort)51228);
                Debug.Assert(pack.afy == (float)2.7810096E38F);
                Debug.Assert(pack.lon_int == (int) -1337590134);
                Debug.Assert(pack.yaw == (float) -2.1687885E38F);
                Debug.Assert(pack.alt == (float)5.7532145E37F);
                Debug.Assert(pack.lat_int == (int)2060533694);
                Debug.Assert(pack.target_system == (byte)(byte)169);
                Debug.Assert(pack.afz == (float)1.8482356E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3599089214U);
                Debug.Assert(pack.vz == (float)2.592904E38F);
                Debug.Assert(pack.yaw_rate == (float) -9.006224E37F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT);
                Debug.Assert(pack.afx == (float)2.0940813E38F);
                Debug.Assert(pack.target_component == (byte)(byte)235);
                Debug.Assert(pack.vy == (float)1.180845E37F);
                Debug.Assert(pack.vx == (float)1.1014023E38F);
            };
            DemoDevice.SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.vy = (float)1.180845E37F;
            p86.afz = (float)1.8482356E38F;
            p86.alt = (float)5.7532145E37F;
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p86.vx = (float)1.1014023E38F;
            p86.lat_int = (int)2060533694;
            p86.afy = (float)2.7810096E38F;
            p86.afx = (float)2.0940813E38F;
            p86.yaw_rate = (float) -9.006224E37F;
            p86.target_system = (byte)(byte)169;
            p86.lon_int = (int) -1337590134;
            p86.target_component = (byte)(byte)235;
            p86.time_boot_ms = (uint)3599089214U;
            p86.type_mask = (ushort)(ushort)51228;
            p86.vz = (float)2.592904E38F;
            p86.yaw = (float) -2.1687885E38F;
            LoopBackDemoChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.afy == (float) -1.2083072E38F);
                Debug.Assert(pack.afx == (float) -3.756781E37F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.afz == (float) -1.7643344E38F);
                Debug.Assert(pack.yaw == (float)8.843929E37F);
                Debug.Assert(pack.alt == (float)2.107209E38F);
                Debug.Assert(pack.vx == (float) -1.0412062E38F);
                Debug.Assert(pack.vy == (float) -9.4099745E36F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)27621);
                Debug.Assert(pack.time_boot_ms == (uint)2487346516U);
                Debug.Assert(pack.lat_int == (int)393920423);
                Debug.Assert(pack.vz == (float)1.4473115E38F);
                Debug.Assert(pack.lon_int == (int)609516271);
                Debug.Assert(pack.yaw_rate == (float) -3.241755E38F);
            };
            DemoDevice.POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.lat_int = (int)393920423;
            p87.yaw_rate = (float) -3.241755E38F;
            p87.vz = (float)1.4473115E38F;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p87.type_mask = (ushort)(ushort)27621;
            p87.afz = (float) -1.7643344E38F;
            p87.time_boot_ms = (uint)2487346516U;
            p87.alt = (float)2.107209E38F;
            p87.afx = (float) -3.756781E37F;
            p87.yaw = (float)8.843929E37F;
            p87.lon_int = (int)609516271;
            p87.vy = (float) -9.4099745E36F;
            p87.vx = (float) -1.0412062E38F;
            p87.afy = (float) -1.2083072E38F;
            LoopBackDemoChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)2.6273965E38F);
                Debug.Assert(pack.pitch == (float) -7.4832645E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3157985481U);
                Debug.Assert(pack.y == (float)7.6464273E37F);
                Debug.Assert(pack.z == (float) -2.700206E38F);
                Debug.Assert(pack.roll == (float)2.2739607E38F);
                Debug.Assert(pack.x == (float)5.800734E37F);
            };
            DemoDevice.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.pitch = (float) -7.4832645E37F;
            p89.y = (float)7.6464273E37F;
            p89.yaw = (float)2.6273965E38F;
            p89.z = (float) -2.700206E38F;
            p89.x = (float)5.800734E37F;
            p89.roll = (float)2.2739607E38F;
            p89.time_boot_ms = (uint)3157985481U;
            LoopBackDemoChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -705241118);
                Debug.Assert(pack.alt == (int)1725761607);
                Debug.Assert(pack.lat == (int)721395792);
                Debug.Assert(pack.pitch == (float)1.9858175E38F);
                Debug.Assert(pack.yacc == (short)(short)13870);
                Debug.Assert(pack.yaw == (float) -3.2795894E38F);
                Debug.Assert(pack.roll == (float)1.9055452E38F);
                Debug.Assert(pack.vz == (short)(short)10722);
                Debug.Assert(pack.vx == (short)(short)30038);
                Debug.Assert(pack.pitchspeed == (float)2.1336434E38F);
                Debug.Assert(pack.zacc == (short)(short) -29723);
                Debug.Assert(pack.xacc == (short)(short) -20618);
                Debug.Assert(pack.yawspeed == (float)1.1844169E38F);
                Debug.Assert(pack.time_usec == (ulong)3233481696838460816L);
                Debug.Assert(pack.vy == (short)(short)14551);
                Debug.Assert(pack.rollspeed == (float)1.0844536E38F);
            };
            DemoDevice.HIL_STATE p90 = LoopBackDemoChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.pitchspeed = (float)2.1336434E38F;
            p90.vx = (short)(short)30038;
            p90.lon = (int) -705241118;
            p90.yaw = (float) -3.2795894E38F;
            p90.vz = (short)(short)10722;
            p90.lat = (int)721395792;
            p90.time_usec = (ulong)3233481696838460816L;
            p90.vy = (short)(short)14551;
            p90.xacc = (short)(short) -20618;
            p90.yawspeed = (float)1.1844169E38F;
            p90.rollspeed = (float)1.0844536E38F;
            p90.zacc = (short)(short) -29723;
            p90.pitch = (float)1.9858175E38F;
            p90.roll = (float)1.9055452E38F;
            p90.alt = (int)1725761607;
            p90.yacc = (short)(short)13870;
            LoopBackDemoChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.aux4 == (float) -3.9346867E36F);
                Debug.Assert(pack.throttle == (float) -1.8703162E38F);
                Debug.Assert(pack.yaw_rudder == (float)2.4750367E38F);
                Debug.Assert(pack.aux3 == (float)3.2194916E38F);
                Debug.Assert(pack.aux1 == (float)2.163611E38F);
                Debug.Assert(pack.aux2 == (float) -1.290665E38F);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_MANUAL_ARMED);
                Debug.Assert(pack.pitch_elevator == (float) -1.9844529E38F);
                Debug.Assert(pack.nav_mode == (byte)(byte)133);
                Debug.Assert(pack.time_usec == (ulong)8150809384200382956L);
                Debug.Assert(pack.roll_ailerons == (float) -2.8296958E37F);
            };
            DemoDevice.HIL_CONTROLS p91 = LoopBackDemoChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.yaw_rudder = (float)2.4750367E38F;
            p91.aux1 = (float)2.163611E38F;
            p91.aux4 = (float) -3.9346867E36F;
            p91.aux3 = (float)3.2194916E38F;
            p91.nav_mode = (byte)(byte)133;
            p91.time_usec = (ulong)8150809384200382956L;
            p91.throttle = (float) -1.8703162E38F;
            p91.aux2 = (float) -1.290665E38F;
            p91.roll_ailerons = (float) -2.8296958E37F;
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_MANUAL_ARMED;
            p91.pitch_elevator = (float) -1.9844529E38F;
            LoopBackDemoChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)60751);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)49220);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)31926);
                Debug.Assert(pack.rssi == (byte)(byte)225);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)29071);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)16098);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)23190);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)27859);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)63463);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)11023);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)37161);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)34913);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)23758);
                Debug.Assert(pack.time_usec == (ulong)62886608398050899L);
            };
            DemoDevice.HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan11_raw = (ushort)(ushort)11023;
            p92.chan9_raw = (ushort)(ushort)31926;
            p92.chan2_raw = (ushort)(ushort)23190;
            p92.chan8_raw = (ushort)(ushort)37161;
            p92.chan7_raw = (ushort)(ushort)34913;
            p92.chan6_raw = (ushort)(ushort)49220;
            p92.chan5_raw = (ushort)(ushort)60751;
            p92.chan12_raw = (ushort)(ushort)16098;
            p92.chan3_raw = (ushort)(ushort)29071;
            p92.chan10_raw = (ushort)(ushort)63463;
            p92.chan1_raw = (ushort)(ushort)27859;
            p92.chan4_raw = (ushort)(ushort)23758;
            p92.rssi = (byte)(byte)225;
            p92.time_usec = (ulong)62886608398050899L;
            LoopBackDemoChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (ulong)8884666200100058849L);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_ARMED);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {2.2642355E36F, -7.2931514E37F, 9.531995E37F, 3.2155231E38F, -2.0588076E38F, 4.6216594E37F, 1.9925234E38F, -2.7790554E38F, -2.2285555E38F, -2.8778758E38F, -2.7847592E38F, 1.6426333E38F, -1.9849439E38F, -3.2128767E38F, 1.9803572E37F, 2.5790739E38F}));
                Debug.Assert(pack.time_usec == (ulong)420740054763667679L);
            };
            DemoDevice.HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.time_usec = (ulong)420740054763667679L;
            p93.flags = (ulong)8884666200100058849L;
            p93.controls_SET(new float[] {2.2642355E36F, -7.2931514E37F, 9.531995E37F, 3.2155231E38F, -2.0588076E38F, 4.6216594E37F, 1.9925234E38F, -2.7790554E38F, -2.2285555E38F, -2.8778758E38F, -2.7847592E38F, 1.6426333E38F, -1.9849439E38F, -3.2128767E38F, 1.9803572E37F, 2.5790739E38F}, 0) ;
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_ARMED;
            LoopBackDemoChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)5810943625240097530L);
                Debug.Assert(pack.flow_comp_m_y == (float) -2.8989342E38F);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float) -1.0213333E38F);
                Debug.Assert(pack.flow_x == (short)(short)26227);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float)1.6513198E38F);
                Debug.Assert(pack.flow_comp_m_x == (float)5.9012843E36F);
                Debug.Assert(pack.sensor_id == (byte)(byte)216);
                Debug.Assert(pack.quality == (byte)(byte)168);
                Debug.Assert(pack.flow_y == (short)(short)1929);
                Debug.Assert(pack.ground_distance == (float)2.5049563E38F);
            };
            DemoDevice.OPTICAL_FLOW p100 = LoopBackDemoChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.flow_comp_m_y = (float) -2.8989342E38F;
            p100.time_usec = (ulong)5810943625240097530L;
            p100.flow_rate_y_SET((float) -1.0213333E38F, PH) ;
            p100.flow_comp_m_x = (float)5.9012843E36F;
            p100.quality = (byte)(byte)168;
            p100.flow_y = (short)(short)1929;
            p100.sensor_id = (byte)(byte)216;
            p100.flow_rate_x_SET((float)1.6513198E38F, PH) ;
            p100.ground_distance = (float)2.5049563E38F;
            p100.flow_x = (short)(short)26227;
            LoopBackDemoChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)2.8448878E37F);
                Debug.Assert(pack.z == (float) -3.33745E38F);
                Debug.Assert(pack.yaw == (float) -1.3063475E38F);
                Debug.Assert(pack.usec == (ulong)1271564752338888292L);
                Debug.Assert(pack.pitch == (float) -1.7193214E38F);
                Debug.Assert(pack.y == (float)2.5799527E38F);
                Debug.Assert(pack.roll == (float) -2.9629268E38F);
            };
            DemoDevice.GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.y = (float)2.5799527E38F;
            p101.pitch = (float) -1.7193214E38F;
            p101.roll = (float) -2.9629268E38F;
            p101.usec = (ulong)1271564752338888292L;
            p101.z = (float) -3.33745E38F;
            p101.x = (float)2.8448878E37F;
            p101.yaw = (float) -1.3063475E38F;
            LoopBackDemoChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -9.0159494E36F);
                Debug.Assert(pack.y == (float)2.0307262E38F);
                Debug.Assert(pack.x == (float) -1.2764953E37F);
                Debug.Assert(pack.yaw == (float)2.7001047E38F);
                Debug.Assert(pack.usec == (ulong)8823988731595822000L);
                Debug.Assert(pack.pitch == (float)1.952084E37F);
                Debug.Assert(pack.roll == (float)3.1043183E38F);
            };
            DemoDevice.VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.pitch = (float)1.952084E37F;
            p102.y = (float)2.0307262E38F;
            p102.usec = (ulong)8823988731595822000L;
            p102.roll = (float)3.1043183E38F;
            p102.z = (float) -9.0159494E36F;
            p102.yaw = (float)2.7001047E38F;
            p102.x = (float) -1.2764953E37F;
            LoopBackDemoChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -7.300643E37F);
                Debug.Assert(pack.z == (float) -2.4330398E37F);
                Debug.Assert(pack.usec == (ulong)2508173342223400961L);
                Debug.Assert(pack.y == (float) -2.2550362E38F);
            };
            DemoDevice.VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)2508173342223400961L;
            p103.z = (float) -2.4330398E37F;
            p103.x = (float) -7.300643E37F;
            p103.y = (float) -2.2550362E38F;
            LoopBackDemoChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)2.0244021E38F);
                Debug.Assert(pack.yaw == (float)1.8421762E38F);
                Debug.Assert(pack.roll == (float) -2.6858632E38F);
                Debug.Assert(pack.z == (float)3.3578209E38F);
                Debug.Assert(pack.pitch == (float) -3.0851443E38F);
                Debug.Assert(pack.y == (float)2.6891542E38F);
                Debug.Assert(pack.usec == (ulong)6304892053985127199L);
            };
            DemoDevice.VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.yaw = (float)1.8421762E38F;
            p104.z = (float)3.3578209E38F;
            p104.x = (float)2.0244021E38F;
            p104.y = (float)2.6891542E38F;
            p104.usec = (ulong)6304892053985127199L;
            p104.pitch = (float) -3.0851443E38F;
            p104.roll = (float) -2.6858632E38F;
            LoopBackDemoChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ymag == (float)3.2684014E38F);
                Debug.Assert(pack.ygyro == (float)2.6718712E38F);
                Debug.Assert(pack.xmag == (float)6.6537946E37F);
                Debug.Assert(pack.time_usec == (ulong)7277580061209845611L);
                Debug.Assert(pack.yacc == (float) -2.469163E38F);
                Debug.Assert(pack.zgyro == (float)9.261256E37F);
                Debug.Assert(pack.zmag == (float) -3.4023101E38F);
                Debug.Assert(pack.xacc == (float)4.2382126E36F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)24295);
                Debug.Assert(pack.xgyro == (float) -1.7385096E38F);
                Debug.Assert(pack.pressure_alt == (float) -1.1679153E38F);
                Debug.Assert(pack.diff_pressure == (float)3.1154803E38F);
                Debug.Assert(pack.zacc == (float) -2.2934629E38F);
                Debug.Assert(pack.temperature == (float) -2.553752E38F);
                Debug.Assert(pack.abs_pressure == (float) -3.451453E36F);
            };
            DemoDevice.HIGHRES_IMU p105 = LoopBackDemoChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.time_usec = (ulong)7277580061209845611L;
            p105.zacc = (float) -2.2934629E38F;
            p105.yacc = (float) -2.469163E38F;
            p105.ygyro = (float)2.6718712E38F;
            p105.xacc = (float)4.2382126E36F;
            p105.diff_pressure = (float)3.1154803E38F;
            p105.pressure_alt = (float) -1.1679153E38F;
            p105.zmag = (float) -3.4023101E38F;
            p105.ymag = (float)3.2684014E38F;
            p105.temperature = (float) -2.553752E38F;
            p105.abs_pressure = (float) -3.451453E36F;
            p105.zgyro = (float)9.261256E37F;
            p105.xmag = (float)6.6537946E37F;
            p105.xgyro = (float) -1.7385096E38F;
            p105.fields_updated = (ushort)(ushort)24295;
            LoopBackDemoChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sensor_id == (byte)(byte)246);
                Debug.Assert(pack.integrated_y == (float)2.5604692E38F);
                Debug.Assert(pack.time_usec == (ulong)8229931423955405387L);
                Debug.Assert(pack.quality == (byte)(byte)217);
                Debug.Assert(pack.integrated_ygyro == (float)1.0175234E38F);
                Debug.Assert(pack.integrated_zgyro == (float) -2.8109357E38F);
                Debug.Assert(pack.distance == (float)1.9786584E38F);
                Debug.Assert(pack.integration_time_us == (uint)3104255775U);
                Debug.Assert(pack.temperature == (short)(short)14774);
                Debug.Assert(pack.integrated_x == (float) -3.4477313E37F);
                Debug.Assert(pack.integrated_xgyro == (float)1.6506261E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)3943305909U);
            };
            DemoDevice.OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.temperature = (short)(short)14774;
            p106.integrated_zgyro = (float) -2.8109357E38F;
            p106.quality = (byte)(byte)217;
            p106.integration_time_us = (uint)3104255775U;
            p106.time_delta_distance_us = (uint)3943305909U;
            p106.time_usec = (ulong)8229931423955405387L;
            p106.integrated_x = (float) -3.4477313E37F;
            p106.sensor_id = (byte)(byte)246;
            p106.integrated_ygyro = (float)1.0175234E38F;
            p106.integrated_y = (float)2.5604692E38F;
            p106.integrated_xgyro = (float)1.6506261E38F;
            p106.distance = (float)1.9786584E38F;
            LoopBackDemoChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zacc == (float)3.055941E37F);
                Debug.Assert(pack.abs_pressure == (float)1.8012293E38F);
                Debug.Assert(pack.zmag == (float) -2.766476E38F);
                Debug.Assert(pack.time_usec == (ulong)5965383993499949349L);
                Debug.Assert(pack.yacc == (float)1.0907149E38F);
                Debug.Assert(pack.ymag == (float)2.9377945E38F);
                Debug.Assert(pack.pressure_alt == (float)2.6425533E37F);
                Debug.Assert(pack.xmag == (float)7.6383783E37F);
                Debug.Assert(pack.xacc == (float)6.044954E37F);
                Debug.Assert(pack.xgyro == (float) -2.771889E38F);
                Debug.Assert(pack.ygyro == (float) -3.0709888E38F);
                Debug.Assert(pack.temperature == (float)1.4054168E38F);
                Debug.Assert(pack.zgyro == (float)2.1847017E37F);
                Debug.Assert(pack.diff_pressure == (float)3.3768431E38F);
                Debug.Assert(pack.fields_updated == (uint)3357019693U);
            };
            DemoDevice.HIL_SENSOR p107 = LoopBackDemoChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.time_usec = (ulong)5965383993499949349L;
            p107.temperature = (float)1.4054168E38F;
            p107.xgyro = (float) -2.771889E38F;
            p107.abs_pressure = (float)1.8012293E38F;
            p107.zgyro = (float)2.1847017E37F;
            p107.zacc = (float)3.055941E37F;
            p107.xmag = (float)7.6383783E37F;
            p107.pressure_alt = (float)2.6425533E37F;
            p107.xacc = (float)6.044954E37F;
            p107.yacc = (float)1.0907149E38F;
            p107.diff_pressure = (float)3.3768431E38F;
            p107.ymag = (float)2.9377945E38F;
            p107.ygyro = (float) -3.0709888E38F;
            p107.zmag = (float) -2.766476E38F;
            p107.fields_updated = (uint)3357019693U;
            LoopBackDemoChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ve == (float)1.4336719E38F);
                Debug.Assert(pack.xacc == (float)1.2083415E37F);
                Debug.Assert(pack.xgyro == (float) -1.3150354E38F);
                Debug.Assert(pack.lon == (float)8.722504E37F);
                Debug.Assert(pack.std_dev_vert == (float) -2.8763167E38F);
                Debug.Assert(pack.q2 == (float) -3.033984E38F);
                Debug.Assert(pack.vn == (float) -2.6885466E38F);
                Debug.Assert(pack.alt == (float) -2.7865255E37F);
                Debug.Assert(pack.pitch == (float) -2.0468758E38F);
                Debug.Assert(pack.ygyro == (float) -1.6739342E38F);
                Debug.Assert(pack.q4 == (float) -6.5127634E37F);
                Debug.Assert(pack.q3 == (float) -7.447712E37F);
                Debug.Assert(pack.yaw == (float) -1.3217059E38F);
                Debug.Assert(pack.vd == (float)3.1286917E38F);
                Debug.Assert(pack.std_dev_horz == (float)1.5405483E38F);
                Debug.Assert(pack.q1 == (float)1.7995562E38F);
                Debug.Assert(pack.roll == (float) -9.460232E37F);
                Debug.Assert(pack.zacc == (float)2.81132E38F);
                Debug.Assert(pack.yacc == (float) -1.0147495E38F);
                Debug.Assert(pack.zgyro == (float) -2.6379164E38F);
                Debug.Assert(pack.lat == (float)1.4032359E38F);
            };
            DemoDevice.SIM_STATE p108 = LoopBackDemoChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.std_dev_vert = (float) -2.8763167E38F;
            p108.zgyro = (float) -2.6379164E38F;
            p108.vd = (float)3.1286917E38F;
            p108.yaw = (float) -1.3217059E38F;
            p108.alt = (float) -2.7865255E37F;
            p108.std_dev_horz = (float)1.5405483E38F;
            p108.roll = (float) -9.460232E37F;
            p108.q1 = (float)1.7995562E38F;
            p108.q3 = (float) -7.447712E37F;
            p108.vn = (float) -2.6885466E38F;
            p108.xacc = (float)1.2083415E37F;
            p108.pitch = (float) -2.0468758E38F;
            p108.xgyro = (float) -1.3150354E38F;
            p108.lat = (float)1.4032359E38F;
            p108.q4 = (float) -6.5127634E37F;
            p108.ve = (float)1.4336719E38F;
            p108.q2 = (float) -3.033984E38F;
            p108.ygyro = (float) -1.6739342E38F;
            p108.lon = (float)8.722504E37F;
            p108.yacc = (float) -1.0147495E38F;
            p108.zacc = (float)2.81132E38F;
            LoopBackDemoChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rxerrors == (ushort)(ushort)9028);
                Debug.Assert(pack.rssi == (byte)(byte)62);
                Debug.Assert(pack.txbuf == (byte)(byte)114);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)9059);
                Debug.Assert(pack.noise == (byte)(byte)249);
                Debug.Assert(pack.remrssi == (byte)(byte)205);
                Debug.Assert(pack.remnoise == (byte)(byte)23);
            };
            DemoDevice.RADIO_STATUS p109 = LoopBackDemoChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rxerrors = (ushort)(ushort)9028;
            p109.remnoise = (byte)(byte)23;
            p109.remrssi = (byte)(byte)205;
            p109.fixed_ = (ushort)(ushort)9059;
            p109.noise = (byte)(byte)249;
            p109.txbuf = (byte)(byte)114;
            p109.rssi = (byte)(byte)62;
            LoopBackDemoChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)174);
                Debug.Assert(pack.target_network == (byte)(byte)134);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)55, (byte)144, (byte)208, (byte)236, (byte)32, (byte)27, (byte)231, (byte)25, (byte)61, (byte)46, (byte)224, (byte)117, (byte)106, (byte)35, (byte)162, (byte)181, (byte)124, (byte)89, (byte)8, (byte)37, (byte)47, (byte)128, (byte)15, (byte)101, (byte)252, (byte)13, (byte)16, (byte)69, (byte)125, (byte)146, (byte)203, (byte)94, (byte)230, (byte)25, (byte)78, (byte)165, (byte)49, (byte)251, (byte)249, (byte)81, (byte)108, (byte)165, (byte)37, (byte)234, (byte)190, (byte)247, (byte)201, (byte)139, (byte)254, (byte)90, (byte)78, (byte)58, (byte)235, (byte)183, (byte)12, (byte)209, (byte)207, (byte)153, (byte)48, (byte)214, (byte)158, (byte)137, (byte)180, (byte)237, (byte)82, (byte)188, (byte)91, (byte)56, (byte)209, (byte)195, (byte)90, (byte)239, (byte)245, (byte)51, (byte)122, (byte)183, (byte)188, (byte)217, (byte)183, (byte)111, (byte)248, (byte)246, (byte)99, (byte)86, (byte)103, (byte)180, (byte)218, (byte)143, (byte)46, (byte)206, (byte)249, (byte)43, (byte)106, (byte)213, (byte)116, (byte)122, (byte)145, (byte)106, (byte)253, (byte)245, (byte)84, (byte)97, (byte)71, (byte)61, (byte)23, (byte)30, (byte)37, (byte)30, (byte)55, (byte)220, (byte)69, (byte)127, (byte)99, (byte)166, (byte)135, (byte)224, (byte)227, (byte)131, (byte)74, (byte)180, (byte)35, (byte)248, (byte)132, (byte)160, (byte)171, (byte)99, (byte)134, (byte)110, (byte)151, (byte)4, (byte)90, (byte)220, (byte)2, (byte)190, (byte)212, (byte)34, (byte)221, (byte)38, (byte)180, (byte)125, (byte)211, (byte)135, (byte)128, (byte)147, (byte)249, (byte)52, (byte)123, (byte)92, (byte)35, (byte)12, (byte)91, (byte)190, (byte)233, (byte)155, (byte)26, (byte)56, (byte)72, (byte)241, (byte)19, (byte)208, (byte)21, (byte)189, (byte)6, (byte)252, (byte)44, (byte)14, (byte)116, (byte)76, (byte)20, (byte)61, (byte)220, (byte)25, (byte)76, (byte)129, (byte)117, (byte)244, (byte)60, (byte)20, (byte)229, (byte)83, (byte)69, (byte)90, (byte)79, (byte)149, (byte)55, (byte)107, (byte)121, (byte)30, (byte)185, (byte)70, (byte)98, (byte)152, (byte)144, (byte)37, (byte)173, (byte)213, (byte)2, (byte)34, (byte)232, (byte)110, (byte)149, (byte)2, (byte)64, (byte)218, (byte)149, (byte)97, (byte)140, (byte)94, (byte)9, (byte)46, (byte)192, (byte)67, (byte)138, (byte)222, (byte)51, (byte)106, (byte)65, (byte)101, (byte)182, (byte)143, (byte)158, (byte)112, (byte)255, (byte)74, (byte)229, (byte)137, (byte)78, (byte)106, (byte)29, (byte)1, (byte)8, (byte)81, (byte)217, (byte)47, (byte)120, (byte)234, (byte)5, (byte)164, (byte)114, (byte)143, (byte)0, (byte)59, (byte)248, (byte)25, (byte)91, (byte)97, (byte)17, (byte)152, (byte)104, (byte)42, (byte)206}));
                Debug.Assert(pack.target_system == (byte)(byte)154);
            };
            DemoDevice.FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_component = (byte)(byte)174;
            p110.payload_SET(new byte[] {(byte)55, (byte)144, (byte)208, (byte)236, (byte)32, (byte)27, (byte)231, (byte)25, (byte)61, (byte)46, (byte)224, (byte)117, (byte)106, (byte)35, (byte)162, (byte)181, (byte)124, (byte)89, (byte)8, (byte)37, (byte)47, (byte)128, (byte)15, (byte)101, (byte)252, (byte)13, (byte)16, (byte)69, (byte)125, (byte)146, (byte)203, (byte)94, (byte)230, (byte)25, (byte)78, (byte)165, (byte)49, (byte)251, (byte)249, (byte)81, (byte)108, (byte)165, (byte)37, (byte)234, (byte)190, (byte)247, (byte)201, (byte)139, (byte)254, (byte)90, (byte)78, (byte)58, (byte)235, (byte)183, (byte)12, (byte)209, (byte)207, (byte)153, (byte)48, (byte)214, (byte)158, (byte)137, (byte)180, (byte)237, (byte)82, (byte)188, (byte)91, (byte)56, (byte)209, (byte)195, (byte)90, (byte)239, (byte)245, (byte)51, (byte)122, (byte)183, (byte)188, (byte)217, (byte)183, (byte)111, (byte)248, (byte)246, (byte)99, (byte)86, (byte)103, (byte)180, (byte)218, (byte)143, (byte)46, (byte)206, (byte)249, (byte)43, (byte)106, (byte)213, (byte)116, (byte)122, (byte)145, (byte)106, (byte)253, (byte)245, (byte)84, (byte)97, (byte)71, (byte)61, (byte)23, (byte)30, (byte)37, (byte)30, (byte)55, (byte)220, (byte)69, (byte)127, (byte)99, (byte)166, (byte)135, (byte)224, (byte)227, (byte)131, (byte)74, (byte)180, (byte)35, (byte)248, (byte)132, (byte)160, (byte)171, (byte)99, (byte)134, (byte)110, (byte)151, (byte)4, (byte)90, (byte)220, (byte)2, (byte)190, (byte)212, (byte)34, (byte)221, (byte)38, (byte)180, (byte)125, (byte)211, (byte)135, (byte)128, (byte)147, (byte)249, (byte)52, (byte)123, (byte)92, (byte)35, (byte)12, (byte)91, (byte)190, (byte)233, (byte)155, (byte)26, (byte)56, (byte)72, (byte)241, (byte)19, (byte)208, (byte)21, (byte)189, (byte)6, (byte)252, (byte)44, (byte)14, (byte)116, (byte)76, (byte)20, (byte)61, (byte)220, (byte)25, (byte)76, (byte)129, (byte)117, (byte)244, (byte)60, (byte)20, (byte)229, (byte)83, (byte)69, (byte)90, (byte)79, (byte)149, (byte)55, (byte)107, (byte)121, (byte)30, (byte)185, (byte)70, (byte)98, (byte)152, (byte)144, (byte)37, (byte)173, (byte)213, (byte)2, (byte)34, (byte)232, (byte)110, (byte)149, (byte)2, (byte)64, (byte)218, (byte)149, (byte)97, (byte)140, (byte)94, (byte)9, (byte)46, (byte)192, (byte)67, (byte)138, (byte)222, (byte)51, (byte)106, (byte)65, (byte)101, (byte)182, (byte)143, (byte)158, (byte)112, (byte)255, (byte)74, (byte)229, (byte)137, (byte)78, (byte)106, (byte)29, (byte)1, (byte)8, (byte)81, (byte)217, (byte)47, (byte)120, (byte)234, (byte)5, (byte)164, (byte)114, (byte)143, (byte)0, (byte)59, (byte)248, (byte)25, (byte)91, (byte)97, (byte)17, (byte)152, (byte)104, (byte)42, (byte)206}, 0) ;
            p110.target_system = (byte)(byte)154;
            p110.target_network = (byte)(byte)134;
            LoopBackDemoChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tc1 == (long)5635736329187538869L);
                Debug.Assert(pack.ts1 == (long)3041082642455421241L);
            };
            DemoDevice.TIMESYNC p111 = LoopBackDemoChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.ts1 = (long)3041082642455421241L;
            p111.tc1 = (long)5635736329187538869L;
            LoopBackDemoChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (uint)2294792323U);
                Debug.Assert(pack.time_usec == (ulong)8179701666374137754L);
            };
            DemoDevice.CAMERA_TRIGGER p112 = LoopBackDemoChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)8179701666374137754L;
            p112.seq = (uint)2294792323U;
            LoopBackDemoChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ve == (short)(short) -5021);
                Debug.Assert(pack.cog == (ushort)(ushort)41554);
                Debug.Assert(pack.vd == (short)(short)12853);
                Debug.Assert(pack.lat == (int)1881758814);
                Debug.Assert(pack.vel == (ushort)(ushort)14159);
                Debug.Assert(pack.vn == (short)(short)29780);
                Debug.Assert(pack.satellites_visible == (byte)(byte)73);
                Debug.Assert(pack.lon == (int) -991173836);
                Debug.Assert(pack.epv == (ushort)(ushort)45736);
                Debug.Assert(pack.alt == (int)985219327);
                Debug.Assert(pack.fix_type == (byte)(byte)227);
                Debug.Assert(pack.eph == (ushort)(ushort)14869);
                Debug.Assert(pack.time_usec == (ulong)9023310013759358130L);
            };
            DemoDevice.HIL_GPS p113 = LoopBackDemoChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.lat = (int)1881758814;
            p113.vn = (short)(short)29780;
            p113.vd = (short)(short)12853;
            p113.ve = (short)(short) -5021;
            p113.vel = (ushort)(ushort)14159;
            p113.eph = (ushort)(ushort)14869;
            p113.cog = (ushort)(ushort)41554;
            p113.alt = (int)985219327;
            p113.time_usec = (ulong)9023310013759358130L;
            p113.lon = (int) -991173836;
            p113.fix_type = (byte)(byte)227;
            p113.satellites_visible = (byte)(byte)73;
            p113.epv = (ushort)(ushort)45736;
            LoopBackDemoChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -21220);
                Debug.Assert(pack.sensor_id == (byte)(byte)105);
                Debug.Assert(pack.integrated_y == (float) -6.6756803E37F);
                Debug.Assert(pack.quality == (byte)(byte)213);
                Debug.Assert(pack.integration_time_us == (uint)943975116U);
                Debug.Assert(pack.integrated_xgyro == (float) -2.7690957E38F);
                Debug.Assert(pack.integrated_zgyro == (float) -2.028847E38F);
                Debug.Assert(pack.time_usec == (ulong)2652781939787075294L);
                Debug.Assert(pack.integrated_x == (float)1.0762348E38F);
                Debug.Assert(pack.integrated_ygyro == (float) -1.6861731E38F);
                Debug.Assert(pack.distance == (float)1.2492525E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)4065634443U);
            };
            DemoDevice.HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.integrated_zgyro = (float) -2.028847E38F;
            p114.integrated_ygyro = (float) -1.6861731E38F;
            p114.time_usec = (ulong)2652781939787075294L;
            p114.sensor_id = (byte)(byte)105;
            p114.integrated_xgyro = (float) -2.7690957E38F;
            p114.integration_time_us = (uint)943975116U;
            p114.quality = (byte)(byte)213;
            p114.time_delta_distance_us = (uint)4065634443U;
            p114.integrated_y = (float) -6.6756803E37F;
            p114.integrated_x = (float)1.0762348E38F;
            p114.temperature = (short)(short) -21220;
            p114.distance = (float)1.2492525E38F;
            LoopBackDemoChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (short)(short)21442);
                Debug.Assert(pack.xacc == (short)(short)17575);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {9.3303844E36F, 6.807555E37F, 2.1663337E38F, -1.2351666E38F}));
                Debug.Assert(pack.yawspeed == (float)1.782958E38F);
                Debug.Assert(pack.time_usec == (ulong)6977845480600588782L);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)35638);
                Debug.Assert(pack.vy == (short)(short)10605);
                Debug.Assert(pack.yacc == (short)(short) -31659);
                Debug.Assert(pack.pitchspeed == (float) -5.2122507E37F);
                Debug.Assert(pack.zacc == (short)(short) -22859);
                Debug.Assert(pack.vz == (short)(short)11211);
                Debug.Assert(pack.lon == (int)609035674);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)37828);
                Debug.Assert(pack.alt == (int) -2025025229);
                Debug.Assert(pack.rollspeed == (float) -2.6344416E38F);
                Debug.Assert(pack.lat == (int) -832843065);
            };
            DemoDevice.HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.xacc = (short)(short)17575;
            p115.vy = (short)(short)10605;
            p115.true_airspeed = (ushort)(ushort)37828;
            p115.yawspeed = (float)1.782958E38F;
            p115.rollspeed = (float) -2.6344416E38F;
            p115.lon = (int)609035674;
            p115.time_usec = (ulong)6977845480600588782L;
            p115.yacc = (short)(short) -31659;
            p115.alt = (int) -2025025229;
            p115.vx = (short)(short)21442;
            p115.vz = (short)(short)11211;
            p115.ind_airspeed = (ushort)(ushort)35638;
            p115.attitude_quaternion_SET(new float[] {9.3303844E36F, 6.807555E37F, 2.1663337E38F, -1.2351666E38F}, 0) ;
            p115.zacc = (short)(short) -22859;
            p115.pitchspeed = (float) -5.2122507E37F;
            p115.lat = (int) -832843065;
            LoopBackDemoChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zmag == (short)(short) -29753);
                Debug.Assert(pack.zacc == (short)(short)6268);
                Debug.Assert(pack.ygyro == (short)(short)26884);
                Debug.Assert(pack.zgyro == (short)(short)13264);
                Debug.Assert(pack.ymag == (short)(short)16407);
                Debug.Assert(pack.yacc == (short)(short)9986);
                Debug.Assert(pack.xacc == (short)(short)19297);
                Debug.Assert(pack.xmag == (short)(short)25565);
                Debug.Assert(pack.xgyro == (short)(short)14969);
                Debug.Assert(pack.time_boot_ms == (uint)2446125838U);
            };
            DemoDevice.SCALED_IMU2 p116 = LoopBackDemoChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.xgyro = (short)(short)14969;
            p116.xacc = (short)(short)19297;
            p116.ygyro = (short)(short)26884;
            p116.xmag = (short)(short)25565;
            p116.zmag = (short)(short) -29753;
            p116.zgyro = (short)(short)13264;
            p116.ymag = (short)(short)16407;
            p116.time_boot_ms = (uint)2446125838U;
            p116.zacc = (short)(short)6268;
            p116.yacc = (short)(short)9986;
            LoopBackDemoChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.end == (ushort)(ushort)12797);
                Debug.Assert(pack.target_system == (byte)(byte)133);
                Debug.Assert(pack.start == (ushort)(ushort)60514);
                Debug.Assert(pack.target_component == (byte)(byte)59);
            };
            DemoDevice.LOG_REQUEST_LIST p117 = LoopBackDemoChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_component = (byte)(byte)59;
            p117.target_system = (byte)(byte)133;
            p117.end = (ushort)(ushort)12797;
            p117.start = (ushort)(ushort)60514;
            LoopBackDemoChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.size == (uint)2305277331U);
                Debug.Assert(pack.time_utc == (uint)1016661132U);
                Debug.Assert(pack.num_logs == (ushort)(ushort)51388);
                Debug.Assert(pack.id == (ushort)(ushort)26349);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)55226);
            };
            DemoDevice.LOG_ENTRY p118 = LoopBackDemoChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.time_utc = (uint)1016661132U;
            p118.last_log_num = (ushort)(ushort)55226;
            p118.size = (uint)2305277331U;
            p118.num_logs = (ushort)(ushort)51388;
            p118.id = (ushort)(ushort)26349;
            LoopBackDemoChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (uint)4045730863U);
                Debug.Assert(pack.target_system == (byte)(byte)71);
                Debug.Assert(pack.target_component == (byte)(byte)117);
                Debug.Assert(pack.id == (ushort)(ushort)39861);
                Debug.Assert(pack.ofs == (uint)2648903905U);
            };
            DemoDevice.LOG_REQUEST_DATA p119 = LoopBackDemoChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.count = (uint)4045730863U;
            p119.target_component = (byte)(byte)117;
            p119.target_system = (byte)(byte)71;
            p119.id = (ushort)(ushort)39861;
            p119.ofs = (uint)2648903905U;
            LoopBackDemoChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (ushort)(ushort)34978);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)6, (byte)29, (byte)44, (byte)130, (byte)220, (byte)82, (byte)242, (byte)67, (byte)157, (byte)186, (byte)36, (byte)12, (byte)165, (byte)12, (byte)163, (byte)71, (byte)14, (byte)195, (byte)86, (byte)151, (byte)26, (byte)11, (byte)68, (byte)98, (byte)171, (byte)11, (byte)182, (byte)76, (byte)29, (byte)166, (byte)45, (byte)139, (byte)168, (byte)82, (byte)225, (byte)22, (byte)35, (byte)22, (byte)153, (byte)126, (byte)171, (byte)130, (byte)236, (byte)156, (byte)60, (byte)249, (byte)134, (byte)65, (byte)240, (byte)27, (byte)146, (byte)31, (byte)184, (byte)5, (byte)15, (byte)129, (byte)38, (byte)196, (byte)182, (byte)37, (byte)232, (byte)123, (byte)18, (byte)56, (byte)200, (byte)187, (byte)11, (byte)16, (byte)157, (byte)80, (byte)155, (byte)58, (byte)134, (byte)249, (byte)24, (byte)144, (byte)173, (byte)6, (byte)141, (byte)39, (byte)16, (byte)135, (byte)203, (byte)48, (byte)38, (byte)94, (byte)198, (byte)234, (byte)252, (byte)109}));
                Debug.Assert(pack.ofs == (uint)806947790U);
                Debug.Assert(pack.count == (byte)(byte)149);
            };
            DemoDevice.LOG_DATA p120 = LoopBackDemoChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.data__SET(new byte[] {(byte)6, (byte)29, (byte)44, (byte)130, (byte)220, (byte)82, (byte)242, (byte)67, (byte)157, (byte)186, (byte)36, (byte)12, (byte)165, (byte)12, (byte)163, (byte)71, (byte)14, (byte)195, (byte)86, (byte)151, (byte)26, (byte)11, (byte)68, (byte)98, (byte)171, (byte)11, (byte)182, (byte)76, (byte)29, (byte)166, (byte)45, (byte)139, (byte)168, (byte)82, (byte)225, (byte)22, (byte)35, (byte)22, (byte)153, (byte)126, (byte)171, (byte)130, (byte)236, (byte)156, (byte)60, (byte)249, (byte)134, (byte)65, (byte)240, (byte)27, (byte)146, (byte)31, (byte)184, (byte)5, (byte)15, (byte)129, (byte)38, (byte)196, (byte)182, (byte)37, (byte)232, (byte)123, (byte)18, (byte)56, (byte)200, (byte)187, (byte)11, (byte)16, (byte)157, (byte)80, (byte)155, (byte)58, (byte)134, (byte)249, (byte)24, (byte)144, (byte)173, (byte)6, (byte)141, (byte)39, (byte)16, (byte)135, (byte)203, (byte)48, (byte)38, (byte)94, (byte)198, (byte)234, (byte)252, (byte)109}, 0) ;
            p120.id = (ushort)(ushort)34978;
            p120.ofs = (uint)806947790U;
            p120.count = (byte)(byte)149;
            LoopBackDemoChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)7);
                Debug.Assert(pack.target_system == (byte)(byte)248);
            };
            DemoDevice.LOG_ERASE p121 = LoopBackDemoChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_component = (byte)(byte)7;
            p121.target_system = (byte)(byte)248;
            LoopBackDemoChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)170);
                Debug.Assert(pack.target_component == (byte)(byte)106);
            };
            DemoDevice.LOG_REQUEST_END p122 = LoopBackDemoChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)170;
            p122.target_component = (byte)(byte)106;
            LoopBackDemoChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)75, (byte)116, (byte)18, (byte)67, (byte)58, (byte)170, (byte)8, (byte)212, (byte)180, (byte)107, (byte)167, (byte)162, (byte)79, (byte)222, (byte)22, (byte)162, (byte)209, (byte)77, (byte)60, (byte)211, (byte)220, (byte)3, (byte)11, (byte)152, (byte)245, (byte)202, (byte)102, (byte)6, (byte)157, (byte)221, (byte)197, (byte)133, (byte)172, (byte)220, (byte)200, (byte)196, (byte)62, (byte)70, (byte)163, (byte)174, (byte)228, (byte)229, (byte)147, (byte)238, (byte)26, (byte)38, (byte)145, (byte)74, (byte)146, (byte)229, (byte)79, (byte)229, (byte)3, (byte)203, (byte)135, (byte)13, (byte)130, (byte)246, (byte)211, (byte)180, (byte)243, (byte)14, (byte)48, (byte)18, (byte)85, (byte)176, (byte)67, (byte)131, (byte)148, (byte)58, (byte)158, (byte)45, (byte)165, (byte)147, (byte)189, (byte)29, (byte)241, (byte)192, (byte)45, (byte)32, (byte)120, (byte)82, (byte)14, (byte)245, (byte)79, (byte)210, (byte)199, (byte)22, (byte)143, (byte)121, (byte)198, (byte)86, (byte)31, (byte)61, (byte)31, (byte)124, (byte)196, (byte)11, (byte)202, (byte)120, (byte)59, (byte)95, (byte)212, (byte)225, (byte)189, (byte)88, (byte)183, (byte)195, (byte)18, (byte)245}));
                Debug.Assert(pack.target_system == (byte)(byte)158);
                Debug.Assert(pack.len == (byte)(byte)230);
                Debug.Assert(pack.target_component == (byte)(byte)77);
            };
            DemoDevice.GPS_INJECT_DATA p123 = LoopBackDemoChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_component = (byte)(byte)77;
            p123.len = (byte)(byte)230;
            p123.target_system = (byte)(byte)158;
            p123.data__SET(new byte[] {(byte)75, (byte)116, (byte)18, (byte)67, (byte)58, (byte)170, (byte)8, (byte)212, (byte)180, (byte)107, (byte)167, (byte)162, (byte)79, (byte)222, (byte)22, (byte)162, (byte)209, (byte)77, (byte)60, (byte)211, (byte)220, (byte)3, (byte)11, (byte)152, (byte)245, (byte)202, (byte)102, (byte)6, (byte)157, (byte)221, (byte)197, (byte)133, (byte)172, (byte)220, (byte)200, (byte)196, (byte)62, (byte)70, (byte)163, (byte)174, (byte)228, (byte)229, (byte)147, (byte)238, (byte)26, (byte)38, (byte)145, (byte)74, (byte)146, (byte)229, (byte)79, (byte)229, (byte)3, (byte)203, (byte)135, (byte)13, (byte)130, (byte)246, (byte)211, (byte)180, (byte)243, (byte)14, (byte)48, (byte)18, (byte)85, (byte)176, (byte)67, (byte)131, (byte)148, (byte)58, (byte)158, (byte)45, (byte)165, (byte)147, (byte)189, (byte)29, (byte)241, (byte)192, (byte)45, (byte)32, (byte)120, (byte)82, (byte)14, (byte)245, (byte)79, (byte)210, (byte)199, (byte)22, (byte)143, (byte)121, (byte)198, (byte)86, (byte)31, (byte)61, (byte)31, (byte)124, (byte)196, (byte)11, (byte)202, (byte)120, (byte)59, (byte)95, (byte)212, (byte)225, (byte)189, (byte)88, (byte)183, (byte)195, (byte)18, (byte)245}, 0) ;
            LoopBackDemoChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.dgps_numch == (byte)(byte)181);
                Debug.Assert(pack.satellites_visible == (byte)(byte)2);
                Debug.Assert(pack.vel == (ushort)(ushort)44174);
                Debug.Assert(pack.cog == (ushort)(ushort)47013);
                Debug.Assert(pack.eph == (ushort)(ushort)47213);
                Debug.Assert(pack.lon == (int)1428926201);
                Debug.Assert(pack.dgps_age == (uint)2166583566U);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
                Debug.Assert(pack.lat == (int)181270907);
                Debug.Assert(pack.epv == (ushort)(ushort)3855);
                Debug.Assert(pack.alt == (int) -1321426064);
                Debug.Assert(pack.time_usec == (ulong)1676880616487071461L);
            };
            DemoDevice.GPS2_RAW p124 = LoopBackDemoChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.cog = (ushort)(ushort)47013;
            p124.satellites_visible = (byte)(byte)2;
            p124.lat = (int)181270907;
            p124.dgps_age = (uint)2166583566U;
            p124.eph = (ushort)(ushort)47213;
            p124.dgps_numch = (byte)(byte)181;
            p124.vel = (ushort)(ushort)44174;
            p124.lon = (int)1428926201;
            p124.alt = (int) -1321426064;
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX;
            p124.time_usec = (ulong)1676880616487071461L;
            p124.epv = (ushort)(ushort)3855;
            LoopBackDemoChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED);
                Debug.Assert(pack.Vservo == (ushort)(ushort)30387);
                Debug.Assert(pack.Vcc == (ushort)(ushort)15060);
            };
            DemoDevice.POWER_STATUS p125 = LoopBackDemoChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)15060;
            p125.Vservo = (ushort)(ushort)30387;
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED;
            LoopBackDemoChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (byte)(byte)251);
                Debug.Assert(pack.timeout == (ushort)(ushort)625);
                Debug.Assert(pack.baudrate == (uint)1865624296U);
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)252, (byte)25, (byte)232, (byte)106, (byte)142, (byte)75, (byte)178, (byte)24, (byte)204, (byte)245, (byte)161, (byte)198, (byte)164, (byte)114, (byte)29, (byte)249, (byte)110, (byte)23, (byte)24, (byte)33, (byte)117, (byte)158, (byte)212, (byte)136, (byte)232, (byte)171, (byte)198, (byte)212, (byte)63, (byte)178, (byte)46, (byte)206, (byte)249, (byte)12, (byte)171, (byte)249, (byte)154, (byte)168, (byte)200, (byte)141, (byte)38, (byte)237, (byte)50, (byte)84, (byte)251, (byte)218, (byte)42, (byte)31, (byte)227, (byte)247, (byte)133, (byte)38, (byte)155, (byte)64, (byte)61, (byte)48, (byte)110, (byte)69, (byte)144, (byte)206, (byte)199, (byte)198, (byte)254, (byte)8, (byte)24, (byte)247, (byte)147, (byte)124, (byte)218, (byte)220}));
                Debug.Assert(pack.device == (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1);
            };
            DemoDevice.SERIAL_CONTROL p126 = LoopBackDemoChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.baudrate = (uint)1865624296U;
            p126.timeout = (ushort)(ushort)625;
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI;
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1;
            p126.data__SET(new byte[] {(byte)252, (byte)25, (byte)232, (byte)106, (byte)142, (byte)75, (byte)178, (byte)24, (byte)204, (byte)245, (byte)161, (byte)198, (byte)164, (byte)114, (byte)29, (byte)249, (byte)110, (byte)23, (byte)24, (byte)33, (byte)117, (byte)158, (byte)212, (byte)136, (byte)232, (byte)171, (byte)198, (byte)212, (byte)63, (byte)178, (byte)46, (byte)206, (byte)249, (byte)12, (byte)171, (byte)249, (byte)154, (byte)168, (byte)200, (byte)141, (byte)38, (byte)237, (byte)50, (byte)84, (byte)251, (byte)218, (byte)42, (byte)31, (byte)227, (byte)247, (byte)133, (byte)38, (byte)155, (byte)64, (byte)61, (byte)48, (byte)110, (byte)69, (byte)144, (byte)206, (byte)199, (byte)198, (byte)254, (byte)8, (byte)24, (byte)247, (byte)147, (byte)124, (byte)218, (byte)220}, 0) ;
            p126.count = (byte)(byte)251;
            LoopBackDemoChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)83);
                Debug.Assert(pack.iar_num_hypotheses == (int) -1467399735);
                Debug.Assert(pack.time_last_baseline_ms == (uint)2156431111U);
                Debug.Assert(pack.rtk_rate == (byte)(byte)230);
                Debug.Assert(pack.baseline_c_mm == (int) -1076101732);
                Debug.Assert(pack.accuracy == (uint)1538128184U);
                Debug.Assert(pack.baseline_a_mm == (int)2102383297);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)47);
                Debug.Assert(pack.baseline_b_mm == (int) -1057580586);
                Debug.Assert(pack.wn == (ushort)(ushort)8010);
                Debug.Assert(pack.nsats == (byte)(byte)130);
                Debug.Assert(pack.rtk_health == (byte)(byte)129);
                Debug.Assert(pack.tow == (uint)1779127193U);
            };
            DemoDevice.GPS_RTK p127 = LoopBackDemoChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.time_last_baseline_ms = (uint)2156431111U;
            p127.accuracy = (uint)1538128184U;
            p127.baseline_a_mm = (int)2102383297;
            p127.iar_num_hypotheses = (int) -1467399735;
            p127.baseline_c_mm = (int) -1076101732;
            p127.baseline_b_mm = (int) -1057580586;
            p127.rtk_health = (byte)(byte)129;
            p127.baseline_coords_type = (byte)(byte)83;
            p127.nsats = (byte)(byte)130;
            p127.wn = (ushort)(ushort)8010;
            p127.rtk_receiver_id = (byte)(byte)47;
            p127.tow = (uint)1779127193U;
            p127.rtk_rate = (byte)(byte)230;
            LoopBackDemoChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_last_baseline_ms == (uint)3173218344U);
                Debug.Assert(pack.rtk_health == (byte)(byte)224);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)148);
                Debug.Assert(pack.rtk_rate == (byte)(byte)171);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)64);
                Debug.Assert(pack.baseline_b_mm == (int)1519817319);
                Debug.Assert(pack.baseline_a_mm == (int) -1977983250);
                Debug.Assert(pack.iar_num_hypotheses == (int)1687122338);
                Debug.Assert(pack.accuracy == (uint)1632559711U);
                Debug.Assert(pack.baseline_c_mm == (int)914798601);
                Debug.Assert(pack.nsats == (byte)(byte)43);
                Debug.Assert(pack.wn == (ushort)(ushort)11128);
                Debug.Assert(pack.tow == (uint)1252062157U);
            };
            DemoDevice.GPS2_RTK p128 = LoopBackDemoChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.baseline_b_mm = (int)1519817319;
            p128.wn = (ushort)(ushort)11128;
            p128.accuracy = (uint)1632559711U;
            p128.nsats = (byte)(byte)43;
            p128.baseline_c_mm = (int)914798601;
            p128.rtk_health = (byte)(byte)224;
            p128.rtk_rate = (byte)(byte)171;
            p128.iar_num_hypotheses = (int)1687122338;
            p128.rtk_receiver_id = (byte)(byte)64;
            p128.tow = (uint)1252062157U;
            p128.time_last_baseline_ms = (uint)3173218344U;
            p128.baseline_coords_type = (byte)(byte)148;
            p128.baseline_a_mm = (int) -1977983250;
            LoopBackDemoChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xmag == (short)(short) -9722);
                Debug.Assert(pack.zmag == (short)(short) -2207);
                Debug.Assert(pack.time_boot_ms == (uint)3089801174U);
                Debug.Assert(pack.xacc == (short)(short) -18209);
                Debug.Assert(pack.ymag == (short)(short) -29888);
                Debug.Assert(pack.xgyro == (short)(short) -10832);
                Debug.Assert(pack.ygyro == (short)(short)27014);
                Debug.Assert(pack.zacc == (short)(short)15957);
                Debug.Assert(pack.yacc == (short)(short)21662);
                Debug.Assert(pack.zgyro == (short)(short)29381);
            };
            DemoDevice.SCALED_IMU3 p129 = LoopBackDemoChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.ymag = (short)(short) -29888;
            p129.xgyro = (short)(short) -10832;
            p129.zgyro = (short)(short)29381;
            p129.xmag = (short)(short) -9722;
            p129.zmag = (short)(short) -2207;
            p129.ygyro = (short)(short)27014;
            p129.xacc = (short)(short) -18209;
            p129.zacc = (short)(short)15957;
            p129.time_boot_ms = (uint)3089801174U;
            p129.yacc = (short)(short)21662;
            LoopBackDemoChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (byte)(byte)250);
                Debug.Assert(pack.height == (ushort)(ushort)11371);
                Debug.Assert(pack.jpg_quality == (byte)(byte)181);
                Debug.Assert(pack.payload == (byte)(byte)135);
                Debug.Assert(pack.packets == (ushort)(ushort)43177);
                Debug.Assert(pack.size == (uint)4268264757U);
                Debug.Assert(pack.width == (ushort)(ushort)25301);
            };
            DemoDevice.DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.jpg_quality = (byte)(byte)181;
            p130.packets = (ushort)(ushort)43177;
            p130.size = (uint)4268264757U;
            p130.width = (ushort)(ushort)25301;
            p130.height = (ushort)(ushort)11371;
            p130.payload = (byte)(byte)135;
            p130.type = (byte)(byte)250;
            LoopBackDemoChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)141, (byte)145, (byte)198, (byte)57, (byte)238, (byte)209, (byte)200, (byte)47, (byte)240, (byte)198, (byte)100, (byte)198, (byte)40, (byte)26, (byte)218, (byte)178, (byte)186, (byte)134, (byte)189, (byte)44, (byte)141, (byte)100, (byte)41, (byte)106, (byte)179, (byte)5, (byte)212, (byte)218, (byte)4, (byte)51, (byte)143, (byte)171, (byte)23, (byte)0, (byte)192, (byte)16, (byte)172, (byte)45, (byte)1, (byte)237, (byte)182, (byte)107, (byte)125, (byte)99, (byte)32, (byte)32, (byte)45, (byte)187, (byte)202, (byte)56, (byte)157, (byte)204, (byte)215, (byte)214, (byte)126, (byte)42, (byte)90, (byte)79, (byte)62, (byte)78, (byte)194, (byte)212, (byte)164, (byte)94, (byte)188, (byte)153, (byte)136, (byte)41, (byte)232, (byte)231, (byte)185, (byte)11, (byte)210, (byte)170, (byte)139, (byte)75, (byte)167, (byte)187, (byte)154, (byte)39, (byte)25, (byte)164, (byte)44, (byte)196, (byte)195, (byte)14, (byte)92, (byte)155, (byte)223, (byte)1, (byte)132, (byte)23, (byte)175, (byte)56, (byte)124, (byte)1, (byte)63, (byte)49, (byte)81, (byte)198, (byte)163, (byte)87, (byte)225, (byte)0, (byte)69, (byte)245, (byte)101, (byte)119, (byte)45, (byte)99, (byte)145, (byte)0, (byte)83, (byte)70, (byte)16, (byte)87, (byte)222, (byte)222, (byte)167, (byte)55, (byte)120, (byte)232, (byte)159, (byte)128, (byte)30, (byte)152, (byte)52, (byte)61, (byte)65, (byte)62, (byte)46, (byte)126, (byte)32, (byte)34, (byte)36, (byte)179, (byte)170, (byte)34, (byte)218, (byte)99, (byte)182, (byte)166, (byte)22, (byte)30, (byte)180, (byte)174, (byte)139, (byte)196, (byte)63, (byte)188, (byte)168, (byte)178, (byte)45, (byte)33, (byte)236, (byte)215, (byte)201, (byte)253, (byte)99, (byte)235, (byte)183, (byte)15, (byte)14, (byte)248, (byte)217, (byte)177, (byte)138, (byte)212, (byte)46, (byte)157, (byte)206, (byte)63, (byte)240, (byte)162, (byte)176, (byte)22, (byte)125, (byte)52, (byte)196, (byte)62, (byte)58, (byte)136, (byte)130, (byte)87, (byte)235, (byte)73, (byte)174, (byte)148, (byte)116, (byte)32, (byte)222, (byte)57, (byte)16, (byte)55, (byte)121, (byte)101, (byte)90, (byte)220, (byte)96, (byte)72, (byte)182, (byte)171, (byte)236, (byte)61, (byte)95, (byte)32, (byte)56, (byte)96, (byte)160, (byte)99, (byte)31, (byte)220, (byte)39, (byte)144, (byte)49, (byte)152, (byte)94, (byte)207, (byte)49, (byte)144, (byte)218, (byte)41, (byte)5, (byte)219, (byte)106, (byte)156, (byte)137, (byte)231, (byte)35, (byte)117, (byte)199, (byte)255, (byte)85, (byte)172, (byte)230, (byte)40, (byte)215, (byte)174, (byte)232, (byte)116, (byte)229, (byte)70, (byte)69, (byte)216, (byte)179, (byte)193, (byte)81, (byte)79, (byte)146, (byte)232, (byte)94, (byte)0, (byte)84}));
                Debug.Assert(pack.seqnr == (ushort)(ushort)55833);
            };
            DemoDevice.ENCAPSULATED_DATA p131 = LoopBackDemoChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)55833;
            p131.data__SET(new byte[] {(byte)141, (byte)145, (byte)198, (byte)57, (byte)238, (byte)209, (byte)200, (byte)47, (byte)240, (byte)198, (byte)100, (byte)198, (byte)40, (byte)26, (byte)218, (byte)178, (byte)186, (byte)134, (byte)189, (byte)44, (byte)141, (byte)100, (byte)41, (byte)106, (byte)179, (byte)5, (byte)212, (byte)218, (byte)4, (byte)51, (byte)143, (byte)171, (byte)23, (byte)0, (byte)192, (byte)16, (byte)172, (byte)45, (byte)1, (byte)237, (byte)182, (byte)107, (byte)125, (byte)99, (byte)32, (byte)32, (byte)45, (byte)187, (byte)202, (byte)56, (byte)157, (byte)204, (byte)215, (byte)214, (byte)126, (byte)42, (byte)90, (byte)79, (byte)62, (byte)78, (byte)194, (byte)212, (byte)164, (byte)94, (byte)188, (byte)153, (byte)136, (byte)41, (byte)232, (byte)231, (byte)185, (byte)11, (byte)210, (byte)170, (byte)139, (byte)75, (byte)167, (byte)187, (byte)154, (byte)39, (byte)25, (byte)164, (byte)44, (byte)196, (byte)195, (byte)14, (byte)92, (byte)155, (byte)223, (byte)1, (byte)132, (byte)23, (byte)175, (byte)56, (byte)124, (byte)1, (byte)63, (byte)49, (byte)81, (byte)198, (byte)163, (byte)87, (byte)225, (byte)0, (byte)69, (byte)245, (byte)101, (byte)119, (byte)45, (byte)99, (byte)145, (byte)0, (byte)83, (byte)70, (byte)16, (byte)87, (byte)222, (byte)222, (byte)167, (byte)55, (byte)120, (byte)232, (byte)159, (byte)128, (byte)30, (byte)152, (byte)52, (byte)61, (byte)65, (byte)62, (byte)46, (byte)126, (byte)32, (byte)34, (byte)36, (byte)179, (byte)170, (byte)34, (byte)218, (byte)99, (byte)182, (byte)166, (byte)22, (byte)30, (byte)180, (byte)174, (byte)139, (byte)196, (byte)63, (byte)188, (byte)168, (byte)178, (byte)45, (byte)33, (byte)236, (byte)215, (byte)201, (byte)253, (byte)99, (byte)235, (byte)183, (byte)15, (byte)14, (byte)248, (byte)217, (byte)177, (byte)138, (byte)212, (byte)46, (byte)157, (byte)206, (byte)63, (byte)240, (byte)162, (byte)176, (byte)22, (byte)125, (byte)52, (byte)196, (byte)62, (byte)58, (byte)136, (byte)130, (byte)87, (byte)235, (byte)73, (byte)174, (byte)148, (byte)116, (byte)32, (byte)222, (byte)57, (byte)16, (byte)55, (byte)121, (byte)101, (byte)90, (byte)220, (byte)96, (byte)72, (byte)182, (byte)171, (byte)236, (byte)61, (byte)95, (byte)32, (byte)56, (byte)96, (byte)160, (byte)99, (byte)31, (byte)220, (byte)39, (byte)144, (byte)49, (byte)152, (byte)94, (byte)207, (byte)49, (byte)144, (byte)218, (byte)41, (byte)5, (byte)219, (byte)106, (byte)156, (byte)137, (byte)231, (byte)35, (byte)117, (byte)199, (byte)255, (byte)85, (byte)172, (byte)230, (byte)40, (byte)215, (byte)174, (byte)232, (byte)116, (byte)229, (byte)70, (byte)69, (byte)216, (byte)179, (byte)193, (byte)81, (byte)79, (byte)146, (byte)232, (byte)94, (byte)0, (byte)84}, 0) ;
            LoopBackDemoChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.orientation == (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_PITCH_180_YAW_270);
                Debug.Assert(pack.current_distance == (ushort)(ushort)59201);
                Debug.Assert(pack.time_boot_ms == (uint)2683803567U);
                Debug.Assert(pack.type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
                Debug.Assert(pack.min_distance == (ushort)(ushort)13776);
                Debug.Assert(pack.covariance == (byte)(byte)217);
                Debug.Assert(pack.max_distance == (ushort)(ushort)265);
                Debug.Assert(pack.id == (byte)(byte)147);
            };
            DemoDevice.DISTANCE_SENSOR p132 = LoopBackDemoChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.id = (byte)(byte)147;
            p132.covariance = (byte)(byte)217;
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN;
            p132.max_distance = (ushort)(ushort)265;
            p132.current_distance = (ushort)(ushort)59201;
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_PITCH_180_YAW_270;
            p132.min_distance = (ushort)(ushort)13776;
            p132.time_boot_ms = (uint)2683803567U;
            LoopBackDemoChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -427122668);
                Debug.Assert(pack.lat == (int)1990147456);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)42950);
                Debug.Assert(pack.mask == (ulong)436224870441021738L);
            };
            DemoDevice.TERRAIN_REQUEST p133 = LoopBackDemoChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.grid_spacing = (ushort)(ushort)42950;
            p133.lat = (int)1990147456;
            p133.lon = (int) -427122668;
            p133.mask = (ulong)436224870441021738L;
            LoopBackDemoChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)40937);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short) -18316, (short) -31031, (short) -29983, (short) -7253, (short)17292, (short) -25584, (short)31473, (short) -20825, (short) -25120, (short)11732, (short)20086, (short) -9963, (short)26878, (short)3986, (short) -12492, (short)14553}));
                Debug.Assert(pack.lon == (int) -1284442717);
                Debug.Assert(pack.gridbit == (byte)(byte)86);
                Debug.Assert(pack.lat == (int)1700236803);
            };
            DemoDevice.TERRAIN_DATA p134 = LoopBackDemoChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.data__SET(new short[] {(short) -18316, (short) -31031, (short) -29983, (short) -7253, (short)17292, (short) -25584, (short)31473, (short) -20825, (short) -25120, (short)11732, (short)20086, (short) -9963, (short)26878, (short)3986, (short) -12492, (short)14553}, 0) ;
            p134.grid_spacing = (ushort)(ushort)40937;
            p134.lon = (int) -1284442717;
            p134.lat = (int)1700236803;
            p134.gridbit = (byte)(byte)86;
            LoopBackDemoChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -2122567672);
                Debug.Assert(pack.lat == (int) -2011927203);
            };
            DemoDevice.TERRAIN_CHECK p135 = LoopBackDemoChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int) -2011927203;
            p135.lon = (int) -2122567672;
            LoopBackDemoChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.spacing == (ushort)(ushort)4026);
                Debug.Assert(pack.lon == (int)1893055084);
                Debug.Assert(pack.lat == (int)617362902);
                Debug.Assert(pack.current_height == (float)3.3759649E38F);
                Debug.Assert(pack.loaded == (ushort)(ushort)64901);
                Debug.Assert(pack.terrain_height == (float)8.3257324E37F);
                Debug.Assert(pack.pending == (ushort)(ushort)14195);
            };
            DemoDevice.TERRAIN_REPORT p136 = LoopBackDemoChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.loaded = (ushort)(ushort)64901;
            p136.lon = (int)1893055084;
            p136.lat = (int)617362902;
            p136.pending = (ushort)(ushort)14195;
            p136.terrain_height = (float)8.3257324E37F;
            p136.current_height = (float)3.3759649E38F;
            p136.spacing = (ushort)(ushort)4026;
            LoopBackDemoChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float)1.5833426E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2021525125U);
                Debug.Assert(pack.press_diff == (float)1.7657052E38F);
                Debug.Assert(pack.temperature == (short)(short) -8373);
            };
            DemoDevice.SCALED_PRESSURE2 p137 = LoopBackDemoChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.press_diff = (float)1.7657052E38F;
            p137.temperature = (short)(short) -8373;
            p137.press_abs = (float)1.5833426E38F;
            p137.time_boot_ms = (uint)2021525125U;
            LoopBackDemoChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)179267761008025686L);
                Debug.Assert(pack.x == (float) -1.940399E38F);
                Debug.Assert(pack.z == (float)7.4051934E37F);
                Debug.Assert(pack.y == (float)2.515625E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.4191148E38F, 2.3349224E38F, -1.8274171E38F, 2.448591E38F}));
            };
            DemoDevice.ATT_POS_MOCAP p138 = LoopBackDemoChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.y = (float)2.515625E38F;
            p138.z = (float)7.4051934E37F;
            p138.time_usec = (ulong)179267761008025686L;
            p138.q_SET(new float[] {-1.4191148E38F, 2.3349224E38F, -1.8274171E38F, 2.448591E38F}, 0) ;
            p138.x = (float) -1.940399E38F;
            LoopBackDemoChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.group_mlx == (byte)(byte)159);
                Debug.Assert(pack.target_system == (byte)(byte)137);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {2.3529932E38F, 6.757444E37F, -2.6971502E38F, 3.0928E38F, -8.387947E37F, -2.6948019E38F, 1.928361E38F, -1.7234848E38F}));
                Debug.Assert(pack.time_usec == (ulong)3020235461247294668L);
                Debug.Assert(pack.target_component == (byte)(byte)171);
            };
            DemoDevice.SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.target_component = (byte)(byte)171;
            p139.group_mlx = (byte)(byte)159;
            p139.controls_SET(new float[] {2.3529932E38F, 6.757444E37F, -2.6971502E38F, 3.0928E38F, -8.387947E37F, -2.6948019E38F, 1.928361E38F, -1.7234848E38F}, 0) ;
            p139.time_usec = (ulong)3020235461247294668L;
            p139.target_system = (byte)(byte)137;
            LoopBackDemoChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-1.2778007E38F, 2.8366935E38F, -9.880662E37F, 3.3967909E38F, 1.4609687E38F, 5.607301E37F, -4.9918397E37F, -5.8037375E37F}));
                Debug.Assert(pack.group_mlx == (byte)(byte)20);
                Debug.Assert(pack.time_usec == (ulong)1767577412834954543L);
            };
            DemoDevice.ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)1767577412834954543L;
            p140.group_mlx = (byte)(byte)20;
            p140.controls_SET(new float[] {-1.2778007E38F, 2.8366935E38F, -9.880662E37F, 3.3967909E38F, 1.4609687E38F, 5.607301E37F, -4.9918397E37F, -5.8037375E37F}, 0) ;
            LoopBackDemoChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.bottom_clearance == (float) -1.4386872E37F);
                Debug.Assert(pack.altitude_relative == (float)2.2995287E38F);
                Debug.Assert(pack.time_usec == (ulong)4212061304125949264L);
                Debug.Assert(pack.altitude_amsl == (float)1.2104984E38F);
                Debug.Assert(pack.altitude_local == (float) -1.1678996E38F);
                Debug.Assert(pack.altitude_terrain == (float)2.4771598E38F);
                Debug.Assert(pack.altitude_monotonic == (float)1.0743636E38F);
            };
            DemoDevice.ALTITUDE p141 = LoopBackDemoChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.altitude_local = (float) -1.1678996E38F;
            p141.altitude_amsl = (float)1.2104984E38F;
            p141.bottom_clearance = (float) -1.4386872E37F;
            p141.time_usec = (ulong)4212061304125949264L;
            p141.altitude_terrain = (float)2.4771598E38F;
            p141.altitude_relative = (float)2.2995287E38F;
            p141.altitude_monotonic = (float)1.0743636E38F;
            LoopBackDemoChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)131, (byte)149, (byte)65, (byte)135, (byte)226, (byte)40, (byte)28, (byte)172, (byte)21, (byte)98, (byte)150, (byte)184, (byte)168, (byte)64, (byte)78, (byte)208, (byte)106, (byte)41, (byte)102, (byte)48, (byte)228, (byte)74, (byte)213, (byte)132, (byte)76, (byte)183, (byte)64, (byte)115, (byte)132, (byte)172, (byte)14, (byte)173, (byte)214, (byte)193, (byte)63, (byte)15, (byte)169, (byte)242, (byte)200, (byte)95, (byte)129, (byte)51, (byte)138, (byte)101, (byte)123, (byte)9, (byte)246, (byte)246, (byte)219, (byte)16, (byte)246, (byte)211, (byte)146, (byte)69, (byte)175, (byte)60, (byte)168, (byte)73, (byte)133, (byte)63, (byte)36, (byte)182, (byte)163, (byte)174, (byte)109, (byte)157, (byte)208, (byte)152, (byte)93, (byte)253, (byte)173, (byte)214, (byte)212, (byte)15, (byte)245, (byte)208, (byte)78, (byte)217, (byte)135, (byte)99, (byte)27, (byte)226, (byte)108, (byte)114, (byte)187, (byte)163, (byte)14, (byte)44, (byte)223, (byte)250, (byte)251, (byte)148, (byte)232, (byte)235, (byte)98, (byte)56, (byte)3, (byte)117, (byte)142, (byte)102, (byte)136, (byte)249, (byte)83, (byte)83, (byte)87, (byte)134, (byte)110, (byte)9, (byte)206, (byte)255, (byte)103, (byte)41, (byte)51, (byte)99, (byte)240, (byte)251, (byte)198, (byte)119, (byte)208, (byte)96}));
                Debug.Assert(pack.request_id == (byte)(byte)145);
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)176, (byte)242, (byte)191, (byte)90, (byte)48, (byte)179, (byte)174, (byte)165, (byte)59, (byte)198, (byte)199, (byte)14, (byte)90, (byte)55, (byte)216, (byte)213, (byte)55, (byte)169, (byte)92, (byte)225, (byte)239, (byte)197, (byte)7, (byte)46, (byte)205, (byte)168, (byte)47, (byte)16, (byte)209, (byte)177, (byte)104, (byte)206, (byte)164, (byte)100, (byte)70, (byte)160, (byte)38, (byte)115, (byte)254, (byte)70, (byte)194, (byte)164, (byte)2, (byte)218, (byte)203, (byte)110, (byte)114, (byte)199, (byte)100, (byte)113, (byte)164, (byte)50, (byte)135, (byte)100, (byte)123, (byte)222, (byte)143, (byte)123, (byte)89, (byte)137, (byte)166, (byte)250, (byte)123, (byte)84, (byte)109, (byte)68, (byte)185, (byte)172, (byte)193, (byte)212, (byte)78, (byte)184, (byte)75, (byte)172, (byte)6, (byte)220, (byte)47, (byte)228, (byte)181, (byte)184, (byte)106, (byte)255, (byte)161, (byte)30, (byte)236, (byte)239, (byte)16, (byte)248, (byte)172, (byte)112, (byte)69, (byte)200, (byte)191, (byte)85, (byte)171, (byte)175, (byte)20, (byte)69, (byte)224, (byte)140, (byte)12, (byte)202, (byte)88, (byte)142, (byte)10, (byte)18, (byte)80, (byte)201, (byte)140, (byte)228, (byte)103, (byte)172, (byte)191, (byte)161, (byte)9, (byte)35, (byte)174, (byte)9, (byte)233, (byte)217}));
                Debug.Assert(pack.uri_type == (byte)(byte)175);
                Debug.Assert(pack.transfer_type == (byte)(byte)244);
            };
            DemoDevice.RESOURCE_REQUEST p142 = LoopBackDemoChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)145;
            p142.uri_SET(new byte[] {(byte)176, (byte)242, (byte)191, (byte)90, (byte)48, (byte)179, (byte)174, (byte)165, (byte)59, (byte)198, (byte)199, (byte)14, (byte)90, (byte)55, (byte)216, (byte)213, (byte)55, (byte)169, (byte)92, (byte)225, (byte)239, (byte)197, (byte)7, (byte)46, (byte)205, (byte)168, (byte)47, (byte)16, (byte)209, (byte)177, (byte)104, (byte)206, (byte)164, (byte)100, (byte)70, (byte)160, (byte)38, (byte)115, (byte)254, (byte)70, (byte)194, (byte)164, (byte)2, (byte)218, (byte)203, (byte)110, (byte)114, (byte)199, (byte)100, (byte)113, (byte)164, (byte)50, (byte)135, (byte)100, (byte)123, (byte)222, (byte)143, (byte)123, (byte)89, (byte)137, (byte)166, (byte)250, (byte)123, (byte)84, (byte)109, (byte)68, (byte)185, (byte)172, (byte)193, (byte)212, (byte)78, (byte)184, (byte)75, (byte)172, (byte)6, (byte)220, (byte)47, (byte)228, (byte)181, (byte)184, (byte)106, (byte)255, (byte)161, (byte)30, (byte)236, (byte)239, (byte)16, (byte)248, (byte)172, (byte)112, (byte)69, (byte)200, (byte)191, (byte)85, (byte)171, (byte)175, (byte)20, (byte)69, (byte)224, (byte)140, (byte)12, (byte)202, (byte)88, (byte)142, (byte)10, (byte)18, (byte)80, (byte)201, (byte)140, (byte)228, (byte)103, (byte)172, (byte)191, (byte)161, (byte)9, (byte)35, (byte)174, (byte)9, (byte)233, (byte)217}, 0) ;
            p142.storage_SET(new byte[] {(byte)131, (byte)149, (byte)65, (byte)135, (byte)226, (byte)40, (byte)28, (byte)172, (byte)21, (byte)98, (byte)150, (byte)184, (byte)168, (byte)64, (byte)78, (byte)208, (byte)106, (byte)41, (byte)102, (byte)48, (byte)228, (byte)74, (byte)213, (byte)132, (byte)76, (byte)183, (byte)64, (byte)115, (byte)132, (byte)172, (byte)14, (byte)173, (byte)214, (byte)193, (byte)63, (byte)15, (byte)169, (byte)242, (byte)200, (byte)95, (byte)129, (byte)51, (byte)138, (byte)101, (byte)123, (byte)9, (byte)246, (byte)246, (byte)219, (byte)16, (byte)246, (byte)211, (byte)146, (byte)69, (byte)175, (byte)60, (byte)168, (byte)73, (byte)133, (byte)63, (byte)36, (byte)182, (byte)163, (byte)174, (byte)109, (byte)157, (byte)208, (byte)152, (byte)93, (byte)253, (byte)173, (byte)214, (byte)212, (byte)15, (byte)245, (byte)208, (byte)78, (byte)217, (byte)135, (byte)99, (byte)27, (byte)226, (byte)108, (byte)114, (byte)187, (byte)163, (byte)14, (byte)44, (byte)223, (byte)250, (byte)251, (byte)148, (byte)232, (byte)235, (byte)98, (byte)56, (byte)3, (byte)117, (byte)142, (byte)102, (byte)136, (byte)249, (byte)83, (byte)83, (byte)87, (byte)134, (byte)110, (byte)9, (byte)206, (byte)255, (byte)103, (byte)41, (byte)51, (byte)99, (byte)240, (byte)251, (byte)198, (byte)119, (byte)208, (byte)96}, 0) ;
            p142.transfer_type = (byte)(byte)244;
            p142.uri_type = (byte)(byte)175;
            LoopBackDemoChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3613402208U);
                Debug.Assert(pack.press_diff == (float)2.7954145E37F);
                Debug.Assert(pack.temperature == (short)(short)16786);
                Debug.Assert(pack.press_abs == (float)1.6508474E38F);
            };
            DemoDevice.SCALED_PRESSURE3 p143 = LoopBackDemoChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.temperature = (short)(short)16786;
            p143.press_abs = (float)1.6508474E38F;
            p143.press_diff = (float)2.7954145E37F;
            p143.time_boot_ms = (uint)3613402208U;
            LoopBackDemoChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.timestamp == (ulong)2567090815519718981L);
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {-3.076769E38F, -1.514851E38F, -3.0043954E38F, 3.155903E37F}));
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {-2.5392293E38F, -1.8070643E38F, -1.3037014E38F}));
                Debug.Assert(pack.vel.SequenceEqual(new float[] {-9.220262E37F, 1.4958462E38F, -5.269423E37F}));
                Debug.Assert(pack.lon == (int) -2045285649);
                Debug.Assert(pack.est_capabilities == (byte)(byte)161);
                Debug.Assert(pack.alt == (float)1.0044038E37F);
                Debug.Assert(pack.rates.SequenceEqual(new float[] {-2.256568E38F, -5.199331E37F, 5.4342173E37F}));
                Debug.Assert(pack.acc.SequenceEqual(new float[] {2.063175E38F, -2.6053023E38F, -1.7661502E38F}));
                Debug.Assert(pack.lat == (int)1407781260);
                Debug.Assert(pack.custom_state == (ulong)5692806601059771589L);
            };
            DemoDevice.FOLLOW_TARGET p144 = LoopBackDemoChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.est_capabilities = (byte)(byte)161;
            p144.rates_SET(new float[] {-2.256568E38F, -5.199331E37F, 5.4342173E37F}, 0) ;
            p144.attitude_q_SET(new float[] {-3.076769E38F, -1.514851E38F, -3.0043954E38F, 3.155903E37F}, 0) ;
            p144.lon = (int) -2045285649;
            p144.timestamp = (ulong)2567090815519718981L;
            p144.custom_state = (ulong)5692806601059771589L;
            p144.position_cov_SET(new float[] {-2.5392293E38F, -1.8070643E38F, -1.3037014E38F}, 0) ;
            p144.vel_SET(new float[] {-9.220262E37F, 1.4958462E38F, -5.269423E37F}, 0) ;
            p144.lat = (int)1407781260;
            p144.alt = (float)1.0044038E37F;
            p144.acc_SET(new float[] {2.063175E38F, -2.6053023E38F, -1.7661502E38F}, 0) ;
            LoopBackDemoChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw_rate == (float)2.7203364E38F);
                Debug.Assert(pack.z_acc == (float) -2.468211E38F);
                Debug.Assert(pack.y_vel == (float) -3.0211655E38F);
                Debug.Assert(pack.roll_rate == (float) -3.3238533E38F);
                Debug.Assert(pack.x_vel == (float)1.5846577E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.1611662E38F, 2.553915E38F, -8.584358E37F, 1.1843998E38F}));
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {-2.5720083E38F, 2.4751553E38F, -4.589865E37F}));
                Debug.Assert(pack.time_usec == (ulong)2011996429978355258L);
                Debug.Assert(pack.z_pos == (float)2.524481E38F);
                Debug.Assert(pack.z_vel == (float) -1.2071416E38F);
                Debug.Assert(pack.y_acc == (float)1.8342466E38F);
                Debug.Assert(pack.x_pos == (float) -2.9820929E38F);
                Debug.Assert(pack.airspeed == (float) -7.255E36F);
                Debug.Assert(pack.y_pos == (float) -2.2613972E38F);
                Debug.Assert(pack.pitch_rate == (float) -2.6163485E38F);
                Debug.Assert(pack.x_acc == (float) -2.6798706E38F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {1.9508648E38F, -3.2238671E38F, 1.104952E38F}));
            };
            DemoDevice.CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.q_SET(new float[] {1.1611662E38F, 2.553915E38F, -8.584358E37F, 1.1843998E38F}, 0) ;
            p146.z_pos = (float)2.524481E38F;
            p146.time_usec = (ulong)2011996429978355258L;
            p146.pos_variance_SET(new float[] {-2.5720083E38F, 2.4751553E38F, -4.589865E37F}, 0) ;
            p146.y_acc = (float)1.8342466E38F;
            p146.x_acc = (float) -2.6798706E38F;
            p146.airspeed = (float) -7.255E36F;
            p146.vel_variance_SET(new float[] {1.9508648E38F, -3.2238671E38F, 1.104952E38F}, 0) ;
            p146.pitch_rate = (float) -2.6163485E38F;
            p146.x_pos = (float) -2.9820929E38F;
            p146.yaw_rate = (float)2.7203364E38F;
            p146.y_pos = (float) -2.2613972E38F;
            p146.y_vel = (float) -3.0211655E38F;
            p146.z_vel = (float) -1.2071416E38F;
            p146.roll_rate = (float) -3.3238533E38F;
            p146.z_acc = (float) -2.468211E38F;
            p146.x_vel = (float)1.5846577E38F;
            LoopBackDemoChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (byte)(byte)64);
                Debug.Assert(pack.current_consumed == (int)954791542);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 100);
                Debug.Assert(pack.battery_function == (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)19888, (ushort)52401, (ushort)16192, (ushort)60418, (ushort)7664, (ushort)58441, (ushort)30120, (ushort)15746, (ushort)63363, (ushort)9889}));
                Debug.Assert(pack.energy_consumed == (int)1350280178);
                Debug.Assert(pack.temperature == (short)(short) -28765);
                Debug.Assert(pack.current_battery == (short)(short) -11954);
                Debug.Assert(pack.type == (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION);
            };
            DemoDevice.BATTERY_STATUS p147 = LoopBackDemoChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.temperature = (short)(short) -28765;
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION;
            p147.energy_consumed = (int)1350280178;
            p147.battery_remaining = (sbyte)(sbyte) - 100;
            p147.current_consumed = (int)954791542;
            p147.voltages_SET(new ushort[] {(ushort)19888, (ushort)52401, (ushort)16192, (ushort)60418, (ushort)7664, (ushort)58441, (ushort)30120, (ushort)15746, (ushort)63363, (ushort)9889}, 0) ;
            p147.current_battery = (short)(short) -11954;
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN;
            p147.id = (byte)(byte)64;
            LoopBackDemoChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)6, (byte)105, (byte)24, (byte)154, (byte)156, (byte)98, (byte)174, (byte)56, (byte)147, (byte)32, (byte)67, (byte)237, (byte)0, (byte)8, (byte)50, (byte)129, (byte)28, (byte)190}));
                Debug.Assert(pack.middleware_sw_version == (uint)2645278856U);
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)93, (byte)1, (byte)226, (byte)75, (byte)28, (byte)25, (byte)56, (byte)251}));
                Debug.Assert(pack.board_version == (uint)1526233400U);
                Debug.Assert(pack.os_sw_version == (uint)3258073919U);
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)230, (byte)25, (byte)141, (byte)248, (byte)193, (byte)175, (byte)250, (byte)206}));
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)139, (byte)96, (byte)108, (byte)5, (byte)51, (byte)170, (byte)44, (byte)84}));
                Debug.Assert(pack.flight_sw_version == (uint)423889437U);
                Debug.Assert(pack.product_id == (ushort)(ushort)50314);
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION);
                Debug.Assert(pack.uid == (ulong)8035746161753861527L);
                Debug.Assert(pack.vendor_id == (ushort)(ushort)48175);
            };
            DemoDevice.AUTOPILOT_VERSION p148 = LoopBackDemoChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.uid = (ulong)8035746161753861527L;
            p148.board_version = (uint)1526233400U;
            p148.middleware_sw_version = (uint)2645278856U;
            p148.os_sw_version = (uint)3258073919U;
            p148.os_custom_version_SET(new byte[] {(byte)139, (byte)96, (byte)108, (byte)5, (byte)51, (byte)170, (byte)44, (byte)84}, 0) ;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION;
            p148.middleware_custom_version_SET(new byte[] {(byte)230, (byte)25, (byte)141, (byte)248, (byte)193, (byte)175, (byte)250, (byte)206}, 0) ;
            p148.flight_custom_version_SET(new byte[] {(byte)93, (byte)1, (byte)226, (byte)75, (byte)28, (byte)25, (byte)56, (byte)251}, 0) ;
            p148.uid2_SET(new byte[] {(byte)6, (byte)105, (byte)24, (byte)154, (byte)156, (byte)98, (byte)174, (byte)56, (byte)147, (byte)32, (byte)67, (byte)237, (byte)0, (byte)8, (byte)50, (byte)129, (byte)28, (byte)190}, 0, PH) ;
            p148.product_id = (ushort)(ushort)50314;
            p148.flight_sw_version = (uint)423889437U;
            p148.vendor_id = (ushort)(ushort)48175;
            LoopBackDemoChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)141);
                Debug.Assert(pack.target_num == (byte)(byte)244);
                Debug.Assert(pack.angle_x == (float) -1.2783998E38F);
                Debug.Assert(pack.time_usec == (ulong)860450882554135084L);
                Debug.Assert(pack.angle_y == (float)2.2514947E38F);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {1.9652462E38F, 2.763203E38F, -2.261088E38F, -5.369401E37F}));
                Debug.Assert(pack.z_TRY(ph) == (float) -5.4510264E37F);
                Debug.Assert(pack.size_y == (float)3.0309986E38F);
                Debug.Assert(pack.size_x == (float) -1.4797206E37F);
                Debug.Assert(pack.y_TRY(ph) == (float) -2.0521608E38F);
                Debug.Assert(pack.distance == (float)1.4628469E37F);
                Debug.Assert(pack.x_TRY(ph) == (float)2.6376024E38F);
            };
            DemoDevice.LANDING_TARGET p149 = LoopBackDemoChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.time_usec = (ulong)860450882554135084L;
            p149.size_y = (float)3.0309986E38F;
            p149.z_SET((float) -5.4510264E37F, PH) ;
            p149.q_SET(new float[] {1.9652462E38F, 2.763203E38F, -2.261088E38F, -5.369401E37F}, 0, PH) ;
            p149.angle_y = (float)2.2514947E38F;
            p149.position_valid_SET((byte)(byte)141, PH) ;
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON;
            p149.x_SET((float)2.6376024E38F, PH) ;
            p149.target_num = (byte)(byte)244;
            p149.distance = (float)1.4628469E37F;
            p149.angle_x = (float) -1.2783998E38F;
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION;
            p149.size_x = (float) -1.4797206E37F;
            p149.y_SET((float) -2.0521608E38F, PH) ;
            LoopBackDemoChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAV_FILTER_BIASReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.accel_0 == (float) -3.1738785E38F);
                Debug.Assert(pack.accel_2 == (float)4.0021824E37F);
                Debug.Assert(pack.gyro_0 == (float) -1.0811784E38F);
                Debug.Assert(pack.usec == (ulong)2089227933792451124L);
                Debug.Assert(pack.gyro_2 == (float) -1.0796633E38F);
                Debug.Assert(pack.accel_1 == (float) -6.4553484E37F);
                Debug.Assert(pack.gyro_1 == (float)2.531297E37F);
            };
            DemoDevice.NAV_FILTER_BIAS p220 = LoopBackDemoChannel.new_NAV_FILTER_BIAS();
            PH.setPack(p220);
            p220.accel_0 = (float) -3.1738785E38F;
            p220.accel_2 = (float)4.0021824E37F;
            p220.gyro_2 = (float) -1.0796633E38F;
            p220.gyro_0 = (float) -1.0811784E38F;
            p220.usec = (ulong)2089227933792451124L;
            p220.gyro_1 = (float)2.531297E37F;
            p220.accel_1 = (float) -6.4553484E37F;
            LoopBackDemoChannel.instance.send(p220);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRADIO_CALIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.aileron.SequenceEqual(new ushort[] {(ushort)59866, (ushort)59963, (ushort)57625}));
                Debug.Assert(pack.gyro.SequenceEqual(new ushort[] {(ushort)28966, (ushort)13079}));
                Debug.Assert(pack.rudder.SequenceEqual(new ushort[] {(ushort)2396, (ushort)20587, (ushort)14326}));
                Debug.Assert(pack.throttle.SequenceEqual(new ushort[] {(ushort)9015, (ushort)15269, (ushort)13029, (ushort)46646, (ushort)36183}));
                Debug.Assert(pack.pitch.SequenceEqual(new ushort[] {(ushort)9032, (ushort)25687, (ushort)5895, (ushort)4624, (ushort)22112}));
                Debug.Assert(pack.elevator.SequenceEqual(new ushort[] {(ushort)38719, (ushort)36000, (ushort)30130}));
            };
            DemoDevice.RADIO_CALIBRATION p221 = LoopBackDemoChannel.new_RADIO_CALIBRATION();
            PH.setPack(p221);
            p221.elevator_SET(new ushort[] {(ushort)38719, (ushort)36000, (ushort)30130}, 0) ;
            p221.aileron_SET(new ushort[] {(ushort)59866, (ushort)59963, (ushort)57625}, 0) ;
            p221.gyro_SET(new ushort[] {(ushort)28966, (ushort)13079}, 0) ;
            p221.throttle_SET(new ushort[] {(ushort)9015, (ushort)15269, (ushort)13029, (ushort)46646, (ushort)36183}, 0) ;
            p221.rudder_SET(new ushort[] {(ushort)2396, (ushort)20587, (ushort)14326}, 0) ;
            p221.pitch_SET(new ushort[] {(ushort)9032, (ushort)25687, (ushort)5895, (ushort)4624, (ushort)22112}, 0) ;
            LoopBackDemoChannel.instance.send(p221);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUALBERTA_SYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.nav_mode == (byte)(byte)187);
                Debug.Assert(pack.pilot == (byte)(byte)147);
                Debug.Assert(pack.mode == (byte)(byte)19);
            };
            DemoDevice.UALBERTA_SYS_STATUS p222 = LoopBackDemoChannel.new_UALBERTA_SYS_STATUS();
            PH.setPack(p222);
            p222.pilot = (byte)(byte)147;
            p222.nav_mode = (byte)(byte)187;
            p222.mode = (byte)(byte)19;
            LoopBackDemoChannel.instance.send(p222);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mag_ratio == (float) -3.1906616E38F);
                Debug.Assert(pack.pos_horiz_accuracy == (float)1.2386772E38F);
                Debug.Assert(pack.vel_ratio == (float)2.5394682E38F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT);
                Debug.Assert(pack.pos_horiz_ratio == (float)2.1244366E38F);
                Debug.Assert(pack.time_usec == (ulong)518015637863841027L);
                Debug.Assert(pack.hagl_ratio == (float)8.543256E37F);
                Debug.Assert(pack.pos_vert_accuracy == (float)3.1866982E38F);
                Debug.Assert(pack.pos_vert_ratio == (float) -6.2871636E37F);
                Debug.Assert(pack.tas_ratio == (float) -8.2948783E37F);
            };
            DemoDevice.ESTIMATOR_STATUS p230 = LoopBackDemoChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.hagl_ratio = (float)8.543256E37F;
            p230.pos_vert_ratio = (float) -6.2871636E37F;
            p230.time_usec = (ulong)518015637863841027L;
            p230.pos_horiz_accuracy = (float)1.2386772E38F;
            p230.pos_vert_accuracy = (float)3.1866982E38F;
            p230.vel_ratio = (float)2.5394682E38F;
            p230.tas_ratio = (float) -8.2948783E37F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT;
            p230.pos_horiz_ratio = (float)2.1244366E38F;
            p230.mag_ratio = (float) -3.1906616E38F;
            LoopBackDemoChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vert_accuracy == (float)1.2929329E37F);
                Debug.Assert(pack.wind_alt == (float)2.9594293E38F);
                Debug.Assert(pack.wind_z == (float)8.1887554E36F);
                Debug.Assert(pack.time_usec == (ulong)6136564524248132770L);
                Debug.Assert(pack.wind_y == (float) -2.9378036E38F);
                Debug.Assert(pack.var_vert == (float)1.460423E38F);
                Debug.Assert(pack.var_horiz == (float) -1.926375E38F);
                Debug.Assert(pack.wind_x == (float)1.0648842E38F);
                Debug.Assert(pack.horiz_accuracy == (float) -1.5504948E38F);
            };
            DemoDevice.WIND_COV p231 = LoopBackDemoChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.wind_x = (float)1.0648842E38F;
            p231.horiz_accuracy = (float) -1.5504948E38F;
            p231.wind_y = (float) -2.9378036E38F;
            p231.wind_alt = (float)2.9594293E38F;
            p231.time_usec = (ulong)6136564524248132770L;
            p231.vert_accuracy = (float)1.2929329E37F;
            p231.wind_z = (float)8.1887554E36F;
            p231.var_vert = (float)1.460423E38F;
            p231.var_horiz = (float) -1.926375E38F;
            LoopBackDemoChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fix_type == (byte)(byte)234);
                Debug.Assert(pack.alt == (float) -2.0583859E38F);
                Debug.Assert(pack.time_week_ms == (uint)341365255U);
                Debug.Assert(pack.hdop == (float) -3.2119942E38F);
                Debug.Assert(pack.gps_id == (byte)(byte)232);
                Debug.Assert(pack.horiz_accuracy == (float) -3.5630238E37F);
                Debug.Assert(pack.lat == (int)600521099);
                Debug.Assert(pack.time_usec == (ulong)8091744986954147077L);
                Debug.Assert(pack.vert_accuracy == (float)3.0404543E38F);
                Debug.Assert(pack.vdop == (float)1.812125E38F);
                Debug.Assert(pack.ve == (float) -3.2906717E38F);
                Debug.Assert(pack.time_week == (ushort)(ushort)44903);
                Debug.Assert(pack.speed_accuracy == (float) -5.938138E37F);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY);
                Debug.Assert(pack.vn == (float)2.4394968E38F);
                Debug.Assert(pack.satellites_visible == (byte)(byte)26);
                Debug.Assert(pack.vd == (float) -1.3978983E38F);
                Debug.Assert(pack.lon == (int)1387391954);
            };
            DemoDevice.GPS_INPUT p232 = LoopBackDemoChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.gps_id = (byte)(byte)232;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY;
            p232.vd = (float) -1.3978983E38F;
            p232.time_week = (ushort)(ushort)44903;
            p232.alt = (float) -2.0583859E38F;
            p232.speed_accuracy = (float) -5.938138E37F;
            p232.horiz_accuracy = (float) -3.5630238E37F;
            p232.time_usec = (ulong)8091744986954147077L;
            p232.time_week_ms = (uint)341365255U;
            p232.vdop = (float)1.812125E38F;
            p232.lat = (int)600521099;
            p232.hdop = (float) -3.2119942E38F;
            p232.ve = (float) -3.2906717E38F;
            p232.lon = (int)1387391954;
            p232.fix_type = (byte)(byte)234;
            p232.vert_accuracy = (float)3.0404543E38F;
            p232.vn = (float)2.4394968E38F;
            p232.satellites_visible = (byte)(byte)26;
            LoopBackDemoChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.len == (byte)(byte)239);
                Debug.Assert(pack.flags == (byte)(byte)71);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)111, (byte)175, (byte)143, (byte)218, (byte)199, (byte)165, (byte)14, (byte)232, (byte)75, (byte)163, (byte)42, (byte)68, (byte)128, (byte)78, (byte)130, (byte)155, (byte)225, (byte)163, (byte)244, (byte)131, (byte)136, (byte)114, (byte)161, (byte)169, (byte)158, (byte)23, (byte)220, (byte)94, (byte)34, (byte)10, (byte)94, (byte)100, (byte)12, (byte)99, (byte)90, (byte)251, (byte)173, (byte)103, (byte)148, (byte)117, (byte)74, (byte)213, (byte)218, (byte)187, (byte)210, (byte)227, (byte)147, (byte)136, (byte)251, (byte)197, (byte)137, (byte)127, (byte)48, (byte)110, (byte)156, (byte)58, (byte)151, (byte)44, (byte)18, (byte)155, (byte)6, (byte)113, (byte)200, (byte)131, (byte)110, (byte)57, (byte)200, (byte)202, (byte)43, (byte)63, (byte)43, (byte)87, (byte)53, (byte)84, (byte)102, (byte)138, (byte)81, (byte)206, (byte)179, (byte)220, (byte)178, (byte)244, (byte)38, (byte)159, (byte)19, (byte)217, (byte)85, (byte)59, (byte)28, (byte)194, (byte)214, (byte)42, (byte)171, (byte)142, (byte)83, (byte)206, (byte)90, (byte)28, (byte)71, (byte)203, (byte)17, (byte)74, (byte)113, (byte)52, (byte)196, (byte)242, (byte)213, (byte)249, (byte)27, (byte)244, (byte)46, (byte)64, (byte)199, (byte)73, (byte)94, (byte)73, (byte)100, (byte)41, (byte)82, (byte)62, (byte)89, (byte)197, (byte)197, (byte)12, (byte)132, (byte)229, (byte)220, (byte)130, (byte)202, (byte)231, (byte)125, (byte)158, (byte)183, (byte)237, (byte)92, (byte)225, (byte)52, (byte)242, (byte)121, (byte)98, (byte)196, (byte)201, (byte)30, (byte)194, (byte)195, (byte)69, (byte)87, (byte)75, (byte)79, (byte)21, (byte)176, (byte)31, (byte)230, (byte)49, (byte)42, (byte)141, (byte)180, (byte)216, (byte)79, (byte)185, (byte)181, (byte)86, (byte)247, (byte)157, (byte)63, (byte)64, (byte)103, (byte)245, (byte)244, (byte)183, (byte)221, (byte)28, (byte)104, (byte)34, (byte)52, (byte)244, (byte)213, (byte)234, (byte)86, (byte)163}));
            };
            DemoDevice.GPS_RTCM_DATA p233 = LoopBackDemoChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)71;
            p233.data__SET(new byte[] {(byte)111, (byte)175, (byte)143, (byte)218, (byte)199, (byte)165, (byte)14, (byte)232, (byte)75, (byte)163, (byte)42, (byte)68, (byte)128, (byte)78, (byte)130, (byte)155, (byte)225, (byte)163, (byte)244, (byte)131, (byte)136, (byte)114, (byte)161, (byte)169, (byte)158, (byte)23, (byte)220, (byte)94, (byte)34, (byte)10, (byte)94, (byte)100, (byte)12, (byte)99, (byte)90, (byte)251, (byte)173, (byte)103, (byte)148, (byte)117, (byte)74, (byte)213, (byte)218, (byte)187, (byte)210, (byte)227, (byte)147, (byte)136, (byte)251, (byte)197, (byte)137, (byte)127, (byte)48, (byte)110, (byte)156, (byte)58, (byte)151, (byte)44, (byte)18, (byte)155, (byte)6, (byte)113, (byte)200, (byte)131, (byte)110, (byte)57, (byte)200, (byte)202, (byte)43, (byte)63, (byte)43, (byte)87, (byte)53, (byte)84, (byte)102, (byte)138, (byte)81, (byte)206, (byte)179, (byte)220, (byte)178, (byte)244, (byte)38, (byte)159, (byte)19, (byte)217, (byte)85, (byte)59, (byte)28, (byte)194, (byte)214, (byte)42, (byte)171, (byte)142, (byte)83, (byte)206, (byte)90, (byte)28, (byte)71, (byte)203, (byte)17, (byte)74, (byte)113, (byte)52, (byte)196, (byte)242, (byte)213, (byte)249, (byte)27, (byte)244, (byte)46, (byte)64, (byte)199, (byte)73, (byte)94, (byte)73, (byte)100, (byte)41, (byte)82, (byte)62, (byte)89, (byte)197, (byte)197, (byte)12, (byte)132, (byte)229, (byte)220, (byte)130, (byte)202, (byte)231, (byte)125, (byte)158, (byte)183, (byte)237, (byte)92, (byte)225, (byte)52, (byte)242, (byte)121, (byte)98, (byte)196, (byte)201, (byte)30, (byte)194, (byte)195, (byte)69, (byte)87, (byte)75, (byte)79, (byte)21, (byte)176, (byte)31, (byte)230, (byte)49, (byte)42, (byte)141, (byte)180, (byte)216, (byte)79, (byte)185, (byte)181, (byte)86, (byte)247, (byte)157, (byte)63, (byte)64, (byte)103, (byte)245, (byte)244, (byte)183, (byte)221, (byte)28, (byte)104, (byte)34, (byte)52, (byte)244, (byte)213, (byte)234, (byte)86, (byte)163}, 0) ;
            p233.len = (byte)(byte)239;
            LoopBackDemoChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.wp_distance == (ushort)(ushort)13458);
                Debug.Assert(pack.failsafe == (byte)(byte)202);
                Debug.Assert(pack.roll == (short)(short)29071);
                Debug.Assert(pack.battery_remaining == (byte)(byte)113);
                Debug.Assert(pack.pitch == (short)(short) -23328);
                Debug.Assert(pack.wp_num == (byte)(byte)125);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)124);
                Debug.Assert(pack.throttle == (sbyte)(sbyte) - 49);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte)42);
                Debug.Assert(pack.heading == (ushort)(ushort)45156);
                Debug.Assert(pack.temperature == (sbyte)(sbyte)50);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
                Debug.Assert(pack.heading_sp == (short)(short) -12500);
                Debug.Assert(pack.gps_nsat == (byte)(byte)11);
                Debug.Assert(pack.groundspeed == (byte)(byte)122);
                Debug.Assert(pack.altitude_amsl == (short)(short) -22132);
                Debug.Assert(pack.airspeed == (byte)(byte)81);
                Debug.Assert(pack.longitude == (int) -894321928);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
                Debug.Assert(pack.gps_fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
                Debug.Assert(pack.latitude == (int) -1093097485);
                Debug.Assert(pack.altitude_sp == (short)(short)4015);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte)21);
                Debug.Assert(pack.custom_mode == (uint)3905024688U);
            };
            DemoDevice.HIGH_LATENCY p234 = LoopBackDemoChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.groundspeed = (byte)(byte)122;
            p234.temperature_air = (sbyte)(sbyte)42;
            p234.airspeed_sp = (byte)(byte)124;
            p234.altitude_sp = (short)(short)4015;
            p234.climb_rate = (sbyte)(sbyte)21;
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            p234.airspeed = (byte)(byte)81;
            p234.wp_distance = (ushort)(ushort)13458;
            p234.latitude = (int) -1093097485;
            p234.battery_remaining = (byte)(byte)113;
            p234.throttle = (sbyte)(sbyte) - 49;
            p234.longitude = (int) -894321928;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF;
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_PPP;
            p234.heading_sp = (short)(short) -12500;
            p234.failsafe = (byte)(byte)202;
            p234.gps_nsat = (byte)(byte)11;
            p234.temperature = (sbyte)(sbyte)50;
            p234.heading = (ushort)(ushort)45156;
            p234.custom_mode = (uint)3905024688U;
            p234.pitch = (short)(short) -23328;
            p234.altitude_amsl = (short)(short) -22132;
            p234.wp_num = (byte)(byte)125;
            p234.roll = (short)(short)29071;
            LoopBackDemoChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vibration_y == (float)1.8876305E38F);
                Debug.Assert(pack.vibration_z == (float)1.9771544E38F);
                Debug.Assert(pack.clipping_2 == (uint)3868343611U);
                Debug.Assert(pack.clipping_0 == (uint)348520264U);
                Debug.Assert(pack.time_usec == (ulong)2014151213077823625L);
                Debug.Assert(pack.vibration_x == (float)1.6772453E38F);
                Debug.Assert(pack.clipping_1 == (uint)3547062204U);
            };
            DemoDevice.VIBRATION p241 = LoopBackDemoChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.clipping_0 = (uint)348520264U;
            p241.clipping_2 = (uint)3868343611U;
            p241.vibration_x = (float)1.6772453E38F;
            p241.time_usec = (ulong)2014151213077823625L;
            p241.vibration_y = (float)1.8876305E38F;
            p241.clipping_1 = (uint)3547062204U;
            p241.vibration_z = (float)1.9771544E38F;
            LoopBackDemoChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.approach_y == (float) -6.990507E37F);
                Debug.Assert(pack.approach_z == (float)3.2840811E38F);
                Debug.Assert(pack.z == (float)1.9332283E36F);
                Debug.Assert(pack.altitude == (int)1746095519);
                Debug.Assert(pack.longitude == (int) -1968554885);
                Debug.Assert(pack.latitude == (int) -1822198947);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)6753225503448515890L);
                Debug.Assert(pack.approach_x == (float)2.7183402E38F);
                Debug.Assert(pack.x == (float) -1.2491533E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.4138146E38F, -2.5149758E38F, -3.2846446E38F, -3.0622043E38F}));
                Debug.Assert(pack.y == (float)3.2383149E38F);
            };
            DemoDevice.HOME_POSITION p242 = LoopBackDemoChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.y = (float)3.2383149E38F;
            p242.q_SET(new float[] {2.4138146E38F, -2.5149758E38F, -3.2846446E38F, -3.0622043E38F}, 0) ;
            p242.x = (float) -1.2491533E38F;
            p242.longitude = (int) -1968554885;
            p242.latitude = (int) -1822198947;
            p242.z = (float)1.9332283E36F;
            p242.time_usec_SET((ulong)6753225503448515890L, PH) ;
            p242.altitude = (int)1746095519;
            p242.approach_x = (float)2.7183402E38F;
            p242.approach_y = (float) -6.990507E37F;
            p242.approach_z = (float)3.2840811E38F;
            LoopBackDemoChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)1.0302673E38F);
                Debug.Assert(pack.target_system == (byte)(byte)105);
                Debug.Assert(pack.latitude == (int)1693879656);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)8095477458116245363L);
                Debug.Assert(pack.x == (float)3.3489532E38F);
                Debug.Assert(pack.approach_y == (float)3.0597295E37F);
                Debug.Assert(pack.z == (float) -2.055255E37F);
                Debug.Assert(pack.longitude == (int)1327259339);
                Debug.Assert(pack.altitude == (int)1886743198);
                Debug.Assert(pack.approach_x == (float) -1.44166E36F);
                Debug.Assert(pack.approach_z == (float) -2.5216726E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {3.3868452E38F, 1.827132E38F, -3.1866274E38F, -3.1985323E38F}));
            };
            DemoDevice.SET_HOME_POSITION p243 = LoopBackDemoChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.z = (float) -2.055255E37F;
            p243.approach_x = (float) -1.44166E36F;
            p243.approach_z = (float) -2.5216726E38F;
            p243.x = (float)3.3489532E38F;
            p243.latitude = (int)1693879656;
            p243.target_system = (byte)(byte)105;
            p243.y = (float)1.0302673E38F;
            p243.altitude = (int)1886743198;
            p243.time_usec_SET((ulong)8095477458116245363L, PH) ;
            p243.longitude = (int)1327259339;
            p243.q_SET(new float[] {3.3868452E38F, 1.827132E38F, -3.1866274E38F, -3.1985323E38F}, 0) ;
            p243.approach_y = (float)3.0597295E37F;
            LoopBackDemoChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_id == (ushort)(ushort)22583);
                Debug.Assert(pack.interval_us == (int)1584176777);
            };
            DemoDevice.MESSAGE_INTERVAL p244 = LoopBackDemoChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)22583;
            p244.interval_us = (int)1584176777;
            LoopBackDemoChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
                Debug.Assert(pack.vtol_state == (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_FW);
            };
            DemoDevice.EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING;
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_FW;
            LoopBackDemoChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.emitter_type == (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LARGE);
                Debug.Assert(pack.altitude_type == (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
                Debug.Assert(pack.ICAO_address == (uint)4155185625U);
                Debug.Assert(pack.heading == (ushort)(ushort)51671);
                Debug.Assert(pack.ver_velocity == (short)(short) -14165);
                Debug.Assert(pack.squawk == (ushort)(ushort)13785);
                Debug.Assert(pack.callsign_LEN(ph) == 7);
                Debug.Assert(pack.callsign_TRY(ph).Equals("LnsdBaf"));
                Debug.Assert(pack.flags == (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_SIMULATED);
                Debug.Assert(pack.lon == (int) -70956726);
                Debug.Assert(pack.lat == (int) -663591907);
                Debug.Assert(pack.tslc == (byte)(byte)78);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)10504);
                Debug.Assert(pack.altitude == (int) -188750932);
            };
            DemoDevice.ADSB_VEHICLE p246 = LoopBackDemoChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.squawk = (ushort)(ushort)13785;
            p246.tslc = (byte)(byte)78;
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_SIMULATED;
            p246.lon = (int) -70956726;
            p246.callsign_SET("LnsdBaf", PH) ;
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LARGE;
            p246.hor_velocity = (ushort)(ushort)10504;
            p246.heading = (ushort)(ushort)51671;
            p246.ver_velocity = (short)(short) -14165;
            p246.lat = (int) -663591907;
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.ICAO_address = (uint)4155185625U;
            p246.altitude = (int) -188750932;
            LoopBackDemoChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.horizontal_minimum_delta == (float)6.93804E37F);
                Debug.Assert(pack.altitude_minimum_delta == (float) -3.3833422E38F);
                Debug.Assert(pack.src_ == (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
                Debug.Assert(pack.threat_level == (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
                Debug.Assert(pack.action == (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND);
                Debug.Assert(pack.id == (uint)3290598076U);
                Debug.Assert(pack.time_to_minimum_delta == (float)2.0358915E38F);
            };
            DemoDevice.COLLISION p247 = LoopBackDemoChannel.new_COLLISION();
            PH.setPack(p247);
            p247.altitude_minimum_delta = (float) -3.3833422E38F;
            p247.id = (uint)3290598076U;
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND;
            p247.horizontal_minimum_delta = (float)6.93804E37F;
            p247.time_to_minimum_delta = (float)2.0358915E38F;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW;
            LoopBackDemoChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_network == (byte)(byte)204);
                Debug.Assert(pack.target_component == (byte)(byte)194);
                Debug.Assert(pack.message_type == (ushort)(ushort)17384);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)171, (byte)22, (byte)254, (byte)97, (byte)26, (byte)138, (byte)188, (byte)55, (byte)240, (byte)17, (byte)212, (byte)11, (byte)124, (byte)237, (byte)63, (byte)132, (byte)73, (byte)251, (byte)141, (byte)17, (byte)213, (byte)1, (byte)194, (byte)214, (byte)221, (byte)114, (byte)82, (byte)146, (byte)89, (byte)95, (byte)127, (byte)193, (byte)129, (byte)204, (byte)239, (byte)15, (byte)96, (byte)51, (byte)214, (byte)205, (byte)205, (byte)34, (byte)14, (byte)128, (byte)138, (byte)212, (byte)94, (byte)247, (byte)148, (byte)165, (byte)231, (byte)195, (byte)114, (byte)79, (byte)64, (byte)99, (byte)204, (byte)19, (byte)77, (byte)28, (byte)221, (byte)193, (byte)230, (byte)18, (byte)250, (byte)167, (byte)94, (byte)85, (byte)38, (byte)144, (byte)118, (byte)91, (byte)145, (byte)61, (byte)51, (byte)224, (byte)48, (byte)254, (byte)28, (byte)125, (byte)66, (byte)111, (byte)0, (byte)152, (byte)68, (byte)179, (byte)89, (byte)60, (byte)135, (byte)227, (byte)132, (byte)178, (byte)76, (byte)45, (byte)129, (byte)20, (byte)129, (byte)240, (byte)142, (byte)227, (byte)160, (byte)250, (byte)192, (byte)27, (byte)61, (byte)246, (byte)207, (byte)192, (byte)193, (byte)85, (byte)241, (byte)161, (byte)130, (byte)225, (byte)188, (byte)201, (byte)123, (byte)101, (byte)43, (byte)32, (byte)206, (byte)243, (byte)82, (byte)108, (byte)56, (byte)128, (byte)231, (byte)75, (byte)159, (byte)255, (byte)34, (byte)230, (byte)169, (byte)139, (byte)21, (byte)36, (byte)211, (byte)104, (byte)109, (byte)150, (byte)125, (byte)3, (byte)117, (byte)53, (byte)226, (byte)20, (byte)242, (byte)69, (byte)173, (byte)183, (byte)191, (byte)60, (byte)217, (byte)125, (byte)166, (byte)111, (byte)195, (byte)150, (byte)251, (byte)197, (byte)253, (byte)96, (byte)9, (byte)50, (byte)94, (byte)36, (byte)38, (byte)117, (byte)76, (byte)179, (byte)204, (byte)249, (byte)205, (byte)214, (byte)213, (byte)91, (byte)200, (byte)83, (byte)191, (byte)7, (byte)133, (byte)108, (byte)161, (byte)117, (byte)162, (byte)53, (byte)151, (byte)43, (byte)232, (byte)147, (byte)158, (byte)57, (byte)129, (byte)125, (byte)187, (byte)58, (byte)218, (byte)133, (byte)184, (byte)31, (byte)94, (byte)187, (byte)242, (byte)244, (byte)255, (byte)136, (byte)187, (byte)34, (byte)234, (byte)43, (byte)178, (byte)209, (byte)124, (byte)31, (byte)254, (byte)204, (byte)150, (byte)145, (byte)243, (byte)86, (byte)188, (byte)229, (byte)194, (byte)4, (byte)233, (byte)198, (byte)71, (byte)124, (byte)52, (byte)80, (byte)242, (byte)196, (byte)227, (byte)212, (byte)187, (byte)44, (byte)226, (byte)180, (byte)141, (byte)122, (byte)196, (byte)188, (byte)18, (byte)133, (byte)40, (byte)161, (byte)139, (byte)115, (byte)113}));
                Debug.Assert(pack.target_system == (byte)(byte)209);
            };
            DemoDevice.V2_EXTENSION p248 = LoopBackDemoChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.payload_SET(new byte[] {(byte)171, (byte)22, (byte)254, (byte)97, (byte)26, (byte)138, (byte)188, (byte)55, (byte)240, (byte)17, (byte)212, (byte)11, (byte)124, (byte)237, (byte)63, (byte)132, (byte)73, (byte)251, (byte)141, (byte)17, (byte)213, (byte)1, (byte)194, (byte)214, (byte)221, (byte)114, (byte)82, (byte)146, (byte)89, (byte)95, (byte)127, (byte)193, (byte)129, (byte)204, (byte)239, (byte)15, (byte)96, (byte)51, (byte)214, (byte)205, (byte)205, (byte)34, (byte)14, (byte)128, (byte)138, (byte)212, (byte)94, (byte)247, (byte)148, (byte)165, (byte)231, (byte)195, (byte)114, (byte)79, (byte)64, (byte)99, (byte)204, (byte)19, (byte)77, (byte)28, (byte)221, (byte)193, (byte)230, (byte)18, (byte)250, (byte)167, (byte)94, (byte)85, (byte)38, (byte)144, (byte)118, (byte)91, (byte)145, (byte)61, (byte)51, (byte)224, (byte)48, (byte)254, (byte)28, (byte)125, (byte)66, (byte)111, (byte)0, (byte)152, (byte)68, (byte)179, (byte)89, (byte)60, (byte)135, (byte)227, (byte)132, (byte)178, (byte)76, (byte)45, (byte)129, (byte)20, (byte)129, (byte)240, (byte)142, (byte)227, (byte)160, (byte)250, (byte)192, (byte)27, (byte)61, (byte)246, (byte)207, (byte)192, (byte)193, (byte)85, (byte)241, (byte)161, (byte)130, (byte)225, (byte)188, (byte)201, (byte)123, (byte)101, (byte)43, (byte)32, (byte)206, (byte)243, (byte)82, (byte)108, (byte)56, (byte)128, (byte)231, (byte)75, (byte)159, (byte)255, (byte)34, (byte)230, (byte)169, (byte)139, (byte)21, (byte)36, (byte)211, (byte)104, (byte)109, (byte)150, (byte)125, (byte)3, (byte)117, (byte)53, (byte)226, (byte)20, (byte)242, (byte)69, (byte)173, (byte)183, (byte)191, (byte)60, (byte)217, (byte)125, (byte)166, (byte)111, (byte)195, (byte)150, (byte)251, (byte)197, (byte)253, (byte)96, (byte)9, (byte)50, (byte)94, (byte)36, (byte)38, (byte)117, (byte)76, (byte)179, (byte)204, (byte)249, (byte)205, (byte)214, (byte)213, (byte)91, (byte)200, (byte)83, (byte)191, (byte)7, (byte)133, (byte)108, (byte)161, (byte)117, (byte)162, (byte)53, (byte)151, (byte)43, (byte)232, (byte)147, (byte)158, (byte)57, (byte)129, (byte)125, (byte)187, (byte)58, (byte)218, (byte)133, (byte)184, (byte)31, (byte)94, (byte)187, (byte)242, (byte)244, (byte)255, (byte)136, (byte)187, (byte)34, (byte)234, (byte)43, (byte)178, (byte)209, (byte)124, (byte)31, (byte)254, (byte)204, (byte)150, (byte)145, (byte)243, (byte)86, (byte)188, (byte)229, (byte)194, (byte)4, (byte)233, (byte)198, (byte)71, (byte)124, (byte)52, (byte)80, (byte)242, (byte)196, (byte)227, (byte)212, (byte)187, (byte)44, (byte)226, (byte)180, (byte)141, (byte)122, (byte)196, (byte)188, (byte)18, (byte)133, (byte)40, (byte)161, (byte)139, (byte)115, (byte)113}, 0) ;
            p248.message_type = (ushort)(ushort)17384;
            p248.target_network = (byte)(byte)204;
            p248.target_system = (byte)(byte)209;
            p248.target_component = (byte)(byte)194;
            LoopBackDemoChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (byte)(byte)211);
                Debug.Assert(pack.address == (ushort)(ushort)46524);
                Debug.Assert(pack.ver == (byte)(byte)113);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte)66, (sbyte)82, (sbyte)92, (sbyte) - 111, (sbyte)54, (sbyte)31, (sbyte)124, (sbyte) - 125, (sbyte)32, (sbyte)23, (sbyte)125, (sbyte) - 107, (sbyte) - 32, (sbyte) - 91, (sbyte)125, (sbyte)19, (sbyte) - 110, (sbyte) - 111, (sbyte) - 9, (sbyte)43, (sbyte)17, (sbyte)59, (sbyte) - 110, (sbyte) - 78, (sbyte) - 14, (sbyte)117, (sbyte)125, (sbyte)115, (sbyte) - 38, (sbyte)83, (sbyte) - 27, (sbyte) - 67}));
            };
            DemoDevice.MEMORY_VECT p249 = LoopBackDemoChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.value_SET(new sbyte[] {(sbyte)66, (sbyte)82, (sbyte)92, (sbyte) - 111, (sbyte)54, (sbyte)31, (sbyte)124, (sbyte) - 125, (sbyte)32, (sbyte)23, (sbyte)125, (sbyte) - 107, (sbyte) - 32, (sbyte) - 91, (sbyte)125, (sbyte)19, (sbyte) - 110, (sbyte) - 111, (sbyte) - 9, (sbyte)43, (sbyte)17, (sbyte)59, (sbyte) - 110, (sbyte) - 78, (sbyte) - 14, (sbyte)117, (sbyte)125, (sbyte)115, (sbyte) - 38, (sbyte)83, (sbyte) - 27, (sbyte) - 67}, 0) ;
            p249.type = (byte)(byte)211;
            p249.ver = (byte)(byte)113;
            p249.address = (ushort)(ushort)46524;
            LoopBackDemoChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)193706279987472702L);
                Debug.Assert(pack.name_LEN(ph) == 4);
                Debug.Assert(pack.name_TRY(ph).Equals("tklu"));
                Debug.Assert(pack.x == (float) -2.5066387E38F);
                Debug.Assert(pack.z == (float)4.2249048E37F);
                Debug.Assert(pack.y == (float)1.3036335E38F);
            };
            DemoDevice.DEBUG_VECT p250 = LoopBackDemoChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.z = (float)4.2249048E37F;
            p250.y = (float)1.3036335E38F;
            p250.x = (float) -2.5066387E38F;
            p250.name_SET("tklu", PH) ;
            p250.time_usec = (ulong)193706279987472702L;
            LoopBackDemoChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 7);
                Debug.Assert(pack.name_TRY(ph).Equals("IQeIpdu"));
                Debug.Assert(pack.value == (float)1.2923043E38F);
                Debug.Assert(pack.time_boot_ms == (uint)232248093U);
            };
            DemoDevice.NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)232248093U;
            p251.value = (float)1.2923043E38F;
            p251.name_SET("IQeIpdu", PH) ;
            LoopBackDemoChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (int)641222081);
                Debug.Assert(pack.time_boot_ms == (uint)418886684U);
                Debug.Assert(pack.name_LEN(ph) == 9);
                Debug.Assert(pack.name_TRY(ph).Equals("IkflvAkJy"));
            };
            DemoDevice.NAMED_VALUE_INT p252 = LoopBackDemoChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.value = (int)641222081;
            p252.time_boot_ms = (uint)418886684U;
            p252.name_SET("IkflvAkJy", PH) ;
            LoopBackDemoChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.text_LEN(ph) == 44);
                Debug.Assert(pack.text_TRY(ph).Equals("zahlablvwaerbatohmlckzvtwqxedeytnqdmurxrnfIl"));
                Debug.Assert(pack.severity == (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_INFO);
            };
            DemoDevice.STATUSTEXT p253 = LoopBackDemoChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.text_SET("zahlablvwaerbatohmlckzvtwqxedeytnqdmurxrnfIl", PH) ;
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_INFO;
            LoopBackDemoChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (float) -1.300079E38F);
                Debug.Assert(pack.time_boot_ms == (uint)658987444U);
                Debug.Assert(pack.ind == (byte)(byte)59);
            };
            DemoDevice.DEBUG p254 = LoopBackDemoChannel.new_DEBUG();
            PH.setPack(p254);
            p254.value = (float) -1.300079E38F;
            p254.time_boot_ms = (uint)658987444U;
            p254.ind = (byte)(byte)59;
            LoopBackDemoChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)109);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)40, (byte)106, (byte)120, (byte)252, (byte)231, (byte)126, (byte)39, (byte)153, (byte)171, (byte)41, (byte)236, (byte)18, (byte)52, (byte)179, (byte)14, (byte)183, (byte)48, (byte)123, (byte)173, (byte)143, (byte)90, (byte)193, (byte)40, (byte)49, (byte)139, (byte)21, (byte)173, (byte)17, (byte)107, (byte)196, (byte)162, (byte)42}));
                Debug.Assert(pack.target_component == (byte)(byte)226);
                Debug.Assert(pack.initial_timestamp == (ulong)2995429742950168500L);
            };
            DemoDevice.SETUP_SIGNING p256 = LoopBackDemoChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.secret_key_SET(new byte[] {(byte)40, (byte)106, (byte)120, (byte)252, (byte)231, (byte)126, (byte)39, (byte)153, (byte)171, (byte)41, (byte)236, (byte)18, (byte)52, (byte)179, (byte)14, (byte)183, (byte)48, (byte)123, (byte)173, (byte)143, (byte)90, (byte)193, (byte)40, (byte)49, (byte)139, (byte)21, (byte)173, (byte)17, (byte)107, (byte)196, (byte)162, (byte)42}, 0) ;
            p256.initial_timestamp = (ulong)2995429742950168500L;
            p256.target_system = (byte)(byte)109;
            p256.target_component = (byte)(byte)226;
            LoopBackDemoChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.last_change_ms == (uint)2894170019U);
                Debug.Assert(pack.state == (byte)(byte)23);
                Debug.Assert(pack.time_boot_ms == (uint)517781723U);
            };
            DemoDevice.BUTTON_CHANGE p257 = LoopBackDemoChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)517781723U;
            p257.last_change_ms = (uint)2894170019U;
            p257.state = (byte)(byte)23;
            LoopBackDemoChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)179);
                Debug.Assert(pack.target_component == (byte)(byte)185);
                Debug.Assert(pack.tune_LEN(ph) == 17);
                Debug.Assert(pack.tune_TRY(ph).Equals("sdzdjmorvckwdvcvv"));
            };
            DemoDevice.PLAY_TUNE p258 = LoopBackDemoChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.tune_SET("sdzdjmorvckwdvcvv", PH) ;
            p258.target_system = (byte)(byte)179;
            p258.target_component = (byte)(byte)185;
            LoopBackDemoChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.firmware_version == (uint)635918639U);
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)72, (byte)28, (byte)220, (byte)239, (byte)88, (byte)110, (byte)91, (byte)17, (byte)244, (byte)77, (byte)8, (byte)255, (byte)79, (byte)155, (byte)78, (byte)228, (byte)91, (byte)2, (byte)0, (byte)220, (byte)188, (byte)182, (byte)216, (byte)20, (byte)98, (byte)125, (byte)123, (byte)169, (byte)188, (byte)67, (byte)80, (byte)127}));
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)17981);
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 10);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("iakycExkcz"));
                Debug.Assert(pack.focal_length == (float)1.2569784E38F);
                Debug.Assert(pack.lens_id == (byte)(byte)152);
                Debug.Assert(pack.time_boot_ms == (uint)2156926696U);
                Debug.Assert(pack.sensor_size_h == (float) -2.0448232E38F);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)40460);
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES);
                Debug.Assert(pack.sensor_size_v == (float)9.901985E37F);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)3573);
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)51, (byte)76, (byte)210, (byte)45, (byte)21, (byte)182, (byte)163, (byte)229, (byte)42, (byte)191, (byte)223, (byte)225, (byte)8, (byte)180, (byte)20, (byte)175, (byte)178, (byte)163, (byte)15, (byte)113, (byte)161, (byte)219, (byte)134, (byte)219, (byte)12, (byte)58, (byte)232, (byte)17, (byte)239, (byte)94, (byte)244, (byte)42}));
            };
            DemoDevice.CAMERA_INFORMATION p259 = LoopBackDemoChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.focal_length = (float)1.2569784E38F;
            p259.cam_definition_uri_SET("iakycExkcz", PH) ;
            p259.model_name_SET(new byte[] {(byte)51, (byte)76, (byte)210, (byte)45, (byte)21, (byte)182, (byte)163, (byte)229, (byte)42, (byte)191, (byte)223, (byte)225, (byte)8, (byte)180, (byte)20, (byte)175, (byte)178, (byte)163, (byte)15, (byte)113, (byte)161, (byte)219, (byte)134, (byte)219, (byte)12, (byte)58, (byte)232, (byte)17, (byte)239, (byte)94, (byte)244, (byte)42}, 0) ;
            p259.time_boot_ms = (uint)2156926696U;
            p259.cam_definition_version = (ushort)(ushort)17981;
            p259.sensor_size_v = (float)9.901985E37F;
            p259.sensor_size_h = (float) -2.0448232E38F;
            p259.resolution_v = (ushort)(ushort)40460;
            p259.lens_id = (byte)(byte)152;
            p259.vendor_name_SET(new byte[] {(byte)72, (byte)28, (byte)220, (byte)239, (byte)88, (byte)110, (byte)91, (byte)17, (byte)244, (byte)77, (byte)8, (byte)255, (byte)79, (byte)155, (byte)78, (byte)228, (byte)91, (byte)2, (byte)0, (byte)220, (byte)188, (byte)182, (byte)216, (byte)20, (byte)98, (byte)125, (byte)123, (byte)169, (byte)188, (byte)67, (byte)80, (byte)127}, 0) ;
            p259.resolution_h = (ushort)(ushort)3573;
            p259.firmware_version = (uint)635918639U;
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES;
            LoopBackDemoChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode_id == (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
                Debug.Assert(pack.time_boot_ms == (uint)3083534119U);
            };
            DemoDevice.CAMERA_SETTINGS p260 = LoopBackDemoChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY;
            p260.time_boot_ms = (uint)3083534119U;
            LoopBackDemoChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.status == (byte)(byte)160);
                Debug.Assert(pack.write_speed == (float)3.147537E38F);
                Debug.Assert(pack.storage_id == (byte)(byte)3);
                Debug.Assert(pack.used_capacity == (float)2.0120167E38F);
                Debug.Assert(pack.available_capacity == (float) -1.4664326E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1020435299U);
                Debug.Assert(pack.total_capacity == (float) -1.6852726E38F);
                Debug.Assert(pack.read_speed == (float)1.5918864E38F);
                Debug.Assert(pack.storage_count == (byte)(byte)211);
            };
            DemoDevice.STORAGE_INFORMATION p261 = LoopBackDemoChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.storage_count = (byte)(byte)211;
            p261.total_capacity = (float) -1.6852726E38F;
            p261.read_speed = (float)1.5918864E38F;
            p261.time_boot_ms = (uint)1020435299U;
            p261.available_capacity = (float) -1.4664326E38F;
            p261.write_speed = (float)3.147537E38F;
            p261.used_capacity = (float)2.0120167E38F;
            p261.status = (byte)(byte)160;
            p261.storage_id = (byte)(byte)3;
            LoopBackDemoChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.recording_time_ms == (uint)1642098728U);
                Debug.Assert(pack.video_status == (byte)(byte)71);
                Debug.Assert(pack.image_status == (byte)(byte)63);
                Debug.Assert(pack.image_interval == (float) -1.7822812E38F);
                Debug.Assert(pack.time_boot_ms == (uint)35156480U);
                Debug.Assert(pack.available_capacity == (float) -7.845537E37F);
            };
            DemoDevice.CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.image_interval = (float) -1.7822812E38F;
            p262.video_status = (byte)(byte)71;
            p262.available_capacity = (float) -7.845537E37F;
            p262.image_status = (byte)(byte)63;
            p262.time_boot_ms = (uint)35156480U;
            p262.recording_time_ms = (uint)1642098728U;
            LoopBackDemoChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_utc == (ulong)1532929940395946125L);
                Debug.Assert(pack.file_url_LEN(ph) == 109);
                Debug.Assert(pack.file_url_TRY(ph).Equals("glYdjdxlqcpbtBtQjhlgdhjGtpYkrFijypescxjmpkougieHbtpcnndxbaptgqbwumngakgtdpkawzZstwbvhuDlpqPmgFaEoauxUqLgjbxgy"));
                Debug.Assert(pack.alt == (int) -1980650588);
                Debug.Assert(pack.camera_id == (byte)(byte)60);
                Debug.Assert(pack.relative_alt == (int)638331596);
                Debug.Assert(pack.lat == (int)1432912966);
                Debug.Assert(pack.time_boot_ms == (uint)1395941692U);
                Debug.Assert(pack.capture_result == (sbyte)(sbyte) - 44);
                Debug.Assert(pack.lon == (int) -444511236);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.749257E38F, 6.527337E37F, -2.6397179E38F, -1.6907466E38F}));
                Debug.Assert(pack.image_index == (int) -1524888153);
            };
            DemoDevice.CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.lon = (int) -444511236;
            p263.relative_alt = (int)638331596;
            p263.image_index = (int) -1524888153;
            p263.file_url_SET("glYdjdxlqcpbtBtQjhlgdhjGtpYkrFijypescxjmpkougieHbtpcnndxbaptgqbwumngakgtdpkawzZstwbvhuDlpqPmgFaEoauxUqLgjbxgy", PH) ;
            p263.capture_result = (sbyte)(sbyte) - 44;
            p263.q_SET(new float[] {-1.749257E38F, 6.527337E37F, -2.6397179E38F, -1.6907466E38F}, 0) ;
            p263.lat = (int)1432912966;
            p263.time_boot_ms = (uint)1395941692U;
            p263.camera_id = (byte)(byte)60;
            p263.alt = (int) -1980650588;
            p263.time_utc = (ulong)1532929940395946125L;
            LoopBackDemoChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.arming_time_utc == (ulong)7555334583075954620L);
                Debug.Assert(pack.time_boot_ms == (uint)3747388646U);
                Debug.Assert(pack.flight_uuid == (ulong)1481820994140372175L);
                Debug.Assert(pack.takeoff_time_utc == (ulong)5894193344014004419L);
            };
            DemoDevice.FLIGHT_INFORMATION p264 = LoopBackDemoChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)3747388646U;
            p264.flight_uuid = (ulong)1481820994140372175L;
            p264.arming_time_utc = (ulong)7555334583075954620L;
            p264.takeoff_time_utc = (ulong)5894193344014004419L;
            LoopBackDemoChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)58873866U);
                Debug.Assert(pack.pitch == (float)2.4711944E38F);
                Debug.Assert(pack.yaw == (float) -4.5984335E37F);
                Debug.Assert(pack.roll == (float) -2.9044106E38F);
            };
            DemoDevice.MOUNT_ORIENTATION p265 = LoopBackDemoChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.roll = (float) -2.9044106E38F;
            p265.pitch = (float)2.4711944E38F;
            p265.time_boot_ms = (uint)58873866U;
            p265.yaw = (float) -4.5984335E37F;
            LoopBackDemoChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)228);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)126, (byte)38, (byte)249, (byte)124, (byte)109, (byte)103, (byte)83, (byte)253, (byte)164, (byte)208, (byte)59, (byte)230, (byte)110, (byte)239, (byte)32, (byte)172, (byte)57, (byte)127, (byte)115, (byte)144, (byte)138, (byte)21, (byte)224, (byte)128, (byte)187, (byte)18, (byte)214, (byte)244, (byte)183, (byte)49, (byte)250, (byte)91, (byte)134, (byte)109, (byte)147, (byte)247, (byte)110, (byte)9, (byte)124, (byte)44, (byte)255, (byte)8, (byte)243, (byte)250, (byte)189, (byte)160, (byte)37, (byte)58, (byte)75, (byte)168, (byte)158, (byte)123, (byte)117, (byte)87, (byte)119, (byte)57, (byte)211, (byte)140, (byte)228, (byte)254, (byte)96, (byte)185, (byte)102, (byte)116, (byte)200, (byte)52, (byte)144, (byte)247, (byte)150, (byte)201, (byte)223, (byte)116, (byte)228, (byte)136, (byte)68, (byte)151, (byte)135, (byte)207, (byte)32, (byte)93, (byte)65, (byte)36, (byte)208, (byte)36, (byte)246, (byte)102, (byte)147, (byte)76, (byte)121, (byte)31, (byte)220, (byte)49, (byte)120, (byte)92, (byte)132, (byte)181, (byte)143, (byte)119, (byte)214, (byte)150, (byte)219, (byte)164, (byte)193, (byte)151, (byte)108, (byte)135, (byte)6, (byte)249, (byte)194, (byte)162, (byte)234, (byte)26, (byte)12, (byte)209, (byte)210, (byte)7, (byte)57, (byte)152, (byte)118, (byte)112, (byte)3, (byte)215, (byte)159, (byte)124, (byte)179, (byte)143, (byte)218, (byte)233, (byte)93, (byte)213, (byte)228, (byte)85, (byte)11, (byte)99, (byte)40, (byte)6, (byte)140, (byte)140, (byte)37, (byte)188, (byte)98, (byte)157, (byte)106, (byte)248, (byte)233, (byte)90, (byte)73, (byte)112, (byte)193, (byte)29, (byte)96, (byte)66, (byte)146, (byte)159, (byte)100, (byte)127, (byte)59, (byte)230, (byte)211, (byte)137, (byte)131, (byte)80, (byte)221, (byte)4, (byte)235, (byte)140, (byte)138, (byte)173, (byte)31, (byte)217, (byte)115, (byte)79, (byte)42, (byte)232, (byte)40, (byte)15, (byte)4, (byte)64, (byte)103, (byte)93, (byte)89, (byte)132, (byte)186, (byte)112, (byte)40, (byte)70, (byte)199, (byte)189, (byte)23, (byte)44, (byte)131, (byte)102, (byte)133, (byte)77, (byte)122, (byte)195, (byte)65, (byte)23, (byte)148, (byte)44, (byte)85, (byte)226, (byte)0, (byte)160, (byte)212, (byte)127, (byte)93, (byte)22, (byte)72, (byte)13, (byte)144, (byte)185, (byte)65, (byte)51, (byte)82, (byte)209, (byte)210, (byte)193, (byte)116, (byte)147, (byte)205, (byte)205, (byte)166, (byte)72, (byte)132, (byte)57, (byte)230, (byte)243, (byte)214, (byte)198, (byte)0, (byte)227, (byte)158, (byte)43, (byte)188, (byte)68, (byte)33, (byte)47, (byte)158, (byte)157, (byte)125, (byte)247, (byte)168, (byte)251, (byte)140, (byte)16, (byte)244, (byte)228, (byte)120}));
                Debug.Assert(pack.sequence == (ushort)(ushort)8279);
                Debug.Assert(pack.first_message_offset == (byte)(byte)57);
                Debug.Assert(pack.target_component == (byte)(byte)87);
                Debug.Assert(pack.length == (byte)(byte)59);
            };
            DemoDevice.LOGGING_DATA p266 = LoopBackDemoChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_component = (byte)(byte)87;
            p266.first_message_offset = (byte)(byte)57;
            p266.sequence = (ushort)(ushort)8279;
            p266.data__SET(new byte[] {(byte)126, (byte)38, (byte)249, (byte)124, (byte)109, (byte)103, (byte)83, (byte)253, (byte)164, (byte)208, (byte)59, (byte)230, (byte)110, (byte)239, (byte)32, (byte)172, (byte)57, (byte)127, (byte)115, (byte)144, (byte)138, (byte)21, (byte)224, (byte)128, (byte)187, (byte)18, (byte)214, (byte)244, (byte)183, (byte)49, (byte)250, (byte)91, (byte)134, (byte)109, (byte)147, (byte)247, (byte)110, (byte)9, (byte)124, (byte)44, (byte)255, (byte)8, (byte)243, (byte)250, (byte)189, (byte)160, (byte)37, (byte)58, (byte)75, (byte)168, (byte)158, (byte)123, (byte)117, (byte)87, (byte)119, (byte)57, (byte)211, (byte)140, (byte)228, (byte)254, (byte)96, (byte)185, (byte)102, (byte)116, (byte)200, (byte)52, (byte)144, (byte)247, (byte)150, (byte)201, (byte)223, (byte)116, (byte)228, (byte)136, (byte)68, (byte)151, (byte)135, (byte)207, (byte)32, (byte)93, (byte)65, (byte)36, (byte)208, (byte)36, (byte)246, (byte)102, (byte)147, (byte)76, (byte)121, (byte)31, (byte)220, (byte)49, (byte)120, (byte)92, (byte)132, (byte)181, (byte)143, (byte)119, (byte)214, (byte)150, (byte)219, (byte)164, (byte)193, (byte)151, (byte)108, (byte)135, (byte)6, (byte)249, (byte)194, (byte)162, (byte)234, (byte)26, (byte)12, (byte)209, (byte)210, (byte)7, (byte)57, (byte)152, (byte)118, (byte)112, (byte)3, (byte)215, (byte)159, (byte)124, (byte)179, (byte)143, (byte)218, (byte)233, (byte)93, (byte)213, (byte)228, (byte)85, (byte)11, (byte)99, (byte)40, (byte)6, (byte)140, (byte)140, (byte)37, (byte)188, (byte)98, (byte)157, (byte)106, (byte)248, (byte)233, (byte)90, (byte)73, (byte)112, (byte)193, (byte)29, (byte)96, (byte)66, (byte)146, (byte)159, (byte)100, (byte)127, (byte)59, (byte)230, (byte)211, (byte)137, (byte)131, (byte)80, (byte)221, (byte)4, (byte)235, (byte)140, (byte)138, (byte)173, (byte)31, (byte)217, (byte)115, (byte)79, (byte)42, (byte)232, (byte)40, (byte)15, (byte)4, (byte)64, (byte)103, (byte)93, (byte)89, (byte)132, (byte)186, (byte)112, (byte)40, (byte)70, (byte)199, (byte)189, (byte)23, (byte)44, (byte)131, (byte)102, (byte)133, (byte)77, (byte)122, (byte)195, (byte)65, (byte)23, (byte)148, (byte)44, (byte)85, (byte)226, (byte)0, (byte)160, (byte)212, (byte)127, (byte)93, (byte)22, (byte)72, (byte)13, (byte)144, (byte)185, (byte)65, (byte)51, (byte)82, (byte)209, (byte)210, (byte)193, (byte)116, (byte)147, (byte)205, (byte)205, (byte)166, (byte)72, (byte)132, (byte)57, (byte)230, (byte)243, (byte)214, (byte)198, (byte)0, (byte)227, (byte)158, (byte)43, (byte)188, (byte)68, (byte)33, (byte)47, (byte)158, (byte)157, (byte)125, (byte)247, (byte)168, (byte)251, (byte)140, (byte)16, (byte)244, (byte)228, (byte)120}, 0) ;
            p266.length = (byte)(byte)59;
            p266.target_system = (byte)(byte)228;
            LoopBackDemoChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.length == (byte)(byte)64);
                Debug.Assert(pack.sequence == (ushort)(ushort)61026);
                Debug.Assert(pack.first_message_offset == (byte)(byte)131);
                Debug.Assert(pack.target_system == (byte)(byte)147);
                Debug.Assert(pack.target_component == (byte)(byte)185);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)121, (byte)215, (byte)29, (byte)125, (byte)181, (byte)245, (byte)76, (byte)79, (byte)215, (byte)71, (byte)252, (byte)225, (byte)176, (byte)20, (byte)119, (byte)248, (byte)86, (byte)153, (byte)191, (byte)165, (byte)213, (byte)190, (byte)224, (byte)164, (byte)8, (byte)203, (byte)146, (byte)216, (byte)224, (byte)104, (byte)104, (byte)10, (byte)206, (byte)160, (byte)94, (byte)61, (byte)235, (byte)35, (byte)35, (byte)37, (byte)229, (byte)253, (byte)94, (byte)76, (byte)64, (byte)143, (byte)117, (byte)213, (byte)42, (byte)59, (byte)43, (byte)23, (byte)67, (byte)23, (byte)49, (byte)139, (byte)185, (byte)44, (byte)224, (byte)153, (byte)39, (byte)62, (byte)173, (byte)38, (byte)95, (byte)4, (byte)52, (byte)242, (byte)223, (byte)127, (byte)11, (byte)203, (byte)126, (byte)62, (byte)3, (byte)207, (byte)184, (byte)75, (byte)212, (byte)28, (byte)180, (byte)212, (byte)16, (byte)185, (byte)248, (byte)207, (byte)254, (byte)22, (byte)118, (byte)254, (byte)78, (byte)87, (byte)254, (byte)88, (byte)36, (byte)124, (byte)184, (byte)68, (byte)35, (byte)220, (byte)32, (byte)99, (byte)20, (byte)124, (byte)183, (byte)176, (byte)251, (byte)104, (byte)239, (byte)215, (byte)155, (byte)201, (byte)31, (byte)197, (byte)226, (byte)6, (byte)145, (byte)90, (byte)13, (byte)116, (byte)100, (byte)152, (byte)159, (byte)3, (byte)56, (byte)94, (byte)219, (byte)230, (byte)212, (byte)3, (byte)79, (byte)192, (byte)180, (byte)82, (byte)210, (byte)115, (byte)75, (byte)41, (byte)5, (byte)161, (byte)207, (byte)66, (byte)153, (byte)159, (byte)179, (byte)185, (byte)59, (byte)148, (byte)104, (byte)73, (byte)237, (byte)230, (byte)151, (byte)159, (byte)194, (byte)141, (byte)149, (byte)15, (byte)50, (byte)64, (byte)69, (byte)195, (byte)162, (byte)99, (byte)26, (byte)139, (byte)190, (byte)186, (byte)40, (byte)165, (byte)147, (byte)17, (byte)219, (byte)114, (byte)107, (byte)210, (byte)23, (byte)181, (byte)119, (byte)102, (byte)53, (byte)168, (byte)49, (byte)183, (byte)131, (byte)117, (byte)20, (byte)23, (byte)52, (byte)209, (byte)76, (byte)175, (byte)170, (byte)33, (byte)128, (byte)190, (byte)172, (byte)49, (byte)70, (byte)151, (byte)1, (byte)214, (byte)33, (byte)58, (byte)35, (byte)110, (byte)150, (byte)76, (byte)89, (byte)121, (byte)173, (byte)22, (byte)118, (byte)26, (byte)215, (byte)224, (byte)188, (byte)146, (byte)66, (byte)170, (byte)40, (byte)106, (byte)162, (byte)20, (byte)221, (byte)197, (byte)212, (byte)177, (byte)181, (byte)176, (byte)247, (byte)139, (byte)155, (byte)241, (byte)32, (byte)19, (byte)66, (byte)203, (byte)217, (byte)34, (byte)226, (byte)104, (byte)43, (byte)181, (byte)26, (byte)172, (byte)29, (byte)159, (byte)90}));
            };
            DemoDevice.LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.data__SET(new byte[] {(byte)121, (byte)215, (byte)29, (byte)125, (byte)181, (byte)245, (byte)76, (byte)79, (byte)215, (byte)71, (byte)252, (byte)225, (byte)176, (byte)20, (byte)119, (byte)248, (byte)86, (byte)153, (byte)191, (byte)165, (byte)213, (byte)190, (byte)224, (byte)164, (byte)8, (byte)203, (byte)146, (byte)216, (byte)224, (byte)104, (byte)104, (byte)10, (byte)206, (byte)160, (byte)94, (byte)61, (byte)235, (byte)35, (byte)35, (byte)37, (byte)229, (byte)253, (byte)94, (byte)76, (byte)64, (byte)143, (byte)117, (byte)213, (byte)42, (byte)59, (byte)43, (byte)23, (byte)67, (byte)23, (byte)49, (byte)139, (byte)185, (byte)44, (byte)224, (byte)153, (byte)39, (byte)62, (byte)173, (byte)38, (byte)95, (byte)4, (byte)52, (byte)242, (byte)223, (byte)127, (byte)11, (byte)203, (byte)126, (byte)62, (byte)3, (byte)207, (byte)184, (byte)75, (byte)212, (byte)28, (byte)180, (byte)212, (byte)16, (byte)185, (byte)248, (byte)207, (byte)254, (byte)22, (byte)118, (byte)254, (byte)78, (byte)87, (byte)254, (byte)88, (byte)36, (byte)124, (byte)184, (byte)68, (byte)35, (byte)220, (byte)32, (byte)99, (byte)20, (byte)124, (byte)183, (byte)176, (byte)251, (byte)104, (byte)239, (byte)215, (byte)155, (byte)201, (byte)31, (byte)197, (byte)226, (byte)6, (byte)145, (byte)90, (byte)13, (byte)116, (byte)100, (byte)152, (byte)159, (byte)3, (byte)56, (byte)94, (byte)219, (byte)230, (byte)212, (byte)3, (byte)79, (byte)192, (byte)180, (byte)82, (byte)210, (byte)115, (byte)75, (byte)41, (byte)5, (byte)161, (byte)207, (byte)66, (byte)153, (byte)159, (byte)179, (byte)185, (byte)59, (byte)148, (byte)104, (byte)73, (byte)237, (byte)230, (byte)151, (byte)159, (byte)194, (byte)141, (byte)149, (byte)15, (byte)50, (byte)64, (byte)69, (byte)195, (byte)162, (byte)99, (byte)26, (byte)139, (byte)190, (byte)186, (byte)40, (byte)165, (byte)147, (byte)17, (byte)219, (byte)114, (byte)107, (byte)210, (byte)23, (byte)181, (byte)119, (byte)102, (byte)53, (byte)168, (byte)49, (byte)183, (byte)131, (byte)117, (byte)20, (byte)23, (byte)52, (byte)209, (byte)76, (byte)175, (byte)170, (byte)33, (byte)128, (byte)190, (byte)172, (byte)49, (byte)70, (byte)151, (byte)1, (byte)214, (byte)33, (byte)58, (byte)35, (byte)110, (byte)150, (byte)76, (byte)89, (byte)121, (byte)173, (byte)22, (byte)118, (byte)26, (byte)215, (byte)224, (byte)188, (byte)146, (byte)66, (byte)170, (byte)40, (byte)106, (byte)162, (byte)20, (byte)221, (byte)197, (byte)212, (byte)177, (byte)181, (byte)176, (byte)247, (byte)139, (byte)155, (byte)241, (byte)32, (byte)19, (byte)66, (byte)203, (byte)217, (byte)34, (byte)226, (byte)104, (byte)43, (byte)181, (byte)26, (byte)172, (byte)29, (byte)159, (byte)90}, 0) ;
            p267.length = (byte)(byte)64;
            p267.target_system = (byte)(byte)147;
            p267.sequence = (ushort)(ushort)61026;
            p267.first_message_offset = (byte)(byte)131;
            p267.target_component = (byte)(byte)185;
            LoopBackDemoChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)195);
                Debug.Assert(pack.target_component == (byte)(byte)70);
                Debug.Assert(pack.sequence == (ushort)(ushort)44338);
            };
            DemoDevice.LOGGING_ACK p268 = LoopBackDemoChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)195;
            p268.target_component = (byte)(byte)70;
            p268.sequence = (ushort)(ushort)44338;
            LoopBackDemoChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.framerate == (float)2.7483633E37F);
                Debug.Assert(pack.rotation == (ushort)(ushort)29651);
                Debug.Assert(pack.camera_id == (byte)(byte)12);
                Debug.Assert(pack.uri_LEN(ph) == 23);
                Debug.Assert(pack.uri_TRY(ph).Equals("rsrpiomoahkphnzltfsfkrt"));
                Debug.Assert(pack.resolution_v == (ushort)(ushort)36917);
                Debug.Assert(pack.bitrate == (uint)2043429609U);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)62603);
                Debug.Assert(pack.status == (byte)(byte)251);
            };
            DemoDevice.VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.rotation = (ushort)(ushort)29651;
            p269.status = (byte)(byte)251;
            p269.resolution_v = (ushort)(ushort)36917;
            p269.bitrate = (uint)2043429609U;
            p269.resolution_h = (ushort)(ushort)62603;
            p269.framerate = (float)2.7483633E37F;
            p269.camera_id = (byte)(byte)12;
            p269.uri_SET("rsrpiomoahkphnzltfsfkrt", PH) ;
            LoopBackDemoChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.resolution_v == (ushort)(ushort)43932);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)9649);
                Debug.Assert(pack.target_component == (byte)(byte)215);
                Debug.Assert(pack.target_system == (byte)(byte)85);
                Debug.Assert(pack.rotation == (ushort)(ushort)32035);
                Debug.Assert(pack.camera_id == (byte)(byte)254);
                Debug.Assert(pack.uri_LEN(ph) == 175);
                Debug.Assert(pack.uri_TRY(ph).Equals("wdywkbhajahfntknmbvzwtydWbsrQdmekkvhdnjvjruppiaxeaaprctlJsyzplaxllssujRjzrpAqdzoNqgfEpcmTxkwqbrmhQwktachdtfedsqujpfqvxaxwabikllTbmjtxvzfzajfbpnrbqkuaMUFleWlTqmskmtxrkghmogcixd"));
                Debug.Assert(pack.bitrate == (uint)4135226549U);
                Debug.Assert(pack.framerate == (float) -3.3822967E38F);
            };
            DemoDevice.SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.resolution_h = (ushort)(ushort)9649;
            p270.framerate = (float) -3.3822967E38F;
            p270.resolution_v = (ushort)(ushort)43932;
            p270.target_component = (byte)(byte)215;
            p270.bitrate = (uint)4135226549U;
            p270.uri_SET("wdywkbhajahfntknmbvzwtydWbsrQdmekkvhdnjvjruppiaxeaaprctlJsyzplaxllssujRjzrpAqdzoNqgfEpcmTxkwqbrmhQwktachdtfedsqujpfqvxaxwabikllTbmjtxvzfzajfbpnrbqkuaMUFleWlTqmskmtxrkghmogcixd", PH) ;
            p270.rotation = (ushort)(ushort)32035;
            p270.target_system = (byte)(byte)85;
            p270.camera_id = (byte)(byte)254;
            LoopBackDemoChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.password_LEN(ph) == 31);
                Debug.Assert(pack.password_TRY(ph).Equals("lkoafrfocVqsbvcltubyuyrxXjpgowu"));
                Debug.Assert(pack.ssid_LEN(ph) == 2);
                Debug.Assert(pack.ssid_TRY(ph).Equals("op"));
            };
            DemoDevice.WIFI_CONFIG_AP p299 = LoopBackDemoChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("op", PH) ;
            p299.password_SET("lkoafrfocVqsbvcltubyuyrxXjpgowu", PH) ;
            LoopBackDemoChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.version == (ushort)(ushort)49405);
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)218, (byte)183, (byte)174, (byte)245, (byte)12, (byte)240, (byte)14, (byte)149}));
                Debug.Assert(pack.max_version == (ushort)(ushort)6052);
                Debug.Assert(pack.min_version == (ushort)(ushort)52119);
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)143, (byte)34, (byte)50, (byte)23, (byte)79, (byte)88, (byte)232, (byte)205}));
            };
            DemoDevice.PROTOCOL_VERSION p300 = LoopBackDemoChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.library_version_hash_SET(new byte[] {(byte)218, (byte)183, (byte)174, (byte)245, (byte)12, (byte)240, (byte)14, (byte)149}, 0) ;
            p300.max_version = (ushort)(ushort)6052;
            p300.version = (ushort)(ushort)49405;
            p300.min_version = (ushort)(ushort)52119;
            p300.spec_version_hash_SET(new byte[] {(byte)143, (byte)34, (byte)50, (byte)23, (byte)79, (byte)88, (byte)232, (byte)205}, 0) ;
            LoopBackDemoChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.health == (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
                Debug.Assert(pack.time_usec == (ulong)1224472523756836682L);
                Debug.Assert(pack.uptime_sec == (uint)2109376553U);
                Debug.Assert(pack.sub_mode == (byte)(byte)168);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)53976);
                Debug.Assert(pack.mode == (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION);
            };
            DemoDevice.UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR;
            p310.sub_mode = (byte)(byte)168;
            p310.vendor_specific_status_code = (ushort)(ushort)53976;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION;
            p310.uptime_sec = (uint)2109376553U;
            p310.time_usec = (ulong)1224472523756836682L;
            LoopBackDemoChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sw_version_minor == (byte)(byte)229);
                Debug.Assert(pack.hw_version_minor == (byte)(byte)57);
                Debug.Assert(pack.name_LEN(ph) == 6);
                Debug.Assert(pack.name_TRY(ph).Equals("mQczag"));
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)55, (byte)162, (byte)110, (byte)68, (byte)113, (byte)167, (byte)80, (byte)133, (byte)123, (byte)40, (byte)242, (byte)1, (byte)130, (byte)184, (byte)128, (byte)149}));
                Debug.Assert(pack.sw_version_major == (byte)(byte)193);
                Debug.Assert(pack.uptime_sec == (uint)397343693U);
                Debug.Assert(pack.hw_version_major == (byte)(byte)1);
                Debug.Assert(pack.sw_vcs_commit == (uint)383941205U);
                Debug.Assert(pack.time_usec == (ulong)5736975327848337972L);
            };
            DemoDevice.UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.hw_version_minor = (byte)(byte)57;
            p311.time_usec = (ulong)5736975327848337972L;
            p311.hw_unique_id_SET(new byte[] {(byte)55, (byte)162, (byte)110, (byte)68, (byte)113, (byte)167, (byte)80, (byte)133, (byte)123, (byte)40, (byte)242, (byte)1, (byte)130, (byte)184, (byte)128, (byte)149}, 0) ;
            p311.sw_version_minor = (byte)(byte)229;
            p311.sw_vcs_commit = (uint)383941205U;
            p311.sw_version_major = (byte)(byte)193;
            p311.uptime_sec = (uint)397343693U;
            p311.hw_version_major = (byte)(byte)1;
            p311.name_SET("mQczag", PH) ;
            LoopBackDemoChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 3);
                Debug.Assert(pack.param_id_TRY(ph).Equals("dji"));
                Debug.Assert(pack.target_system == (byte)(byte)99);
                Debug.Assert(pack.param_index == (short)(short)24865);
                Debug.Assert(pack.target_component == (byte)(byte)88);
            };
            DemoDevice.PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.param_index = (short)(short)24865;
            p320.target_system = (byte)(byte)99;
            p320.target_component = (byte)(byte)88;
            p320.param_id_SET("dji", PH) ;
            LoopBackDemoChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)16);
                Debug.Assert(pack.target_component == (byte)(byte)191);
            };
            DemoDevice.PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)16;
            p321.target_component = (byte)(byte)191;
            LoopBackDemoChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_count == (ushort)(ushort)40172);
                Debug.Assert(pack.param_index == (ushort)(ushort)31710);
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
                Debug.Assert(pack.param_id_LEN(ph) == 8);
                Debug.Assert(pack.param_id_TRY(ph).Equals("czcZoncm"));
                Debug.Assert(pack.param_value_LEN(ph) == 93);
                Debug.Assert(pack.param_value_TRY(ph).Equals("lswmSdmckuuvuXhevjihpqvcriemoaxybqhkgfbwxlyrwxlayyhkNraygucyrkdszmcXazssqrvmgbdkebhjjanOytosi"));
            };
            DemoDevice.PARAM_EXT_VALUE p322 = LoopBackDemoChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_index = (ushort)(ushort)31710;
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8;
            p322.param_id_SET("czcZoncm", PH) ;
            p322.param_count = (ushort)(ushort)40172;
            p322.param_value_SET("lswmSdmckuuvuXhevjihpqvcriemoaxybqhkgfbwxlyrwxlayyhkNraygucyrkdszmcXazssqrvmgbdkebhjjanOytosi", PH) ;
            LoopBackDemoChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)154);
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("Vqgmc"));
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
                Debug.Assert(pack.target_component == (byte)(byte)147);
                Debug.Assert(pack.param_value_LEN(ph) == 18);
                Debug.Assert(pack.param_value_TRY(ph).Equals("ykiaorvmgmmseqGfdo"));
            };
            DemoDevice.PARAM_EXT_SET p323 = LoopBackDemoChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.param_id_SET("Vqgmc", PH) ;
            p323.target_system = (byte)(byte)154;
            p323.target_component = (byte)(byte)147;
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64;
            p323.param_value_SET("ykiaorvmgmmseqGfdo", PH) ;
            LoopBackDemoChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_LEN(ph) == 52);
                Debug.Assert(pack.param_value_TRY(ph).Equals("kehmsrlnfafatbJplqpdtryJmqqfxwdpxauanUpupisPdvPvomUq"));
                Debug.Assert(pack.param_result == (PARAM_ACK)PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED);
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8);
                Debug.Assert(pack.param_id_LEN(ph) == 3);
                Debug.Assert(pack.param_id_TRY(ph).Equals("vsp"));
            };
            DemoDevice.PARAM_EXT_ACK p324 = LoopBackDemoChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED;
            p324.param_id_SET("vsp", PH) ;
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8;
            p324.param_value_SET("kehmsrlnfafatbJplqpdtryJmqqfxwdpxauanUpupisPdvPvomUq", PH) ;
            LoopBackDemoChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)2685693042165528861L);
                Debug.Assert(pack.max_distance == (ushort)(ushort)46974);
                Debug.Assert(pack.min_distance == (ushort)(ushort)49387);
                Debug.Assert(pack.increment == (byte)(byte)188);
                Debug.Assert(pack.sensor_type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)54395, (ushort)2806, (ushort)681, (ushort)44856, (ushort)35280, (ushort)60645, (ushort)37725, (ushort)56005, (ushort)15035, (ushort)3653, (ushort)15862, (ushort)44361, (ushort)44790, (ushort)52099, (ushort)23666, (ushort)53573, (ushort)2325, (ushort)26686, (ushort)24402, (ushort)55653, (ushort)16884, (ushort)27156, (ushort)52829, (ushort)33682, (ushort)42776, (ushort)25758, (ushort)779, (ushort)40622, (ushort)52978, (ushort)27304, (ushort)11125, (ushort)36706, (ushort)26204, (ushort)63995, (ushort)47050, (ushort)2733, (ushort)62964, (ushort)47097, (ushort)24594, (ushort)27664, (ushort)18756, (ushort)26417, (ushort)7645, (ushort)17830, (ushort)19384, (ushort)34716, (ushort)28962, (ushort)41574, (ushort)13366, (ushort)14529, (ushort)50701, (ushort)43994, (ushort)19807, (ushort)32525, (ushort)30880, (ushort)62766, (ushort)21586, (ushort)35483, (ushort)1579, (ushort)64185, (ushort)52196, (ushort)62902, (ushort)55109, (ushort)26412, (ushort)21860, (ushort)32135, (ushort)9082, (ushort)11458, (ushort)362, (ushort)49472, (ushort)40143, (ushort)18061}));
            };
            DemoDevice.OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.max_distance = (ushort)(ushort)46974;
            p330.min_distance = (ushort)(ushort)49387;
            p330.time_usec = (ulong)2685693042165528861L;
            p330.distances_SET(new ushort[] {(ushort)54395, (ushort)2806, (ushort)681, (ushort)44856, (ushort)35280, (ushort)60645, (ushort)37725, (ushort)56005, (ushort)15035, (ushort)3653, (ushort)15862, (ushort)44361, (ushort)44790, (ushort)52099, (ushort)23666, (ushort)53573, (ushort)2325, (ushort)26686, (ushort)24402, (ushort)55653, (ushort)16884, (ushort)27156, (ushort)52829, (ushort)33682, (ushort)42776, (ushort)25758, (ushort)779, (ushort)40622, (ushort)52978, (ushort)27304, (ushort)11125, (ushort)36706, (ushort)26204, (ushort)63995, (ushort)47050, (ushort)2733, (ushort)62964, (ushort)47097, (ushort)24594, (ushort)27664, (ushort)18756, (ushort)26417, (ushort)7645, (ushort)17830, (ushort)19384, (ushort)34716, (ushort)28962, (ushort)41574, (ushort)13366, (ushort)14529, (ushort)50701, (ushort)43994, (ushort)19807, (ushort)32525, (ushort)30880, (ushort)62766, (ushort)21586, (ushort)35483, (ushort)1579, (ushort)64185, (ushort)52196, (ushort)62902, (ushort)55109, (ushort)26412, (ushort)21860, (ushort)32135, (ushort)9082, (ushort)11458, (ushort)362, (ushort)49472, (ushort)40143, (ushort)18061}, 0) ;
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER;
            p330.increment = (byte)(byte)188;
            LoopBackDemoChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
        }
    }
}