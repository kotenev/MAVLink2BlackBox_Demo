
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
                Debug.Assert(pack.custom_mode == (uint)570090059U);
                Debug.Assert(pack.type == (MAV_TYPE)MAV_TYPE.MAV_TYPE_SURFACE_BOAT);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED);
                Debug.Assert(pack.system_status == (MAV_STATE)MAV_STATE.MAV_STATE_EMERGENCY);
                Debug.Assert(pack.mavlink_version == (byte)(byte)250);
                Debug.Assert(pack.autopilot == (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_PPZ);
            };
            DemoDevice.HEARTBEAT p0 = LoopBackDemoChannel.new_HEARTBEAT();
            PH.setPack(p0);
            p0.mavlink_version = (byte)(byte)250;
            p0.system_status = (MAV_STATE)MAV_STATE.MAV_STATE_EMERGENCY;
            p0.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED;
            p0.custom_mode = (uint)570090059U;
            p0.autopilot = (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_PPZ;
            p0.type = (MAV_TYPE)MAV_TYPE.MAV_TYPE_SURFACE_BOAT;
            LoopBackDemoChannel.instance.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.load == (ushort)(ushort)56442);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)73);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)14921);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)29991);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)24672);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)18558);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)13301);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)61383);
                Debug.Assert(pack.current_battery == (short)(short)7505);
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)4723);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH);
            };
            DemoDevice.SYS_STATUS p1 = LoopBackDemoChannel.new_SYS_STATUS();
            PH.setPack(p1);
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG;
            p1.current_battery = (short)(short)7505;
            p1.errors_count1 = (ushort)(ushort)29991;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION;
            p1.drop_rate_comm = (ushort)(ushort)4723;
            p1.load = (ushort)(ushort)56442;
            p1.errors_count2 = (ushort)(ushort)24672;
            p1.errors_comm = (ushort)(ushort)13301;
            p1.errors_count4 = (ushort)(ushort)14921;
            p1.battery_remaining = (sbyte)(sbyte)73;
            p1.errors_count3 = (ushort)(ushort)18558;
            p1.voltage_battery = (ushort)(ushort)61383;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH;
            LoopBackDemoChannel.instance.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_unix_usec == (ulong)8304381036868474028L);
                Debug.Assert(pack.time_boot_ms == (uint)1152046604U);
            };
            DemoDevice.SYSTEM_TIME p2 = LoopBackDemoChannel.new_SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)8304381036868474028L;
            p2.time_boot_ms = (uint)1152046604U;
            LoopBackDemoChannel.instance.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.type_mask == (ushort)(ushort)7789);
                Debug.Assert(pack.time_boot_ms == (uint)312985068U);
                Debug.Assert(pack.yaw == (float) -2.1342374E38F);
                Debug.Assert(pack.vy == (float)3.3379806E38F);
                Debug.Assert(pack.yaw_rate == (float) -1.5903244E38F);
                Debug.Assert(pack.afz == (float) -1.9148694E38F);
                Debug.Assert(pack.afy == (float) -2.4546936E38F);
                Debug.Assert(pack.y == (float) -2.706269E38F);
                Debug.Assert(pack.x == (float)9.445843E37F);
                Debug.Assert(pack.afx == (float)6.0324014E37F);
                Debug.Assert(pack.vx == (float)1.237441E38F);
                Debug.Assert(pack.z == (float) -3.1017218E38F);
                Debug.Assert(pack.vz == (float)2.9990577E38F);
            };
            DemoDevice.POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.vx = (float)1.237441E38F;
            p3.type_mask = (ushort)(ushort)7789;
            p3.afx = (float)6.0324014E37F;
            p3.afz = (float) -1.9148694E38F;
            p3.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p3.x = (float)9.445843E37F;
            p3.vz = (float)2.9990577E38F;
            p3.yaw_rate = (float) -1.5903244E38F;
            p3.time_boot_ms = (uint)312985068U;
            p3.y = (float) -2.706269E38F;
            p3.vy = (float)3.3379806E38F;
            p3.yaw = (float) -2.1342374E38F;
            p3.z = (float) -3.1017218E38F;
            p3.afy = (float) -2.4546936E38F;
            LoopBackDemoChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (uint)3482970076U);
                Debug.Assert(pack.time_usec == (ulong)2420963743217187929L);
                Debug.Assert(pack.target_system == (byte)(byte)102);
                Debug.Assert(pack.target_component == (byte)(byte)192);
            };
            DemoDevice.PING p4 = LoopBackDemoChannel.new_PING();
            PH.setPack(p4);
            p4.target_component = (byte)(byte)192;
            p4.target_system = (byte)(byte)102;
            p4.time_usec = (ulong)2420963743217187929L;
            p4.seq = (uint)3482970076U;
            LoopBackDemoChannel.instance.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.version == (byte)(byte)91);
                Debug.Assert(pack.passkey_LEN(ph) == 8);
                Debug.Assert(pack.passkey_TRY(ph).Equals("mjhlnJnm"));
                Debug.Assert(pack.control_request == (byte)(byte)105);
                Debug.Assert(pack.target_system == (byte)(byte)171);
            };
            DemoDevice.CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.version = (byte)(byte)91;
            p5.control_request = (byte)(byte)105;
            p5.passkey_SET("mjhlnJnm", PH) ;
            p5.target_system = (byte)(byte)171;
            LoopBackDemoChannel.instance.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gcs_system_id == (byte)(byte)155);
                Debug.Assert(pack.control_request == (byte)(byte)190);
                Debug.Assert(pack.ack == (byte)(byte)159);
            };
            DemoDevice.CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.gcs_system_id = (byte)(byte)155;
            p6.ack = (byte)(byte)159;
            p6.control_request = (byte)(byte)190;
            LoopBackDemoChannel.instance.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 25);
                Debug.Assert(pack.key_TRY(ph).Equals("scfZmfcvzbycbjnrdedpygzqm"));
            };
            DemoDevice.AUTH_KEY p7 = LoopBackDemoChannel.new_AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("scfZmfcvzbycbjnrdedpygzqm", PH) ;
            LoopBackDemoChannel.instance.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)134);
                Debug.Assert(pack.base_mode == (MAV_MODE)MAV_MODE.MAV_MODE_AUTO_DISARMED);
                Debug.Assert(pack.custom_mode == (uint)1489603235U);
            };
            DemoDevice.SET_MODE p11 = LoopBackDemoChannel.new_SET_MODE();
            PH.setPack(p11);
            p11.base_mode = (MAV_MODE)MAV_MODE.MAV_MODE_AUTO_DISARMED;
            p11.target_system = (byte)(byte)134;
            p11.custom_mode = (uint)1489603235U;
            LoopBackDemoChannel.instance.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)75);
                Debug.Assert(pack.param_index == (short)(short)17757);
                Debug.Assert(pack.param_id_LEN(ph) == 11);
                Debug.Assert(pack.param_id_TRY(ph).Equals("wvwTddgyshw"));
                Debug.Assert(pack.target_system == (byte)(byte)38);
            };
            DemoDevice.PARAM_REQUEST_READ p20 = LoopBackDemoChannel.new_PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.param_id_SET("wvwTddgyshw", PH) ;
            p20.param_index = (short)(short)17757;
            p20.target_system = (byte)(byte)38;
            p20.target_component = (byte)(byte)75;
            LoopBackDemoChannel.instance.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)249);
                Debug.Assert(pack.target_component == (byte)(byte)211);
            };
            DemoDevice.PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.new_PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_component = (byte)(byte)211;
            p21.target_system = (byte)(byte)249;
            LoopBackDemoChannel.instance.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (ushort)(ushort)47133);
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64);
                Debug.Assert(pack.param_value == (float) -1.1115167E38F);
                Debug.Assert(pack.param_id_LEN(ph) == 14);
                Debug.Assert(pack.param_id_TRY(ph).Equals("jbmvgactfphYDh"));
                Debug.Assert(pack.param_count == (ushort)(ushort)32545);
            };
            DemoDevice.PARAM_VALUE p22 = LoopBackDemoChannel.new_PARAM_VALUE();
            PH.setPack(p22);
            p22.param_id_SET("jbmvgactfphYDh", PH) ;
            p22.param_count = (ushort)(ushort)32545;
            p22.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64;
            p22.param_value = (float) -1.1115167E38F;
            p22.param_index = (ushort)(ushort)47133;
            LoopBackDemoChannel.instance.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)87);
                Debug.Assert(pack.param_id_LEN(ph) == 13);
                Debug.Assert(pack.param_id_TRY(ph).Equals("yjcfislpcvnkZ"));
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32);
                Debug.Assert(pack.param_value == (float) -2.3856448E38F);
                Debug.Assert(pack.target_system == (byte)(byte)203);
            };
            DemoDevice.PARAM_SET p23 = LoopBackDemoChannel.new_PARAM_SET();
            PH.setPack(p23);
            p23.param_id_SET("yjcfislpcvnkZ", PH) ;
            p23.target_system = (byte)(byte)203;
            p23.target_component = (byte)(byte)87;
            p23.param_value = (float) -2.3856448E38F;
            p23.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32;
            LoopBackDemoChannel.instance.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vel == (ushort)(ushort)52806);
                Debug.Assert(pack.alt == (int)1068028482);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)3228578461U);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int)1316078032);
                Debug.Assert(pack.cog == (ushort)(ushort)60716);
                Debug.Assert(pack.eph == (ushort)(ushort)60619);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)6633008U);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)3304969945U);
                Debug.Assert(pack.time_usec == (ulong)4739703684460820371L);
                Debug.Assert(pack.lat == (int) -1502618558);
                Debug.Assert(pack.satellites_visible == (byte)(byte)76);
                Debug.Assert(pack.lon == (int)699423000);
                Debug.Assert(pack.epv == (ushort)(ushort)19350);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)2950245508U);
            };
            DemoDevice.GPS_RAW_INT p24 = LoopBackDemoChannel.new_GPS_RAW_INT();
            PH.setPack(p24);
            p24.vel = (ushort)(ushort)52806;
            p24.epv = (ushort)(ushort)19350;
            p24.lon = (int)699423000;
            p24.satellites_visible = (byte)(byte)76;
            p24.h_acc_SET((uint)3228578461U, PH) ;
            p24.vel_acc_SET((uint)3304969945U, PH) ;
            p24.hdg_acc_SET((uint)2950245508U, PH) ;
            p24.lat = (int) -1502618558;
            p24.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_PPP;
            p24.eph = (ushort)(ushort)60619;
            p24.time_usec = (ulong)4739703684460820371L;
            p24.v_acc_SET((uint)6633008U, PH) ;
            p24.alt_ellipsoid_SET((int)1316078032, PH) ;
            p24.alt = (int)1068028482;
            p24.cog = (ushort)(ushort)60716;
            LoopBackDemoChannel.instance.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellites_visible == (byte)(byte)113);
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)238, (byte)104, (byte)63, (byte)135, (byte)36, (byte)72, (byte)66, (byte)170, (byte)221, (byte)142, (byte)2, (byte)73, (byte)5, (byte)198, (byte)222, (byte)154, (byte)239, (byte)73, (byte)18, (byte)166}));
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)248, (byte)229, (byte)236, (byte)254, (byte)178, (byte)30, (byte)247, (byte)248, (byte)148, (byte)99, (byte)122, (byte)107, (byte)49, (byte)227, (byte)79, (byte)38, (byte)12, (byte)10, (byte)185, (byte)2}));
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)108, (byte)190, (byte)137, (byte)40, (byte)27, (byte)1, (byte)58, (byte)88, (byte)237, (byte)195, (byte)156, (byte)127, (byte)124, (byte)89, (byte)155, (byte)246, (byte)187, (byte)219, (byte)90, (byte)121}));
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)39, (byte)48, (byte)35, (byte)103, (byte)129, (byte)229, (byte)255, (byte)228, (byte)105, (byte)49, (byte)94, (byte)202, (byte)2, (byte)199, (byte)160, (byte)74, (byte)169, (byte)12, (byte)61, (byte)53}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)164, (byte)49, (byte)5, (byte)218, (byte)65, (byte)131, (byte)88, (byte)236, (byte)212, (byte)212, (byte)250, (byte)48, (byte)78, (byte)128, (byte)163, (byte)74, (byte)153, (byte)220, (byte)109, (byte)1}));
            };
            DemoDevice.GPS_STATUS p25 = LoopBackDemoChannel.new_GPS_STATUS();
            PH.setPack(p25);
            p25.satellites_visible = (byte)(byte)113;
            p25.satellite_azimuth_SET(new byte[] {(byte)164, (byte)49, (byte)5, (byte)218, (byte)65, (byte)131, (byte)88, (byte)236, (byte)212, (byte)212, (byte)250, (byte)48, (byte)78, (byte)128, (byte)163, (byte)74, (byte)153, (byte)220, (byte)109, (byte)1}, 0) ;
            p25.satellite_used_SET(new byte[] {(byte)39, (byte)48, (byte)35, (byte)103, (byte)129, (byte)229, (byte)255, (byte)228, (byte)105, (byte)49, (byte)94, (byte)202, (byte)2, (byte)199, (byte)160, (byte)74, (byte)169, (byte)12, (byte)61, (byte)53}, 0) ;
            p25.satellite_prn_SET(new byte[] {(byte)108, (byte)190, (byte)137, (byte)40, (byte)27, (byte)1, (byte)58, (byte)88, (byte)237, (byte)195, (byte)156, (byte)127, (byte)124, (byte)89, (byte)155, (byte)246, (byte)187, (byte)219, (byte)90, (byte)121}, 0) ;
            p25.satellite_snr_SET(new byte[] {(byte)248, (byte)229, (byte)236, (byte)254, (byte)178, (byte)30, (byte)247, (byte)248, (byte)148, (byte)99, (byte)122, (byte)107, (byte)49, (byte)227, (byte)79, (byte)38, (byte)12, (byte)10, (byte)185, (byte)2}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)238, (byte)104, (byte)63, (byte)135, (byte)36, (byte)72, (byte)66, (byte)170, (byte)221, (byte)142, (byte)2, (byte)73, (byte)5, (byte)198, (byte)222, (byte)154, (byte)239, (byte)73, (byte)18, (byte)166}, 0) ;
            LoopBackDemoChannel.instance.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (short)(short)20813);
                Debug.Assert(pack.xmag == (short)(short)5540);
                Debug.Assert(pack.time_boot_ms == (uint)912194457U);
                Debug.Assert(pack.xacc == (short)(short)27437);
                Debug.Assert(pack.ymag == (short)(short)25808);
                Debug.Assert(pack.zgyro == (short)(short) -14258);
                Debug.Assert(pack.xgyro == (short)(short)25459);
                Debug.Assert(pack.ygyro == (short)(short)9070);
                Debug.Assert(pack.zacc == (short)(short)31081);
                Debug.Assert(pack.zmag == (short)(short) -17709);
            };
            DemoDevice.SCALED_IMU p26 = LoopBackDemoChannel.new_SCALED_IMU();
            PH.setPack(p26);
            p26.xacc = (short)(short)27437;
            p26.zmag = (short)(short) -17709;
            p26.xgyro = (short)(short)25459;
            p26.zgyro = (short)(short) -14258;
            p26.yacc = (short)(short)20813;
            p26.time_boot_ms = (uint)912194457U;
            p26.xmag = (short)(short)5540;
            p26.ygyro = (short)(short)9070;
            p26.zacc = (short)(short)31081;
            p26.ymag = (short)(short)25808;
            LoopBackDemoChannel.instance.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zacc == (short)(short)31639);
                Debug.Assert(pack.yacc == (short)(short) -32241);
                Debug.Assert(pack.xacc == (short)(short) -16303);
                Debug.Assert(pack.xmag == (short)(short) -14817);
                Debug.Assert(pack.time_usec == (ulong)7757604591040764900L);
                Debug.Assert(pack.ymag == (short)(short) -16262);
                Debug.Assert(pack.zgyro == (short)(short) -18031);
                Debug.Assert(pack.xgyro == (short)(short) -3966);
                Debug.Assert(pack.ygyro == (short)(short)4715);
                Debug.Assert(pack.zmag == (short)(short) -16175);
            };
            DemoDevice.RAW_IMU p27 = LoopBackDemoChannel.new_RAW_IMU();
            PH.setPack(p27);
            p27.zacc = (short)(short)31639;
            p27.zgyro = (short)(short) -18031;
            p27.ygyro = (short)(short)4715;
            p27.yacc = (short)(short) -32241;
            p27.time_usec = (ulong)7757604591040764900L;
            p27.xmag = (short)(short) -14817;
            p27.xacc = (short)(short) -16303;
            p27.ymag = (short)(short) -16262;
            p27.xgyro = (short)(short) -3966;
            p27.zmag = (short)(short) -16175;
            LoopBackDemoChannel.instance.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -17121);
                Debug.Assert(pack.press_diff2 == (short)(short) -27521);
                Debug.Assert(pack.time_usec == (ulong)2243854205713554697L);
                Debug.Assert(pack.press_diff1 == (short)(short)14344);
                Debug.Assert(pack.press_abs == (short)(short)5384);
            };
            DemoDevice.RAW_PRESSURE p28 = LoopBackDemoChannel.new_RAW_PRESSURE();
            PH.setPack(p28);
            p28.press_diff1 = (short)(short)14344;
            p28.temperature = (short)(short) -17121;
            p28.press_abs = (short)(short)5384;
            p28.press_diff2 = (short)(short) -27521;
            p28.time_usec = (ulong)2243854205713554697L;
            LoopBackDemoChannel.instance.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2633444405U);
                Debug.Assert(pack.press_abs == (float) -2.4055658E38F);
                Debug.Assert(pack.temperature == (short)(short) -9841);
                Debug.Assert(pack.press_diff == (float)3.021106E37F);
            };
            DemoDevice.SCALED_PRESSURE p29 = LoopBackDemoChannel.new_SCALED_PRESSURE();
            PH.setPack(p29);
            p29.press_diff = (float)3.021106E37F;
            p29.time_boot_ms = (uint)2633444405U;
            p29.press_abs = (float) -2.4055658E38F;
            p29.temperature = (short)(short) -9841;
            LoopBackDemoChannel.instance.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitchspeed == (float)1.1910272E38F);
                Debug.Assert(pack.rollspeed == (float)5.0989146E37F);
                Debug.Assert(pack.pitch == (float) -1.781256E38F);
                Debug.Assert(pack.yawspeed == (float)1.9958398E38F);
                Debug.Assert(pack.roll == (float)3.0658229E37F);
                Debug.Assert(pack.time_boot_ms == (uint)276367765U);
                Debug.Assert(pack.yaw == (float)2.094449E38F);
            };
            DemoDevice.ATTITUDE p30 = LoopBackDemoChannel.new_ATTITUDE();
            PH.setPack(p30);
            p30.pitchspeed = (float)1.1910272E38F;
            p30.pitch = (float) -1.781256E38F;
            p30.yaw = (float)2.094449E38F;
            p30.yawspeed = (float)1.9958398E38F;
            p30.roll = (float)3.0658229E37F;
            p30.rollspeed = (float)5.0989146E37F;
            p30.time_boot_ms = (uint)276367765U;
            LoopBackDemoChannel.instance.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q4 == (float)1.6851345E38F);
                Debug.Assert(pack.q1 == (float) -2.7161122E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2715186655U);
                Debug.Assert(pack.yawspeed == (float)6.4473075E37F);
                Debug.Assert(pack.rollspeed == (float)3.2283246E38F);
                Debug.Assert(pack.pitchspeed == (float)3.3561374E38F);
                Debug.Assert(pack.q3 == (float)3.205976E38F);
                Debug.Assert(pack.q2 == (float) -9.808246E37F);
            };
            DemoDevice.ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.yawspeed = (float)6.4473075E37F;
            p31.q4 = (float)1.6851345E38F;
            p31.pitchspeed = (float)3.3561374E38F;
            p31.q1 = (float) -2.7161122E38F;
            p31.time_boot_ms = (uint)2715186655U;
            p31.rollspeed = (float)3.2283246E38F;
            p31.q3 = (float)3.205976E38F;
            p31.q2 = (float) -9.808246E37F;
            LoopBackDemoChannel.instance.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)7.05493E36F);
                Debug.Assert(pack.z == (float) -6.436947E36F);
                Debug.Assert(pack.vy == (float) -2.9436597E38F);
                Debug.Assert(pack.vx == (float) -9.319312E36F);
                Debug.Assert(pack.x == (float)2.4778026E36F);
                Debug.Assert(pack.time_boot_ms == (uint)1980635250U);
                Debug.Assert(pack.vz == (float) -2.7742805E38F);
            };
            DemoDevice.LOCAL_POSITION_NED p32 = LoopBackDemoChannel.new_LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.vz = (float) -2.7742805E38F;
            p32.time_boot_ms = (uint)1980635250U;
            p32.y = (float)7.05493E36F;
            p32.x = (float)2.4778026E36F;
            p32.vx = (float) -9.319312E36F;
            p32.vy = (float) -2.9436597E38F;
            p32.z = (float) -6.436947E36F;
            LoopBackDemoChannel.instance.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (short)(short)27293);
                Debug.Assert(pack.lon == (int) -1679813388);
                Debug.Assert(pack.relative_alt == (int) -1963355373);
                Debug.Assert(pack.lat == (int)1900764736);
                Debug.Assert(pack.hdg == (ushort)(ushort)53979);
                Debug.Assert(pack.vz == (short)(short)30315);
                Debug.Assert(pack.alt == (int) -1499432330);
                Debug.Assert(pack.vx == (short)(short)5514);
                Debug.Assert(pack.time_boot_ms == (uint)2165600937U);
            };
            DemoDevice.GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.vz = (short)(short)30315;
            p33.relative_alt = (int) -1963355373;
            p33.time_boot_ms = (uint)2165600937U;
            p33.vx = (short)(short)5514;
            p33.lon = (int) -1679813388;
            p33.hdg = (ushort)(ushort)53979;
            p33.lat = (int)1900764736;
            p33.alt = (int) -1499432330;
            p33.vy = (short)(short)27293;
            LoopBackDemoChannel.instance.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan3_scaled == (short)(short)22436);
                Debug.Assert(pack.chan4_scaled == (short)(short)22708);
                Debug.Assert(pack.chan6_scaled == (short)(short)11304);
                Debug.Assert(pack.chan8_scaled == (short)(short)5608);
                Debug.Assert(pack.chan1_scaled == (short)(short)27764);
                Debug.Assert(pack.chan7_scaled == (short)(short) -1796);
                Debug.Assert(pack.port == (byte)(byte)83);
                Debug.Assert(pack.chan5_scaled == (short)(short) -4266);
                Debug.Assert(pack.rssi == (byte)(byte)83);
                Debug.Assert(pack.time_boot_ms == (uint)1816229546U);
                Debug.Assert(pack.chan2_scaled == (short)(short)24732);
            };
            DemoDevice.RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.new_RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.chan8_scaled = (short)(short)5608;
            p34.chan6_scaled = (short)(short)11304;
            p34.time_boot_ms = (uint)1816229546U;
            p34.chan2_scaled = (short)(short)24732;
            p34.rssi = (byte)(byte)83;
            p34.chan5_scaled = (short)(short) -4266;
            p34.chan3_scaled = (short)(short)22436;
            p34.chan7_scaled = (short)(short) -1796;
            p34.port = (byte)(byte)83;
            p34.chan4_scaled = (short)(short)22708;
            p34.chan1_scaled = (short)(short)27764;
            LoopBackDemoChannel.instance.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)56377);
                Debug.Assert(pack.rssi == (byte)(byte)214);
                Debug.Assert(pack.port == (byte)(byte)241);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)39473);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)45749);
                Debug.Assert(pack.time_boot_ms == (uint)682387637U);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)7503);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)55047);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)23297);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)7645);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)1028);
            };
            DemoDevice.RC_CHANNELS_RAW p35 = LoopBackDemoChannel.new_RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.chan2_raw = (ushort)(ushort)7503;
            p35.chan6_raw = (ushort)(ushort)45749;
            p35.port = (byte)(byte)241;
            p35.chan5_raw = (ushort)(ushort)56377;
            p35.rssi = (byte)(byte)214;
            p35.chan8_raw = (ushort)(ushort)1028;
            p35.chan4_raw = (ushort)(ushort)55047;
            p35.time_boot_ms = (uint)682387637U;
            p35.chan1_raw = (ushort)(ushort)39473;
            p35.chan7_raw = (ushort)(ushort)7645;
            p35.chan3_raw = (ushort)(ushort)23297;
            LoopBackDemoChannel.instance.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)12396);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)860);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)11847);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)12652);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)18049);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)3042);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)42608);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)54037);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)51563);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)65378);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)58769);
                Debug.Assert(pack.time_usec == (uint)3407745657U);
                Debug.Assert(pack.port == (byte)(byte)30);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)41617);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)29353);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)1556);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)34817);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)46597);
            };
            DemoDevice.SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.new_SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo2_raw = (ushort)(ushort)12396;
            p36.servo10_raw_SET((ushort)(ushort)1556, PH) ;
            p36.servo12_raw_SET((ushort)(ushort)46597, PH) ;
            p36.servo8_raw = (ushort)(ushort)51563;
            p36.servo11_raw_SET((ushort)(ushort)860, PH) ;
            p36.servo6_raw = (ushort)(ushort)34817;
            p36.servo13_raw_SET((ushort)(ushort)29353, PH) ;
            p36.servo16_raw_SET((ushort)(ushort)54037, PH) ;
            p36.servo4_raw = (ushort)(ushort)58769;
            p36.servo9_raw_SET((ushort)(ushort)3042, PH) ;
            p36.servo7_raw = (ushort)(ushort)18049;
            p36.time_usec = (uint)3407745657U;
            p36.servo14_raw_SET((ushort)(ushort)12652, PH) ;
            p36.servo5_raw = (ushort)(ushort)42608;
            p36.servo1_raw = (ushort)(ushort)11847;
            p36.servo3_raw = (ushort)(ushort)65378;
            p36.port = (byte)(byte)30;
            p36.servo15_raw_SET((ushort)(ushort)41617, PH) ;
            LoopBackDemoChannel.instance.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_index == (short)(short) -31371);
                Debug.Assert(pack.target_system == (byte)(byte)115);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)192);
                Debug.Assert(pack.end_index == (short)(short)16460);
            };
            DemoDevice.MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.new_MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.target_component = (byte)(byte)192;
            p37.end_index = (short)(short)16460;
            p37.start_index = (short)(short) -31371;
            p37.target_system = (byte)(byte)115;
            p37.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            LoopBackDemoChannel.instance.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.end_index == (short)(short)19615);
                Debug.Assert(pack.start_index == (short)(short)9971);
                Debug.Assert(pack.target_system == (byte)(byte)243);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)30);
            };
            DemoDevice.MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.new_MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.end_index = (short)(short)19615;
            p38.start_index = (short)(short)9971;
            p38.target_component = (byte)(byte)30;
            p38.target_system = (byte)(byte)243;
            p38.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            LoopBackDemoChannel.instance.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param4 == (float)6.3829316E37F);
                Debug.Assert(pack.z == (float)2.0541203E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT);
                Debug.Assert(pack.param3 == (float)2.4703056E38F);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION);
                Debug.Assert(pack.param1 == (float) -9.905992E36F);
                Debug.Assert(pack.autocontinue == (byte)(byte)230);
                Debug.Assert(pack.target_system == (byte)(byte)203);
                Debug.Assert(pack.seq == (ushort)(ushort)23749);
                Debug.Assert(pack.x == (float)2.0041876E38F);
                Debug.Assert(pack.param2 == (float)2.9504971E38F);
                Debug.Assert(pack.target_component == (byte)(byte)110);
                Debug.Assert(pack.current == (byte)(byte)8);
                Debug.Assert(pack.y == (float) -2.7590142E38F);
            };
            DemoDevice.MISSION_ITEM p39 = LoopBackDemoChannel.new_MISSION_ITEM();
            PH.setPack(p39);
            p39.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p39.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p39.param1 = (float) -9.905992E36F;
            p39.command = (MAV_CMD)MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION;
            p39.current = (byte)(byte)8;
            p39.param3 = (float)2.4703056E38F;
            p39.y = (float) -2.7590142E38F;
            p39.param4 = (float)6.3829316E37F;
            p39.autocontinue = (byte)(byte)230;
            p39.param2 = (float)2.9504971E38F;
            p39.seq = (ushort)(ushort)23749;
            p39.target_component = (byte)(byte)110;
            p39.x = (float)2.0041876E38F;
            p39.target_system = (byte)(byte)203;
            p39.z = (float)2.0541203E38F;
            LoopBackDemoChannel.instance.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)8);
                Debug.Assert(pack.seq == (ushort)(ushort)189);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)112);
            };
            DemoDevice.MISSION_REQUEST p40 = LoopBackDemoChannel.new_MISSION_REQUEST();
            PH.setPack(p40);
            p40.seq = (ushort)(ushort)189;
            p40.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p40.target_component = (byte)(byte)112;
            p40.target_system = (byte)(byte)8;
            LoopBackDemoChannel.instance.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)37302);
                Debug.Assert(pack.target_system == (byte)(byte)249);
                Debug.Assert(pack.target_component == (byte)(byte)188);
            };
            DemoDevice.MISSION_SET_CURRENT p41 = LoopBackDemoChannel.new_MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_system = (byte)(byte)249;
            p41.seq = (ushort)(ushort)37302;
            p41.target_component = (byte)(byte)188;
            LoopBackDemoChannel.instance.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)44281);
            };
            DemoDevice.MISSION_CURRENT p42 = LoopBackDemoChannel.new_MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)44281;
            LoopBackDemoChannel.instance.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)22);
                Debug.Assert(pack.target_component == (byte)(byte)127);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            };
            DemoDevice.MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.new_MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.target_system = (byte)(byte)22;
            p43.target_component = (byte)(byte)127;
            p43.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            LoopBackDemoChannel.instance.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)202);
                Debug.Assert(pack.count == (ushort)(ushort)40610);
                Debug.Assert(pack.target_system == (byte)(byte)147);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            };
            DemoDevice.MISSION_COUNT p44 = LoopBackDemoChannel.new_MISSION_COUNT();
            PH.setPack(p44);
            p44.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p44.target_component = (byte)(byte)202;
            p44.count = (ushort)(ushort)40610;
            p44.target_system = (byte)(byte)147;
            LoopBackDemoChannel.instance.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)117);
                Debug.Assert(pack.target_system == (byte)(byte)21);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            };
            DemoDevice.MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.new_MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_component = (byte)(byte)117;
            p45.target_system = (byte)(byte)21;
            p45.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            LoopBackDemoChannel.instance.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)40396);
            };
            DemoDevice.MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.new_MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)40396;
            LoopBackDemoChannel.instance.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_system == (byte)(byte)136);
                Debug.Assert(pack.type == (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED);
                Debug.Assert(pack.target_component == (byte)(byte)80);
            };
            DemoDevice.MISSION_ACK p47 = LoopBackDemoChannel.new_MISSION_ACK();
            PH.setPack(p47);
            p47.target_system = (byte)(byte)136;
            p47.target_component = (byte)(byte)80;
            p47.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p47.type = (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED;
            LoopBackDemoChannel.instance.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.latitude == (int) -431829271);
                Debug.Assert(pack.altitude == (int)316305206);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)4273430348676590499L);
                Debug.Assert(pack.longitude == (int) -1361613766);
                Debug.Assert(pack.target_system == (byte)(byte)11);
            };
            DemoDevice.SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.new_SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.time_usec_SET((ulong)4273430348676590499L, PH) ;
            p48.longitude = (int) -1361613766;
            p48.latitude = (int) -431829271;
            p48.altitude = (int)316305206;
            p48.target_system = (byte)(byte)11;
            LoopBackDemoChannel.instance.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.longitude == (int)282758308);
                Debug.Assert(pack.latitude == (int) -390201510);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)8642158956959518816L);
                Debug.Assert(pack.altitude == (int)1858700979);
            };
            DemoDevice.GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.new_GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.altitude = (int)1858700979;
            p49.latitude = (int) -390201510;
            p49.time_usec_SET((ulong)8642158956959518816L, PH) ;
            p49.longitude = (int)282758308;
            LoopBackDemoChannel.instance.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)154);
                Debug.Assert(pack.scale == (float)3.2571282E38F);
                Debug.Assert(pack.param_id_LEN(ph) == 1);
                Debug.Assert(pack.param_id_TRY(ph).Equals("e"));
                Debug.Assert(pack.param_value_min == (float)8.790701E37F);
                Debug.Assert(pack.param_value0 == (float) -2.0663127E38F);
                Debug.Assert(pack.target_component == (byte)(byte)92);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)114);
                Debug.Assert(pack.param_value_max == (float)1.2956962E38F);
                Debug.Assert(pack.param_index == (short)(short) -6463);
            };
            DemoDevice.PARAM_MAP_RC p50 = LoopBackDemoChannel.new_PARAM_MAP_RC();
            PH.setPack(p50);
            p50.param_value0 = (float) -2.0663127E38F;
            p50.param_value_max = (float)1.2956962E38F;
            p50.target_system = (byte)(byte)154;
            p50.parameter_rc_channel_index = (byte)(byte)114;
            p50.param_value_min = (float)8.790701E37F;
            p50.param_id_SET("e", PH) ;
            p50.target_component = (byte)(byte)92;
            p50.param_index = (short)(short) -6463;
            p50.scale = (float)3.2571282E38F;
            LoopBackDemoChannel.instance.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)141);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)14);
                Debug.Assert(pack.seq == (ushort)(ushort)46037);
            };
            DemoDevice.MISSION_REQUEST_INT p51 = LoopBackDemoChannel.new_MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.target_system = (byte)(byte)141;
            p51.target_component = (byte)(byte)14;
            p51.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p51.seq = (ushort)(ushort)46037;
            LoopBackDemoChannel.instance.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p1z == (float) -2.3926295E38F);
                Debug.Assert(pack.p2y == (float)2.5783417E38F);
                Debug.Assert(pack.p1y == (float)5.382356E37F);
                Debug.Assert(pack.target_system == (byte)(byte)117);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.p2z == (float)2.8211633E38F);
                Debug.Assert(pack.p2x == (float) -2.9756962E38F);
                Debug.Assert(pack.target_component == (byte)(byte)143);
                Debug.Assert(pack.p1x == (float)2.51236E37F);
            };
            DemoDevice.SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.new_SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p1x = (float)2.51236E37F;
            p54.p1y = (float)5.382356E37F;
            p54.p2x = (float) -2.9756962E38F;
            p54.p2z = (float)2.8211633E38F;
            p54.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p54.target_system = (byte)(byte)117;
            p54.p1z = (float) -2.3926295E38F;
            p54.p2y = (float)2.5783417E38F;
            p54.target_component = (byte)(byte)143;
            LoopBackDemoChannel.instance.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p2y == (float)2.652571E38F);
                Debug.Assert(pack.p1y == (float) -1.6480961E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.p1x == (float) -1.6919052E38F);
                Debug.Assert(pack.p2z == (float)1.4585321E38F);
                Debug.Assert(pack.p2x == (float)9.123806E37F);
                Debug.Assert(pack.p1z == (float)1.5394726E38F);
            };
            DemoDevice.SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.new_SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p1x = (float) -1.6919052E38F;
            p55.p2y = (float)2.652571E38F;
            p55.p2x = (float)9.123806E37F;
            p55.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p55.p1y = (float) -1.6480961E38F;
            p55.p2z = (float)1.4585321E38F;
            p55.p1z = (float)1.5394726E38F;
            LoopBackDemoChannel.instance.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.4794554E37F, 2.3319461E38F, 2.9177824E38F, 8.1938237E37F}));
                Debug.Assert(pack.pitchspeed == (float) -1.7111902E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {2.2122647E38F, -2.149E38F, -1.9001162E38F, -1.807063E38F, -2.1212098E38F, 1.1663448E38F, -2.770619E38F, -4.0993452E37F, 1.8957678E38F}));
                Debug.Assert(pack.time_usec == (ulong)543485807628363339L);
                Debug.Assert(pack.rollspeed == (float)1.155284E38F);
                Debug.Assert(pack.yawspeed == (float) -1.9100442E38F);
            };
            DemoDevice.ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.q_SET(new float[] {-2.4794554E37F, 2.3319461E38F, 2.9177824E38F, 8.1938237E37F}, 0) ;
            p61.yawspeed = (float) -1.9100442E38F;
            p61.covariance_SET(new float[] {2.2122647E38F, -2.149E38F, -1.9001162E38F, -1.807063E38F, -2.1212098E38F, 1.1663448E38F, -2.770619E38F, -4.0993452E37F, 1.8957678E38F}, 0) ;
            p61.time_usec = (ulong)543485807628363339L;
            p61.pitchspeed = (float) -1.7111902E38F;
            p61.rollspeed = (float)1.155284E38F;
            LoopBackDemoChannel.instance.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.nav_bearing == (short)(short) -18162);
                Debug.Assert(pack.aspd_error == (float) -1.3045699E38F);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)32967);
                Debug.Assert(pack.xtrack_error == (float) -1.2006779E38F);
                Debug.Assert(pack.target_bearing == (short)(short) -11583);
                Debug.Assert(pack.alt_error == (float) -6.4283596E37F);
                Debug.Assert(pack.nav_roll == (float)2.9874089E38F);
                Debug.Assert(pack.nav_pitch == (float)2.2771127E37F);
            };
            DemoDevice.NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.new_NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.alt_error = (float) -6.4283596E37F;
            p62.aspd_error = (float) -1.3045699E38F;
            p62.nav_pitch = (float)2.2771127E37F;
            p62.nav_roll = (float)2.9874089E38F;
            p62.wp_dist = (ushort)(ushort)32967;
            p62.xtrack_error = (float) -1.2006779E38F;
            p62.target_bearing = (short)(short) -11583;
            p62.nav_bearing = (short)(short) -18162;
            LoopBackDemoChannel.instance.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float)2.770996E38F);
                Debug.Assert(pack.alt == (int)62519551);
                Debug.Assert(pack.time_usec == (ulong)2263069556810647137L);
                Debug.Assert(pack.relative_alt == (int)233023102);
                Debug.Assert(pack.vz == (float) -5.516103E37F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {2.267864E38F, -1.5150979E38F, 2.016882E38F, -8.350663E37F, 9.460112E37F, -2.1747514E38F, -2.6397795E38F, 2.6500921E38F, -1.2669987E38F, 2.5588696E38F, -3.1097147E38F, -2.1883769E38F, 2.5760613E38F, 8.637774E37F, -1.6751342E38F, -1.0645908E38F, -1.8329694E38F, -1.8681812E37F, 2.3553936E38F, 4.295755E35F, -3.2882368E38F, 1.9082358E38F, 1.646044E38F, -4.330478E37F, -1.966579E38F, 1.4591171E38F, -2.9240896E38F, 3.6896104E37F, -1.1779293E38F, -9.983724E37F, -4.3557595E37F, 8.053627E37F, 1.1933294E37F, -1.6354386E38F, -2.8100538E38F, -7.3879123E37F}));
                Debug.Assert(pack.lat == (int) -952566233);
                Debug.Assert(pack.lon == (int) -719417083);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
                Debug.Assert(pack.vx == (float)2.6367672E38F);
            };
            DemoDevice.GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.lon = (int) -719417083;
            p63.vy = (float)2.770996E38F;
            p63.alt = (int)62519551;
            p63.lat = (int) -952566233;
            p63.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS;
            p63.vx = (float)2.6367672E38F;
            p63.time_usec = (ulong)2263069556810647137L;
            p63.vz = (float) -5.516103E37F;
            p63.relative_alt = (int)233023102;
            p63.covariance_SET(new float[] {2.267864E38F, -1.5150979E38F, 2.016882E38F, -8.350663E37F, 9.460112E37F, -2.1747514E38F, -2.6397795E38F, 2.6500921E38F, -1.2669987E38F, 2.5588696E38F, -3.1097147E38F, -2.1883769E38F, 2.5760613E38F, 8.637774E37F, -1.6751342E38F, -1.0645908E38F, -1.8329694E38F, -1.8681812E37F, 2.3553936E38F, 4.295755E35F, -3.2882368E38F, 1.9082358E38F, 1.646044E38F, -4.330478E37F, -1.966579E38F, 1.4591171E38F, -2.9240896E38F, 3.6896104E37F, -1.1779293E38F, -9.983724E37F, -4.3557595E37F, 8.053627E37F, 1.1933294E37F, -1.6354386E38F, -2.8100538E38F, -7.3879123E37F}, 0) ;
            LoopBackDemoChannel.instance.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -2.571015E38F);
                Debug.Assert(pack.time_usec == (ulong)9199862718567197758L);
                Debug.Assert(pack.vy == (float) -2.3752973E38F);
                Debug.Assert(pack.ay == (float)4.7136558E36F);
                Debug.Assert(pack.vx == (float) -1.7350626E38F);
                Debug.Assert(pack.az == (float) -6.760558E37F);
                Debug.Assert(pack.ax == (float)3.2356191E38F);
                Debug.Assert(pack.x == (float)9.297549E37F);
                Debug.Assert(pack.vz == (float)3.0982435E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {3.2781256E38F, -5.885022E37F, -1.4248833E38F, 1.7192021E38F, 7.5897025E37F, 3.9915435E37F, 2.8552586E38F, -1.2722703E38F, 2.7520536E38F, -3.6847393E37F, 1.2683481E38F, 7.157713E37F, -2.0285505E38F, -2.4438156E38F, 2.9911824E38F, -2.3647784E37F, 2.9215077E38F, 2.8837124E38F, -1.3223241E38F, 2.4680092E38F, 2.9790947E38F, 1.0810291E38F, -3.3742969E38F, -2.3666998E38F, -2.2044083E38F, 2.919927E38F, 3.3229984E38F, -2.1623878E38F, 1.6542362E38F, 1.3533973E38F, -2.4667075E38F, -3.3680393E38F, 3.0251733E38F, -2.7269458E38F, -6.7370133E37F, -2.9853425E38F, 6.716488E37F, -1.4439326E38F, 2.1694284E38F, -7.707743E37F, -1.4462226E38F, -2.7172448E38F, -3.5203567E37F, 1.149554E38F, 2.5061872E38F}));
                Debug.Assert(pack.z == (float)2.8350793E38F);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
            };
            DemoDevice.LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.vx = (float) -1.7350626E38F;
            p64.y = (float) -2.571015E38F;
            p64.vz = (float)3.0982435E38F;
            p64.covariance_SET(new float[] {3.2781256E38F, -5.885022E37F, -1.4248833E38F, 1.7192021E38F, 7.5897025E37F, 3.9915435E37F, 2.8552586E38F, -1.2722703E38F, 2.7520536E38F, -3.6847393E37F, 1.2683481E38F, 7.157713E37F, -2.0285505E38F, -2.4438156E38F, 2.9911824E38F, -2.3647784E37F, 2.9215077E38F, 2.8837124E38F, -1.3223241E38F, 2.4680092E38F, 2.9790947E38F, 1.0810291E38F, -3.3742969E38F, -2.3666998E38F, -2.2044083E38F, 2.919927E38F, 3.3229984E38F, -2.1623878E38F, 1.6542362E38F, 1.3533973E38F, -2.4667075E38F, -3.3680393E38F, 3.0251733E38F, -2.7269458E38F, -6.7370133E37F, -2.9853425E38F, 6.716488E37F, -1.4439326E38F, 2.1694284E38F, -7.707743E37F, -1.4462226E38F, -2.7172448E38F, -3.5203567E37F, 1.149554E38F, 2.5061872E38F}, 0) ;
            p64.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION;
            p64.z = (float)2.8350793E38F;
            p64.vy = (float) -2.3752973E38F;
            p64.x = (float)9.297549E37F;
            p64.time_usec = (ulong)9199862718567197758L;
            p64.ay = (float)4.7136558E36F;
            p64.ax = (float)3.2356191E38F;
            p64.az = (float) -6.760558E37F;
            LoopBackDemoChannel.instance.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)64963);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)14569);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)17638);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)6146);
                Debug.Assert(pack.chancount == (byte)(byte)162);
                Debug.Assert(pack.time_boot_ms == (uint)3122743914U);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)37524);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)467);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)52018);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)34433);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)61244);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)63647);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)3288);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)29337);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)7647);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)7629);
                Debug.Assert(pack.rssi == (byte)(byte)81);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)12874);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)45452);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)45626);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)42205);
            };
            DemoDevice.RC_CHANNELS p65 = LoopBackDemoChannel.new_RC_CHANNELS();
            PH.setPack(p65);
            p65.chan7_raw = (ushort)(ushort)45452;
            p65.chan2_raw = (ushort)(ushort)45626;
            p65.chan9_raw = (ushort)(ushort)14569;
            p65.rssi = (byte)(byte)81;
            p65.chan6_raw = (ushort)(ushort)34433;
            p65.chan16_raw = (ushort)(ushort)6146;
            p65.chancount = (byte)(byte)162;
            p65.chan3_raw = (ushort)(ushort)63647;
            p65.chan10_raw = (ushort)(ushort)64963;
            p65.chan4_raw = (ushort)(ushort)29337;
            p65.chan11_raw = (ushort)(ushort)17638;
            p65.time_boot_ms = (uint)3122743914U;
            p65.chan12_raw = (ushort)(ushort)42205;
            p65.chan18_raw = (ushort)(ushort)61244;
            p65.chan17_raw = (ushort)(ushort)52018;
            p65.chan5_raw = (ushort)(ushort)7629;
            p65.chan15_raw = (ushort)(ushort)12874;
            p65.chan1_raw = (ushort)(ushort)467;
            p65.chan8_raw = (ushort)(ushort)37524;
            p65.chan13_raw = (ushort)(ushort)3288;
            p65.chan14_raw = (ushort)(ushort)7647;
            LoopBackDemoChannel.instance.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)203);
                Debug.Assert(pack.req_stream_id == (byte)(byte)231);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)45229);
                Debug.Assert(pack.start_stop == (byte)(byte)79);
                Debug.Assert(pack.target_system == (byte)(byte)8);
            };
            DemoDevice.REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.new_REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.req_message_rate = (ushort)(ushort)45229;
            p66.start_stop = (byte)(byte)79;
            p66.req_stream_id = (byte)(byte)231;
            p66.target_component = (byte)(byte)203;
            p66.target_system = (byte)(byte)8;
            LoopBackDemoChannel.instance.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.stream_id == (byte)(byte)77);
                Debug.Assert(pack.on_off == (byte)(byte)183);
                Debug.Assert(pack.message_rate == (ushort)(ushort)59352);
            };
            DemoDevice.DATA_STREAM p67 = LoopBackDemoChannel.new_DATA_STREAM();
            PH.setPack(p67);
            p67.message_rate = (ushort)(ushort)59352;
            p67.stream_id = (byte)(byte)77;
            p67.on_off = (byte)(byte)183;
            LoopBackDemoChannel.instance.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (short)(short) -31104);
                Debug.Assert(pack.target == (byte)(byte)12);
                Debug.Assert(pack.buttons == (ushort)(ushort)28482);
                Debug.Assert(pack.x == (short)(short) -11717);
                Debug.Assert(pack.r == (short)(short) -3110);
                Debug.Assert(pack.y == (short)(short)1253);
            };
            DemoDevice.MANUAL_CONTROL p69 = LoopBackDemoChannel.new_MANUAL_CONTROL();
            PH.setPack(p69);
            p69.y = (short)(short)1253;
            p69.r = (short)(short) -3110;
            p69.target = (byte)(byte)12;
            p69.x = (short)(short) -11717;
            p69.buttons = (ushort)(ushort)28482;
            p69.z = (short)(short) -31104;
            LoopBackDemoChannel.instance.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)4914);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)25954);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)32855);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)61023);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)37382);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)6821);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)9216);
                Debug.Assert(pack.target_component == (byte)(byte)122);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)8914);
                Debug.Assert(pack.target_system == (byte)(byte)30);
            };
            DemoDevice.RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.new_RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan7_raw = (ushort)(ushort)37382;
            p70.chan4_raw = (ushort)(ushort)25954;
            p70.target_component = (byte)(byte)122;
            p70.chan2_raw = (ushort)(ushort)6821;
            p70.chan8_raw = (ushort)(ushort)32855;
            p70.chan1_raw = (ushort)(ushort)61023;
            p70.chan3_raw = (ushort)(ushort)4914;
            p70.chan6_raw = (ushort)(ushort)9216;
            p70.chan5_raw = (ushort)(ushort)8914;
            p70.target_system = (byte)(byte)30;
            LoopBackDemoChannel.instance.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.autocontinue == (byte)(byte)190);
                Debug.Assert(pack.param2 == (float) -3.3210063E38F);
                Debug.Assert(pack.x == (int)359445964);
                Debug.Assert(pack.param4 == (float) -1.3116189E38F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_GET_MID_LEVEL_COMMANDS);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_component == (byte)(byte)57);
                Debug.Assert(pack.target_system == (byte)(byte)178);
                Debug.Assert(pack.current == (byte)(byte)121);
                Debug.Assert(pack.param1 == (float) -2.6416966E38F);
                Debug.Assert(pack.z == (float)2.8919757E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.param3 == (float)1.3753113E38F);
                Debug.Assert(pack.seq == (ushort)(ushort)2449);
                Debug.Assert(pack.y == (int) -443592622);
            };
            DemoDevice.MISSION_ITEM_INT p73 = LoopBackDemoChannel.new_MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.current = (byte)(byte)121;
            p73.seq = (ushort)(ushort)2449;
            p73.target_system = (byte)(byte)178;
            p73.autocontinue = (byte)(byte)190;
            p73.command = (MAV_CMD)MAV_CMD.MAV_CMD_GET_MID_LEVEL_COMMANDS;
            p73.y = (int) -443592622;
            p73.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p73.x = (int)359445964;
            p73.param4 = (float) -1.3116189E38F;
            p73.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p73.target_component = (byte)(byte)57;
            p73.param2 = (float) -3.3210063E38F;
            p73.z = (float)2.8919757E38F;
            p73.param3 = (float)1.3753113E38F;
            p73.param1 = (float) -2.6416966E38F;
            LoopBackDemoChannel.instance.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.throttle == (ushort)(ushort)48322);
                Debug.Assert(pack.alt == (float)1.643345E38F);
                Debug.Assert(pack.airspeed == (float)3.311855E38F);
                Debug.Assert(pack.heading == (short)(short)1744);
                Debug.Assert(pack.groundspeed == (float) -3.059086E38F);
                Debug.Assert(pack.climb == (float) -3.1551398E38F);
            };
            DemoDevice.VFR_HUD p74 = LoopBackDemoChannel.new_VFR_HUD();
            PH.setPack(p74);
            p74.alt = (float)1.643345E38F;
            p74.heading = (short)(short)1744;
            p74.groundspeed = (float) -3.059086E38F;
            p74.climb = (float) -3.1551398E38F;
            p74.throttle = (ushort)(ushort)48322;
            p74.airspeed = (float)3.311855E38F;
            LoopBackDemoChannel.instance.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (int) -1301055542);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.autocontinue == (byte)(byte)251);
                Debug.Assert(pack.param2 == (float)2.843728E38F);
                Debug.Assert(pack.target_component == (byte)(byte)71);
                Debug.Assert(pack.x == (int)2085594344);
                Debug.Assert(pack.param1 == (float) -1.592226E38F);
                Debug.Assert(pack.current == (byte)(byte)199);
                Debug.Assert(pack.param3 == (float) -5.180471E37F);
                Debug.Assert(pack.param4 == (float) -2.2894115E38F);
                Debug.Assert(pack.z == (float) -1.275068E38F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_WAYPOINT_USER_2);
                Debug.Assert(pack.target_system == (byte)(byte)189);
            };
            DemoDevice.COMMAND_INT p75 = LoopBackDemoChannel.new_COMMAND_INT();
            PH.setPack(p75);
            p75.current = (byte)(byte)199;
            p75.param1 = (float) -1.592226E38F;
            p75.z = (float) -1.275068E38F;
            p75.autocontinue = (byte)(byte)251;
            p75.param3 = (float) -5.180471E37F;
            p75.command = (MAV_CMD)MAV_CMD.MAV_CMD_WAYPOINT_USER_2;
            p75.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION;
            p75.target_system = (byte)(byte)189;
            p75.param2 = (float)2.843728E38F;
            p75.target_component = (byte)(byte)71;
            p75.param4 = (float) -2.2894115E38F;
            p75.y = (int) -1301055542;
            p75.x = (int)2085594344;
            LoopBackDemoChannel.instance.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param4 == (float)2.3411957E38F);
                Debug.Assert(pack.param5 == (float)1.6150493E38F);
                Debug.Assert(pack.target_system == (byte)(byte)46);
                Debug.Assert(pack.param3 == (float) -3.2205197E38F);
                Debug.Assert(pack.param6 == (float)1.1365479E38F);
                Debug.Assert(pack.confirmation == (byte)(byte)101);
                Debug.Assert(pack.param7 == (float) -4.0555794E37F);
                Debug.Assert(pack.param2 == (float) -2.8074692E38F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL);
                Debug.Assert(pack.param1 == (float) -2.2515568E38F);
                Debug.Assert(pack.target_component == (byte)(byte)69);
            };
            DemoDevice.COMMAND_LONG p76 = LoopBackDemoChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.param2 = (float) -2.8074692E38F;
            p76.target_system = (byte)(byte)46;
            p76.param5 = (float)1.6150493E38F;
            p76.param1 = (float) -2.2515568E38F;
            p76.param4 = (float)2.3411957E38F;
            p76.param3 = (float) -3.2205197E38F;
            p76.param7 = (float) -4.0555794E37F;
            p76.confirmation = (byte)(byte)101;
            p76.param6 = (float)1.1365479E38F;
            p76.target_component = (byte)(byte)69;
            p76.command = (MAV_CMD)MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL;
            LoopBackDemoChannel.instance.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.result_param2_TRY(ph) == (int) -1681111148);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)182);
                Debug.Assert(pack.result == (MAV_RESULT)MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_MOTOR_TEST);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)37);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)126);
            };
            DemoDevice.COMMAND_ACK p77 = LoopBackDemoChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.progress_SET((byte)(byte)126, PH) ;
            p77.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_MOTOR_TEST;
            p77.target_component_SET((byte)(byte)37, PH) ;
            p77.result = (MAV_RESULT)MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED;
            p77.result_param2_SET((int) -1681111148, PH) ;
            p77.target_system_SET((byte)(byte)182, PH) ;
            LoopBackDemoChannel.instance.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float)3.6873778E37F);
                Debug.Assert(pack.thrust == (float)2.8124492E38F);
                Debug.Assert(pack.mode_switch == (byte)(byte)42);
                Debug.Assert(pack.yaw == (float) -2.3506554E38F);
                Debug.Assert(pack.roll == (float) -9.446949E37F);
                Debug.Assert(pack.time_boot_ms == (uint)1841898989U);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)196);
            };
            DemoDevice.MANUAL_SETPOINT p81 = LoopBackDemoChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.pitch = (float)3.6873778E37F;
            p81.mode_switch = (byte)(byte)42;
            p81.yaw = (float) -2.3506554E38F;
            p81.manual_override_switch = (byte)(byte)196;
            p81.time_boot_ms = (uint)1841898989U;
            p81.thrust = (float)2.8124492E38F;
            p81.roll = (float) -9.446949E37F;
            LoopBackDemoChannel.instance.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.body_pitch_rate == (float)1.3848616E38F);
                Debug.Assert(pack.type_mask == (byte)(byte)79);
                Debug.Assert(pack.target_system == (byte)(byte)55);
                Debug.Assert(pack.body_yaw_rate == (float)3.2621416E38F);
                Debug.Assert(pack.target_component == (byte)(byte)125);
                Debug.Assert(pack.thrust == (float)2.7900489E38F);
                Debug.Assert(pack.time_boot_ms == (uint)4034072804U);
                Debug.Assert(pack.body_roll_rate == (float)1.4160395E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.6117219E38F, 1.3647886E38F, 8.715657E37F, 3.1075728E38F}));
            };
            DemoDevice.SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.body_roll_rate = (float)1.4160395E37F;
            p82.q_SET(new float[] {-1.6117219E38F, 1.3647886E38F, 8.715657E37F, 3.1075728E38F}, 0) ;
            p82.target_system = (byte)(byte)55;
            p82.time_boot_ms = (uint)4034072804U;
            p82.body_pitch_rate = (float)1.3848616E38F;
            p82.thrust = (float)2.7900489E38F;
            p82.target_component = (byte)(byte)125;
            p82.type_mask = (byte)(byte)79;
            p82.body_yaw_rate = (float)3.2621416E38F;
            LoopBackDemoChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.body_roll_rate == (float) -1.0884769E38F);
                Debug.Assert(pack.time_boot_ms == (uint)851215221U);
                Debug.Assert(pack.body_pitch_rate == (float)2.0464059E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-3.537917E37F, 2.1338344E38F, 2.0417707E38F, 3.2837235E38F}));
                Debug.Assert(pack.type_mask == (byte)(byte)59);
                Debug.Assert(pack.thrust == (float) -2.7908074E38F);
                Debug.Assert(pack.body_yaw_rate == (float) -1.791606E38F);
            };
            DemoDevice.ATTITUDE_TARGET p83 = LoopBackDemoChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.thrust = (float) -2.7908074E38F;
            p83.time_boot_ms = (uint)851215221U;
            p83.body_roll_rate = (float) -1.0884769E38F;
            p83.type_mask = (byte)(byte)59;
            p83.body_pitch_rate = (float)2.0464059E38F;
            p83.q_SET(new float[] {-3.537917E37F, 2.1338344E38F, 2.0417707E38F, 3.2837235E38F}, 0) ;
            p83.body_yaw_rate = (float) -1.791606E38F;
            LoopBackDemoChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float)2.664507E38F);
                Debug.Assert(pack.yaw_rate == (float) -5.9209947E37F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
                Debug.Assert(pack.yaw == (float) -1.6026038E38F);
                Debug.Assert(pack.target_system == (byte)(byte)34);
                Debug.Assert(pack.z == (float)1.2764956E38F);
                Debug.Assert(pack.x == (float)1.798568E38F);
                Debug.Assert(pack.afy == (float) -2.5871972E38F);
                Debug.Assert(pack.time_boot_ms == (uint)596005117U);
                Debug.Assert(pack.target_component == (byte)(byte)150);
                Debug.Assert(pack.y == (float)3.1224396E38F);
                Debug.Assert(pack.afz == (float)2.197543E38F);
                Debug.Assert(pack.vz == (float) -1.5323407E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)13342);
                Debug.Assert(pack.vx == (float)3.4458163E37F);
                Debug.Assert(pack.afx == (float) -1.8843772E38F);
            };
            DemoDevice.SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.afx = (float) -1.8843772E38F;
            p84.z = (float)1.2764956E38F;
            p84.vy = (float)2.664507E38F;
            p84.type_mask = (ushort)(ushort)13342;
            p84.target_component = (byte)(byte)150;
            p84.y = (float)3.1224396E38F;
            p84.time_boot_ms = (uint)596005117U;
            p84.yaw = (float) -1.6026038E38F;
            p84.yaw_rate = (float) -5.9209947E37F;
            p84.vx = (float)3.4458163E37F;
            p84.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p84.target_system = (byte)(byte)34;
            p84.afy = (float) -2.5871972E38F;
            p84.x = (float)1.798568E38F;
            p84.vz = (float) -1.5323407E38F;
            p84.afz = (float)2.197543E38F;
            LoopBackDemoChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (float)2.4154058E38F);
                Debug.Assert(pack.yaw_rate == (float) -3.1898886E38F);
                Debug.Assert(pack.afz == (float)5.9054274E37F);
                Debug.Assert(pack.vy == (float) -1.0824539E38F);
                Debug.Assert(pack.lat_int == (int) -703012291);
                Debug.Assert(pack.target_component == (byte)(byte)150);
                Debug.Assert(pack.vz == (float)2.024187E38F);
                Debug.Assert(pack.lon_int == (int)1286530086);
                Debug.Assert(pack.afx == (float) -2.8831216E38F);
                Debug.Assert(pack.afy == (float)1.0908093E37F);
                Debug.Assert(pack.target_system == (byte)(byte)80);
                Debug.Assert(pack.time_boot_ms == (uint)1725166862U);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.yaw == (float)2.91007E38F);
                Debug.Assert(pack.vx == (float)3.0733236E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)48666);
            };
            DemoDevice.SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.afy = (float)1.0908093E37F;
            p86.yaw_rate = (float) -3.1898886E38F;
            p86.lon_int = (int)1286530086;
            p86.type_mask = (ushort)(ushort)48666;
            p86.vz = (float)2.024187E38F;
            p86.alt = (float)2.4154058E38F;
            p86.yaw = (float)2.91007E38F;
            p86.vx = (float)3.0733236E38F;
            p86.target_system = (byte)(byte)80;
            p86.target_component = (byte)(byte)150;
            p86.vy = (float) -1.0824539E38F;
            p86.afz = (float)5.9054274E37F;
            p86.time_boot_ms = (uint)1725166862U;
            p86.lat_int = (int) -703012291;
            p86.afx = (float) -2.8831216E38F;
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            LoopBackDemoChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED);
                Debug.Assert(pack.afx == (float)6.089233E37F);
                Debug.Assert(pack.vx == (float)2.4294912E37F);
                Debug.Assert(pack.vy == (float) -6.595231E37F);
                Debug.Assert(pack.vz == (float) -2.1648645E38F);
                Debug.Assert(pack.lat_int == (int) -696120246);
                Debug.Assert(pack.afz == (float) -1.9249132E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)1842);
                Debug.Assert(pack.yaw == (float) -2.0364643E38F);
                Debug.Assert(pack.yaw_rate == (float)8.591324E37F);
                Debug.Assert(pack.time_boot_ms == (uint)1368444137U);
                Debug.Assert(pack.afy == (float)1.0357604E38F);
                Debug.Assert(pack.alt == (float) -8.703019E37F);
                Debug.Assert(pack.lon_int == (int)528927280);
            };
            DemoDevice.POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.afy = (float)1.0357604E38F;
            p87.vz = (float) -2.1648645E38F;
            p87.vx = (float)2.4294912E37F;
            p87.time_boot_ms = (uint)1368444137U;
            p87.vy = (float) -6.595231E37F;
            p87.afx = (float)6.089233E37F;
            p87.lat_int = (int) -696120246;
            p87.afz = (float) -1.9249132E38F;
            p87.yaw_rate = (float)8.591324E37F;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p87.yaw = (float) -2.0364643E38F;
            p87.type_mask = (ushort)(ushort)1842;
            p87.lon_int = (int)528927280;
            p87.alt = (float) -8.703019E37F;
            LoopBackDemoChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -1.3487195E38F);
                Debug.Assert(pack.x == (float)1.4642809E38F);
                Debug.Assert(pack.time_boot_ms == (uint)4286079223U);
                Debug.Assert(pack.roll == (float)3.002634E38F);
                Debug.Assert(pack.pitch == (float)2.5391119E38F);
                Debug.Assert(pack.yaw == (float) -9.952137E37F);
                Debug.Assert(pack.y == (float) -2.5895238E38F);
            };
            DemoDevice.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.y = (float) -2.5895238E38F;
            p89.time_boot_ms = (uint)4286079223U;
            p89.yaw = (float) -9.952137E37F;
            p89.z = (float) -1.3487195E38F;
            p89.pitch = (float)2.5391119E38F;
            p89.x = (float)1.4642809E38F;
            p89.roll = (float)3.002634E38F;
            LoopBackDemoChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)3872624089433728309L);
                Debug.Assert(pack.zacc == (short)(short) -20612);
                Debug.Assert(pack.pitchspeed == (float)1.9627773E38F);
                Debug.Assert(pack.pitch == (float) -1.8998302E37F);
                Debug.Assert(pack.alt == (int) -236646199);
                Debug.Assert(pack.lat == (int) -315615830);
                Debug.Assert(pack.vz == (short)(short)11958);
                Debug.Assert(pack.yawspeed == (float)9.209131E37F);
                Debug.Assert(pack.yaw == (float)3.279295E38F);
                Debug.Assert(pack.yacc == (short)(short)20967);
                Debug.Assert(pack.xacc == (short)(short) -13705);
                Debug.Assert(pack.rollspeed == (float) -2.4871712E38F);
                Debug.Assert(pack.vy == (short)(short)2067);
                Debug.Assert(pack.roll == (float) -8.21972E37F);
                Debug.Assert(pack.vx == (short)(short) -23659);
                Debug.Assert(pack.lon == (int)865055462);
            };
            DemoDevice.HIL_STATE p90 = LoopBackDemoChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.yacc = (short)(short)20967;
            p90.alt = (int) -236646199;
            p90.xacc = (short)(short) -13705;
            p90.yawspeed = (float)9.209131E37F;
            p90.rollspeed = (float) -2.4871712E38F;
            p90.pitchspeed = (float)1.9627773E38F;
            p90.vx = (short)(short) -23659;
            p90.yaw = (float)3.279295E38F;
            p90.zacc = (short)(short) -20612;
            p90.pitch = (float) -1.8998302E37F;
            p90.roll = (float) -8.21972E37F;
            p90.lat = (int) -315615830;
            p90.vz = (short)(short)11958;
            p90.lon = (int)865055462;
            p90.vy = (short)(short)2067;
            p90.time_usec = (ulong)3872624089433728309L;
            LoopBackDemoChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.throttle == (float)2.1548833E38F);
                Debug.Assert(pack.roll_ailerons == (float)1.3976513E38F);
                Debug.Assert(pack.yaw_rudder == (float) -1.7171005E38F);
                Debug.Assert(pack.aux3 == (float)1.8343563E38F);
                Debug.Assert(pack.aux4 == (float)3.7192673E37F);
                Debug.Assert(pack.time_usec == (ulong)6008252899650617209L);
                Debug.Assert(pack.aux2 == (float)2.709295E38F);
                Debug.Assert(pack.nav_mode == (byte)(byte)252);
                Debug.Assert(pack.pitch_elevator == (float) -2.1700237E38F);
                Debug.Assert(pack.aux1 == (float) -3.0203974E38F);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_MANUAL_DISARMED);
            };
            DemoDevice.HIL_CONTROLS p91 = LoopBackDemoChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.roll_ailerons = (float)1.3976513E38F;
            p91.aux3 = (float)1.8343563E38F;
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_MANUAL_DISARMED;
            p91.pitch_elevator = (float) -2.1700237E38F;
            p91.yaw_rudder = (float) -1.7171005E38F;
            p91.time_usec = (ulong)6008252899650617209L;
            p91.aux1 = (float) -3.0203974E38F;
            p91.throttle = (float)2.1548833E38F;
            p91.aux2 = (float)2.709295E38F;
            p91.aux4 = (float)3.7192673E37F;
            p91.nav_mode = (byte)(byte)252;
            LoopBackDemoChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)32617);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)52958);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)44127);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)20423);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)52239);
                Debug.Assert(pack.time_usec == (ulong)3387747231480967312L);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)46119);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)5396);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)27992);
                Debug.Assert(pack.rssi == (byte)(byte)190);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)16788);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)816);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)59642);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)56095);
            };
            DemoDevice.HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.time_usec = (ulong)3387747231480967312L;
            p92.chan8_raw = (ushort)(ushort)46119;
            p92.chan7_raw = (ushort)(ushort)816;
            p92.chan9_raw = (ushort)(ushort)59642;
            p92.chan1_raw = (ushort)(ushort)5396;
            p92.chan10_raw = (ushort)(ushort)27992;
            p92.chan11_raw = (ushort)(ushort)56095;
            p92.chan5_raw = (ushort)(ushort)16788;
            p92.rssi = (byte)(byte)190;
            p92.chan2_raw = (ushort)(ushort)52958;
            p92.chan3_raw = (ushort)(ushort)52239;
            p92.chan12_raw = (ushort)(ushort)44127;
            p92.chan6_raw = (ushort)(ushort)32617;
            p92.chan4_raw = (ushort)(ushort)20423;
            LoopBackDemoChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (ulong)7502471511187419974L);
                Debug.Assert(pack.time_usec == (ulong)6064383846745954591L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-4.8698197E37F, -1.0370266E38F, 1.83771E38F, -1.5164421E38F, 1.6382353E38F, 2.2512093E38F, -3.3474586E38F, -2.153994E38F, -2.950252E38F, 1.1230977E38F, 2.0078551E38F, 2.4969023E38F, 1.9350106E38F, 1.7447555E38F, 2.6546601E38F, -1.6699241E38F}));
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_MANUAL_DISARMED);
            };
            DemoDevice.HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.time_usec = (ulong)6064383846745954591L;
            p93.controls_SET(new float[] {-4.8698197E37F, -1.0370266E38F, 1.83771E38F, -1.5164421E38F, 1.6382353E38F, 2.2512093E38F, -3.3474586E38F, -2.153994E38F, -2.950252E38F, 1.1230977E38F, 2.0078551E38F, 2.4969023E38F, 1.9350106E38F, 1.7447555E38F, 2.6546601E38F, -1.6699241E38F}, 0) ;
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_MANUAL_DISARMED;
            p93.flags = (ulong)7502471511187419974L;
            LoopBackDemoChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.quality == (byte)(byte)71);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float) -1.2444868E38F);
                Debug.Assert(pack.flow_x == (short)(short) -31486);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float)9.114711E37F);
                Debug.Assert(pack.sensor_id == (byte)(byte)25);
                Debug.Assert(pack.flow_y == (short)(short) -23410);
                Debug.Assert(pack.time_usec == (ulong)2774595095389811853L);
                Debug.Assert(pack.flow_comp_m_y == (float) -3.3166973E38F);
                Debug.Assert(pack.ground_distance == (float) -2.0960854E38F);
                Debug.Assert(pack.flow_comp_m_x == (float)1.2332748E38F);
            };
            DemoDevice.OPTICAL_FLOW p100 = LoopBackDemoChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.flow_rate_y_SET((float) -1.2444868E38F, PH) ;
            p100.flow_x = (short)(short) -31486;
            p100.flow_comp_m_x = (float)1.2332748E38F;
            p100.quality = (byte)(byte)71;
            p100.time_usec = (ulong)2774595095389811853L;
            p100.flow_rate_x_SET((float)9.114711E37F, PH) ;
            p100.sensor_id = (byte)(byte)25;
            p100.flow_comp_m_y = (float) -3.3166973E38F;
            p100.ground_distance = (float) -2.0960854E38F;
            p100.flow_y = (short)(short) -23410;
            LoopBackDemoChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -1.8830355E38F);
                Debug.Assert(pack.usec == (ulong)6074534162210500743L);
                Debug.Assert(pack.roll == (float) -1.3834987E38F);
                Debug.Assert(pack.yaw == (float) -2.711627E38F);
                Debug.Assert(pack.z == (float) -2.479002E38F);
                Debug.Assert(pack.pitch == (float)1.7313903E38F);
                Debug.Assert(pack.y == (float)2.8230901E38F);
            };
            DemoDevice.GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.yaw = (float) -2.711627E38F;
            p101.pitch = (float)1.7313903E38F;
            p101.x = (float) -1.8830355E38F;
            p101.y = (float)2.8230901E38F;
            p101.roll = (float) -1.3834987E38F;
            p101.usec = (ulong)6074534162210500743L;
            p101.z = (float) -2.479002E38F;
            LoopBackDemoChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)1.2274183E38F);
                Debug.Assert(pack.yaw == (float) -2.3755296E38F);
                Debug.Assert(pack.usec == (ulong)3398273980406583815L);
                Debug.Assert(pack.x == (float) -2.5590986E38F);
                Debug.Assert(pack.roll == (float) -3.2885772E37F);
                Debug.Assert(pack.y == (float)2.70424E38F);
                Debug.Assert(pack.pitch == (float)2.7084059E38F);
            };
            DemoDevice.VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.roll = (float) -3.2885772E37F;
            p102.yaw = (float) -2.3755296E38F;
            p102.z = (float)1.2274183E38F;
            p102.y = (float)2.70424E38F;
            p102.pitch = (float)2.7084059E38F;
            p102.usec = (ulong)3398273980406583815L;
            p102.x = (float) -2.5590986E38F;
            LoopBackDemoChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.usec == (ulong)3722378034775858535L);
                Debug.Assert(pack.z == (float)1.2943046E37F);
                Debug.Assert(pack.y == (float)3.3568156E37F);
                Debug.Assert(pack.x == (float) -7.550686E37F);
            };
            DemoDevice.VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.x = (float) -7.550686E37F;
            p103.z = (float)1.2943046E37F;
            p103.usec = (ulong)3722378034775858535L;
            p103.y = (float)3.3568156E37F;
            LoopBackDemoChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)6.9601036E37F);
                Debug.Assert(pack.z == (float) -3.313135E38F);
                Debug.Assert(pack.pitch == (float)2.1360277E36F);
                Debug.Assert(pack.x == (float)2.6924835E37F);
                Debug.Assert(pack.roll == (float) -1.1773359E38F);
                Debug.Assert(pack.usec == (ulong)5816967486553881944L);
                Debug.Assert(pack.yaw == (float)2.6295513E38F);
            };
            DemoDevice.VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.yaw = (float)2.6295513E38F;
            p104.pitch = (float)2.1360277E36F;
            p104.z = (float) -3.313135E38F;
            p104.roll = (float) -1.1773359E38F;
            p104.y = (float)6.9601036E37F;
            p104.usec = (ulong)5816967486553881944L;
            p104.x = (float)2.6924835E37F;
            LoopBackDemoChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (float) -3.295388E38F);
                Debug.Assert(pack.time_usec == (ulong)7047154108422919855L);
                Debug.Assert(pack.abs_pressure == (float)2.4187017E38F);
                Debug.Assert(pack.ymag == (float)2.6966305E38F);
                Debug.Assert(pack.zacc == (float) -2.9401326E38F);
                Debug.Assert(pack.zmag == (float)2.1261036E38F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)54319);
                Debug.Assert(pack.temperature == (float)3.104468E38F);
                Debug.Assert(pack.zgyro == (float) -2.9255767E38F);
                Debug.Assert(pack.xmag == (float)1.5091297E38F);
                Debug.Assert(pack.yacc == (float) -1.6271134E38F);
                Debug.Assert(pack.xgyro == (float) -5.7802165E37F);
                Debug.Assert(pack.pressure_alt == (float)1.0779261E38F);
                Debug.Assert(pack.diff_pressure == (float)7.688251E37F);
                Debug.Assert(pack.xacc == (float)4.3504126E37F);
            };
            DemoDevice.HIGHRES_IMU p105 = LoopBackDemoChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.xmag = (float)1.5091297E38F;
            p105.zacc = (float) -2.9401326E38F;
            p105.temperature = (float)3.104468E38F;
            p105.ygyro = (float) -3.295388E38F;
            p105.diff_pressure = (float)7.688251E37F;
            p105.xgyro = (float) -5.7802165E37F;
            p105.zmag = (float)2.1261036E38F;
            p105.pressure_alt = (float)1.0779261E38F;
            p105.ymag = (float)2.6966305E38F;
            p105.yacc = (float) -1.6271134E38F;
            p105.fields_updated = (ushort)(ushort)54319;
            p105.zgyro = (float) -2.9255767E38F;
            p105.abs_pressure = (float)2.4187017E38F;
            p105.xacc = (float)4.3504126E37F;
            p105.time_usec = (ulong)7047154108422919855L;
            LoopBackDemoChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integration_time_us == (uint)3370060711U);
                Debug.Assert(pack.quality == (byte)(byte)147);
                Debug.Assert(pack.temperature == (short)(short) -17274);
                Debug.Assert(pack.sensor_id == (byte)(byte)156);
                Debug.Assert(pack.integrated_y == (float)2.4193189E38F);
                Debug.Assert(pack.time_usec == (ulong)7466868809015153588L);
                Debug.Assert(pack.integrated_x == (float) -2.556138E38F);
                Debug.Assert(pack.distance == (float) -2.5738506E38F);
                Debug.Assert(pack.integrated_ygyro == (float)1.7298508E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)3327187357U);
                Debug.Assert(pack.integrated_xgyro == (float) -2.8576862E38F);
                Debug.Assert(pack.integrated_zgyro == (float) -3.6319475E37F);
            };
            DemoDevice.OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.quality = (byte)(byte)147;
            p106.integration_time_us = (uint)3370060711U;
            p106.sensor_id = (byte)(byte)156;
            p106.integrated_y = (float)2.4193189E38F;
            p106.integrated_xgyro = (float) -2.8576862E38F;
            p106.integrated_ygyro = (float)1.7298508E38F;
            p106.distance = (float) -2.5738506E38F;
            p106.integrated_x = (float) -2.556138E38F;
            p106.time_usec = (ulong)7466868809015153588L;
            p106.integrated_zgyro = (float) -3.6319475E37F;
            p106.time_delta_distance_us = (uint)3327187357U;
            p106.temperature = (short)(short) -17274;
            LoopBackDemoChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (float) -1.4429693E37F);
                Debug.Assert(pack.pressure_alt == (float) -2.5295775E38F);
                Debug.Assert(pack.fields_updated == (uint)2043986462U);
                Debug.Assert(pack.zacc == (float)6.4347334E37F);
                Debug.Assert(pack.temperature == (float)2.0416103E38F);
                Debug.Assert(pack.time_usec == (ulong)857133345540107889L);
                Debug.Assert(pack.ymag == (float) -2.6862013E38F);
                Debug.Assert(pack.abs_pressure == (float)6.012513E37F);
                Debug.Assert(pack.zmag == (float)2.3051068E38F);
                Debug.Assert(pack.zgyro == (float) -2.4494196E38F);
                Debug.Assert(pack.xgyro == (float)8.4510513E37F);
                Debug.Assert(pack.yacc == (float) -2.0757124E37F);
                Debug.Assert(pack.diff_pressure == (float) -3.3647402E37F);
                Debug.Assert(pack.xmag == (float) -1.8747232E38F);
                Debug.Assert(pack.ygyro == (float)1.3134213E38F);
            };
            DemoDevice.HIL_SENSOR p107 = LoopBackDemoChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.xmag = (float) -1.8747232E38F;
            p107.xacc = (float) -1.4429693E37F;
            p107.pressure_alt = (float) -2.5295775E38F;
            p107.xgyro = (float)8.4510513E37F;
            p107.diff_pressure = (float) -3.3647402E37F;
            p107.time_usec = (ulong)857133345540107889L;
            p107.ygyro = (float)1.3134213E38F;
            p107.temperature = (float)2.0416103E38F;
            p107.ymag = (float) -2.6862013E38F;
            p107.zmag = (float)2.3051068E38F;
            p107.yacc = (float) -2.0757124E37F;
            p107.zacc = (float)6.4347334E37F;
            p107.abs_pressure = (float)6.012513E37F;
            p107.fields_updated = (uint)2043986462U;
            p107.zgyro = (float) -2.4494196E38F;
            LoopBackDemoChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xgyro == (float) -2.4399138E38F);
                Debug.Assert(pack.lon == (float) -9.416716E37F);
                Debug.Assert(pack.xacc == (float)5.35112E37F);
                Debug.Assert(pack.std_dev_horz == (float)2.4373777E38F);
                Debug.Assert(pack.vd == (float)1.2624346E38F);
                Debug.Assert(pack.q4 == (float) -2.1550527E38F);
                Debug.Assert(pack.roll == (float) -2.9512562E37F);
                Debug.Assert(pack.ygyro == (float)2.6464224E38F);
                Debug.Assert(pack.zgyro == (float) -8.686844E37F);
                Debug.Assert(pack.q3 == (float)5.124505E37F);
                Debug.Assert(pack.zacc == (float) -1.4720935E38F);
                Debug.Assert(pack.vn == (float) -5.86607E37F);
                Debug.Assert(pack.q1 == (float) -1.3965583E38F);
                Debug.Assert(pack.ve == (float) -1.9697538E38F);
                Debug.Assert(pack.q2 == (float)2.4143558E38F);
                Debug.Assert(pack.yaw == (float)2.547834E38F);
                Debug.Assert(pack.std_dev_vert == (float)1.5540105E38F);
                Debug.Assert(pack.lat == (float)1.2999473E38F);
                Debug.Assert(pack.pitch == (float) -9.748416E37F);
                Debug.Assert(pack.alt == (float) -2.3807786E38F);
                Debug.Assert(pack.yacc == (float) -2.3222487E38F);
            };
            DemoDevice.SIM_STATE p108 = LoopBackDemoChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q2 = (float)2.4143558E38F;
            p108.q4 = (float) -2.1550527E38F;
            p108.zacc = (float) -1.4720935E38F;
            p108.std_dev_vert = (float)1.5540105E38F;
            p108.std_dev_horz = (float)2.4373777E38F;
            p108.q1 = (float) -1.3965583E38F;
            p108.xgyro = (float) -2.4399138E38F;
            p108.pitch = (float) -9.748416E37F;
            p108.vd = (float)1.2624346E38F;
            p108.lon = (float) -9.416716E37F;
            p108.yaw = (float)2.547834E38F;
            p108.vn = (float) -5.86607E37F;
            p108.ve = (float) -1.9697538E38F;
            p108.roll = (float) -2.9512562E37F;
            p108.zgyro = (float) -8.686844E37F;
            p108.xacc = (float)5.35112E37F;
            p108.q3 = (float)5.124505E37F;
            p108.yacc = (float) -2.3222487E38F;
            p108.ygyro = (float)2.6464224E38F;
            p108.alt = (float) -2.3807786E38F;
            p108.lat = (float)1.2999473E38F;
            LoopBackDemoChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.remrssi == (byte)(byte)104);
                Debug.Assert(pack.remnoise == (byte)(byte)17);
                Debug.Assert(pack.noise == (byte)(byte)133);
                Debug.Assert(pack.rssi == (byte)(byte)22);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)23701);
                Debug.Assert(pack.txbuf == (byte)(byte)99);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)58276);
            };
            DemoDevice.RADIO_STATUS p109 = LoopBackDemoChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rssi = (byte)(byte)22;
            p109.rxerrors = (ushort)(ushort)58276;
            p109.txbuf = (byte)(byte)99;
            p109.noise = (byte)(byte)133;
            p109.remrssi = (byte)(byte)104;
            p109.remnoise = (byte)(byte)17;
            p109.fixed_ = (ushort)(ushort)23701;
            LoopBackDemoChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_network == (byte)(byte)215);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)96, (byte)156, (byte)58, (byte)26, (byte)186, (byte)147, (byte)179, (byte)77, (byte)5, (byte)225, (byte)51, (byte)127, (byte)209, (byte)224, (byte)221, (byte)81, (byte)41, (byte)246, (byte)58, (byte)79, (byte)181, (byte)32, (byte)167, (byte)205, (byte)18, (byte)136, (byte)88, (byte)239, (byte)34, (byte)65, (byte)86, (byte)117, (byte)245, (byte)76, (byte)240, (byte)211, (byte)172, (byte)17, (byte)104, (byte)91, (byte)165, (byte)176, (byte)158, (byte)58, (byte)75, (byte)60, (byte)49, (byte)136, (byte)233, (byte)56, (byte)181, (byte)0, (byte)89, (byte)67, (byte)112, (byte)83, (byte)148, (byte)248, (byte)135, (byte)236, (byte)251, (byte)217, (byte)218, (byte)34, (byte)155, (byte)77, (byte)92, (byte)44, (byte)241, (byte)169, (byte)179, (byte)46, (byte)156, (byte)54, (byte)183, (byte)172, (byte)1, (byte)138, (byte)58, (byte)175, (byte)127, (byte)70, (byte)178, (byte)148, (byte)146, (byte)27, (byte)198, (byte)237, (byte)191, (byte)36, (byte)184, (byte)163, (byte)33, (byte)133, (byte)132, (byte)119, (byte)217, (byte)178, (byte)24, (byte)81, (byte)38, (byte)129, (byte)173, (byte)112, (byte)208, (byte)197, (byte)115, (byte)219, (byte)38, (byte)196, (byte)219, (byte)229, (byte)251, (byte)100, (byte)53, (byte)127, (byte)6, (byte)146, (byte)44, (byte)80, (byte)92, (byte)30, (byte)121, (byte)125, (byte)76, (byte)250, (byte)238, (byte)145, (byte)136, (byte)72, (byte)37, (byte)77, (byte)57, (byte)33, (byte)163, (byte)17, (byte)111, (byte)42, (byte)89, (byte)220, (byte)2, (byte)15, (byte)191, (byte)87, (byte)181, (byte)218, (byte)111, (byte)12, (byte)8, (byte)183, (byte)201, (byte)201, (byte)49, (byte)52, (byte)50, (byte)140, (byte)9, (byte)255, (byte)26, (byte)238, (byte)191, (byte)22, (byte)185, (byte)214, (byte)19, (byte)186, (byte)246, (byte)26, (byte)208, (byte)162, (byte)146, (byte)185, (byte)148, (byte)73, (byte)93, (byte)66, (byte)42, (byte)238, (byte)175, (byte)94, (byte)147, (byte)83, (byte)224, (byte)247, (byte)142, (byte)226, (byte)4, (byte)67, (byte)177, (byte)21, (byte)36, (byte)60, (byte)3, (byte)246, (byte)228, (byte)68, (byte)86, (byte)60, (byte)209, (byte)229, (byte)115, (byte)40, (byte)249, (byte)211, (byte)128, (byte)180, (byte)216, (byte)141, (byte)218, (byte)200, (byte)53, (byte)112, (byte)31, (byte)153, (byte)64, (byte)34, (byte)175, (byte)198, (byte)207, (byte)52, (byte)25, (byte)39, (byte)215, (byte)74, (byte)241, (byte)0, (byte)184, (byte)18, (byte)249, (byte)80, (byte)162, (byte)110, (byte)187, (byte)8, (byte)65, (byte)30, (byte)156, (byte)128, (byte)190, (byte)92, (byte)223, (byte)163, (byte)101, (byte)49, (byte)6, (byte)97, (byte)34, (byte)9, (byte)248, (byte)221, (byte)176}));
                Debug.Assert(pack.target_system == (byte)(byte)162);
                Debug.Assert(pack.target_component == (byte)(byte)141);
            };
            DemoDevice.FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)215;
            p110.target_system = (byte)(byte)162;
            p110.target_component = (byte)(byte)141;
            p110.payload_SET(new byte[] {(byte)96, (byte)156, (byte)58, (byte)26, (byte)186, (byte)147, (byte)179, (byte)77, (byte)5, (byte)225, (byte)51, (byte)127, (byte)209, (byte)224, (byte)221, (byte)81, (byte)41, (byte)246, (byte)58, (byte)79, (byte)181, (byte)32, (byte)167, (byte)205, (byte)18, (byte)136, (byte)88, (byte)239, (byte)34, (byte)65, (byte)86, (byte)117, (byte)245, (byte)76, (byte)240, (byte)211, (byte)172, (byte)17, (byte)104, (byte)91, (byte)165, (byte)176, (byte)158, (byte)58, (byte)75, (byte)60, (byte)49, (byte)136, (byte)233, (byte)56, (byte)181, (byte)0, (byte)89, (byte)67, (byte)112, (byte)83, (byte)148, (byte)248, (byte)135, (byte)236, (byte)251, (byte)217, (byte)218, (byte)34, (byte)155, (byte)77, (byte)92, (byte)44, (byte)241, (byte)169, (byte)179, (byte)46, (byte)156, (byte)54, (byte)183, (byte)172, (byte)1, (byte)138, (byte)58, (byte)175, (byte)127, (byte)70, (byte)178, (byte)148, (byte)146, (byte)27, (byte)198, (byte)237, (byte)191, (byte)36, (byte)184, (byte)163, (byte)33, (byte)133, (byte)132, (byte)119, (byte)217, (byte)178, (byte)24, (byte)81, (byte)38, (byte)129, (byte)173, (byte)112, (byte)208, (byte)197, (byte)115, (byte)219, (byte)38, (byte)196, (byte)219, (byte)229, (byte)251, (byte)100, (byte)53, (byte)127, (byte)6, (byte)146, (byte)44, (byte)80, (byte)92, (byte)30, (byte)121, (byte)125, (byte)76, (byte)250, (byte)238, (byte)145, (byte)136, (byte)72, (byte)37, (byte)77, (byte)57, (byte)33, (byte)163, (byte)17, (byte)111, (byte)42, (byte)89, (byte)220, (byte)2, (byte)15, (byte)191, (byte)87, (byte)181, (byte)218, (byte)111, (byte)12, (byte)8, (byte)183, (byte)201, (byte)201, (byte)49, (byte)52, (byte)50, (byte)140, (byte)9, (byte)255, (byte)26, (byte)238, (byte)191, (byte)22, (byte)185, (byte)214, (byte)19, (byte)186, (byte)246, (byte)26, (byte)208, (byte)162, (byte)146, (byte)185, (byte)148, (byte)73, (byte)93, (byte)66, (byte)42, (byte)238, (byte)175, (byte)94, (byte)147, (byte)83, (byte)224, (byte)247, (byte)142, (byte)226, (byte)4, (byte)67, (byte)177, (byte)21, (byte)36, (byte)60, (byte)3, (byte)246, (byte)228, (byte)68, (byte)86, (byte)60, (byte)209, (byte)229, (byte)115, (byte)40, (byte)249, (byte)211, (byte)128, (byte)180, (byte)216, (byte)141, (byte)218, (byte)200, (byte)53, (byte)112, (byte)31, (byte)153, (byte)64, (byte)34, (byte)175, (byte)198, (byte)207, (byte)52, (byte)25, (byte)39, (byte)215, (byte)74, (byte)241, (byte)0, (byte)184, (byte)18, (byte)249, (byte)80, (byte)162, (byte)110, (byte)187, (byte)8, (byte)65, (byte)30, (byte)156, (byte)128, (byte)190, (byte)92, (byte)223, (byte)163, (byte)101, (byte)49, (byte)6, (byte)97, (byte)34, (byte)9, (byte)248, (byte)221, (byte)176}, 0) ;
            LoopBackDemoChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tc1 == (long)3981910442451021766L);
                Debug.Assert(pack.ts1 == (long)8872752501079234011L);
            };
            DemoDevice.TIMESYNC p111 = LoopBackDemoChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.ts1 = (long)8872752501079234011L;
            p111.tc1 = (long)3981910442451021766L;
            LoopBackDemoChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)6795830610302622259L);
                Debug.Assert(pack.seq == (uint)1069670529U);
            };
            DemoDevice.CAMERA_TRIGGER p112 = LoopBackDemoChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)6795830610302622259L;
            p112.seq = (uint)1069670529U;
            LoopBackDemoChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)1566933152);
                Debug.Assert(pack.eph == (ushort)(ushort)14518);
                Debug.Assert(pack.time_usec == (ulong)7711269567214774670L);
                Debug.Assert(pack.epv == (ushort)(ushort)63151);
                Debug.Assert(pack.cog == (ushort)(ushort)54803);
                Debug.Assert(pack.vn == (short)(short) -11810);
                Debug.Assert(pack.alt == (int) -659068259);
                Debug.Assert(pack.ve == (short)(short) -17867);
                Debug.Assert(pack.vel == (ushort)(ushort)51971);
                Debug.Assert(pack.vd == (short)(short) -22107);
                Debug.Assert(pack.fix_type == (byte)(byte)95);
                Debug.Assert(pack.lat == (int)919640814);
                Debug.Assert(pack.satellites_visible == (byte)(byte)191);
            };
            DemoDevice.HIL_GPS p113 = LoopBackDemoChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.cog = (ushort)(ushort)54803;
            p113.vel = (ushort)(ushort)51971;
            p113.time_usec = (ulong)7711269567214774670L;
            p113.alt = (int) -659068259;
            p113.satellites_visible = (byte)(byte)191;
            p113.eph = (ushort)(ushort)14518;
            p113.ve = (short)(short) -17867;
            p113.vn = (short)(short) -11810;
            p113.epv = (ushort)(ushort)63151;
            p113.lat = (int)919640814;
            p113.lon = (int)1566933152;
            p113.vd = (short)(short) -22107;
            p113.fix_type = (byte)(byte)95;
            LoopBackDemoChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_y == (float)4.7516833E37F);
                Debug.Assert(pack.temperature == (short)(short) -11042);
                Debug.Assert(pack.time_usec == (ulong)1875258133694228903L);
                Debug.Assert(pack.integration_time_us == (uint)3207731447U);
                Debug.Assert(pack.integrated_xgyro == (float) -5.2438383E36F);
                Debug.Assert(pack.quality == (byte)(byte)191);
                Debug.Assert(pack.time_delta_distance_us == (uint)257436203U);
                Debug.Assert(pack.distance == (float)8.3143423E37F);
                Debug.Assert(pack.integrated_x == (float)9.70586E37F);
                Debug.Assert(pack.integrated_zgyro == (float)2.551386E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)139);
                Debug.Assert(pack.integrated_ygyro == (float) -1.0501668E38F);
            };
            DemoDevice.HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.quality = (byte)(byte)191;
            p114.sensor_id = (byte)(byte)139;
            p114.integration_time_us = (uint)3207731447U;
            p114.integrated_ygyro = (float) -1.0501668E38F;
            p114.integrated_x = (float)9.70586E37F;
            p114.temperature = (short)(short) -11042;
            p114.time_delta_distance_us = (uint)257436203U;
            p114.time_usec = (ulong)1875258133694228903L;
            p114.integrated_zgyro = (float)2.551386E38F;
            p114.integrated_xgyro = (float) -5.2438383E36F;
            p114.integrated_y = (float)4.7516833E37F;
            p114.distance = (float)8.3143423E37F;
            LoopBackDemoChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (short)(short) -20243);
                Debug.Assert(pack.yawspeed == (float) -1.5747145E38F);
                Debug.Assert(pack.zacc == (short)(short)22798);
                Debug.Assert(pack.vz == (short)(short) -25512);
                Debug.Assert(pack.time_usec == (ulong)2684730392667336355L);
                Debug.Assert(pack.xacc == (short)(short)17189);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)25049);
                Debug.Assert(pack.lat == (int) -1568486493);
                Debug.Assert(pack.yacc == (short)(short)4562);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {2.8030187E38F, 1.1266419E38F, 2.2411493E38F, -8.6309576E36F}));
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)12783);
                Debug.Assert(pack.alt == (int)1336977296);
                Debug.Assert(pack.rollspeed == (float) -1.4759206E38F);
                Debug.Assert(pack.pitchspeed == (float) -2.7957939E38F);
                Debug.Assert(pack.lon == (int)999976847);
                Debug.Assert(pack.vx == (short)(short)24938);
            };
            DemoDevice.HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.lon = (int)999976847;
            p115.zacc = (short)(short)22798;
            p115.yawspeed = (float) -1.5747145E38F;
            p115.xacc = (short)(short)17189;
            p115.true_airspeed = (ushort)(ushort)12783;
            p115.vx = (short)(short)24938;
            p115.attitude_quaternion_SET(new float[] {2.8030187E38F, 1.1266419E38F, 2.2411493E38F, -8.6309576E36F}, 0) ;
            p115.yacc = (short)(short)4562;
            p115.pitchspeed = (float) -2.7957939E38F;
            p115.time_usec = (ulong)2684730392667336355L;
            p115.ind_airspeed = (ushort)(ushort)25049;
            p115.lat = (int) -1568486493;
            p115.rollspeed = (float) -1.4759206E38F;
            p115.vy = (short)(short) -20243;
            p115.alt = (int)1336977296;
            p115.vz = (short)(short) -25512;
            LoopBackDemoChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xmag == (short)(short) -841);
                Debug.Assert(pack.xgyro == (short)(short) -23904);
                Debug.Assert(pack.ymag == (short)(short) -2772);
                Debug.Assert(pack.time_boot_ms == (uint)5971069U);
                Debug.Assert(pack.zgyro == (short)(short)24745);
                Debug.Assert(pack.yacc == (short)(short)13229);
                Debug.Assert(pack.zmag == (short)(short) -32010);
                Debug.Assert(pack.xacc == (short)(short) -27964);
                Debug.Assert(pack.zacc == (short)(short)4774);
                Debug.Assert(pack.ygyro == (short)(short)32275);
            };
            DemoDevice.SCALED_IMU2 p116 = LoopBackDemoChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.zacc = (short)(short)4774;
            p116.ygyro = (short)(short)32275;
            p116.time_boot_ms = (uint)5971069U;
            p116.xmag = (short)(short) -841;
            p116.zgyro = (short)(short)24745;
            p116.xacc = (short)(short) -27964;
            p116.ymag = (short)(short) -2772;
            p116.yacc = (short)(short)13229;
            p116.xgyro = (short)(short) -23904;
            p116.zmag = (short)(short) -32010;
            LoopBackDemoChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)176);
                Debug.Assert(pack.start == (ushort)(ushort)44065);
                Debug.Assert(pack.end == (ushort)(ushort)44635);
                Debug.Assert(pack.target_component == (byte)(byte)25);
            };
            DemoDevice.LOG_REQUEST_LIST p117 = LoopBackDemoChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_system = (byte)(byte)176;
            p117.start = (ushort)(ushort)44065;
            p117.target_component = (byte)(byte)25;
            p117.end = (ushort)(ushort)44635;
            LoopBackDemoChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (ushort)(ushort)26186);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)45199);
                Debug.Assert(pack.num_logs == (ushort)(ushort)55963);
                Debug.Assert(pack.size == (uint)3551175276U);
                Debug.Assert(pack.time_utc == (uint)2935579552U);
            };
            DemoDevice.LOG_ENTRY p118 = LoopBackDemoChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.time_utc = (uint)2935579552U;
            p118.last_log_num = (ushort)(ushort)45199;
            p118.num_logs = (ushort)(ushort)55963;
            p118.size = (uint)3551175276U;
            p118.id = (ushort)(ushort)26186;
            LoopBackDemoChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)25);
                Debug.Assert(pack.target_component == (byte)(byte)185);
                Debug.Assert(pack.ofs == (uint)2161305028U);
                Debug.Assert(pack.count == (uint)2788358910U);
                Debug.Assert(pack.id == (ushort)(ushort)14984);
            };
            DemoDevice.LOG_REQUEST_DATA p119 = LoopBackDemoChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.count = (uint)2788358910U;
            p119.ofs = (uint)2161305028U;
            p119.id = (ushort)(ushort)14984;
            p119.target_system = (byte)(byte)25;
            p119.target_component = (byte)(byte)185;
            LoopBackDemoChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)197, (byte)92, (byte)129, (byte)216, (byte)161, (byte)83, (byte)249, (byte)12, (byte)55, (byte)192, (byte)210, (byte)172, (byte)205, (byte)157, (byte)202, (byte)36, (byte)17, (byte)128, (byte)86, (byte)144, (byte)71, (byte)234, (byte)99, (byte)246, (byte)171, (byte)192, (byte)175, (byte)87, (byte)103, (byte)52, (byte)164, (byte)240, (byte)230, (byte)84, (byte)208, (byte)32, (byte)157, (byte)109, (byte)206, (byte)104, (byte)189, (byte)105, (byte)25, (byte)74, (byte)19, (byte)119, (byte)63, (byte)4, (byte)14, (byte)65, (byte)15, (byte)70, (byte)143, (byte)72, (byte)240, (byte)184, (byte)57, (byte)101, (byte)10, (byte)233, (byte)12, (byte)30, (byte)126, (byte)210, (byte)98, (byte)21, (byte)230, (byte)10, (byte)126, (byte)28, (byte)186, (byte)199, (byte)105, (byte)192, (byte)85, (byte)232, (byte)165, (byte)181, (byte)192, (byte)23, (byte)36, (byte)218, (byte)67, (byte)132, (byte)197, (byte)223, (byte)52, (byte)248, (byte)213, (byte)143}));
                Debug.Assert(pack.ofs == (uint)1931261930U);
                Debug.Assert(pack.count == (byte)(byte)109);
                Debug.Assert(pack.id == (ushort)(ushort)55944);
            };
            DemoDevice.LOG_DATA p120 = LoopBackDemoChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)55944;
            p120.data__SET(new byte[] {(byte)197, (byte)92, (byte)129, (byte)216, (byte)161, (byte)83, (byte)249, (byte)12, (byte)55, (byte)192, (byte)210, (byte)172, (byte)205, (byte)157, (byte)202, (byte)36, (byte)17, (byte)128, (byte)86, (byte)144, (byte)71, (byte)234, (byte)99, (byte)246, (byte)171, (byte)192, (byte)175, (byte)87, (byte)103, (byte)52, (byte)164, (byte)240, (byte)230, (byte)84, (byte)208, (byte)32, (byte)157, (byte)109, (byte)206, (byte)104, (byte)189, (byte)105, (byte)25, (byte)74, (byte)19, (byte)119, (byte)63, (byte)4, (byte)14, (byte)65, (byte)15, (byte)70, (byte)143, (byte)72, (byte)240, (byte)184, (byte)57, (byte)101, (byte)10, (byte)233, (byte)12, (byte)30, (byte)126, (byte)210, (byte)98, (byte)21, (byte)230, (byte)10, (byte)126, (byte)28, (byte)186, (byte)199, (byte)105, (byte)192, (byte)85, (byte)232, (byte)165, (byte)181, (byte)192, (byte)23, (byte)36, (byte)218, (byte)67, (byte)132, (byte)197, (byte)223, (byte)52, (byte)248, (byte)213, (byte)143}, 0) ;
            p120.ofs = (uint)1931261930U;
            p120.count = (byte)(byte)109;
            LoopBackDemoChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)229);
                Debug.Assert(pack.target_component == (byte)(byte)169);
            };
            DemoDevice.LOG_ERASE p121 = LoopBackDemoChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_component = (byte)(byte)169;
            p121.target_system = (byte)(byte)229;
            LoopBackDemoChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)88);
                Debug.Assert(pack.target_component == (byte)(byte)191);
            };
            DemoDevice.LOG_REQUEST_END p122 = LoopBackDemoChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_component = (byte)(byte)191;
            p122.target_system = (byte)(byte)88;
            LoopBackDemoChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)117, (byte)236, (byte)160, (byte)153, (byte)60, (byte)168, (byte)54, (byte)68, (byte)12, (byte)16, (byte)253, (byte)163, (byte)48, (byte)217, (byte)113, (byte)156, (byte)87, (byte)252, (byte)120, (byte)112, (byte)133, (byte)243, (byte)3, (byte)211, (byte)201, (byte)79, (byte)144, (byte)242, (byte)147, (byte)185, (byte)224, (byte)162, (byte)254, (byte)145, (byte)88, (byte)161, (byte)27, (byte)18, (byte)69, (byte)226, (byte)28, (byte)207, (byte)119, (byte)159, (byte)198, (byte)59, (byte)96, (byte)10, (byte)191, (byte)174, (byte)154, (byte)125, (byte)122, (byte)128, (byte)88, (byte)26, (byte)72, (byte)82, (byte)203, (byte)220, (byte)170, (byte)57, (byte)57, (byte)235, (byte)236, (byte)133, (byte)149, (byte)51, (byte)242, (byte)151, (byte)192, (byte)1, (byte)222, (byte)204, (byte)52, (byte)236, (byte)212, (byte)24, (byte)70, (byte)24, (byte)225, (byte)99, (byte)150, (byte)198, (byte)63, (byte)75, (byte)93, (byte)157, (byte)226, (byte)242, (byte)248, (byte)32, (byte)134, (byte)0, (byte)163, (byte)232, (byte)169, (byte)34, (byte)214, (byte)83, (byte)214, (byte)189, (byte)62, (byte)31, (byte)246, (byte)233, (byte)76, (byte)75, (byte)208, (byte)232}));
                Debug.Assert(pack.target_system == (byte)(byte)233);
                Debug.Assert(pack.target_component == (byte)(byte)104);
                Debug.Assert(pack.len == (byte)(byte)159);
            };
            DemoDevice.GPS_INJECT_DATA p123 = LoopBackDemoChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)233;
            p123.target_component = (byte)(byte)104;
            p123.len = (byte)(byte)159;
            p123.data__SET(new byte[] {(byte)117, (byte)236, (byte)160, (byte)153, (byte)60, (byte)168, (byte)54, (byte)68, (byte)12, (byte)16, (byte)253, (byte)163, (byte)48, (byte)217, (byte)113, (byte)156, (byte)87, (byte)252, (byte)120, (byte)112, (byte)133, (byte)243, (byte)3, (byte)211, (byte)201, (byte)79, (byte)144, (byte)242, (byte)147, (byte)185, (byte)224, (byte)162, (byte)254, (byte)145, (byte)88, (byte)161, (byte)27, (byte)18, (byte)69, (byte)226, (byte)28, (byte)207, (byte)119, (byte)159, (byte)198, (byte)59, (byte)96, (byte)10, (byte)191, (byte)174, (byte)154, (byte)125, (byte)122, (byte)128, (byte)88, (byte)26, (byte)72, (byte)82, (byte)203, (byte)220, (byte)170, (byte)57, (byte)57, (byte)235, (byte)236, (byte)133, (byte)149, (byte)51, (byte)242, (byte)151, (byte)192, (byte)1, (byte)222, (byte)204, (byte)52, (byte)236, (byte)212, (byte)24, (byte)70, (byte)24, (byte)225, (byte)99, (byte)150, (byte)198, (byte)63, (byte)75, (byte)93, (byte)157, (byte)226, (byte)242, (byte)248, (byte)32, (byte)134, (byte)0, (byte)163, (byte)232, (byte)169, (byte)34, (byte)214, (byte)83, (byte)214, (byte)189, (byte)62, (byte)31, (byte)246, (byte)233, (byte)76, (byte)75, (byte)208, (byte)232}, 0) ;
            LoopBackDemoChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.epv == (ushort)(ushort)33649);
                Debug.Assert(pack.lat == (int)94425818);
                Debug.Assert(pack.time_usec == (ulong)6909989796358044248L);
                Debug.Assert(pack.lon == (int) -1706801138);
                Debug.Assert(pack.cog == (ushort)(ushort)6485);
                Debug.Assert(pack.eph == (ushort)(ushort)11957);
                Debug.Assert(pack.satellites_visible == (byte)(byte)132);
                Debug.Assert(pack.vel == (ushort)(ushort)44831);
                Debug.Assert(pack.alt == (int) -1202543901);
                Debug.Assert(pack.dgps_numch == (byte)(byte)181);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
                Debug.Assert(pack.dgps_age == (uint)2124178917U);
            };
            DemoDevice.GPS2_RAW p124 = LoopBackDemoChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.eph = (ushort)(ushort)11957;
            p124.cog = (ushort)(ushort)6485;
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p124.dgps_numch = (byte)(byte)181;
            p124.lon = (int) -1706801138;
            p124.dgps_age = (uint)2124178917U;
            p124.alt = (int) -1202543901;
            p124.lat = (int)94425818;
            p124.vel = (ushort)(ushort)44831;
            p124.satellites_visible = (byte)(byte)132;
            p124.time_usec = (ulong)6909989796358044248L;
            p124.epv = (ushort)(ushort)33649;
            LoopBackDemoChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Vcc == (ushort)(ushort)6560);
                Debug.Assert(pack.flags == (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT);
                Debug.Assert(pack.Vservo == (ushort)(ushort)4041);
            };
            DemoDevice.POWER_STATUS p125 = LoopBackDemoChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT;
            p125.Vservo = (ushort)(ushort)4041;
            p125.Vcc = (ushort)(ushort)6560;
            LoopBackDemoChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (byte)(byte)75);
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE);
                Debug.Assert(pack.timeout == (ushort)(ushort)58982);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)33, (byte)16, (byte)114, (byte)48, (byte)118, (byte)117, (byte)197, (byte)215, (byte)180, (byte)233, (byte)129, (byte)70, (byte)228, (byte)28, (byte)115, (byte)11, (byte)22, (byte)130, (byte)215, (byte)254, (byte)117, (byte)60, (byte)234, (byte)200, (byte)126, (byte)97, (byte)53, (byte)216, (byte)9, (byte)205, (byte)146, (byte)95, (byte)26, (byte)225, (byte)149, (byte)244, (byte)9, (byte)254, (byte)238, (byte)61, (byte)94, (byte)45, (byte)247, (byte)188, (byte)253, (byte)33, (byte)237, (byte)212, (byte)123, (byte)21, (byte)215, (byte)180, (byte)118, (byte)172, (byte)173, (byte)34, (byte)163, (byte)213, (byte)218, (byte)166, (byte)233, (byte)78, (byte)88, (byte)65, (byte)18, (byte)173, (byte)243, (byte)17, (byte)6, (byte)203}));
                Debug.Assert(pack.baudrate == (uint)3095189829U);
                Debug.Assert(pack.device == (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1);
            };
            DemoDevice.SERIAL_CONTROL p126 = LoopBackDemoChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.timeout = (ushort)(ushort)58982;
            p126.count = (byte)(byte)75;
            p126.baudrate = (uint)3095189829U;
            p126.data__SET(new byte[] {(byte)33, (byte)16, (byte)114, (byte)48, (byte)118, (byte)117, (byte)197, (byte)215, (byte)180, (byte)233, (byte)129, (byte)70, (byte)228, (byte)28, (byte)115, (byte)11, (byte)22, (byte)130, (byte)215, (byte)254, (byte)117, (byte)60, (byte)234, (byte)200, (byte)126, (byte)97, (byte)53, (byte)216, (byte)9, (byte)205, (byte)146, (byte)95, (byte)26, (byte)225, (byte)149, (byte)244, (byte)9, (byte)254, (byte)238, (byte)61, (byte)94, (byte)45, (byte)247, (byte)188, (byte)253, (byte)33, (byte)237, (byte)212, (byte)123, (byte)21, (byte)215, (byte)180, (byte)118, (byte)172, (byte)173, (byte)34, (byte)163, (byte)213, (byte)218, (byte)166, (byte)233, (byte)78, (byte)88, (byte)65, (byte)18, (byte)173, (byte)243, (byte)17, (byte)6, (byte)203}, 0) ;
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1;
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE;
            LoopBackDemoChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_last_baseline_ms == (uint)3877931638U);
                Debug.Assert(pack.accuracy == (uint)2456595822U);
                Debug.Assert(pack.nsats == (byte)(byte)253);
                Debug.Assert(pack.tow == (uint)1045133335U);
                Debug.Assert(pack.baseline_b_mm == (int) -1980845368);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)57);
                Debug.Assert(pack.iar_num_hypotheses == (int) -799172543);
                Debug.Assert(pack.rtk_rate == (byte)(byte)38);
                Debug.Assert(pack.rtk_health == (byte)(byte)63);
                Debug.Assert(pack.baseline_a_mm == (int)1711525426);
                Debug.Assert(pack.wn == (ushort)(ushort)54212);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)242);
                Debug.Assert(pack.baseline_c_mm == (int)2025478498);
            };
            DemoDevice.GPS_RTK p127 = LoopBackDemoChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.accuracy = (uint)2456595822U;
            p127.baseline_b_mm = (int) -1980845368;
            p127.rtk_rate = (byte)(byte)38;
            p127.baseline_c_mm = (int)2025478498;
            p127.time_last_baseline_ms = (uint)3877931638U;
            p127.baseline_a_mm = (int)1711525426;
            p127.rtk_health = (byte)(byte)63;
            p127.nsats = (byte)(byte)253;
            p127.tow = (uint)1045133335U;
            p127.rtk_receiver_id = (byte)(byte)242;
            p127.baseline_coords_type = (byte)(byte)57;
            p127.iar_num_hypotheses = (int) -799172543;
            p127.wn = (ushort)(ushort)54212;
            LoopBackDemoChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baseline_b_mm == (int)793866306);
                Debug.Assert(pack.time_last_baseline_ms == (uint)203248719U);
                Debug.Assert(pack.baseline_c_mm == (int)1999485974);
                Debug.Assert(pack.rtk_rate == (byte)(byte)71);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)156);
                Debug.Assert(pack.rtk_health == (byte)(byte)35);
                Debug.Assert(pack.tow == (uint)217941790U);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)236);
                Debug.Assert(pack.baseline_a_mm == (int)1674919593);
                Debug.Assert(pack.accuracy == (uint)1046126936U);
                Debug.Assert(pack.nsats == (byte)(byte)107);
                Debug.Assert(pack.iar_num_hypotheses == (int) -414012249);
                Debug.Assert(pack.wn == (ushort)(ushort)51395);
            };
            DemoDevice.GPS2_RTK p128 = LoopBackDemoChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.iar_num_hypotheses = (int) -414012249;
            p128.baseline_a_mm = (int)1674919593;
            p128.baseline_b_mm = (int)793866306;
            p128.time_last_baseline_ms = (uint)203248719U;
            p128.wn = (ushort)(ushort)51395;
            p128.rtk_rate = (byte)(byte)71;
            p128.accuracy = (uint)1046126936U;
            p128.tow = (uint)217941790U;
            p128.baseline_c_mm = (int)1999485974;
            p128.baseline_coords_type = (byte)(byte)156;
            p128.rtk_receiver_id = (byte)(byte)236;
            p128.nsats = (byte)(byte)107;
            p128.rtk_health = (byte)(byte)35;
            LoopBackDemoChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zacc == (short)(short) -23775);
                Debug.Assert(pack.xacc == (short)(short)5753);
                Debug.Assert(pack.ygyro == (short)(short)11125);
                Debug.Assert(pack.xgyro == (short)(short)11030);
                Debug.Assert(pack.yacc == (short)(short)10004);
                Debug.Assert(pack.ymag == (short)(short)13307);
                Debug.Assert(pack.time_boot_ms == (uint)1059134212U);
                Debug.Assert(pack.zmag == (short)(short) -26378);
                Debug.Assert(pack.xmag == (short)(short) -14069);
                Debug.Assert(pack.zgyro == (short)(short) -17867);
            };
            DemoDevice.SCALED_IMU3 p129 = LoopBackDemoChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.xgyro = (short)(short)11030;
            p129.zmag = (short)(short) -26378;
            p129.xmag = (short)(short) -14069;
            p129.zgyro = (short)(short) -17867;
            p129.ymag = (short)(short)13307;
            p129.ygyro = (short)(short)11125;
            p129.zacc = (short)(short) -23775;
            p129.time_boot_ms = (uint)1059134212U;
            p129.xacc = (short)(short)5753;
            p129.yacc = (short)(short)10004;
            LoopBackDemoChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.height == (ushort)(ushort)2016);
                Debug.Assert(pack.jpg_quality == (byte)(byte)168);
                Debug.Assert(pack.payload == (byte)(byte)113);
                Debug.Assert(pack.type == (byte)(byte)255);
                Debug.Assert(pack.size == (uint)138056494U);
                Debug.Assert(pack.width == (ushort)(ushort)48787);
                Debug.Assert(pack.packets == (ushort)(ushort)6792);
            };
            DemoDevice.DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.width = (ushort)(ushort)48787;
            p130.payload = (byte)(byte)113;
            p130.type = (byte)(byte)255;
            p130.packets = (ushort)(ushort)6792;
            p130.jpg_quality = (byte)(byte)168;
            p130.height = (ushort)(ushort)2016;
            p130.size = (uint)138056494U;
            LoopBackDemoChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seqnr == (ushort)(ushort)40015);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)30, (byte)240, (byte)32, (byte)4, (byte)183, (byte)166, (byte)63, (byte)245, (byte)99, (byte)71, (byte)195, (byte)24, (byte)179, (byte)151, (byte)168, (byte)122, (byte)187, (byte)106, (byte)193, (byte)195, (byte)84, (byte)10, (byte)202, (byte)75, (byte)20, (byte)229, (byte)128, (byte)13, (byte)63, (byte)52, (byte)103, (byte)34, (byte)199, (byte)10, (byte)52, (byte)87, (byte)188, (byte)43, (byte)97, (byte)98, (byte)22, (byte)19, (byte)8, (byte)236, (byte)223, (byte)155, (byte)162, (byte)46, (byte)244, (byte)149, (byte)207, (byte)8, (byte)128, (byte)57, (byte)169, (byte)255, (byte)67, (byte)231, (byte)25, (byte)202, (byte)67, (byte)219, (byte)45, (byte)56, (byte)233, (byte)166, (byte)0, (byte)31, (byte)58, (byte)255, (byte)208, (byte)105, (byte)92, (byte)23, (byte)158, (byte)24, (byte)140, (byte)27, (byte)136, (byte)235, (byte)198, (byte)208, (byte)144, (byte)168, (byte)222, (byte)40, (byte)233, (byte)60, (byte)98, (byte)249, (byte)196, (byte)16, (byte)146, (byte)24, (byte)199, (byte)93, (byte)10, (byte)88, (byte)59, (byte)95, (byte)161, (byte)51, (byte)44, (byte)222, (byte)65, (byte)138, (byte)94, (byte)87, (byte)156, (byte)110, (byte)85, (byte)82, (byte)11, (byte)179, (byte)78, (byte)4, (byte)156, (byte)214, (byte)224, (byte)103, (byte)163, (byte)38, (byte)188, (byte)185, (byte)228, (byte)39, (byte)14, (byte)171, (byte)5, (byte)199, (byte)251, (byte)249, (byte)200, (byte)38, (byte)16, (byte)40, (byte)248, (byte)202, (byte)61, (byte)34, (byte)171, (byte)4, (byte)139, (byte)135, (byte)98, (byte)222, (byte)120, (byte)74, (byte)32, (byte)43, (byte)237, (byte)41, (byte)86, (byte)169, (byte)250, (byte)29, (byte)238, (byte)222, (byte)239, (byte)111, (byte)22, (byte)60, (byte)204, (byte)126, (byte)241, (byte)57, (byte)234, (byte)220, (byte)194, (byte)151, (byte)211, (byte)147, (byte)106, (byte)164, (byte)244, (byte)244, (byte)11, (byte)14, (byte)242, (byte)70, (byte)69, (byte)53, (byte)230, (byte)232, (byte)49, (byte)95, (byte)182, (byte)239, (byte)155, (byte)217, (byte)98, (byte)198, (byte)197, (byte)133, (byte)79, (byte)40, (byte)35, (byte)134, (byte)80, (byte)16, (byte)139, (byte)84, (byte)115, (byte)24, (byte)148, (byte)43, (byte)196, (byte)34, (byte)168, (byte)43, (byte)217, (byte)125, (byte)66, (byte)197, (byte)82, (byte)30, (byte)27, (byte)230, (byte)1, (byte)222, (byte)179, (byte)159, (byte)211, (byte)224, (byte)67, (byte)89, (byte)54, (byte)198, (byte)188, (byte)251, (byte)135, (byte)34, (byte)176, (byte)110, (byte)29, (byte)87, (byte)179, (byte)205, (byte)3, (byte)13, (byte)122, (byte)195, (byte)215, (byte)203, (byte)101, (byte)51, (byte)53, (byte)55, (byte)70, (byte)226, (byte)122, (byte)168, (byte)173}));
            };
            DemoDevice.ENCAPSULATED_DATA p131 = LoopBackDemoChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.data__SET(new byte[] {(byte)30, (byte)240, (byte)32, (byte)4, (byte)183, (byte)166, (byte)63, (byte)245, (byte)99, (byte)71, (byte)195, (byte)24, (byte)179, (byte)151, (byte)168, (byte)122, (byte)187, (byte)106, (byte)193, (byte)195, (byte)84, (byte)10, (byte)202, (byte)75, (byte)20, (byte)229, (byte)128, (byte)13, (byte)63, (byte)52, (byte)103, (byte)34, (byte)199, (byte)10, (byte)52, (byte)87, (byte)188, (byte)43, (byte)97, (byte)98, (byte)22, (byte)19, (byte)8, (byte)236, (byte)223, (byte)155, (byte)162, (byte)46, (byte)244, (byte)149, (byte)207, (byte)8, (byte)128, (byte)57, (byte)169, (byte)255, (byte)67, (byte)231, (byte)25, (byte)202, (byte)67, (byte)219, (byte)45, (byte)56, (byte)233, (byte)166, (byte)0, (byte)31, (byte)58, (byte)255, (byte)208, (byte)105, (byte)92, (byte)23, (byte)158, (byte)24, (byte)140, (byte)27, (byte)136, (byte)235, (byte)198, (byte)208, (byte)144, (byte)168, (byte)222, (byte)40, (byte)233, (byte)60, (byte)98, (byte)249, (byte)196, (byte)16, (byte)146, (byte)24, (byte)199, (byte)93, (byte)10, (byte)88, (byte)59, (byte)95, (byte)161, (byte)51, (byte)44, (byte)222, (byte)65, (byte)138, (byte)94, (byte)87, (byte)156, (byte)110, (byte)85, (byte)82, (byte)11, (byte)179, (byte)78, (byte)4, (byte)156, (byte)214, (byte)224, (byte)103, (byte)163, (byte)38, (byte)188, (byte)185, (byte)228, (byte)39, (byte)14, (byte)171, (byte)5, (byte)199, (byte)251, (byte)249, (byte)200, (byte)38, (byte)16, (byte)40, (byte)248, (byte)202, (byte)61, (byte)34, (byte)171, (byte)4, (byte)139, (byte)135, (byte)98, (byte)222, (byte)120, (byte)74, (byte)32, (byte)43, (byte)237, (byte)41, (byte)86, (byte)169, (byte)250, (byte)29, (byte)238, (byte)222, (byte)239, (byte)111, (byte)22, (byte)60, (byte)204, (byte)126, (byte)241, (byte)57, (byte)234, (byte)220, (byte)194, (byte)151, (byte)211, (byte)147, (byte)106, (byte)164, (byte)244, (byte)244, (byte)11, (byte)14, (byte)242, (byte)70, (byte)69, (byte)53, (byte)230, (byte)232, (byte)49, (byte)95, (byte)182, (byte)239, (byte)155, (byte)217, (byte)98, (byte)198, (byte)197, (byte)133, (byte)79, (byte)40, (byte)35, (byte)134, (byte)80, (byte)16, (byte)139, (byte)84, (byte)115, (byte)24, (byte)148, (byte)43, (byte)196, (byte)34, (byte)168, (byte)43, (byte)217, (byte)125, (byte)66, (byte)197, (byte)82, (byte)30, (byte)27, (byte)230, (byte)1, (byte)222, (byte)179, (byte)159, (byte)211, (byte)224, (byte)67, (byte)89, (byte)54, (byte)198, (byte)188, (byte)251, (byte)135, (byte)34, (byte)176, (byte)110, (byte)29, (byte)87, (byte)179, (byte)205, (byte)3, (byte)13, (byte)122, (byte)195, (byte)215, (byte)203, (byte)101, (byte)51, (byte)53, (byte)55, (byte)70, (byte)226, (byte)122, (byte)168, (byte)173}, 0) ;
            p131.seqnr = (ushort)(ushort)40015;
            LoopBackDemoChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.covariance == (byte)(byte)142);
                Debug.Assert(pack.orientation == (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_225);
                Debug.Assert(pack.min_distance == (ushort)(ushort)20799);
                Debug.Assert(pack.time_boot_ms == (uint)3574994318U);
                Debug.Assert(pack.max_distance == (ushort)(ushort)32184);
                Debug.Assert(pack.type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
                Debug.Assert(pack.current_distance == (ushort)(ushort)15720);
                Debug.Assert(pack.id == (byte)(byte)187);
            };
            DemoDevice.DISTANCE_SENSOR p132 = LoopBackDemoChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.current_distance = (ushort)(ushort)15720;
            p132.max_distance = (ushort)(ushort)32184;
            p132.id = (byte)(byte)187;
            p132.time_boot_ms = (uint)3574994318U;
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_225;
            p132.covariance = (byte)(byte)142;
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR;
            p132.min_distance = (ushort)(ushort)20799;
            LoopBackDemoChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)19195);
                Debug.Assert(pack.mask == (ulong)3464417612024904594L);
                Debug.Assert(pack.lon == (int) -525866952);
                Debug.Assert(pack.lat == (int) -15552693);
            };
            DemoDevice.TERRAIN_REQUEST p133 = LoopBackDemoChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.mask = (ulong)3464417612024904594L;
            p133.lat = (int) -15552693;
            p133.grid_spacing = (ushort)(ushort)19195;
            p133.lon = (int) -525866952;
            LoopBackDemoChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gridbit == (byte)(byte)158);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short) -5850, (short) -22974, (short) -15839, (short)28058, (short) -15662, (short)24859, (short) -10465, (short)14433, (short)12452, (short)10100, (short)30780, (short) -29702, (short)19494, (short) -15307, (short)23908, (short) -29837}));
                Debug.Assert(pack.lon == (int) -950735228);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)46709);
                Debug.Assert(pack.lat == (int) -215242989);
            };
            DemoDevice.TERRAIN_DATA p134 = LoopBackDemoChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int) -215242989;
            p134.data__SET(new short[] {(short) -5850, (short) -22974, (short) -15839, (short)28058, (short) -15662, (short)24859, (short) -10465, (short)14433, (short)12452, (short)10100, (short)30780, (short) -29702, (short)19494, (short) -15307, (short)23908, (short) -29837}, 0) ;
            p134.lon = (int) -950735228;
            p134.grid_spacing = (ushort)(ushort)46709;
            p134.gridbit = (byte)(byte)158;
            LoopBackDemoChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)2006050664);
                Debug.Assert(pack.lat == (int) -796071280);
            };
            DemoDevice.TERRAIN_CHECK p135 = LoopBackDemoChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lon = (int)2006050664;
            p135.lat = (int) -796071280;
            LoopBackDemoChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.terrain_height == (float)4.282422E37F);
                Debug.Assert(pack.pending == (ushort)(ushort)10853);
                Debug.Assert(pack.lat == (int)672311045);
                Debug.Assert(pack.spacing == (ushort)(ushort)56123);
                Debug.Assert(pack.loaded == (ushort)(ushort)741);
                Debug.Assert(pack.lon == (int) -1311307326);
                Debug.Assert(pack.current_height == (float) -1.3700029E38F);
            };
            DemoDevice.TERRAIN_REPORT p136 = LoopBackDemoChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.terrain_height = (float)4.282422E37F;
            p136.loaded = (ushort)(ushort)741;
            p136.lat = (int)672311045;
            p136.spacing = (ushort)(ushort)56123;
            p136.pending = (ushort)(ushort)10853;
            p136.lon = (int) -1311307326;
            p136.current_height = (float) -1.3700029E38F;
            LoopBackDemoChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff == (float) -7.4705413E37F);
                Debug.Assert(pack.time_boot_ms == (uint)994844504U);
                Debug.Assert(pack.temperature == (short)(short)3638);
                Debug.Assert(pack.press_abs == (float) -2.939732E38F);
            };
            DemoDevice.SCALED_PRESSURE2 p137 = LoopBackDemoChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.temperature = (short)(short)3638;
            p137.press_diff = (float) -7.4705413E37F;
            p137.time_boot_ms = (uint)994844504U;
            p137.press_abs = (float) -2.939732E38F;
            LoopBackDemoChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)7843764236385152246L);
                Debug.Assert(pack.z == (float) -2.3912156E38F);
                Debug.Assert(pack.y == (float)2.6442044E38F);
                Debug.Assert(pack.x == (float) -3.047628E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.3718128E37F, 2.588202E38F, -6.873196E37F, 7.982084E37F}));
            };
            DemoDevice.ATT_POS_MOCAP p138 = LoopBackDemoChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.y = (float)2.6442044E38F;
            p138.z = (float) -2.3912156E38F;
            p138.q_SET(new float[] {-1.3718128E37F, 2.588202E38F, -6.873196E37F, 7.982084E37F}, 0) ;
            p138.x = (float) -3.047628E38F;
            p138.time_usec = (ulong)7843764236385152246L;
            LoopBackDemoChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.group_mlx == (byte)(byte)206);
                Debug.Assert(pack.target_system == (byte)(byte)195);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {4.5955006E37F, 1.8986495E38F, 2.5957166E38F, 1.4975649E38F, 1.3474639E38F, -2.5844796E38F, -1.7324717E38F, -1.6279718E38F}));
                Debug.Assert(pack.target_component == (byte)(byte)24);
                Debug.Assert(pack.time_usec == (ulong)615279208571474690L);
            };
            DemoDevice.SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.target_system = (byte)(byte)195;
            p139.controls_SET(new float[] {4.5955006E37F, 1.8986495E38F, 2.5957166E38F, 1.4975649E38F, 1.3474639E38F, -2.5844796E38F, -1.7324717E38F, -1.6279718E38F}, 0) ;
            p139.group_mlx = (byte)(byte)206;
            p139.target_component = (byte)(byte)24;
            p139.time_usec = (ulong)615279208571474690L;
            LoopBackDemoChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.group_mlx == (byte)(byte)255);
                Debug.Assert(pack.time_usec == (ulong)6178080586112561440L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {1.3691268E38F, -1.0840216E38F, -1.5778084E38F, -2.4779375E38F, 1.312057E38F, -2.0537986E38F, -1.7813937E38F, -1.1523496E38F}));
            };
            DemoDevice.ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)6178080586112561440L;
            p140.controls_SET(new float[] {1.3691268E38F, -1.0840216E38F, -1.5778084E38F, -2.4779375E38F, 1.312057E38F, -2.0537986E38F, -1.7813937E38F, -1.1523496E38F}, 0) ;
            p140.group_mlx = (byte)(byte)255;
            LoopBackDemoChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)3594735575078055460L);
                Debug.Assert(pack.altitude_monotonic == (float)5.6299015E37F);
                Debug.Assert(pack.altitude_relative == (float)3.1451732E38F);
                Debug.Assert(pack.altitude_terrain == (float)8.1310285E36F);
                Debug.Assert(pack.altitude_local == (float) -3.2049248E38F);
                Debug.Assert(pack.bottom_clearance == (float)3.0882553E38F);
                Debug.Assert(pack.altitude_amsl == (float)5.744638E37F);
            };
            DemoDevice.ALTITUDE p141 = LoopBackDemoChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.altitude_amsl = (float)5.744638E37F;
            p141.altitude_terrain = (float)8.1310285E36F;
            p141.time_usec = (ulong)3594735575078055460L;
            p141.bottom_clearance = (float)3.0882553E38F;
            p141.altitude_relative = (float)3.1451732E38F;
            p141.altitude_monotonic = (float)5.6299015E37F;
            p141.altitude_local = (float) -3.2049248E38F;
            LoopBackDemoChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uri_type == (byte)(byte)48);
                Debug.Assert(pack.transfer_type == (byte)(byte)30);
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)63, (byte)6, (byte)155, (byte)233, (byte)97, (byte)223, (byte)137, (byte)3, (byte)65, (byte)158, (byte)196, (byte)125, (byte)65, (byte)63, (byte)223, (byte)125, (byte)5, (byte)8, (byte)188, (byte)10, (byte)137, (byte)66, (byte)233, (byte)224, (byte)154, (byte)178, (byte)201, (byte)118, (byte)88, (byte)100, (byte)177, (byte)16, (byte)221, (byte)88, (byte)146, (byte)141, (byte)1, (byte)89, (byte)168, (byte)52, (byte)133, (byte)89, (byte)228, (byte)149, (byte)90, (byte)24, (byte)66, (byte)65, (byte)118, (byte)208, (byte)197, (byte)177, (byte)248, (byte)6, (byte)158, (byte)77, (byte)213, (byte)34, (byte)3, (byte)76, (byte)89, (byte)29, (byte)194, (byte)200, (byte)68, (byte)3, (byte)183, (byte)229, (byte)171, (byte)25, (byte)167, (byte)146, (byte)57, (byte)165, (byte)54, (byte)235, (byte)32, (byte)79, (byte)68, (byte)212, (byte)16, (byte)89, (byte)38, (byte)26, (byte)158, (byte)47, (byte)128, (byte)199, (byte)228, (byte)8, (byte)44, (byte)155, (byte)219, (byte)84, (byte)200, (byte)18, (byte)127, (byte)34, (byte)110, (byte)140, (byte)167, (byte)131, (byte)140, (byte)243, (byte)27, (byte)38, (byte)191, (byte)115, (byte)217, (byte)253, (byte)40, (byte)106, (byte)174, (byte)29, (byte)153, (byte)127, (byte)21, (byte)172, (byte)203, (byte)117}));
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)184, (byte)207, (byte)227, (byte)137, (byte)186, (byte)120, (byte)36, (byte)180, (byte)80, (byte)37, (byte)228, (byte)171, (byte)255, (byte)130, (byte)248, (byte)97, (byte)236, (byte)196, (byte)23, (byte)210, (byte)58, (byte)3, (byte)150, (byte)32, (byte)156, (byte)209, (byte)187, (byte)122, (byte)213, (byte)31, (byte)186, (byte)70, (byte)236, (byte)24, (byte)141, (byte)103, (byte)162, (byte)82, (byte)137, (byte)238, (byte)234, (byte)62, (byte)111, (byte)216, (byte)106, (byte)101, (byte)134, (byte)201, (byte)209, (byte)17, (byte)237, (byte)117, (byte)48, (byte)20, (byte)156, (byte)236, (byte)27, (byte)253, (byte)177, (byte)165, (byte)181, (byte)116, (byte)64, (byte)222, (byte)249, (byte)178, (byte)215, (byte)244, (byte)110, (byte)124, (byte)94, (byte)147, (byte)18, (byte)18, (byte)147, (byte)171, (byte)26, (byte)182, (byte)176, (byte)179, (byte)128, (byte)242, (byte)26, (byte)24, (byte)26, (byte)138, (byte)102, (byte)240, (byte)144, (byte)39, (byte)157, (byte)216, (byte)107, (byte)158, (byte)14, (byte)17, (byte)93, (byte)103, (byte)152, (byte)38, (byte)116, (byte)105, (byte)238, (byte)242, (byte)79, (byte)176, (byte)29, (byte)192, (byte)225, (byte)178, (byte)82, (byte)232, (byte)255, (byte)8, (byte)163, (byte)50, (byte)90, (byte)23, (byte)245, (byte)188}));
                Debug.Assert(pack.request_id == (byte)(byte)26);
            };
            DemoDevice.RESOURCE_REQUEST p142 = LoopBackDemoChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.uri_SET(new byte[] {(byte)184, (byte)207, (byte)227, (byte)137, (byte)186, (byte)120, (byte)36, (byte)180, (byte)80, (byte)37, (byte)228, (byte)171, (byte)255, (byte)130, (byte)248, (byte)97, (byte)236, (byte)196, (byte)23, (byte)210, (byte)58, (byte)3, (byte)150, (byte)32, (byte)156, (byte)209, (byte)187, (byte)122, (byte)213, (byte)31, (byte)186, (byte)70, (byte)236, (byte)24, (byte)141, (byte)103, (byte)162, (byte)82, (byte)137, (byte)238, (byte)234, (byte)62, (byte)111, (byte)216, (byte)106, (byte)101, (byte)134, (byte)201, (byte)209, (byte)17, (byte)237, (byte)117, (byte)48, (byte)20, (byte)156, (byte)236, (byte)27, (byte)253, (byte)177, (byte)165, (byte)181, (byte)116, (byte)64, (byte)222, (byte)249, (byte)178, (byte)215, (byte)244, (byte)110, (byte)124, (byte)94, (byte)147, (byte)18, (byte)18, (byte)147, (byte)171, (byte)26, (byte)182, (byte)176, (byte)179, (byte)128, (byte)242, (byte)26, (byte)24, (byte)26, (byte)138, (byte)102, (byte)240, (byte)144, (byte)39, (byte)157, (byte)216, (byte)107, (byte)158, (byte)14, (byte)17, (byte)93, (byte)103, (byte)152, (byte)38, (byte)116, (byte)105, (byte)238, (byte)242, (byte)79, (byte)176, (byte)29, (byte)192, (byte)225, (byte)178, (byte)82, (byte)232, (byte)255, (byte)8, (byte)163, (byte)50, (byte)90, (byte)23, (byte)245, (byte)188}, 0) ;
            p142.storage_SET(new byte[] {(byte)63, (byte)6, (byte)155, (byte)233, (byte)97, (byte)223, (byte)137, (byte)3, (byte)65, (byte)158, (byte)196, (byte)125, (byte)65, (byte)63, (byte)223, (byte)125, (byte)5, (byte)8, (byte)188, (byte)10, (byte)137, (byte)66, (byte)233, (byte)224, (byte)154, (byte)178, (byte)201, (byte)118, (byte)88, (byte)100, (byte)177, (byte)16, (byte)221, (byte)88, (byte)146, (byte)141, (byte)1, (byte)89, (byte)168, (byte)52, (byte)133, (byte)89, (byte)228, (byte)149, (byte)90, (byte)24, (byte)66, (byte)65, (byte)118, (byte)208, (byte)197, (byte)177, (byte)248, (byte)6, (byte)158, (byte)77, (byte)213, (byte)34, (byte)3, (byte)76, (byte)89, (byte)29, (byte)194, (byte)200, (byte)68, (byte)3, (byte)183, (byte)229, (byte)171, (byte)25, (byte)167, (byte)146, (byte)57, (byte)165, (byte)54, (byte)235, (byte)32, (byte)79, (byte)68, (byte)212, (byte)16, (byte)89, (byte)38, (byte)26, (byte)158, (byte)47, (byte)128, (byte)199, (byte)228, (byte)8, (byte)44, (byte)155, (byte)219, (byte)84, (byte)200, (byte)18, (byte)127, (byte)34, (byte)110, (byte)140, (byte)167, (byte)131, (byte)140, (byte)243, (byte)27, (byte)38, (byte)191, (byte)115, (byte)217, (byte)253, (byte)40, (byte)106, (byte)174, (byte)29, (byte)153, (byte)127, (byte)21, (byte)172, (byte)203, (byte)117}, 0) ;
            p142.transfer_type = (byte)(byte)30;
            p142.uri_type = (byte)(byte)48;
            p142.request_id = (byte)(byte)26;
            LoopBackDemoChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2776746423U);
                Debug.Assert(pack.press_abs == (float)8.609084E37F);
                Debug.Assert(pack.temperature == (short)(short)859);
                Debug.Assert(pack.press_diff == (float) -1.5280337E38F);
            };
            DemoDevice.SCALED_PRESSURE3 p143 = LoopBackDemoChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.press_abs = (float)8.609084E37F;
            p143.press_diff = (float) -1.5280337E38F;
            p143.temperature = (short)(short)859;
            p143.time_boot_ms = (uint)2776746423U;
            LoopBackDemoChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {2.0567436E38F, 2.3096718E37F, -8.550814E37F}));
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {-3.356645E38F, -3.8025145E37F, 2.364E38F, -1.8740561E38F}));
                Debug.Assert(pack.custom_state == (ulong)1659280759425216350L);
                Debug.Assert(pack.alt == (float)3.1054884E38F);
                Debug.Assert(pack.acc.SequenceEqual(new float[] {2.0915373E38F, 4.703952E36F, -2.058266E38F}));
                Debug.Assert(pack.lon == (int)1026236241);
                Debug.Assert(pack.est_capabilities == (byte)(byte)75);
                Debug.Assert(pack.timestamp == (ulong)3955261623094898043L);
                Debug.Assert(pack.vel.SequenceEqual(new float[] {2.7446895E38F, 1.1618402E38F, 2.7553558E38F}));
                Debug.Assert(pack.rates.SequenceEqual(new float[] {2.1528999E38F, -3.3554896E38F, -2.0124146E38F}));
                Debug.Assert(pack.lat == (int) -1973399960);
            };
            DemoDevice.FOLLOW_TARGET p144 = LoopBackDemoChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.alt = (float)3.1054884E38F;
            p144.vel_SET(new float[] {2.7446895E38F, 1.1618402E38F, 2.7553558E38F}, 0) ;
            p144.lat = (int) -1973399960;
            p144.lon = (int)1026236241;
            p144.attitude_q_SET(new float[] {-3.356645E38F, -3.8025145E37F, 2.364E38F, -1.8740561E38F}, 0) ;
            p144.custom_state = (ulong)1659280759425216350L;
            p144.acc_SET(new float[] {2.0915373E38F, 4.703952E36F, -2.058266E38F}, 0) ;
            p144.rates_SET(new float[] {2.1528999E38F, -3.3554896E38F, -2.0124146E38F}, 0) ;
            p144.est_capabilities = (byte)(byte)75;
            p144.timestamp = (ulong)3955261623094898043L;
            p144.position_cov_SET(new float[] {2.0567436E38F, 2.3096718E37F, -8.550814E37F}, 0) ;
            LoopBackDemoChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.airspeed == (float)1.1785888E38F);
                Debug.Assert(pack.y_acc == (float) -1.3191539E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {9.089581E37F, 9.654513E37F, 1.5342888E38F, 2.70109E38F}));
                Debug.Assert(pack.z_pos == (float)3.173524E38F);
                Debug.Assert(pack.z_acc == (float) -3.337535E38F);
                Debug.Assert(pack.roll_rate == (float)3.1436749E37F);
                Debug.Assert(pack.y_pos == (float) -2.4740106E38F);
                Debug.Assert(pack.x_vel == (float) -3.002593E38F);
                Debug.Assert(pack.time_usec == (ulong)4435497734451167107L);
                Debug.Assert(pack.yaw_rate == (float)1.7842482E38F);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {-1.725908E38F, -2.2174756E37F, 2.402456E38F}));
                Debug.Assert(pack.y_vel == (float)2.8793817E38F);
                Debug.Assert(pack.pitch_rate == (float) -2.4923104E38F);
                Debug.Assert(pack.x_acc == (float) -2.625168E38F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {-9.427529E37F, 2.2267145E38F, 3.2358848E38F}));
                Debug.Assert(pack.x_pos == (float)2.4723178E38F);
                Debug.Assert(pack.z_vel == (float) -3.3264186E38F);
            };
            DemoDevice.CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.time_usec = (ulong)4435497734451167107L;
            p146.y_acc = (float) -1.3191539E38F;
            p146.z_pos = (float)3.173524E38F;
            p146.q_SET(new float[] {9.089581E37F, 9.654513E37F, 1.5342888E38F, 2.70109E38F}, 0) ;
            p146.x_acc = (float) -2.625168E38F;
            p146.y_pos = (float) -2.4740106E38F;
            p146.z_vel = (float) -3.3264186E38F;
            p146.y_vel = (float)2.8793817E38F;
            p146.pos_variance_SET(new float[] {-1.725908E38F, -2.2174756E37F, 2.402456E38F}, 0) ;
            p146.vel_variance_SET(new float[] {-9.427529E37F, 2.2267145E38F, 3.2358848E38F}, 0) ;
            p146.airspeed = (float)1.1785888E38F;
            p146.yaw_rate = (float)1.7842482E38F;
            p146.x_pos = (float)2.4723178E38F;
            p146.pitch_rate = (float) -2.4923104E38F;
            p146.z_acc = (float) -3.337535E38F;
            p146.roll_rate = (float)3.1436749E37F;
            p146.x_vel = (float) -3.002593E38F;
            LoopBackDemoChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.energy_consumed == (int) -1270771976);
                Debug.Assert(pack.id == (byte)(byte)197);
                Debug.Assert(pack.temperature == (short)(short)29869);
                Debug.Assert(pack.battery_function == (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS);
                Debug.Assert(pack.current_battery == (short)(short) -15824);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)57513, (ushort)25320, (ushort)21901, (ushort)59811, (ushort)53168, (ushort)37340, (ushort)54364, (ushort)32498, (ushort)29037, (ushort)25724}));
                Debug.Assert(pack.current_consumed == (int) -46081723);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)87);
                Debug.Assert(pack.type == (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH);
            };
            DemoDevice.BATTERY_STATUS p147 = LoopBackDemoChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH;
            p147.current_consumed = (int) -46081723;
            p147.voltages_SET(new ushort[] {(ushort)57513, (ushort)25320, (ushort)21901, (ushort)59811, (ushort)53168, (ushort)37340, (ushort)54364, (ushort)32498, (ushort)29037, (ushort)25724}, 0) ;
            p147.battery_remaining = (sbyte)(sbyte)87;
            p147.id = (byte)(byte)197;
            p147.temperature = (short)(short)29869;
            p147.current_battery = (short)(short) -15824;
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS;
            p147.energy_consumed = (int) -1270771976;
            LoopBackDemoChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)43, (byte)190, (byte)221, (byte)233, (byte)128, (byte)67, (byte)117, (byte)83}));
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)231, (byte)152, (byte)129, (byte)124, (byte)84, (byte)30, (byte)140, (byte)209}));
                Debug.Assert(pack.product_id == (ushort)(ushort)3167);
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)185, (byte)252, (byte)46, (byte)133, (byte)178, (byte)177, (byte)68, (byte)121}));
                Debug.Assert(pack.middleware_sw_version == (uint)2733039016U);
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP);
                Debug.Assert(pack.os_sw_version == (uint)2761557698U);
                Debug.Assert(pack.flight_sw_version == (uint)899113841U);
                Debug.Assert(pack.vendor_id == (ushort)(ushort)16831);
                Debug.Assert(pack.uid == (ulong)2286234126963354981L);
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)25, (byte)193, (byte)191, (byte)94, (byte)194, (byte)17, (byte)86, (byte)169, (byte)32, (byte)88, (byte)97, (byte)184, (byte)198, (byte)16, (byte)65, (byte)81, (byte)115, (byte)234}));
                Debug.Assert(pack.board_version == (uint)54113210U);
            };
            DemoDevice.AUTOPILOT_VERSION p148 = LoopBackDemoChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.uid2_SET(new byte[] {(byte)25, (byte)193, (byte)191, (byte)94, (byte)194, (byte)17, (byte)86, (byte)169, (byte)32, (byte)88, (byte)97, (byte)184, (byte)198, (byte)16, (byte)65, (byte)81, (byte)115, (byte)234}, 0, PH) ;
            p148.flight_sw_version = (uint)899113841U;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP;
            p148.middleware_custom_version_SET(new byte[] {(byte)231, (byte)152, (byte)129, (byte)124, (byte)84, (byte)30, (byte)140, (byte)209}, 0) ;
            p148.flight_custom_version_SET(new byte[] {(byte)185, (byte)252, (byte)46, (byte)133, (byte)178, (byte)177, (byte)68, (byte)121}, 0) ;
            p148.os_custom_version_SET(new byte[] {(byte)43, (byte)190, (byte)221, (byte)233, (byte)128, (byte)67, (byte)117, (byte)83}, 0) ;
            p148.uid = (ulong)2286234126963354981L;
            p148.os_sw_version = (uint)2761557698U;
            p148.product_id = (ushort)(ushort)3167;
            p148.board_version = (uint)54113210U;
            p148.middleware_sw_version = (uint)2733039016U;
            p148.vendor_id = (ushort)(ushort)16831;
            LoopBackDemoChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.angle_x == (float)3.9951629E37F);
                Debug.Assert(pack.type == (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
                Debug.Assert(pack.y_TRY(ph) == (float) -2.4720108E38F);
                Debug.Assert(pack.x_TRY(ph) == (float) -1.5598601E38F);
                Debug.Assert(pack.angle_y == (float)1.2533121E38F);
                Debug.Assert(pack.time_usec == (ulong)7382457929488285735L);
                Debug.Assert(pack.distance == (float) -1.0215358E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.z_TRY(ph) == (float)1.2207117E38F);
                Debug.Assert(pack.size_y == (float)1.494893E38F);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)230);
                Debug.Assert(pack.target_num == (byte)(byte)95);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {2.9593358E38F, -1.5509501E38F, -1.1791781E38F, 2.477248E38F}));
                Debug.Assert(pack.size_x == (float)1.2043923E38F);
            };
            DemoDevice.LANDING_TARGET p149 = LoopBackDemoChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.target_num = (byte)(byte)95;
            p149.size_y = (float)1.494893E38F;
            p149.x_SET((float) -1.5598601E38F, PH) ;
            p149.time_usec = (ulong)7382457929488285735L;
            p149.q_SET(new float[] {2.9593358E38F, -1.5509501E38F, -1.1791781E38F, 2.477248E38F}, 0, PH) ;
            p149.distance = (float) -1.0215358E38F;
            p149.size_x = (float)1.2043923E38F;
            p149.angle_y = (float)1.2533121E38F;
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p149.y_SET((float) -2.4720108E38F, PH) ;
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER;
            p149.z_SET((float)1.2207117E38F, PH) ;
            p149.angle_x = (float)3.9951629E37F;
            p149.position_valid_SET((byte)(byte)230, PH) ;
            LoopBackDemoChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCPU_LOADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sensLoad == (byte)(byte)19);
                Debug.Assert(pack.ctrlLoad == (byte)(byte)86);
                Debug.Assert(pack.batVolt == (ushort)(ushort)7965);
            };
            DemoDevice.CPU_LOAD p170 = LoopBackDemoChannel.new_CPU_LOAD();
            PH.setPack(p170);
            p170.batVolt = (ushort)(ushort)7965;
            p170.sensLoad = (byte)(byte)19;
            p170.ctrlLoad = (byte)(byte)86;
            LoopBackDemoChannel.instance.send(p170);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSENSOR_BIASReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.axBias == (float) -2.1685356E38F);
                Debug.Assert(pack.gyBias == (float) -8.336836E37F);
                Debug.Assert(pack.gzBias == (float)1.3968567E38F);
                Debug.Assert(pack.gxBias == (float) -2.4379698E38F);
                Debug.Assert(pack.azBias == (float)8.843455E37F);
                Debug.Assert(pack.ayBias == (float)1.1371121E37F);
            };
            DemoDevice.SENSOR_BIAS p172 = LoopBackDemoChannel.new_SENSOR_BIAS();
            PH.setPack(p172);
            p172.azBias = (float)8.843455E37F;
            p172.axBias = (float) -2.1685356E38F;
            p172.gzBias = (float)1.3968567E38F;
            p172.gxBias = (float) -2.4379698E38F;
            p172.gyBias = (float) -8.336836E37F;
            p172.ayBias = (float)1.1371121E37F;
            LoopBackDemoChannel.instance.send(p172);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDIAGNOSTICReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.diagSh2 == (short)(short) -1588);
                Debug.Assert(pack.diagFl3 == (float) -1.7433295E38F);
                Debug.Assert(pack.diagSh3 == (short)(short)16032);
                Debug.Assert(pack.diagFl1 == (float) -2.6257725E38F);
                Debug.Assert(pack.diagFl2 == (float) -3.1175713E38F);
                Debug.Assert(pack.diagSh1 == (short)(short)18957);
            };
            DemoDevice.DIAGNOSTIC p173 = LoopBackDemoChannel.new_DIAGNOSTIC();
            PH.setPack(p173);
            p173.diagFl2 = (float) -3.1175713E38F;
            p173.diagFl1 = (float) -2.6257725E38F;
            p173.diagSh1 = (short)(short)18957;
            p173.diagFl3 = (float) -1.7433295E38F;
            p173.diagSh3 = (short)(short)16032;
            p173.diagSh2 = (short)(short) -1588;
            LoopBackDemoChannel.instance.send(p173);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSLUGS_NAVIGATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.toWP == (byte)(byte)104);
                Debug.Assert(pack.dist2Go == (float)3.2689942E38F);
                Debug.Assert(pack.phi_c == (float)7.703587E37F);
                Debug.Assert(pack.psiDot_c == (float)2.426643E38F);
                Debug.Assert(pack.fromWP == (byte)(byte)22);
                Debug.Assert(pack.totalDist == (float)2.5349363E38F);
                Debug.Assert(pack.h_c == (ushort)(ushort)38651);
                Debug.Assert(pack.theta_c == (float)1.3811659E38F);
                Debug.Assert(pack.u_m == (float) -2.2440176E38F);
                Debug.Assert(pack.ay_body == (float)1.6577078E37F);
            };
            DemoDevice.SLUGS_NAVIGATION p176 = LoopBackDemoChannel.new_SLUGS_NAVIGATION();
            PH.setPack(p176);
            p176.dist2Go = (float)3.2689942E38F;
            p176.ay_body = (float)1.6577078E37F;
            p176.phi_c = (float)7.703587E37F;
            p176.psiDot_c = (float)2.426643E38F;
            p176.u_m = (float) -2.2440176E38F;
            p176.fromWP = (byte)(byte)22;
            p176.totalDist = (float)2.5349363E38F;
            p176.h_c = (ushort)(ushort)38651;
            p176.toWP = (byte)(byte)104;
            p176.theta_c = (float)1.3811659E38F;
            LoopBackDemoChannel.instance.send(p176);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDATA_LOGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fl_5 == (float) -8.825245E37F);
                Debug.Assert(pack.fl_2 == (float)5.565028E37F);
                Debug.Assert(pack.fl_1 == (float) -3.6421886E37F);
                Debug.Assert(pack.fl_6 == (float) -7.029075E37F);
                Debug.Assert(pack.fl_3 == (float) -2.3212187E38F);
                Debug.Assert(pack.fl_4 == (float) -6.102969E37F);
            };
            DemoDevice.DATA_LOG p177 = LoopBackDemoChannel.new_DATA_LOG();
            PH.setPack(p177);
            p177.fl_3 = (float) -2.3212187E38F;
            p177.fl_1 = (float) -3.6421886E37F;
            p177.fl_2 = (float)5.565028E37F;
            p177.fl_5 = (float) -8.825245E37F;
            p177.fl_4 = (float) -6.102969E37F;
            p177.fl_6 = (float) -7.029075E37F;
            LoopBackDemoChannel.instance.send(p177);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_DATE_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sec == (byte)(byte)94);
                Debug.Assert(pack.min == (byte)(byte)254);
                Debug.Assert(pack.hour == (byte)(byte)227);
                Debug.Assert(pack.year == (byte)(byte)234);
                Debug.Assert(pack.visSat == (byte)(byte)215);
                Debug.Assert(pack.month == (byte)(byte)117);
                Debug.Assert(pack.clockStat == (byte)(byte)170);
                Debug.Assert(pack.day == (byte)(byte)187);
                Debug.Assert(pack.percentUsed == (byte)(byte)101);
                Debug.Assert(pack.sigUsedMask == (byte)(byte)45);
                Debug.Assert(pack.useSat == (byte)(byte)125);
                Debug.Assert(pack.GppGl == (byte)(byte)197);
            };
            DemoDevice.GPS_DATE_TIME p179 = LoopBackDemoChannel.new_GPS_DATE_TIME();
            PH.setPack(p179);
            p179.useSat = (byte)(byte)125;
            p179.year = (byte)(byte)234;
            p179.month = (byte)(byte)117;
            p179.min = (byte)(byte)254;
            p179.sigUsedMask = (byte)(byte)45;
            p179.clockStat = (byte)(byte)170;
            p179.hour = (byte)(byte)227;
            p179.visSat = (byte)(byte)215;
            p179.sec = (byte)(byte)94;
            p179.percentUsed = (byte)(byte)101;
            p179.day = (byte)(byte)187;
            p179.GppGl = (byte)(byte)197;
            LoopBackDemoChannel.instance.send(p179);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMID_LVL_CMDSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rCommand == (float) -2.094151E38F);
                Debug.Assert(pack.target == (byte)(byte)90);
                Debug.Assert(pack.uCommand == (float)3.2330165E38F);
                Debug.Assert(pack.hCommand == (float) -2.2448985E38F);
            };
            DemoDevice.MID_LVL_CMDS p180 = LoopBackDemoChannel.new_MID_LVL_CMDS();
            PH.setPack(p180);
            p180.target = (byte)(byte)90;
            p180.uCommand = (float)3.2330165E38F;
            p180.rCommand = (float) -2.094151E38F;
            p180.hCommand = (float) -2.2448985E38F;
            LoopBackDemoChannel.instance.send(p180);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCTRL_SRFC_PTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target == (byte)(byte)60);
                Debug.Assert(pack.bitfieldPt == (ushort)(ushort)7014);
            };
            DemoDevice.CTRL_SRFC_PT p181 = LoopBackDemoChannel.new_CTRL_SRFC_PT();
            PH.setPack(p181);
            p181.target = (byte)(byte)60;
            p181.bitfieldPt = (ushort)(ushort)7014;
            LoopBackDemoChannel.instance.send(p181);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSLUGS_CAMERA_ORDERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.moveHome == (sbyte)(sbyte) - 118);
                Debug.Assert(pack.zoom == (sbyte)(sbyte)15);
                Debug.Assert(pack.tilt == (sbyte)(sbyte)40);
                Debug.Assert(pack.target == (byte)(byte)106);
                Debug.Assert(pack.pan == (sbyte)(sbyte) - 11);
            };
            DemoDevice.SLUGS_CAMERA_ORDER p184 = LoopBackDemoChannel.new_SLUGS_CAMERA_ORDER();
            PH.setPack(p184);
            p184.zoom = (sbyte)(sbyte)15;
            p184.tilt = (sbyte)(sbyte)40;
            p184.target = (byte)(byte)106;
            p184.pan = (sbyte)(sbyte) - 11;
            p184.moveHome = (sbyte)(sbyte) - 118;
            LoopBackDemoChannel.instance.send(p184);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCONTROL_SURFACEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.bControl == (float) -2.6059343E38F);
                Debug.Assert(pack.mControl == (float)2.1019779E38F);
                Debug.Assert(pack.target == (byte)(byte)55);
                Debug.Assert(pack.idSurface == (byte)(byte)55);
            };
            DemoDevice.CONTROL_SURFACE p185 = LoopBackDemoChannel.new_CONTROL_SURFACE();
            PH.setPack(p185);
            p185.bControl = (float) -2.6059343E38F;
            p185.target = (byte)(byte)55;
            p185.mControl = (float)2.1019779E38F;
            p185.idSurface = (byte)(byte)55;
            LoopBackDemoChannel.instance.send(p185);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSLUGS_MOBILE_LOCATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target == (byte)(byte)21);
                Debug.Assert(pack.longitude == (float)7.438913E37F);
                Debug.Assert(pack.latitude == (float)2.1923427E38F);
            };
            DemoDevice.SLUGS_MOBILE_LOCATION p186 = LoopBackDemoChannel.new_SLUGS_MOBILE_LOCATION();
            PH.setPack(p186);
            p186.longitude = (float)7.438913E37F;
            p186.latitude = (float)2.1923427E38F;
            p186.target = (byte)(byte)21;
            LoopBackDemoChannel.instance.send(p186);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSLUGS_CONFIGURATION_CAMERAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.idOrder == (byte)(byte)249);
                Debug.Assert(pack.order == (byte)(byte)63);
                Debug.Assert(pack.target == (byte)(byte)11);
            };
            DemoDevice.SLUGS_CONFIGURATION_CAMERA p188 = LoopBackDemoChannel.new_SLUGS_CONFIGURATION_CAMERA();
            PH.setPack(p188);
            p188.target = (byte)(byte)11;
            p188.idOrder = (byte)(byte)249;
            p188.order = (byte)(byte)63;
            LoopBackDemoChannel.instance.send(p188);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnISR_LOCATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.latitude == (float) -8.889194E37F);
                Debug.Assert(pack.height == (float) -2.6740004E38F);
                Debug.Assert(pack.option1 == (byte)(byte)74);
                Debug.Assert(pack.option3 == (byte)(byte)231);
                Debug.Assert(pack.target == (byte)(byte)33);
                Debug.Assert(pack.longitude == (float) -2.8632495E38F);
                Debug.Assert(pack.option2 == (byte)(byte)243);
            };
            DemoDevice.ISR_LOCATION p189 = LoopBackDemoChannel.new_ISR_LOCATION();
            PH.setPack(p189);
            p189.latitude = (float) -8.889194E37F;
            p189.option3 = (byte)(byte)231;
            p189.target = (byte)(byte)33;
            p189.longitude = (float) -2.8632495E38F;
            p189.option2 = (byte)(byte)243;
            p189.option1 = (byte)(byte)74;
            p189.height = (float) -2.6740004E38F;
            LoopBackDemoChannel.instance.send(p189);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVOLT_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.r2Type == (byte)(byte)176);
                Debug.Assert(pack.reading2 == (ushort)(ushort)20394);
                Debug.Assert(pack.voltage == (ushort)(ushort)52892);
            };
            DemoDevice.VOLT_SENSOR p191 = LoopBackDemoChannel.new_VOLT_SENSOR();
            PH.setPack(p191);
            p191.reading2 = (ushort)(ushort)20394;
            p191.r2Type = (byte)(byte)176;
            p191.voltage = (ushort)(ushort)52892;
            LoopBackDemoChannel.instance.send(p191);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPTZ_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tilt == (short)(short) -29429);
                Debug.Assert(pack.zoom == (byte)(byte)95);
                Debug.Assert(pack.pan == (short)(short) -25636);
            };
            DemoDevice.PTZ_STATUS p192 = LoopBackDemoChannel.new_PTZ_STATUS();
            PH.setPack(p192);
            p192.tilt = (short)(short) -29429;
            p192.zoom = (byte)(byte)95;
            p192.pan = (short)(short) -25636;
            LoopBackDemoChannel.instance.send(p192);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUAV_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.speed == (float) -9.963671E37F);
                Debug.Assert(pack.target == (byte)(byte)71);
                Debug.Assert(pack.altitude == (float)2.377105E37F);
                Debug.Assert(pack.latitude == (float)2.2184472E38F);
                Debug.Assert(pack.longitude == (float)4.3119115E37F);
                Debug.Assert(pack.course == (float) -1.0447829E38F);
            };
            DemoDevice.UAV_STATUS p193 = LoopBackDemoChannel.new_UAV_STATUS();
            PH.setPack(p193);
            p193.target = (byte)(byte)71;
            p193.longitude = (float)4.3119115E37F;
            p193.course = (float) -1.0447829E38F;
            p193.altitude = (float)2.377105E37F;
            p193.latitude = (float)2.2184472E38F;
            p193.speed = (float) -9.963671E37F;
            LoopBackDemoChannel.instance.send(p193);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSTATUS_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gpsQuality == (byte)(byte)106);
                Debug.Assert(pack.csFails == (ushort)(ushort)12861);
                Debug.Assert(pack.magVar == (float)2.988318E38F);
                Debug.Assert(pack.posStatus == (byte)(byte)97);
                Debug.Assert(pack.modeInd == (byte)(byte)17);
                Debug.Assert(pack.msgsType == (byte)(byte)28);
                Debug.Assert(pack.magDir == (sbyte)(sbyte) - 100);
            };
            DemoDevice.STATUS_GPS p194 = LoopBackDemoChannel.new_STATUS_GPS();
            PH.setPack(p194);
            p194.modeInd = (byte)(byte)17;
            p194.csFails = (ushort)(ushort)12861;
            p194.msgsType = (byte)(byte)28;
            p194.posStatus = (byte)(byte)97;
            p194.magVar = (float)2.988318E38F;
            p194.magDir = (sbyte)(sbyte) - 100;
            p194.gpsQuality = (byte)(byte)106;
            LoopBackDemoChannel.instance.send(p194);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNOVATEL_DIAGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.posSolAge == (float) -3.0250508E38F);
                Debug.Assert(pack.timeStatus == (byte)(byte)114);
                Debug.Assert(pack.csFails == (ushort)(ushort)56536);
                Debug.Assert(pack.velType == (byte)(byte)181);
                Debug.Assert(pack.solStatus == (byte)(byte)208);
                Debug.Assert(pack.posType == (byte)(byte)135);
                Debug.Assert(pack.receiverStatus == (uint)28197695U);
            };
            DemoDevice.NOVATEL_DIAG p195 = LoopBackDemoChannel.new_NOVATEL_DIAG();
            PH.setPack(p195);
            p195.timeStatus = (byte)(byte)114;
            p195.solStatus = (byte)(byte)208;
            p195.receiverStatus = (uint)28197695U;
            p195.velType = (byte)(byte)181;
            p195.posSolAge = (float) -3.0250508E38F;
            p195.posType = (byte)(byte)135;
            p195.csFails = (ushort)(ushort)56536;
            LoopBackDemoChannel.instance.send(p195);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSENSOR_DIAGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.float2 == (float)1.3222531E38F);
                Debug.Assert(pack.char1 == (sbyte)(sbyte)45);
                Debug.Assert(pack.int1 == (short)(short) -17091);
                Debug.Assert(pack.float1 == (float) -1.3806178E38F);
            };
            DemoDevice.SENSOR_DIAG p196 = LoopBackDemoChannel.new_SENSOR_DIAG();
            PH.setPack(p196);
            p196.char1 = (sbyte)(sbyte)45;
            p196.float1 = (float) -1.3806178E38F;
            p196.int1 = (short)(short) -17091;
            p196.float2 = (float)1.3222531E38F;
            LoopBackDemoChannel.instance.send(p196);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnBOOTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.version == (uint)2562340367U);
            };
            DemoDevice.BOOT p197 = LoopBackDemoChannel.new_BOOT();
            PH.setPack(p197);
            p197.version = (uint)2562340367U;
            LoopBackDemoChannel.instance.send(p197);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vel_ratio == (float)2.2113984E38F);
                Debug.Assert(pack.pos_horiz_accuracy == (float)2.1552099E38F);
                Debug.Assert(pack.pos_vert_ratio == (float)7.861809E37F);
                Debug.Assert(pack.tas_ratio == (float) -2.8363047E38F);
                Debug.Assert(pack.pos_vert_accuracy == (float) -2.4223028E38F);
                Debug.Assert(pack.hagl_ratio == (float) -2.4033609E38F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL);
                Debug.Assert(pack.time_usec == (ulong)3031091690657637384L);
                Debug.Assert(pack.pos_horiz_ratio == (float) -9.203316E37F);
                Debug.Assert(pack.mag_ratio == (float)9.110801E37F);
            };
            DemoDevice.ESTIMATOR_STATUS p230 = LoopBackDemoChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.pos_vert_ratio = (float)7.861809E37F;
            p230.time_usec = (ulong)3031091690657637384L;
            p230.tas_ratio = (float) -2.8363047E38F;
            p230.pos_horiz_ratio = (float) -9.203316E37F;
            p230.mag_ratio = (float)9.110801E37F;
            p230.vel_ratio = (float)2.2113984E38F;
            p230.pos_vert_accuracy = (float) -2.4223028E38F;
            p230.hagl_ratio = (float) -2.4033609E38F;
            p230.pos_horiz_accuracy = (float)2.1552099E38F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL;
            LoopBackDemoChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.wind_x == (float)1.2113459E38F);
                Debug.Assert(pack.wind_alt == (float)4.0950783E37F);
                Debug.Assert(pack.horiz_accuracy == (float)3.1823082E38F);
                Debug.Assert(pack.time_usec == (ulong)1023015693731534998L);
                Debug.Assert(pack.var_horiz == (float) -1.7780084E38F);
                Debug.Assert(pack.wind_y == (float) -2.3617195E37F);
                Debug.Assert(pack.vert_accuracy == (float) -9.802027E37F);
                Debug.Assert(pack.var_vert == (float)6.509994E36F);
                Debug.Assert(pack.wind_z == (float) -3.2894355E38F);
            };
            DemoDevice.WIND_COV p231 = LoopBackDemoChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.wind_y = (float) -2.3617195E37F;
            p231.var_horiz = (float) -1.7780084E38F;
            p231.var_vert = (float)6.509994E36F;
            p231.wind_z = (float) -3.2894355E38F;
            p231.time_usec = (ulong)1023015693731534998L;
            p231.vert_accuracy = (float) -9.802027E37F;
            p231.wind_alt = (float)4.0950783E37F;
            p231.horiz_accuracy = (float)3.1823082E38F;
            p231.wind_x = (float)1.2113459E38F;
            LoopBackDemoChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)3673549753334880711L);
                Debug.Assert(pack.speed_accuracy == (float)2.5231172E38F);
                Debug.Assert(pack.ve == (float)6.400514E37F);
                Debug.Assert(pack.alt == (float)7.893381E37F);
                Debug.Assert(pack.time_week == (ushort)(ushort)49492);
                Debug.Assert(pack.lat == (int) -830753300);
                Debug.Assert(pack.fix_type == (byte)(byte)35);
                Debug.Assert(pack.lon == (int)188215853);
                Debug.Assert(pack.hdop == (float)3.2329396E38F);
                Debug.Assert(pack.vert_accuracy == (float)2.8329815E38F);
                Debug.Assert(pack.time_week_ms == (uint)1191679387U);
                Debug.Assert(pack.gps_id == (byte)(byte)5);
                Debug.Assert(pack.vn == (float) -1.7478446E38F);
                Debug.Assert(pack.vd == (float) -1.2931166E38F);
                Debug.Assert(pack.horiz_accuracy == (float) -1.969959E38F);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ);
                Debug.Assert(pack.satellites_visible == (byte)(byte)136);
                Debug.Assert(pack.vdop == (float) -2.7524719E38F);
            };
            DemoDevice.GPS_INPUT p232 = LoopBackDemoChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.gps_id = (byte)(byte)5;
            p232.vd = (float) -1.2931166E38F;
            p232.lat = (int) -830753300;
            p232.vert_accuracy = (float)2.8329815E38F;
            p232.speed_accuracy = (float)2.5231172E38F;
            p232.vn = (float) -1.7478446E38F;
            p232.lon = (int)188215853;
            p232.satellites_visible = (byte)(byte)136;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ;
            p232.time_usec = (ulong)3673549753334880711L;
            p232.alt = (float)7.893381E37F;
            p232.fix_type = (byte)(byte)35;
            p232.hdop = (float)3.2329396E38F;
            p232.time_week_ms = (uint)1191679387U;
            p232.vdop = (float) -2.7524719E38F;
            p232.ve = (float)6.400514E37F;
            p232.horiz_accuracy = (float) -1.969959E38F;
            p232.time_week = (ushort)(ushort)49492;
            LoopBackDemoChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.len == (byte)(byte)172);
                Debug.Assert(pack.flags == (byte)(byte)236);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)178, (byte)186, (byte)98, (byte)38, (byte)10, (byte)180, (byte)62, (byte)56, (byte)55, (byte)79, (byte)242, (byte)198, (byte)92, (byte)183, (byte)36, (byte)155, (byte)82, (byte)118, (byte)65, (byte)182, (byte)116, (byte)37, (byte)123, (byte)248, (byte)176, (byte)231, (byte)48, (byte)146, (byte)78, (byte)72, (byte)229, (byte)24, (byte)235, (byte)192, (byte)2, (byte)222, (byte)17, (byte)0, (byte)175, (byte)25, (byte)184, (byte)126, (byte)154, (byte)193, (byte)244, (byte)78, (byte)157, (byte)58, (byte)35, (byte)13, (byte)190, (byte)227, (byte)106, (byte)119, (byte)20, (byte)209, (byte)36, (byte)107, (byte)28, (byte)27, (byte)235, (byte)24, (byte)64, (byte)195, (byte)131, (byte)227, (byte)122, (byte)207, (byte)222, (byte)10, (byte)144, (byte)251, (byte)112, (byte)66, (byte)176, (byte)50, (byte)120, (byte)66, (byte)146, (byte)242, (byte)131, (byte)104, (byte)162, (byte)17, (byte)223, (byte)105, (byte)51, (byte)18, (byte)220, (byte)146, (byte)110, (byte)174, (byte)168, (byte)238, (byte)94, (byte)31, (byte)252, (byte)31, (byte)240, (byte)187, (byte)208, (byte)129, (byte)246, (byte)36, (byte)13, (byte)122, (byte)236, (byte)237, (byte)148, (byte)45, (byte)58, (byte)42, (byte)138, (byte)103, (byte)139, (byte)91, (byte)71, (byte)0, (byte)217, (byte)133, (byte)88, (byte)143, (byte)239, (byte)122, (byte)247, (byte)33, (byte)169, (byte)106, (byte)97, (byte)42, (byte)26, (byte)72, (byte)213, (byte)95, (byte)48, (byte)33, (byte)43, (byte)158, (byte)166, (byte)149, (byte)170, (byte)35, (byte)180, (byte)246, (byte)125, (byte)54, (byte)119, (byte)200, (byte)115, (byte)70, (byte)234, (byte)43, (byte)95, (byte)92, (byte)170, (byte)251, (byte)31, (byte)108, (byte)24, (byte)232, (byte)200, (byte)28, (byte)197, (byte)179, (byte)163, (byte)229, (byte)174, (byte)65, (byte)178, (byte)218, (byte)116, (byte)99, (byte)28, (byte)3, (byte)239, (byte)83, (byte)57, (byte)211, (byte)29, (byte)94}));
            };
            DemoDevice.GPS_RTCM_DATA p233 = LoopBackDemoChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.data__SET(new byte[] {(byte)178, (byte)186, (byte)98, (byte)38, (byte)10, (byte)180, (byte)62, (byte)56, (byte)55, (byte)79, (byte)242, (byte)198, (byte)92, (byte)183, (byte)36, (byte)155, (byte)82, (byte)118, (byte)65, (byte)182, (byte)116, (byte)37, (byte)123, (byte)248, (byte)176, (byte)231, (byte)48, (byte)146, (byte)78, (byte)72, (byte)229, (byte)24, (byte)235, (byte)192, (byte)2, (byte)222, (byte)17, (byte)0, (byte)175, (byte)25, (byte)184, (byte)126, (byte)154, (byte)193, (byte)244, (byte)78, (byte)157, (byte)58, (byte)35, (byte)13, (byte)190, (byte)227, (byte)106, (byte)119, (byte)20, (byte)209, (byte)36, (byte)107, (byte)28, (byte)27, (byte)235, (byte)24, (byte)64, (byte)195, (byte)131, (byte)227, (byte)122, (byte)207, (byte)222, (byte)10, (byte)144, (byte)251, (byte)112, (byte)66, (byte)176, (byte)50, (byte)120, (byte)66, (byte)146, (byte)242, (byte)131, (byte)104, (byte)162, (byte)17, (byte)223, (byte)105, (byte)51, (byte)18, (byte)220, (byte)146, (byte)110, (byte)174, (byte)168, (byte)238, (byte)94, (byte)31, (byte)252, (byte)31, (byte)240, (byte)187, (byte)208, (byte)129, (byte)246, (byte)36, (byte)13, (byte)122, (byte)236, (byte)237, (byte)148, (byte)45, (byte)58, (byte)42, (byte)138, (byte)103, (byte)139, (byte)91, (byte)71, (byte)0, (byte)217, (byte)133, (byte)88, (byte)143, (byte)239, (byte)122, (byte)247, (byte)33, (byte)169, (byte)106, (byte)97, (byte)42, (byte)26, (byte)72, (byte)213, (byte)95, (byte)48, (byte)33, (byte)43, (byte)158, (byte)166, (byte)149, (byte)170, (byte)35, (byte)180, (byte)246, (byte)125, (byte)54, (byte)119, (byte)200, (byte)115, (byte)70, (byte)234, (byte)43, (byte)95, (byte)92, (byte)170, (byte)251, (byte)31, (byte)108, (byte)24, (byte)232, (byte)200, (byte)28, (byte)197, (byte)179, (byte)163, (byte)229, (byte)174, (byte)65, (byte)178, (byte)218, (byte)116, (byte)99, (byte)28, (byte)3, (byte)239, (byte)83, (byte)57, (byte)211, (byte)29, (byte)94}, 0) ;
            p233.len = (byte)(byte)172;
            p233.flags = (byte)(byte)236;
            LoopBackDemoChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte) - 102);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)68);
                Debug.Assert(pack.gps_fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte) - 117);
                Debug.Assert(pack.wp_num == (byte)(byte)113);
                Debug.Assert(pack.heading == (ushort)(ushort)15809);
                Debug.Assert(pack.failsafe == (byte)(byte)228);
                Debug.Assert(pack.throttle == (sbyte)(sbyte)85);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
                Debug.Assert(pack.custom_mode == (uint)4253870890U);
                Debug.Assert(pack.airspeed == (byte)(byte)255);
                Debug.Assert(pack.groundspeed == (byte)(byte)105);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED);
                Debug.Assert(pack.longitude == (int)1616176619);
                Debug.Assert(pack.gps_nsat == (byte)(byte)37);
                Debug.Assert(pack.altitude_amsl == (short)(short) -20397);
                Debug.Assert(pack.altitude_sp == (short)(short) -6911);
                Debug.Assert(pack.pitch == (short)(short) -24853);
                Debug.Assert(pack.latitude == (int) -820952341);
                Debug.Assert(pack.battery_remaining == (byte)(byte)213);
                Debug.Assert(pack.heading_sp == (short)(short) -18427);
                Debug.Assert(pack.temperature == (sbyte)(sbyte)27);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)28009);
                Debug.Assert(pack.roll == (short)(short)13422);
            };
            DemoDevice.HIGH_LATENCY p234 = LoopBackDemoChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.heading = (ushort)(ushort)15809;
            p234.pitch = (short)(short) -24853;
            p234.altitude_sp = (short)(short) -6911;
            p234.wp_distance = (ushort)(ushort)28009;
            p234.latitude = (int) -820952341;
            p234.roll = (short)(short)13422;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF;
            p234.temperature = (sbyte)(sbyte)27;
            p234.battery_remaining = (byte)(byte)213;
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p234.custom_mode = (uint)4253870890U;
            p234.gps_nsat = (byte)(byte)37;
            p234.groundspeed = (byte)(byte)105;
            p234.altitude_amsl = (short)(short) -20397;
            p234.wp_num = (byte)(byte)113;
            p234.throttle = (sbyte)(sbyte)85;
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED;
            p234.airspeed = (byte)(byte)255;
            p234.heading_sp = (short)(short) -18427;
            p234.failsafe = (byte)(byte)228;
            p234.temperature_air = (sbyte)(sbyte) - 117;
            p234.airspeed_sp = (byte)(byte)68;
            p234.climb_rate = (sbyte)(sbyte) - 102;
            p234.longitude = (int)1616176619;
            LoopBackDemoChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vibration_x == (float)2.511694E37F);
                Debug.Assert(pack.vibration_y == (float) -2.0421123E38F);
                Debug.Assert(pack.vibration_z == (float) -2.3522409E38F);
                Debug.Assert(pack.clipping_2 == (uint)1852476359U);
                Debug.Assert(pack.clipping_1 == (uint)1681893034U);
                Debug.Assert(pack.clipping_0 == (uint)1246447821U);
                Debug.Assert(pack.time_usec == (ulong)1812433593686600011L);
            };
            DemoDevice.VIBRATION p241 = LoopBackDemoChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.vibration_y = (float) -2.0421123E38F;
            p241.clipping_0 = (uint)1246447821U;
            p241.vibration_x = (float)2.511694E37F;
            p241.time_usec = (ulong)1812433593686600011L;
            p241.vibration_z = (float) -2.3522409E38F;
            p241.clipping_2 = (uint)1852476359U;
            p241.clipping_1 = (uint)1681893034U;
            LoopBackDemoChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)2.4818735E38F);
                Debug.Assert(pack.longitude == (int) -702489478);
                Debug.Assert(pack.latitude == (int) -2019863153);
                Debug.Assert(pack.y == (float) -3.1451024E38F);
                Debug.Assert(pack.altitude == (int) -911546853);
                Debug.Assert(pack.approach_z == (float) -7.243643E36F);
                Debug.Assert(pack.z == (float)1.2035744E38F);
                Debug.Assert(pack.approach_x == (float)3.139174E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.5973257E38F, 2.0474097E38F, -3.092354E38F, 1.5440284E38F}));
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)4568034236085091355L);
                Debug.Assert(pack.approach_y == (float)2.258207E37F);
            };
            DemoDevice.HOME_POSITION p242 = LoopBackDemoChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.approach_x = (float)3.139174E38F;
            p242.altitude = (int) -911546853;
            p242.approach_z = (float) -7.243643E36F;
            p242.time_usec_SET((ulong)4568034236085091355L, PH) ;
            p242.latitude = (int) -2019863153;
            p242.longitude = (int) -702489478;
            p242.x = (float)2.4818735E38F;
            p242.q_SET(new float[] {1.5973257E38F, 2.0474097E38F, -3.092354E38F, 1.5440284E38F}, 0) ;
            p242.z = (float)1.2035744E38F;
            p242.y = (float) -3.1451024E38F;
            p242.approach_y = (float)2.258207E37F;
            LoopBackDemoChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)7664148706982798069L);
                Debug.Assert(pack.latitude == (int) -2034492892);
                Debug.Assert(pack.y == (float) -2.019447E38F);
                Debug.Assert(pack.approach_z == (float)2.655296E38F);
                Debug.Assert(pack.x == (float)6.3486437E37F);
                Debug.Assert(pack.altitude == (int)1177462052);
                Debug.Assert(pack.longitude == (int)1809341314);
                Debug.Assert(pack.target_system == (byte)(byte)245);
                Debug.Assert(pack.z == (float)1.2282157E38F);
                Debug.Assert(pack.approach_x == (float) -5.0086635E37F);
                Debug.Assert(pack.approach_y == (float) -2.264476E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.7074919E38F, 5.9760026E37F, -5.770499E37F, 7.323353E37F}));
            };
            DemoDevice.SET_HOME_POSITION p243 = LoopBackDemoChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.approach_x = (float) -5.0086635E37F;
            p243.approach_y = (float) -2.264476E38F;
            p243.target_system = (byte)(byte)245;
            p243.longitude = (int)1809341314;
            p243.z = (float)1.2282157E38F;
            p243.approach_z = (float)2.655296E38F;
            p243.y = (float) -2.019447E38F;
            p243.altitude = (int)1177462052;
            p243.latitude = (int) -2034492892;
            p243.q_SET(new float[] {-1.7074919E38F, 5.9760026E37F, -5.770499E37F, 7.323353E37F}, 0) ;
            p243.time_usec_SET((ulong)7664148706982798069L, PH) ;
            p243.x = (float)6.3486437E37F;
            LoopBackDemoChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_id == (ushort)(ushort)59892);
                Debug.Assert(pack.interval_us == (int) -1262736552);
            };
            DemoDevice.MESSAGE_INTERVAL p244 = LoopBackDemoChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.interval_us = (int) -1262736552;
            p244.message_id = (ushort)(ushort)59892;
            LoopBackDemoChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vtol_state == (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
            };
            DemoDevice.EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED;
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED;
            LoopBackDemoChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.squawk == (ushort)(ushort)25383);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)23432);
                Debug.Assert(pack.lat == (int) -1391785519);
                Debug.Assert(pack.lon == (int)370747950);
                Debug.Assert(pack.altitude_type == (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
                Debug.Assert(pack.heading == (ushort)(ushort)55330);
                Debug.Assert(pack.tslc == (byte)(byte)63);
                Debug.Assert(pack.altitude == (int)831074314);
                Debug.Assert(pack.flags == (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE);
                Debug.Assert(pack.ver_velocity == (short)(short) -7582);
                Debug.Assert(pack.ICAO_address == (uint)349428988U);
                Debug.Assert(pack.emitter_type == (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE);
                Debug.Assert(pack.callsign_LEN(ph) == 7);
                Debug.Assert(pack.callsign_TRY(ph).Equals("prmowra"));
            };
            DemoDevice.ADSB_VEHICLE p246 = LoopBackDemoChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.lon = (int)370747950;
            p246.tslc = (byte)(byte)63;
            p246.ICAO_address = (uint)349428988U;
            p246.altitude = (int)831074314;
            p246.heading = (ushort)(ushort)55330;
            p246.callsign_SET("prmowra", PH) ;
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE;
            p246.lat = (int) -1391785519;
            p246.hor_velocity = (ushort)(ushort)23432;
            p246.ver_velocity = (short)(short) -7582;
            p246.squawk = (ushort)(ushort)25383;
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE;
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            LoopBackDemoChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.action == (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER);
                Debug.Assert(pack.src_ == (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
                Debug.Assert(pack.time_to_minimum_delta == (float)4.563959E37F);
                Debug.Assert(pack.threat_level == (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
                Debug.Assert(pack.horizontal_minimum_delta == (float)1.009253E37F);
                Debug.Assert(pack.id == (uint)3141637267U);
                Debug.Assert(pack.altitude_minimum_delta == (float) -1.1099389E38F);
            };
            DemoDevice.COLLISION p247 = LoopBackDemoChannel.new_COLLISION();
            PH.setPack(p247);
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER;
            p247.time_to_minimum_delta = (float)4.563959E37F;
            p247.horizontal_minimum_delta = (float)1.009253E37F;
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.id = (uint)3141637267U;
            p247.altitude_minimum_delta = (float) -1.1099389E38F;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH;
            LoopBackDemoChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)0, (byte)133, (byte)213, (byte)147, (byte)237, (byte)118, (byte)226, (byte)177, (byte)234, (byte)36, (byte)157, (byte)151, (byte)223, (byte)57, (byte)123, (byte)139, (byte)62, (byte)205, (byte)174, (byte)141, (byte)121, (byte)172, (byte)156, (byte)169, (byte)177, (byte)117, (byte)33, (byte)7, (byte)124, (byte)109, (byte)246, (byte)255, (byte)139, (byte)199, (byte)247, (byte)15, (byte)162, (byte)34, (byte)58, (byte)183, (byte)136, (byte)238, (byte)159, (byte)10, (byte)169, (byte)158, (byte)86, (byte)223, (byte)97, (byte)204, (byte)98, (byte)228, (byte)161, (byte)195, (byte)155, (byte)89, (byte)14, (byte)235, (byte)95, (byte)252, (byte)121, (byte)96, (byte)171, (byte)184, (byte)89, (byte)240, (byte)155, (byte)189, (byte)209, (byte)96, (byte)37, (byte)239, (byte)157, (byte)194, (byte)0, (byte)220, (byte)49, (byte)94, (byte)1, (byte)159, (byte)52, (byte)210, (byte)204, (byte)241, (byte)151, (byte)151, (byte)253, (byte)133, (byte)222, (byte)173, (byte)169, (byte)238, (byte)95, (byte)171, (byte)140, (byte)11, (byte)145, (byte)192, (byte)169, (byte)100, (byte)107, (byte)115, (byte)169, (byte)150, (byte)53, (byte)105, (byte)54, (byte)28, (byte)109, (byte)116, (byte)103, (byte)71, (byte)217, (byte)14, (byte)247, (byte)1, (byte)2, (byte)181, (byte)181, (byte)115, (byte)248, (byte)119, (byte)244, (byte)70, (byte)241, (byte)94, (byte)181, (byte)173, (byte)4, (byte)20, (byte)201, (byte)48, (byte)120, (byte)216, (byte)20, (byte)186, (byte)1, (byte)99, (byte)70, (byte)129, (byte)112, (byte)163, (byte)5, (byte)169, (byte)80, (byte)215, (byte)247, (byte)14, (byte)168, (byte)165, (byte)56, (byte)119, (byte)54, (byte)48, (byte)120, (byte)27, (byte)244, (byte)237, (byte)219, (byte)53, (byte)110, (byte)88, (byte)208, (byte)106, (byte)197, (byte)114, (byte)176, (byte)72, (byte)243, (byte)255, (byte)95, (byte)181, (byte)6, (byte)248, (byte)137, (byte)4, (byte)192, (byte)90, (byte)241, (byte)113, (byte)36, (byte)4, (byte)13, (byte)218, (byte)231, (byte)154, (byte)232, (byte)83, (byte)51, (byte)254, (byte)231, (byte)52, (byte)67, (byte)165, (byte)179, (byte)143, (byte)111, (byte)61, (byte)186, (byte)205, (byte)176, (byte)80, (byte)55, (byte)123, (byte)220, (byte)143, (byte)107, (byte)213, (byte)14, (byte)218, (byte)26, (byte)225, (byte)136, (byte)72, (byte)4, (byte)180, (byte)93, (byte)21, (byte)105, (byte)216, (byte)150, (byte)195, (byte)182, (byte)153, (byte)72, (byte)119, (byte)132, (byte)132, (byte)34, (byte)9, (byte)111, (byte)76, (byte)169, (byte)162, (byte)187, (byte)66, (byte)197, (byte)128, (byte)140, (byte)224, (byte)147, (byte)69, (byte)137, (byte)84, (byte)93, (byte)72, (byte)52, (byte)104, (byte)116}));
                Debug.Assert(pack.target_component == (byte)(byte)26);
                Debug.Assert(pack.target_network == (byte)(byte)162);
                Debug.Assert(pack.target_system == (byte)(byte)203);
                Debug.Assert(pack.message_type == (ushort)(ushort)3221);
            };
            DemoDevice.V2_EXTENSION p248 = LoopBackDemoChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.message_type = (ushort)(ushort)3221;
            p248.target_system = (byte)(byte)203;
            p248.target_component = (byte)(byte)26;
            p248.payload_SET(new byte[] {(byte)0, (byte)133, (byte)213, (byte)147, (byte)237, (byte)118, (byte)226, (byte)177, (byte)234, (byte)36, (byte)157, (byte)151, (byte)223, (byte)57, (byte)123, (byte)139, (byte)62, (byte)205, (byte)174, (byte)141, (byte)121, (byte)172, (byte)156, (byte)169, (byte)177, (byte)117, (byte)33, (byte)7, (byte)124, (byte)109, (byte)246, (byte)255, (byte)139, (byte)199, (byte)247, (byte)15, (byte)162, (byte)34, (byte)58, (byte)183, (byte)136, (byte)238, (byte)159, (byte)10, (byte)169, (byte)158, (byte)86, (byte)223, (byte)97, (byte)204, (byte)98, (byte)228, (byte)161, (byte)195, (byte)155, (byte)89, (byte)14, (byte)235, (byte)95, (byte)252, (byte)121, (byte)96, (byte)171, (byte)184, (byte)89, (byte)240, (byte)155, (byte)189, (byte)209, (byte)96, (byte)37, (byte)239, (byte)157, (byte)194, (byte)0, (byte)220, (byte)49, (byte)94, (byte)1, (byte)159, (byte)52, (byte)210, (byte)204, (byte)241, (byte)151, (byte)151, (byte)253, (byte)133, (byte)222, (byte)173, (byte)169, (byte)238, (byte)95, (byte)171, (byte)140, (byte)11, (byte)145, (byte)192, (byte)169, (byte)100, (byte)107, (byte)115, (byte)169, (byte)150, (byte)53, (byte)105, (byte)54, (byte)28, (byte)109, (byte)116, (byte)103, (byte)71, (byte)217, (byte)14, (byte)247, (byte)1, (byte)2, (byte)181, (byte)181, (byte)115, (byte)248, (byte)119, (byte)244, (byte)70, (byte)241, (byte)94, (byte)181, (byte)173, (byte)4, (byte)20, (byte)201, (byte)48, (byte)120, (byte)216, (byte)20, (byte)186, (byte)1, (byte)99, (byte)70, (byte)129, (byte)112, (byte)163, (byte)5, (byte)169, (byte)80, (byte)215, (byte)247, (byte)14, (byte)168, (byte)165, (byte)56, (byte)119, (byte)54, (byte)48, (byte)120, (byte)27, (byte)244, (byte)237, (byte)219, (byte)53, (byte)110, (byte)88, (byte)208, (byte)106, (byte)197, (byte)114, (byte)176, (byte)72, (byte)243, (byte)255, (byte)95, (byte)181, (byte)6, (byte)248, (byte)137, (byte)4, (byte)192, (byte)90, (byte)241, (byte)113, (byte)36, (byte)4, (byte)13, (byte)218, (byte)231, (byte)154, (byte)232, (byte)83, (byte)51, (byte)254, (byte)231, (byte)52, (byte)67, (byte)165, (byte)179, (byte)143, (byte)111, (byte)61, (byte)186, (byte)205, (byte)176, (byte)80, (byte)55, (byte)123, (byte)220, (byte)143, (byte)107, (byte)213, (byte)14, (byte)218, (byte)26, (byte)225, (byte)136, (byte)72, (byte)4, (byte)180, (byte)93, (byte)21, (byte)105, (byte)216, (byte)150, (byte)195, (byte)182, (byte)153, (byte)72, (byte)119, (byte)132, (byte)132, (byte)34, (byte)9, (byte)111, (byte)76, (byte)169, (byte)162, (byte)187, (byte)66, (byte)197, (byte)128, (byte)140, (byte)224, (byte)147, (byte)69, (byte)137, (byte)84, (byte)93, (byte)72, (byte)52, (byte)104, (byte)116}, 0) ;
            p248.target_network = (byte)(byte)162;
            LoopBackDemoChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (byte)(byte)1);
                Debug.Assert(pack.address == (ushort)(ushort)37421);
                Debug.Assert(pack.ver == (byte)(byte)131);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte) - 56, (sbyte)0, (sbyte) - 54, (sbyte) - 63, (sbyte)118, (sbyte) - 127, (sbyte) - 107, (sbyte)112, (sbyte) - 91, (sbyte)120, (sbyte) - 91, (sbyte)48, (sbyte)90, (sbyte) - 59, (sbyte)77, (sbyte) - 56, (sbyte) - 116, (sbyte) - 53, (sbyte)67, (sbyte) - 116, (sbyte)80, (sbyte) - 44, (sbyte)80, (sbyte)91, (sbyte)89, (sbyte)30, (sbyte)88, (sbyte) - 42, (sbyte) - 31, (sbyte) - 85, (sbyte) - 21, (sbyte)29}));
            };
            DemoDevice.MEMORY_VECT p249 = LoopBackDemoChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)37421;
            p249.type = (byte)(byte)1;
            p249.ver = (byte)(byte)131;
            p249.value_SET(new sbyte[] {(sbyte) - 56, (sbyte)0, (sbyte) - 54, (sbyte) - 63, (sbyte)118, (sbyte) - 127, (sbyte) - 107, (sbyte)112, (sbyte) - 91, (sbyte)120, (sbyte) - 91, (sbyte)48, (sbyte)90, (sbyte) - 59, (sbyte)77, (sbyte) - 56, (sbyte) - 116, (sbyte) - 53, (sbyte)67, (sbyte) - 116, (sbyte)80, (sbyte) - 44, (sbyte)80, (sbyte)91, (sbyte)89, (sbyte)30, (sbyte)88, (sbyte) - 42, (sbyte) - 31, (sbyte) - 85, (sbyte) - 21, (sbyte)29}, 0) ;
            LoopBackDemoChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)9.1885825E36F);
                Debug.Assert(pack.z == (float) -2.2898218E38F);
                Debug.Assert(pack.x == (float) -1.0731144E38F);
                Debug.Assert(pack.time_usec == (ulong)1096322828782112155L);
                Debug.Assert(pack.name_LEN(ph) == 6);
                Debug.Assert(pack.name_TRY(ph).Equals("jsldmz"));
            };
            DemoDevice.DEBUG_VECT p250 = LoopBackDemoChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.y = (float)9.1885825E36F;
            p250.z = (float) -2.2898218E38F;
            p250.time_usec = (ulong)1096322828782112155L;
            p250.name_SET("jsldmz", PH) ;
            p250.x = (float) -1.0731144E38F;
            LoopBackDemoChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 4);
                Debug.Assert(pack.name_TRY(ph).Equals("fkvo"));
                Debug.Assert(pack.value == (float) -2.3235228E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1017999527U);
            };
            DemoDevice.NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)1017999527U;
            p251.name_SET("fkvo", PH) ;
            p251.value = (float) -2.3235228E38F;
            LoopBackDemoChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3286640824U);
                Debug.Assert(pack.name_LEN(ph) == 8);
                Debug.Assert(pack.name_TRY(ph).Equals("mqvxrvGa"));
                Debug.Assert(pack.value == (int)2117016627);
            };
            DemoDevice.NAMED_VALUE_INT p252 = LoopBackDemoChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.name_SET("mqvxrvGa", PH) ;
            p252.time_boot_ms = (uint)3286640824U;
            p252.value = (int)2117016627;
            LoopBackDemoChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.text_LEN(ph) == 36);
                Debug.Assert(pack.text_TRY(ph).Equals("UkgwzKpzPvlrzwptcBnbdwggvsxEbbgihlyf"));
                Debug.Assert(pack.severity == (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_CRITICAL);
            };
            DemoDevice.STATUSTEXT p253 = LoopBackDemoChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.text_SET("UkgwzKpzPvlrzwptcBnbdwggvsxEbbgihlyf", PH) ;
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_CRITICAL;
            LoopBackDemoChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1908261245U);
                Debug.Assert(pack.ind == (byte)(byte)19);
                Debug.Assert(pack.value == (float)2.1460317E38F);
            };
            DemoDevice.DEBUG p254 = LoopBackDemoChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)1908261245U;
            p254.ind = (byte)(byte)19;
            p254.value = (float)2.1460317E38F;
            LoopBackDemoChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)20, (byte)123, (byte)49, (byte)111, (byte)62, (byte)53, (byte)187, (byte)170, (byte)222, (byte)56, (byte)44, (byte)178, (byte)30, (byte)207, (byte)54, (byte)127, (byte)17, (byte)194, (byte)243, (byte)58, (byte)89, (byte)244, (byte)240, (byte)112, (byte)82, (byte)8, (byte)100, (byte)32, (byte)102, (byte)22, (byte)8, (byte)52}));
                Debug.Assert(pack.target_component == (byte)(byte)239);
                Debug.Assert(pack.target_system == (byte)(byte)113);
                Debug.Assert(pack.initial_timestamp == (ulong)6952915092685264823L);
            };
            DemoDevice.SETUP_SIGNING p256 = LoopBackDemoChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.initial_timestamp = (ulong)6952915092685264823L;
            p256.target_component = (byte)(byte)239;
            p256.target_system = (byte)(byte)113;
            p256.secret_key_SET(new byte[] {(byte)20, (byte)123, (byte)49, (byte)111, (byte)62, (byte)53, (byte)187, (byte)170, (byte)222, (byte)56, (byte)44, (byte)178, (byte)30, (byte)207, (byte)54, (byte)127, (byte)17, (byte)194, (byte)243, (byte)58, (byte)89, (byte)244, (byte)240, (byte)112, (byte)82, (byte)8, (byte)100, (byte)32, (byte)102, (byte)22, (byte)8, (byte)52}, 0) ;
            LoopBackDemoChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.state == (byte)(byte)31);
                Debug.Assert(pack.last_change_ms == (uint)1814722340U);
                Debug.Assert(pack.time_boot_ms == (uint)3193142754U);
            };
            DemoDevice.BUTTON_CHANGE p257 = LoopBackDemoChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)3193142754U;
            p257.state = (byte)(byte)31;
            p257.last_change_ms = (uint)1814722340U;
            LoopBackDemoChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tune_LEN(ph) == 18);
                Debug.Assert(pack.tune_TRY(ph).Equals("apmuyzyliasdrfczjv"));
                Debug.Assert(pack.target_system == (byte)(byte)173);
                Debug.Assert(pack.target_component == (byte)(byte)99);
            };
            DemoDevice.PLAY_TUNE p258 = LoopBackDemoChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)173;
            p258.tune_SET("apmuyzyliasdrfczjv", PH) ;
            p258.target_component = (byte)(byte)99;
            LoopBackDemoChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)49594);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)48506);
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)7912);
                Debug.Assert(pack.firmware_version == (uint)2336255625U);
                Debug.Assert(pack.focal_length == (float)2.3820575E38F);
                Debug.Assert(pack.sensor_size_h == (float) -5.5779917E37F);
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 22);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("mvshadrPrwjxkMhajwcudI"));
                Debug.Assert(pack.sensor_size_v == (float)6.0772696E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3999330253U);
                Debug.Assert(pack.lens_id == (byte)(byte)47);
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)115, (byte)145, (byte)98, (byte)96, (byte)76, (byte)198, (byte)97, (byte)230, (byte)83, (byte)227, (byte)198, (byte)26, (byte)150, (byte)176, (byte)40, (byte)247, (byte)180, (byte)37, (byte)213, (byte)82, (byte)32, (byte)57, (byte)141, (byte)29, (byte)93, (byte)127, (byte)141, (byte)203, (byte)214, (byte)106, (byte)217, (byte)240}));
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)168, (byte)85, (byte)224, (byte)63, (byte)52, (byte)49, (byte)242, (byte)255, (byte)137, (byte)212, (byte)92, (byte)84, (byte)165, (byte)138, (byte)124, (byte)178, (byte)207, (byte)186, (byte)200, (byte)220, (byte)157, (byte)19, (byte)135, (byte)224, (byte)205, (byte)112, (byte)188, (byte)104, (byte)185, (byte)207, (byte)220, (byte)206}));
            };
            DemoDevice.CAMERA_INFORMATION p259 = LoopBackDemoChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.cam_definition_uri_SET("mvshadrPrwjxkMhajwcudI", PH) ;
            p259.vendor_name_SET(new byte[] {(byte)115, (byte)145, (byte)98, (byte)96, (byte)76, (byte)198, (byte)97, (byte)230, (byte)83, (byte)227, (byte)198, (byte)26, (byte)150, (byte)176, (byte)40, (byte)247, (byte)180, (byte)37, (byte)213, (byte)82, (byte)32, (byte)57, (byte)141, (byte)29, (byte)93, (byte)127, (byte)141, (byte)203, (byte)214, (byte)106, (byte)217, (byte)240}, 0) ;
            p259.cam_definition_version = (ushort)(ushort)7912;
            p259.focal_length = (float)2.3820575E38F;
            p259.lens_id = (byte)(byte)47;
            p259.firmware_version = (uint)2336255625U;
            p259.resolution_v = (ushort)(ushort)48506;
            p259.sensor_size_v = (float)6.0772696E37F;
            p259.time_boot_ms = (uint)3999330253U;
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE;
            p259.model_name_SET(new byte[] {(byte)168, (byte)85, (byte)224, (byte)63, (byte)52, (byte)49, (byte)242, (byte)255, (byte)137, (byte)212, (byte)92, (byte)84, (byte)165, (byte)138, (byte)124, (byte)178, (byte)207, (byte)186, (byte)200, (byte)220, (byte)157, (byte)19, (byte)135, (byte)224, (byte)205, (byte)112, (byte)188, (byte)104, (byte)185, (byte)207, (byte)220, (byte)206}, 0) ;
            p259.resolution_h = (ushort)(ushort)49594;
            p259.sensor_size_h = (float) -5.5779917E37F;
            LoopBackDemoChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)619018126U);
                Debug.Assert(pack.mode_id == (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
            };
            DemoDevice.CAMERA_SETTINGS p260 = LoopBackDemoChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY;
            p260.time_boot_ms = (uint)619018126U;
            LoopBackDemoChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)4204641248U);
                Debug.Assert(pack.storage_id == (byte)(byte)44);
                Debug.Assert(pack.write_speed == (float) -1.2592937E38F);
                Debug.Assert(pack.read_speed == (float) -3.0974892E38F);
                Debug.Assert(pack.storage_count == (byte)(byte)185);
                Debug.Assert(pack.status == (byte)(byte)124);
                Debug.Assert(pack.total_capacity == (float) -2.5568967E38F);
                Debug.Assert(pack.available_capacity == (float) -2.332585E38F);
                Debug.Assert(pack.used_capacity == (float) -7.195432E37F);
            };
            DemoDevice.STORAGE_INFORMATION p261 = LoopBackDemoChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.storage_count = (byte)(byte)185;
            p261.storage_id = (byte)(byte)44;
            p261.read_speed = (float) -3.0974892E38F;
            p261.status = (byte)(byte)124;
            p261.available_capacity = (float) -2.332585E38F;
            p261.total_capacity = (float) -2.5568967E38F;
            p261.write_speed = (float) -1.2592937E38F;
            p261.time_boot_ms = (uint)4204641248U;
            p261.used_capacity = (float) -7.195432E37F;
            LoopBackDemoChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.recording_time_ms == (uint)469423830U);
                Debug.Assert(pack.time_boot_ms == (uint)2850311848U);
                Debug.Assert(pack.available_capacity == (float) -2.7332502E38F);
                Debug.Assert(pack.image_status == (byte)(byte)91);
                Debug.Assert(pack.image_interval == (float)6.6962304E37F);
                Debug.Assert(pack.video_status == (byte)(byte)166);
            };
            DemoDevice.CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.image_status = (byte)(byte)91;
            p262.available_capacity = (float) -2.7332502E38F;
            p262.image_interval = (float)6.6962304E37F;
            p262.time_boot_ms = (uint)2850311848U;
            p262.recording_time_ms = (uint)469423830U;
            p262.video_status = (byte)(byte)166;
            LoopBackDemoChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (int) -2025113380);
                Debug.Assert(pack.time_boot_ms == (uint)3611592611U);
                Debug.Assert(pack.image_index == (int)94508348);
                Debug.Assert(pack.camera_id == (byte)(byte)51);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-8.3525133E37F, 2.9023242E38F, -2.6423572E38F, -2.965018E38F}));
                Debug.Assert(pack.file_url_LEN(ph) == 151);
                Debug.Assert(pack.file_url_TRY(ph).Equals("ehzqtdgqzEljjnmXhveapjnucgtehoafAlapoluZxvcqtpQyvayzbmncudwujekkDyubdubpzayFmlmwqyqjiXWhebjhjhbkmleaimohhGqOnkrvgsmlthwrkwlvbnZzfNjxrpwmrgldjtaytpmklvb"));
                Debug.Assert(pack.relative_alt == (int)461040744);
                Debug.Assert(pack.lon == (int) -587685116);
                Debug.Assert(pack.lat == (int) -232669435);
                Debug.Assert(pack.capture_result == (sbyte)(sbyte)117);
                Debug.Assert(pack.time_utc == (ulong)1787830715801715991L);
            };
            DemoDevice.CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.file_url_SET("ehzqtdgqzEljjnmXhveapjnucgtehoafAlapoluZxvcqtpQyvayzbmncudwujekkDyubdubpzayFmlmwqyqjiXWhebjhjhbkmleaimohhGqOnkrvgsmlthwrkwlvbnZzfNjxrpwmrgldjtaytpmklvb", PH) ;
            p263.relative_alt = (int)461040744;
            p263.lat = (int) -232669435;
            p263.time_utc = (ulong)1787830715801715991L;
            p263.image_index = (int)94508348;
            p263.camera_id = (byte)(byte)51;
            p263.q_SET(new float[] {-8.3525133E37F, 2.9023242E38F, -2.6423572E38F, -2.965018E38F}, 0) ;
            p263.time_boot_ms = (uint)3611592611U;
            p263.lon = (int) -587685116;
            p263.capture_result = (sbyte)(sbyte)117;
            p263.alt = (int) -2025113380;
            LoopBackDemoChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flight_uuid == (ulong)1120848575073010080L);
                Debug.Assert(pack.arming_time_utc == (ulong)6564332866611245345L);
                Debug.Assert(pack.takeoff_time_utc == (ulong)761292069960163518L);
                Debug.Assert(pack.time_boot_ms == (uint)4223572910U);
            };
            DemoDevice.FLIGHT_INFORMATION p264 = LoopBackDemoChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)4223572910U;
            p264.takeoff_time_utc = (ulong)761292069960163518L;
            p264.arming_time_utc = (ulong)6564332866611245345L;
            p264.flight_uuid = (ulong)1120848575073010080L;
            LoopBackDemoChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)578707900U);
                Debug.Assert(pack.yaw == (float) -2.1474455E38F);
                Debug.Assert(pack.roll == (float) -4.983047E37F);
                Debug.Assert(pack.pitch == (float) -2.1543063E38F);
            };
            DemoDevice.MOUNT_ORIENTATION p265 = LoopBackDemoChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.yaw = (float) -2.1474455E38F;
            p265.time_boot_ms = (uint)578707900U;
            p265.roll = (float) -4.983047E37F;
            p265.pitch = (float) -2.1543063E38F;
            LoopBackDemoChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)78, (byte)188, (byte)23, (byte)97, (byte)206, (byte)196, (byte)40, (byte)36, (byte)105, (byte)34, (byte)225, (byte)108, (byte)215, (byte)205, (byte)7, (byte)242, (byte)160, (byte)144, (byte)204, (byte)106, (byte)70, (byte)65, (byte)196, (byte)76, (byte)199, (byte)87, (byte)91, (byte)124, (byte)243, (byte)72, (byte)214, (byte)244, (byte)209, (byte)78, (byte)114, (byte)76, (byte)13, (byte)49, (byte)220, (byte)229, (byte)105, (byte)5, (byte)237, (byte)48, (byte)28, (byte)167, (byte)185, (byte)247, (byte)225, (byte)61, (byte)12, (byte)232, (byte)234, (byte)122, (byte)68, (byte)135, (byte)145, (byte)83, (byte)116, (byte)244, (byte)96, (byte)162, (byte)249, (byte)61, (byte)89, (byte)119, (byte)48, (byte)55, (byte)152, (byte)127, (byte)23, (byte)175, (byte)180, (byte)230, (byte)49, (byte)239, (byte)250, (byte)156, (byte)146, (byte)213, (byte)229, (byte)116, (byte)168, (byte)61, (byte)96, (byte)151, (byte)191, (byte)78, (byte)50, (byte)91, (byte)85, (byte)140, (byte)189, (byte)134, (byte)126, (byte)20, (byte)43, (byte)57, (byte)239, (byte)51, (byte)1, (byte)183, (byte)161, (byte)12, (byte)81, (byte)201, (byte)77, (byte)122, (byte)225, (byte)40, (byte)33, (byte)237, (byte)32, (byte)126, (byte)114, (byte)131, (byte)38, (byte)24, (byte)239, (byte)186, (byte)2, (byte)31, (byte)199, (byte)2, (byte)174, (byte)83, (byte)185, (byte)162, (byte)169, (byte)15, (byte)248, (byte)158, (byte)31, (byte)182, (byte)9, (byte)2, (byte)7, (byte)110, (byte)215, (byte)4, (byte)67, (byte)39, (byte)28, (byte)25, (byte)46, (byte)63, (byte)6, (byte)227, (byte)248, (byte)138, (byte)20, (byte)221, (byte)55, (byte)198, (byte)163, (byte)78, (byte)23, (byte)69, (byte)188, (byte)114, (byte)237, (byte)103, (byte)197, (byte)211, (byte)218, (byte)188, (byte)232, (byte)29, (byte)161, (byte)107, (byte)120, (byte)116, (byte)55, (byte)27, (byte)219, (byte)159, (byte)38, (byte)125, (byte)172, (byte)248, (byte)218, (byte)59, (byte)143, (byte)168, (byte)187, (byte)175, (byte)70, (byte)231, (byte)8, (byte)45, (byte)39, (byte)216, (byte)57, (byte)203, (byte)248, (byte)91, (byte)218, (byte)81, (byte)154, (byte)50, (byte)147, (byte)24, (byte)16, (byte)165, (byte)233, (byte)83, (byte)217, (byte)225, (byte)25, (byte)240, (byte)219, (byte)209, (byte)90, (byte)139, (byte)115, (byte)115, (byte)241, (byte)14, (byte)148, (byte)240, (byte)27, (byte)135, (byte)133, (byte)111, (byte)201, (byte)21, (byte)71, (byte)58, (byte)78, (byte)140, (byte)217, (byte)185, (byte)117, (byte)56, (byte)75, (byte)11, (byte)168, (byte)6, (byte)100, (byte)54, (byte)98, (byte)255, (byte)138, (byte)236, (byte)55, (byte)32, (byte)117, (byte)11, (byte)1}));
                Debug.Assert(pack.sequence == (ushort)(ushort)26992);
                Debug.Assert(pack.target_component == (byte)(byte)123);
                Debug.Assert(pack.target_system == (byte)(byte)136);
                Debug.Assert(pack.length == (byte)(byte)87);
                Debug.Assert(pack.first_message_offset == (byte)(byte)0);
            };
            DemoDevice.LOGGING_DATA p266 = LoopBackDemoChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_component = (byte)(byte)123;
            p266.data__SET(new byte[] {(byte)78, (byte)188, (byte)23, (byte)97, (byte)206, (byte)196, (byte)40, (byte)36, (byte)105, (byte)34, (byte)225, (byte)108, (byte)215, (byte)205, (byte)7, (byte)242, (byte)160, (byte)144, (byte)204, (byte)106, (byte)70, (byte)65, (byte)196, (byte)76, (byte)199, (byte)87, (byte)91, (byte)124, (byte)243, (byte)72, (byte)214, (byte)244, (byte)209, (byte)78, (byte)114, (byte)76, (byte)13, (byte)49, (byte)220, (byte)229, (byte)105, (byte)5, (byte)237, (byte)48, (byte)28, (byte)167, (byte)185, (byte)247, (byte)225, (byte)61, (byte)12, (byte)232, (byte)234, (byte)122, (byte)68, (byte)135, (byte)145, (byte)83, (byte)116, (byte)244, (byte)96, (byte)162, (byte)249, (byte)61, (byte)89, (byte)119, (byte)48, (byte)55, (byte)152, (byte)127, (byte)23, (byte)175, (byte)180, (byte)230, (byte)49, (byte)239, (byte)250, (byte)156, (byte)146, (byte)213, (byte)229, (byte)116, (byte)168, (byte)61, (byte)96, (byte)151, (byte)191, (byte)78, (byte)50, (byte)91, (byte)85, (byte)140, (byte)189, (byte)134, (byte)126, (byte)20, (byte)43, (byte)57, (byte)239, (byte)51, (byte)1, (byte)183, (byte)161, (byte)12, (byte)81, (byte)201, (byte)77, (byte)122, (byte)225, (byte)40, (byte)33, (byte)237, (byte)32, (byte)126, (byte)114, (byte)131, (byte)38, (byte)24, (byte)239, (byte)186, (byte)2, (byte)31, (byte)199, (byte)2, (byte)174, (byte)83, (byte)185, (byte)162, (byte)169, (byte)15, (byte)248, (byte)158, (byte)31, (byte)182, (byte)9, (byte)2, (byte)7, (byte)110, (byte)215, (byte)4, (byte)67, (byte)39, (byte)28, (byte)25, (byte)46, (byte)63, (byte)6, (byte)227, (byte)248, (byte)138, (byte)20, (byte)221, (byte)55, (byte)198, (byte)163, (byte)78, (byte)23, (byte)69, (byte)188, (byte)114, (byte)237, (byte)103, (byte)197, (byte)211, (byte)218, (byte)188, (byte)232, (byte)29, (byte)161, (byte)107, (byte)120, (byte)116, (byte)55, (byte)27, (byte)219, (byte)159, (byte)38, (byte)125, (byte)172, (byte)248, (byte)218, (byte)59, (byte)143, (byte)168, (byte)187, (byte)175, (byte)70, (byte)231, (byte)8, (byte)45, (byte)39, (byte)216, (byte)57, (byte)203, (byte)248, (byte)91, (byte)218, (byte)81, (byte)154, (byte)50, (byte)147, (byte)24, (byte)16, (byte)165, (byte)233, (byte)83, (byte)217, (byte)225, (byte)25, (byte)240, (byte)219, (byte)209, (byte)90, (byte)139, (byte)115, (byte)115, (byte)241, (byte)14, (byte)148, (byte)240, (byte)27, (byte)135, (byte)133, (byte)111, (byte)201, (byte)21, (byte)71, (byte)58, (byte)78, (byte)140, (byte)217, (byte)185, (byte)117, (byte)56, (byte)75, (byte)11, (byte)168, (byte)6, (byte)100, (byte)54, (byte)98, (byte)255, (byte)138, (byte)236, (byte)55, (byte)32, (byte)117, (byte)11, (byte)1}, 0) ;
            p266.length = (byte)(byte)87;
            p266.first_message_offset = (byte)(byte)0;
            p266.target_system = (byte)(byte)136;
            p266.sequence = (ushort)(ushort)26992;
            LoopBackDemoChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)77);
                Debug.Assert(pack.first_message_offset == (byte)(byte)152);
                Debug.Assert(pack.length == (byte)(byte)89);
                Debug.Assert(pack.target_component == (byte)(byte)129);
                Debug.Assert(pack.sequence == (ushort)(ushort)29210);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)131, (byte)30, (byte)211, (byte)81, (byte)42, (byte)0, (byte)188, (byte)233, (byte)182, (byte)152, (byte)32, (byte)10, (byte)52, (byte)21, (byte)210, (byte)121, (byte)26, (byte)203, (byte)17, (byte)152, (byte)173, (byte)11, (byte)253, (byte)210, (byte)148, (byte)219, (byte)84, (byte)45, (byte)54, (byte)173, (byte)239, (byte)7, (byte)146, (byte)154, (byte)145, (byte)12, (byte)1, (byte)93, (byte)89, (byte)227, (byte)89, (byte)253, (byte)151, (byte)123, (byte)24, (byte)219, (byte)243, (byte)135, (byte)84, (byte)174, (byte)190, (byte)55, (byte)162, (byte)45, (byte)24, (byte)249, (byte)101, (byte)98, (byte)190, (byte)19, (byte)131, (byte)233, (byte)159, (byte)206, (byte)168, (byte)177, (byte)33, (byte)157, (byte)182, (byte)39, (byte)130, (byte)63, (byte)84, (byte)69, (byte)92, (byte)98, (byte)97, (byte)234, (byte)186, (byte)17, (byte)155, (byte)135, (byte)32, (byte)204, (byte)28, (byte)226, (byte)194, (byte)48, (byte)55, (byte)240, (byte)214, (byte)214, (byte)152, (byte)124, (byte)45, (byte)139, (byte)253, (byte)98, (byte)230, (byte)254, (byte)207, (byte)207, (byte)80, (byte)78, (byte)255, (byte)167, (byte)243, (byte)16, (byte)110, (byte)126, (byte)21, (byte)60, (byte)33, (byte)160, (byte)146, (byte)16, (byte)47, (byte)184, (byte)216, (byte)130, (byte)56, (byte)62, (byte)35, (byte)1, (byte)51, (byte)105, (byte)62, (byte)102, (byte)59, (byte)230, (byte)78, (byte)240, (byte)160, (byte)105, (byte)218, (byte)96, (byte)18, (byte)79, (byte)11, (byte)174, (byte)142, (byte)7, (byte)167, (byte)57, (byte)248, (byte)117, (byte)146, (byte)103, (byte)194, (byte)191, (byte)8, (byte)222, (byte)140, (byte)121, (byte)164, (byte)241, (byte)51, (byte)220, (byte)44, (byte)133, (byte)51, (byte)191, (byte)188, (byte)63, (byte)205, (byte)254, (byte)253, (byte)3, (byte)222, (byte)228, (byte)216, (byte)0, (byte)178, (byte)121, (byte)124, (byte)237, (byte)68, (byte)114, (byte)1, (byte)38, (byte)116, (byte)215, (byte)21, (byte)94, (byte)196, (byte)67, (byte)183, (byte)143, (byte)68, (byte)182, (byte)29, (byte)68, (byte)7, (byte)77, (byte)120, (byte)33, (byte)59, (byte)207, (byte)127, (byte)209, (byte)68, (byte)201, (byte)108, (byte)29, (byte)78, (byte)67, (byte)219, (byte)122, (byte)210, (byte)21, (byte)64, (byte)243, (byte)8, (byte)221, (byte)13, (byte)11, (byte)107, (byte)48, (byte)151, (byte)156, (byte)148, (byte)160, (byte)92, (byte)161, (byte)178, (byte)93, (byte)222, (byte)157, (byte)239, (byte)173, (byte)45, (byte)22, (byte)186, (byte)204, (byte)143, (byte)226, (byte)143, (byte)223, (byte)55, (byte)242, (byte)253, (byte)29, (byte)27, (byte)166, (byte)126, (byte)117, (byte)93, (byte)85, (byte)57}));
            };
            DemoDevice.LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_system = (byte)(byte)77;
            p267.target_component = (byte)(byte)129;
            p267.sequence = (ushort)(ushort)29210;
            p267.data__SET(new byte[] {(byte)131, (byte)30, (byte)211, (byte)81, (byte)42, (byte)0, (byte)188, (byte)233, (byte)182, (byte)152, (byte)32, (byte)10, (byte)52, (byte)21, (byte)210, (byte)121, (byte)26, (byte)203, (byte)17, (byte)152, (byte)173, (byte)11, (byte)253, (byte)210, (byte)148, (byte)219, (byte)84, (byte)45, (byte)54, (byte)173, (byte)239, (byte)7, (byte)146, (byte)154, (byte)145, (byte)12, (byte)1, (byte)93, (byte)89, (byte)227, (byte)89, (byte)253, (byte)151, (byte)123, (byte)24, (byte)219, (byte)243, (byte)135, (byte)84, (byte)174, (byte)190, (byte)55, (byte)162, (byte)45, (byte)24, (byte)249, (byte)101, (byte)98, (byte)190, (byte)19, (byte)131, (byte)233, (byte)159, (byte)206, (byte)168, (byte)177, (byte)33, (byte)157, (byte)182, (byte)39, (byte)130, (byte)63, (byte)84, (byte)69, (byte)92, (byte)98, (byte)97, (byte)234, (byte)186, (byte)17, (byte)155, (byte)135, (byte)32, (byte)204, (byte)28, (byte)226, (byte)194, (byte)48, (byte)55, (byte)240, (byte)214, (byte)214, (byte)152, (byte)124, (byte)45, (byte)139, (byte)253, (byte)98, (byte)230, (byte)254, (byte)207, (byte)207, (byte)80, (byte)78, (byte)255, (byte)167, (byte)243, (byte)16, (byte)110, (byte)126, (byte)21, (byte)60, (byte)33, (byte)160, (byte)146, (byte)16, (byte)47, (byte)184, (byte)216, (byte)130, (byte)56, (byte)62, (byte)35, (byte)1, (byte)51, (byte)105, (byte)62, (byte)102, (byte)59, (byte)230, (byte)78, (byte)240, (byte)160, (byte)105, (byte)218, (byte)96, (byte)18, (byte)79, (byte)11, (byte)174, (byte)142, (byte)7, (byte)167, (byte)57, (byte)248, (byte)117, (byte)146, (byte)103, (byte)194, (byte)191, (byte)8, (byte)222, (byte)140, (byte)121, (byte)164, (byte)241, (byte)51, (byte)220, (byte)44, (byte)133, (byte)51, (byte)191, (byte)188, (byte)63, (byte)205, (byte)254, (byte)253, (byte)3, (byte)222, (byte)228, (byte)216, (byte)0, (byte)178, (byte)121, (byte)124, (byte)237, (byte)68, (byte)114, (byte)1, (byte)38, (byte)116, (byte)215, (byte)21, (byte)94, (byte)196, (byte)67, (byte)183, (byte)143, (byte)68, (byte)182, (byte)29, (byte)68, (byte)7, (byte)77, (byte)120, (byte)33, (byte)59, (byte)207, (byte)127, (byte)209, (byte)68, (byte)201, (byte)108, (byte)29, (byte)78, (byte)67, (byte)219, (byte)122, (byte)210, (byte)21, (byte)64, (byte)243, (byte)8, (byte)221, (byte)13, (byte)11, (byte)107, (byte)48, (byte)151, (byte)156, (byte)148, (byte)160, (byte)92, (byte)161, (byte)178, (byte)93, (byte)222, (byte)157, (byte)239, (byte)173, (byte)45, (byte)22, (byte)186, (byte)204, (byte)143, (byte)226, (byte)143, (byte)223, (byte)55, (byte)242, (byte)253, (byte)29, (byte)27, (byte)166, (byte)126, (byte)117, (byte)93, (byte)85, (byte)57}, 0) ;
            p267.first_message_offset = (byte)(byte)152;
            p267.length = (byte)(byte)89;
            LoopBackDemoChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)237);
                Debug.Assert(pack.sequence == (ushort)(ushort)14552);
                Debug.Assert(pack.target_system == (byte)(byte)106);
            };
            DemoDevice.LOGGING_ACK p268 = LoopBackDemoChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_component = (byte)(byte)237;
            p268.target_system = (byte)(byte)106;
            p268.sequence = (ushort)(ushort)14552;
            LoopBackDemoChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.resolution_v == (ushort)(ushort)60121);
                Debug.Assert(pack.camera_id == (byte)(byte)78);
                Debug.Assert(pack.status == (byte)(byte)165);
                Debug.Assert(pack.bitrate == (uint)1508260001U);
                Debug.Assert(pack.framerate == (float)1.9135431E38F);
                Debug.Assert(pack.uri_LEN(ph) == 150);
                Debug.Assert(pack.uri_TRY(ph).Equals("ieZjdwzjkbtzElldYcaDybUnrZwdzejcdhakzGjHjHhucbzpbwyuysjuVzrublnrheqHlhypubignpkLnwjphtxzVmxiiowGiirKdqspazhdsfxxhuogazhjxczaezsydokiSafajzoduikbdhflmy"));
                Debug.Assert(pack.resolution_h == (ushort)(ushort)28792);
                Debug.Assert(pack.rotation == (ushort)(ushort)16592);
            };
            DemoDevice.VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.framerate = (float)1.9135431E38F;
            p269.camera_id = (byte)(byte)78;
            p269.bitrate = (uint)1508260001U;
            p269.resolution_h = (ushort)(ushort)28792;
            p269.status = (byte)(byte)165;
            p269.resolution_v = (ushort)(ushort)60121;
            p269.rotation = (ushort)(ushort)16592;
            p269.uri_SET("ieZjdwzjkbtzElldYcaDybUnrZwdzejcdhakzGjHjHhucbzpbwyuysjuVzrublnrheqHlhypubignpkLnwjphtxzVmxiiowGiirKdqspazhdsfxxhuogazhjxczaezsydokiSafajzoduikbdhflmy", PH) ;
            LoopBackDemoChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uri_LEN(ph) == 117);
                Debug.Assert(pack.uri_TRY(ph).Equals("ctvgIoeqYnmhyxzbWsclafcspzbgbouefwxdwObjfZzvfTxVoHcxloacmkxeffuwxyigkyczrlwpqrrnkymLslllwcoaDeynpmxsxxabnzbytilzbkbSx"));
                Debug.Assert(pack.bitrate == (uint)3301242793U);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)54078);
                Debug.Assert(pack.target_system == (byte)(byte)238);
                Debug.Assert(pack.target_component == (byte)(byte)57);
                Debug.Assert(pack.rotation == (ushort)(ushort)25373);
                Debug.Assert(pack.framerate == (float) -6.605688E37F);
                Debug.Assert(pack.camera_id == (byte)(byte)155);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)50366);
            };
            DemoDevice.SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.bitrate = (uint)3301242793U;
            p270.camera_id = (byte)(byte)155;
            p270.rotation = (ushort)(ushort)25373;
            p270.framerate = (float) -6.605688E37F;
            p270.resolution_h = (ushort)(ushort)50366;
            p270.resolution_v = (ushort)(ushort)54078;
            p270.target_component = (byte)(byte)57;
            p270.target_system = (byte)(byte)238;
            p270.uri_SET("ctvgIoeqYnmhyxzbWsclafcspzbgbouefwxdwObjfZzvfTxVoHcxloacmkxeffuwxyigkyczrlwpqrrnkymLslllwcoaDeynpmxsxxabnzbytilzbkbSx", PH) ;
            LoopBackDemoChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ssid_LEN(ph) == 25);
                Debug.Assert(pack.ssid_TRY(ph).Equals("dcEbwtmwtcvCdfyyzjpxqgmDH"));
                Debug.Assert(pack.password_LEN(ph) == 7);
                Debug.Assert(pack.password_TRY(ph).Equals("ibpbkeh"));
            };
            DemoDevice.WIFI_CONFIG_AP p299 = LoopBackDemoChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.password_SET("ibpbkeh", PH) ;
            p299.ssid_SET("dcEbwtmwtcvCdfyyzjpxqgmDH", PH) ;
            LoopBackDemoChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)61, (byte)99, (byte)217, (byte)47, (byte)31, (byte)132, (byte)1, (byte)205}));
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)249, (byte)190, (byte)180, (byte)238, (byte)100, (byte)180, (byte)128, (byte)104}));
                Debug.Assert(pack.version == (ushort)(ushort)34709);
                Debug.Assert(pack.min_version == (ushort)(ushort)23503);
                Debug.Assert(pack.max_version == (ushort)(ushort)3939);
            };
            DemoDevice.PROTOCOL_VERSION p300 = LoopBackDemoChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.library_version_hash_SET(new byte[] {(byte)249, (byte)190, (byte)180, (byte)238, (byte)100, (byte)180, (byte)128, (byte)104}, 0) ;
            p300.version = (ushort)(ushort)34709;
            p300.max_version = (ushort)(ushort)3939;
            p300.min_version = (ushort)(ushort)23503;
            p300.spec_version_hash_SET(new byte[] {(byte)61, (byte)99, (byte)217, (byte)47, (byte)31, (byte)132, (byte)1, (byte)205}, 0) ;
            LoopBackDemoChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)54369);
                Debug.Assert(pack.health == (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING);
                Debug.Assert(pack.mode == (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE);
                Debug.Assert(pack.time_usec == (ulong)1907480762787020982L);
                Debug.Assert(pack.sub_mode == (byte)(byte)150);
                Debug.Assert(pack.uptime_sec == (uint)4227816163U);
            };
            DemoDevice.UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.uptime_sec = (uint)4227816163U;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE;
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING;
            p310.sub_mode = (byte)(byte)150;
            p310.vendor_specific_status_code = (ushort)(ushort)54369;
            p310.time_usec = (ulong)1907480762787020982L;
            LoopBackDemoChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hw_version_major == (byte)(byte)99);
                Debug.Assert(pack.name_LEN(ph) == 55);
                Debug.Assert(pack.name_TRY(ph).Equals("igOtwArymskljgEfkhkzaauxcxiwjjyyyctgmnfshkejrqyvdidjagf"));
                Debug.Assert(pack.time_usec == (ulong)5305325306517959571L);
                Debug.Assert(pack.hw_version_minor == (byte)(byte)249);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)71, (byte)17, (byte)184, (byte)182, (byte)240, (byte)169, (byte)184, (byte)1, (byte)85, (byte)217, (byte)78, (byte)71, (byte)191, (byte)158, (byte)39, (byte)104}));
                Debug.Assert(pack.uptime_sec == (uint)2833829473U);
                Debug.Assert(pack.sw_version_major == (byte)(byte)147);
                Debug.Assert(pack.sw_version_minor == (byte)(byte)98);
                Debug.Assert(pack.sw_vcs_commit == (uint)3732198523U);
            };
            DemoDevice.UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.sw_version_minor = (byte)(byte)98;
            p311.time_usec = (ulong)5305325306517959571L;
            p311.sw_version_major = (byte)(byte)147;
            p311.uptime_sec = (uint)2833829473U;
            p311.sw_vcs_commit = (uint)3732198523U;
            p311.hw_unique_id_SET(new byte[] {(byte)71, (byte)17, (byte)184, (byte)182, (byte)240, (byte)169, (byte)184, (byte)1, (byte)85, (byte)217, (byte)78, (byte)71, (byte)191, (byte)158, (byte)39, (byte)104}, 0) ;
            p311.name_SET("igOtwArymskljgEfkhkzaauxcxiwjjyyyctgmnfshkejrqyvdidjagf", PH) ;
            p311.hw_version_minor = (byte)(byte)249;
            p311.hw_version_major = (byte)(byte)99;
            LoopBackDemoChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)226);
                Debug.Assert(pack.param_id_LEN(ph) == 1);
                Debug.Assert(pack.param_id_TRY(ph).Equals("r"));
                Debug.Assert(pack.param_index == (short)(short)495);
                Debug.Assert(pack.target_component == (byte)(byte)13);
            };
            DemoDevice.PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.param_index = (short)(short)495;
            p320.target_system = (byte)(byte)226;
            p320.param_id_SET("r", PH) ;
            p320.target_component = (byte)(byte)13;
            LoopBackDemoChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)62);
                Debug.Assert(pack.target_component == (byte)(byte)218);
            };
            DemoDevice.PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)62;
            p321.target_component = (byte)(byte)218;
            LoopBackDemoChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_count == (ushort)(ushort)51314);
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
                Debug.Assert(pack.param_value_LEN(ph) == 99);
                Debug.Assert(pack.param_value_TRY(ph).Equals("wKdykbvvtJpnvkbdkyttbqwaSaaimqxzLhpoqtYtuqdbwjclhckuqgklisyGhkhwFbaynxjlgVbtbWjfteivpkkgarmuteUpceh"));
                Debug.Assert(pack.param_index == (ushort)(ushort)37735);
                Debug.Assert(pack.param_id_LEN(ph) == 10);
                Debug.Assert(pack.param_id_TRY(ph).Equals("zqhvkkgyem"));
            };
            DemoDevice.PARAM_EXT_VALUE p322 = LoopBackDemoChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("zqhvkkgyem", PH) ;
            p322.param_index = (ushort)(ushort)37735;
            p322.param_value_SET("wKdykbvvtJpnvkbdkyttbqwaSaaimqxzLhpoqtYtuqdbwjclhckuqgklisyGhkhwFbaynxjlgVbtbWjfteivpkkgarmuteUpceh", PH) ;
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32;
            p322.param_count = (ushort)(ushort)51314;
            LoopBackDemoChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)95);
                Debug.Assert(pack.target_system == (byte)(byte)2);
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64);
                Debug.Assert(pack.param_value_LEN(ph) == 108);
                Debug.Assert(pack.param_value_TRY(ph).Equals("jxhpxkmfayltiomksuaptjrmpgkusoqybiksvyvuexqdvralwpkKmaeggoamwrgilxpwsEwzncwnktmopbubGlfSebdagctqmwseupqbMjvl"));
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("OwUws"));
            };
            DemoDevice.PARAM_EXT_SET p323 = LoopBackDemoChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.param_value_SET("jxhpxkmfayltiomksuaptjrmpgkusoqybiksvyvuexqdvralwpkKmaeggoamwrgilxpwsEwzncwnktmopbubGlfSebdagctqmwseupqbMjvl", PH) ;
            p323.param_id_SET("OwUws", PH) ;
            p323.target_component = (byte)(byte)95;
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64;
            p323.target_system = (byte)(byte)2;
            LoopBackDemoChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32);
                Debug.Assert(pack.param_value_LEN(ph) == 12);
                Debug.Assert(pack.param_value_TRY(ph).Equals("wlxfubbspfpn"));
                Debug.Assert(pack.param_result == (PARAM_ACK)PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED);
                Debug.Assert(pack.param_id_LEN(ph) == 8);
                Debug.Assert(pack.param_id_TRY(ph).Equals("bqizcufs"));
            };
            DemoDevice.PARAM_EXT_ACK p324 = LoopBackDemoChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("bqizcufs", PH) ;
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32;
            p324.param_value_SET("wlxfubbspfpn", PH) ;
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED;
            LoopBackDemoChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.max_distance == (ushort)(ushort)7092);
                Debug.Assert(pack.min_distance == (ushort)(ushort)63665);
                Debug.Assert(pack.time_usec == (ulong)9217375438312903391L);
                Debug.Assert(pack.sensor_type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)15559, (ushort)40774, (ushort)41245, (ushort)44871, (ushort)37787, (ushort)28574, (ushort)58670, (ushort)48388, (ushort)21168, (ushort)39928, (ushort)26134, (ushort)3601, (ushort)30422, (ushort)55429, (ushort)54304, (ushort)29469, (ushort)63553, (ushort)56275, (ushort)21503, (ushort)63576, (ushort)49928, (ushort)28547, (ushort)23230, (ushort)6239, (ushort)9263, (ushort)63453, (ushort)3254, (ushort)16579, (ushort)55679, (ushort)25252, (ushort)38060, (ushort)2946, (ushort)12795, (ushort)62954, (ushort)35038, (ushort)33169, (ushort)57561, (ushort)6649, (ushort)39351, (ushort)41568, (ushort)15331, (ushort)22276, (ushort)6592, (ushort)23056, (ushort)25824, (ushort)62683, (ushort)30140, (ushort)64180, (ushort)35555, (ushort)49348, (ushort)10234, (ushort)9039, (ushort)21599, (ushort)7079, (ushort)57188, (ushort)2938, (ushort)64934, (ushort)3992, (ushort)5649, (ushort)23140, (ushort)50727, (ushort)43285, (ushort)3067, (ushort)49311, (ushort)61495, (ushort)29766, (ushort)24599, (ushort)3160, (ushort)55951, (ushort)57596, (ushort)4568, (ushort)3794}));
                Debug.Assert(pack.increment == (byte)(byte)71);
            };
            DemoDevice.OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.increment = (byte)(byte)71;
            p330.time_usec = (ulong)9217375438312903391L;
            p330.max_distance = (ushort)(ushort)7092;
            p330.distances_SET(new ushort[] {(ushort)15559, (ushort)40774, (ushort)41245, (ushort)44871, (ushort)37787, (ushort)28574, (ushort)58670, (ushort)48388, (ushort)21168, (ushort)39928, (ushort)26134, (ushort)3601, (ushort)30422, (ushort)55429, (ushort)54304, (ushort)29469, (ushort)63553, (ushort)56275, (ushort)21503, (ushort)63576, (ushort)49928, (ushort)28547, (ushort)23230, (ushort)6239, (ushort)9263, (ushort)63453, (ushort)3254, (ushort)16579, (ushort)55679, (ushort)25252, (ushort)38060, (ushort)2946, (ushort)12795, (ushort)62954, (ushort)35038, (ushort)33169, (ushort)57561, (ushort)6649, (ushort)39351, (ushort)41568, (ushort)15331, (ushort)22276, (ushort)6592, (ushort)23056, (ushort)25824, (ushort)62683, (ushort)30140, (ushort)64180, (ushort)35555, (ushort)49348, (ushort)10234, (ushort)9039, (ushort)21599, (ushort)7079, (ushort)57188, (ushort)2938, (ushort)64934, (ushort)3992, (ushort)5649, (ushort)23140, (ushort)50727, (ushort)43285, (ushort)3067, (ushort)49311, (ushort)61495, (ushort)29766, (ushort)24599, (ushort)3160, (ushort)55951, (ushort)57596, (ushort)4568, (ushort)3794}, 0) ;
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            p330.min_distance = (ushort)(ushort)63665;
            LoopBackDemoChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
        }
    }
}