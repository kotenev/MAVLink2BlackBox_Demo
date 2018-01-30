
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
                Debug.Assert(pack.custom_mode == (uint)2425377368U);
                Debug.Assert(pack.system_status == (MAV_STATE)MAV_STATE.MAV_STATE_ACTIVE);
                Debug.Assert(pack.autopilot == (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY);
                Debug.Assert(pack.mavlink_version == (byte)(byte)136);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED);
                Debug.Assert(pack.type == (MAV_TYPE)MAV_TYPE.MAV_TYPE_GCS);
            };
            DemoDevice.HEARTBEAT p0 = LoopBackDemoChannel.new_HEARTBEAT();
            PH.setPack(p0);
            p0.mavlink_version = (byte)(byte)136;
            p0.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED;
            p0.autopilot = (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY;
            p0.type = (MAV_TYPE)MAV_TYPE.MAV_TYPE_GCS;
            p0.custom_mode = (uint)2425377368U;
            p0.system_status = (MAV_STATE)MAV_STATE.MAV_STATE_ACTIVE;
            LoopBackDemoChannel.instance.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)41);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)3539);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)16088);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)36768);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)28534);
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)51564);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2);
                Debug.Assert(pack.load == (ushort)(ushort)31209);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)38391);
                Debug.Assert(pack.current_battery == (short)(short) -18181);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)47571);
            };
            DemoDevice.SYS_STATUS p1 = LoopBackDemoChannel.new_SYS_STATUS();
            PH.setPack(p1);
            p1.errors_count2 = (ushort)(ushort)28534;
            p1.load = (ushort)(ushort)31209;
            p1.errors_count1 = (ushort)(ushort)3539;
            p1.drop_rate_comm = (ushort)(ushort)51564;
            p1.errors_comm = (ushort)(ushort)16088;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG;
            p1.current_battery = (short)(short) -18181;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN;
            p1.battery_remaining = (sbyte)(sbyte)41;
            p1.errors_count4 = (ushort)(ushort)36768;
            p1.errors_count3 = (ushort)(ushort)47571;
            p1.voltage_battery = (ushort)(ushort)38391;
            LoopBackDemoChannel.instance.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_unix_usec == (ulong)6239389165864021218L);
                Debug.Assert(pack.time_boot_ms == (uint)3746243534U);
            };
            DemoDevice.SYSTEM_TIME p2 = LoopBackDemoChannel.new_SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)6239389165864021218L;
            p2.time_boot_ms = (uint)3746243534U;
            LoopBackDemoChannel.instance.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type_mask == (ushort)(ushort)21698);
                Debug.Assert(pack.yaw == (float) -1.2737644E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.y == (float) -9.576172E37F);
                Debug.Assert(pack.afx == (float)2.5799454E38F);
                Debug.Assert(pack.yaw_rate == (float)2.8574305E38F);
                Debug.Assert(pack.vy == (float)2.4451027E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2476493043U);
                Debug.Assert(pack.vx == (float) -8.189839E37F);
                Debug.Assert(pack.z == (float) -1.8733418E38F);
                Debug.Assert(pack.vz == (float)2.7248981E38F);
                Debug.Assert(pack.afz == (float) -1.0743069E38F);
                Debug.Assert(pack.x == (float) -1.554413E37F);
                Debug.Assert(pack.afy == (float) -3.3520824E38F);
            };
            DemoDevice.POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.y = (float) -9.576172E37F;
            p3.x = (float) -1.554413E37F;
            p3.vx = (float) -8.189839E37F;
            p3.time_boot_ms = (uint)2476493043U;
            p3.vy = (float)2.4451027E38F;
            p3.yaw_rate = (float)2.8574305E38F;
            p3.vz = (float)2.7248981E38F;
            p3.z = (float) -1.8733418E38F;
            p3.afz = (float) -1.0743069E38F;
            p3.type_mask = (ushort)(ushort)21698;
            p3.yaw = (float) -1.2737644E38F;
            p3.afx = (float)2.5799454E38F;
            p3.afy = (float) -3.3520824E38F;
            p3.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            LoopBackDemoChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)71);
                Debug.Assert(pack.target_component == (byte)(byte)167);
                Debug.Assert(pack.time_usec == (ulong)1231201638069574939L);
                Debug.Assert(pack.seq == (uint)997753314U);
            };
            DemoDevice.PING p4 = LoopBackDemoChannel.new_PING();
            PH.setPack(p4);
            p4.time_usec = (ulong)1231201638069574939L;
            p4.seq = (uint)997753314U;
            p4.target_component = (byte)(byte)167;
            p4.target_system = (byte)(byte)71;
            LoopBackDemoChannel.instance.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)24);
                Debug.Assert(pack.version == (byte)(byte)49);
                Debug.Assert(pack.control_request == (byte)(byte)198);
                Debug.Assert(pack.passkey_LEN(ph) == 6);
                Debug.Assert(pack.passkey_TRY(ph).Equals("ulaLkh"));
            };
            DemoDevice.CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.control_request = (byte)(byte)198;
            p5.passkey_SET("ulaLkh", PH) ;
            p5.version = (byte)(byte)49;
            p5.target_system = (byte)(byte)24;
            LoopBackDemoChannel.instance.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gcs_system_id == (byte)(byte)8);
                Debug.Assert(pack.ack == (byte)(byte)196);
                Debug.Assert(pack.control_request == (byte)(byte)62);
            };
            DemoDevice.CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.gcs_system_id = (byte)(byte)8;
            p6.control_request = (byte)(byte)62;
            p6.ack = (byte)(byte)196;
            LoopBackDemoChannel.instance.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 2);
                Debug.Assert(pack.key_TRY(ph).Equals("bh"));
            };
            DemoDevice.AUTH_KEY p7 = LoopBackDemoChannel.new_AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("bh", PH) ;
            LoopBackDemoChannel.instance.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.custom_mode == (uint)57422455U);
                Debug.Assert(pack.base_mode == (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
                Debug.Assert(pack.target_system == (byte)(byte)47);
            };
            DemoDevice.SET_MODE p11 = LoopBackDemoChannel.new_SET_MODE();
            PH.setPack(p11);
            p11.base_mode = (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_DISARMED;
            p11.custom_mode = (uint)57422455U;
            p11.target_system = (byte)(byte)47;
            LoopBackDemoChannel.instance.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 10);
                Debug.Assert(pack.param_id_TRY(ph).Equals("Xdhwvwnhln"));
                Debug.Assert(pack.target_system == (byte)(byte)90);
                Debug.Assert(pack.param_index == (short)(short) -28468);
                Debug.Assert(pack.target_component == (byte)(byte)222);
            };
            DemoDevice.PARAM_REQUEST_READ p20 = LoopBackDemoChannel.new_PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.param_id_SET("Xdhwvwnhln", PH) ;
            p20.target_system = (byte)(byte)90;
            p20.param_index = (short)(short) -28468;
            p20.target_component = (byte)(byte)222;
            LoopBackDemoChannel.instance.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)33);
                Debug.Assert(pack.target_component == (byte)(byte)122);
            };
            DemoDevice.PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.new_PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_component = (byte)(byte)122;
            p21.target_system = (byte)(byte)33;
            LoopBackDemoChannel.instance.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_count == (ushort)(ushort)59307);
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16);
                Debug.Assert(pack.param_value == (float)1.4003698E38F);
                Debug.Assert(pack.param_index == (ushort)(ushort)9760);
                Debug.Assert(pack.param_id_LEN(ph) == 2);
                Debug.Assert(pack.param_id_TRY(ph).Equals("gq"));
            };
            DemoDevice.PARAM_VALUE p22 = LoopBackDemoChannel.new_PARAM_VALUE();
            PH.setPack(p22);
            p22.param_value = (float)1.4003698E38F;
            p22.param_id_SET("gq", PH) ;
            p22.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16;
            p22.param_count = (ushort)(ushort)59307;
            p22.param_index = (ushort)(ushort)9760;
            LoopBackDemoChannel.instance.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)130);
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64);
                Debug.Assert(pack.param_id_LEN(ph) == 7);
                Debug.Assert(pack.param_id_TRY(ph).Equals("fzqpvyo"));
                Debug.Assert(pack.target_component == (byte)(byte)232);
                Debug.Assert(pack.param_value == (float)2.8009077E38F);
            };
            DemoDevice.PARAM_SET p23 = LoopBackDemoChannel.new_PARAM_SET();
            PH.setPack(p23);
            p23.param_value = (float)2.8009077E38F;
            p23.target_system = (byte)(byte)130;
            p23.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64;
            p23.target_component = (byte)(byte)232;
            p23.param_id_SET("fzqpvyo", PH) ;
            LoopBackDemoChannel.instance.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.cog == (ushort)(ushort)2607);
                Debug.Assert(pack.eph == (ushort)(ushort)28142);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int) -192028131);
                Debug.Assert(pack.satellites_visible == (byte)(byte)49);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)2683180987U);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)1700731066U);
                Debug.Assert(pack.epv == (ushort)(ushort)52227);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)3560895551U);
                Debug.Assert(pack.lon == (int) -1630964778);
                Debug.Assert(pack.time_usec == (ulong)798821582678318160L);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)957535825U);
                Debug.Assert(pack.lat == (int)691574064);
                Debug.Assert(pack.vel == (ushort)(ushort)46778);
                Debug.Assert(pack.alt == (int)1130992232);
            };
            DemoDevice.GPS_RAW_INT p24 = LoopBackDemoChannel.new_GPS_RAW_INT();
            PH.setPack(p24);
            p24.satellites_visible = (byte)(byte)49;
            p24.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p24.time_usec = (ulong)798821582678318160L;
            p24.eph = (ushort)(ushort)28142;
            p24.lat = (int)691574064;
            p24.h_acc_SET((uint)1700731066U, PH) ;
            p24.v_acc_SET((uint)2683180987U, PH) ;
            p24.alt_ellipsoid_SET((int) -192028131, PH) ;
            p24.epv = (ushort)(ushort)52227;
            p24.vel_acc_SET((uint)3560895551U, PH) ;
            p24.hdg_acc_SET((uint)957535825U, PH) ;
            p24.vel = (ushort)(ushort)46778;
            p24.cog = (ushort)(ushort)2607;
            p24.alt = (int)1130992232;
            p24.lon = (int) -1630964778;
            LoopBackDemoChannel.instance.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)116, (byte)77, (byte)47, (byte)151, (byte)71, (byte)8, (byte)254, (byte)175, (byte)136, (byte)193, (byte)72, (byte)163, (byte)92, (byte)134, (byte)76, (byte)83, (byte)172, (byte)1, (byte)66, (byte)68}));
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)100, (byte)3, (byte)163, (byte)199, (byte)251, (byte)219, (byte)108, (byte)181, (byte)181, (byte)71, (byte)242, (byte)238, (byte)226, (byte)19, (byte)91, (byte)215, (byte)11, (byte)18, (byte)146, (byte)181}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)74);
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)181, (byte)189, (byte)175, (byte)77, (byte)63, (byte)245, (byte)26, (byte)190, (byte)82, (byte)154, (byte)175, (byte)248, (byte)165, (byte)39, (byte)1, (byte)34, (byte)181, (byte)243, (byte)37, (byte)33}));
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)82, (byte)220, (byte)33, (byte)3, (byte)84, (byte)11, (byte)154, (byte)89, (byte)203, (byte)205, (byte)197, (byte)234, (byte)205, (byte)203, (byte)143, (byte)159, (byte)132, (byte)13, (byte)47, (byte)199}));
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)49, (byte)251, (byte)93, (byte)221, (byte)232, (byte)21, (byte)207, (byte)9, (byte)39, (byte)249, (byte)71, (byte)161, (byte)27, (byte)124, (byte)90, (byte)102, (byte)75, (byte)111, (byte)119, (byte)13}));
            };
            DemoDevice.GPS_STATUS p25 = LoopBackDemoChannel.new_GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_used_SET(new byte[] {(byte)49, (byte)251, (byte)93, (byte)221, (byte)232, (byte)21, (byte)207, (byte)9, (byte)39, (byte)249, (byte)71, (byte)161, (byte)27, (byte)124, (byte)90, (byte)102, (byte)75, (byte)111, (byte)119, (byte)13}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)82, (byte)220, (byte)33, (byte)3, (byte)84, (byte)11, (byte)154, (byte)89, (byte)203, (byte)205, (byte)197, (byte)234, (byte)205, (byte)203, (byte)143, (byte)159, (byte)132, (byte)13, (byte)47, (byte)199}, 0) ;
            p25.satellite_snr_SET(new byte[] {(byte)116, (byte)77, (byte)47, (byte)151, (byte)71, (byte)8, (byte)254, (byte)175, (byte)136, (byte)193, (byte)72, (byte)163, (byte)92, (byte)134, (byte)76, (byte)83, (byte)172, (byte)1, (byte)66, (byte)68}, 0) ;
            p25.satellite_prn_SET(new byte[] {(byte)100, (byte)3, (byte)163, (byte)199, (byte)251, (byte)219, (byte)108, (byte)181, (byte)181, (byte)71, (byte)242, (byte)238, (byte)226, (byte)19, (byte)91, (byte)215, (byte)11, (byte)18, (byte)146, (byte)181}, 0) ;
            p25.satellites_visible = (byte)(byte)74;
            p25.satellite_azimuth_SET(new byte[] {(byte)181, (byte)189, (byte)175, (byte)77, (byte)63, (byte)245, (byte)26, (byte)190, (byte)82, (byte)154, (byte)175, (byte)248, (byte)165, (byte)39, (byte)1, (byte)34, (byte)181, (byte)243, (byte)37, (byte)33}, 0) ;
            LoopBackDemoChannel.instance.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (short)(short) -31313);
                Debug.Assert(pack.zgyro == (short)(short) -2524);
                Debug.Assert(pack.zacc == (short)(short)7911);
                Debug.Assert(pack.yacc == (short)(short) -14665);
                Debug.Assert(pack.xgyro == (short)(short) -5945);
                Debug.Assert(pack.ygyro == (short)(short)2710);
                Debug.Assert(pack.ymag == (short)(short) -29869);
                Debug.Assert(pack.time_boot_ms == (uint)3815791908U);
                Debug.Assert(pack.zmag == (short)(short)10851);
                Debug.Assert(pack.xmag == (short)(short)5025);
            };
            DemoDevice.SCALED_IMU p26 = LoopBackDemoChannel.new_SCALED_IMU();
            PH.setPack(p26);
            p26.yacc = (short)(short) -14665;
            p26.ymag = (short)(short) -29869;
            p26.xgyro = (short)(short) -5945;
            p26.zacc = (short)(short)7911;
            p26.xmag = (short)(short)5025;
            p26.zmag = (short)(short)10851;
            p26.ygyro = (short)(short)2710;
            p26.xacc = (short)(short) -31313;
            p26.zgyro = (short)(short) -2524;
            p26.time_boot_ms = (uint)3815791908U;
            LoopBackDemoChannel.instance.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (short)(short)31898);
                Debug.Assert(pack.zacc == (short)(short)101);
                Debug.Assert(pack.xgyro == (short)(short)18891);
                Debug.Assert(pack.yacc == (short)(short)20330);
                Debug.Assert(pack.xmag == (short)(short)8871);
                Debug.Assert(pack.zmag == (short)(short) -19870);
                Debug.Assert(pack.ygyro == (short)(short)24324);
                Debug.Assert(pack.ymag == (short)(short) -5914);
                Debug.Assert(pack.zgyro == (short)(short) -24076);
                Debug.Assert(pack.time_usec == (ulong)2669459027421311672L);
            };
            DemoDevice.RAW_IMU p27 = LoopBackDemoChannel.new_RAW_IMU();
            PH.setPack(p27);
            p27.zacc = (short)(short)101;
            p27.ygyro = (short)(short)24324;
            p27.zmag = (short)(short) -19870;
            p27.yacc = (short)(short)20330;
            p27.xmag = (short)(short)8871;
            p27.zgyro = (short)(short) -24076;
            p27.xacc = (short)(short)31898;
            p27.xgyro = (short)(short)18891;
            p27.ymag = (short)(short) -5914;
            p27.time_usec = (ulong)2669459027421311672L;
            LoopBackDemoChannel.instance.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff2 == (short)(short)22147);
                Debug.Assert(pack.press_abs == (short)(short)28760);
                Debug.Assert(pack.temperature == (short)(short) -24629);
                Debug.Assert(pack.time_usec == (ulong)3325577226851899044L);
                Debug.Assert(pack.press_diff1 == (short)(short) -17584);
            };
            DemoDevice.RAW_PRESSURE p28 = LoopBackDemoChannel.new_RAW_PRESSURE();
            PH.setPack(p28);
            p28.press_diff2 = (short)(short)22147;
            p28.temperature = (short)(short) -24629;
            p28.time_usec = (ulong)3325577226851899044L;
            p28.press_abs = (short)(short)28760;
            p28.press_diff1 = (short)(short) -17584;
            LoopBackDemoChannel.instance.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -10546);
                Debug.Assert(pack.time_boot_ms == (uint)745207163U);
                Debug.Assert(pack.press_diff == (float)1.550102E38F);
                Debug.Assert(pack.press_abs == (float) -2.4741637E38F);
            };
            DemoDevice.SCALED_PRESSURE p29 = LoopBackDemoChannel.new_SCALED_PRESSURE();
            PH.setPack(p29);
            p29.temperature = (short)(short) -10546;
            p29.press_abs = (float) -2.4741637E38F;
            p29.press_diff = (float)1.550102E38F;
            p29.time_boot_ms = (uint)745207163U;
            LoopBackDemoChannel.instance.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float)1.1361847E38F);
                Debug.Assert(pack.time_boot_ms == (uint)165435679U);
                Debug.Assert(pack.yaw == (float) -2.8153498E38F);
                Debug.Assert(pack.pitchspeed == (float) -1.8715226E38F);
                Debug.Assert(pack.pitch == (float)2.970837E38F);
                Debug.Assert(pack.yawspeed == (float) -2.359439E38F);
                Debug.Assert(pack.rollspeed == (float) -3.1169153E38F);
            };
            DemoDevice.ATTITUDE p30 = LoopBackDemoChannel.new_ATTITUDE();
            PH.setPack(p30);
            p30.yaw = (float) -2.8153498E38F;
            p30.pitch = (float)2.970837E38F;
            p30.yawspeed = (float) -2.359439E38F;
            p30.pitchspeed = (float) -1.8715226E38F;
            p30.rollspeed = (float) -3.1169153E38F;
            p30.time_boot_ms = (uint)165435679U;
            p30.roll = (float)1.1361847E38F;
            LoopBackDemoChannel.instance.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q1 == (float)1.1302297E38F);
                Debug.Assert(pack.q4 == (float) -1.7125862E38F);
                Debug.Assert(pack.yawspeed == (float) -3.0188939E38F);
                Debug.Assert(pack.q2 == (float) -5.3090665E36F);
                Debug.Assert(pack.time_boot_ms == (uint)476054762U);
                Debug.Assert(pack.pitchspeed == (float) -1.6520933E38F);
                Debug.Assert(pack.q3 == (float) -7.801081E37F);
                Debug.Assert(pack.rollspeed == (float) -1.1231242E38F);
            };
            DemoDevice.ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.q4 = (float) -1.7125862E38F;
            p31.q2 = (float) -5.3090665E36F;
            p31.q3 = (float) -7.801081E37F;
            p31.q1 = (float)1.1302297E38F;
            p31.time_boot_ms = (uint)476054762U;
            p31.pitchspeed = (float) -1.6520933E38F;
            p31.yawspeed = (float) -3.0188939E38F;
            p31.rollspeed = (float) -1.1231242E38F;
            LoopBackDemoChannel.instance.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (float) -1.2289197E38F);
                Debug.Assert(pack.y == (float)9.059497E37F);
                Debug.Assert(pack.vy == (float)1.6103423E38F);
                Debug.Assert(pack.vz == (float)3.7797016E37F);
                Debug.Assert(pack.x == (float) -3.1545666E38F);
                Debug.Assert(pack.z == (float) -7.8210457E37F);
                Debug.Assert(pack.time_boot_ms == (uint)4290050519U);
            };
            DemoDevice.LOCAL_POSITION_NED p32 = LoopBackDemoChannel.new_LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.y = (float)9.059497E37F;
            p32.x = (float) -3.1545666E38F;
            p32.vx = (float) -1.2289197E38F;
            p32.vz = (float)3.7797016E37F;
            p32.z = (float) -7.8210457E37F;
            p32.vy = (float)1.6103423E38F;
            p32.time_boot_ms = (uint)4290050519U;
            LoopBackDemoChannel.instance.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)1264653583);
                Debug.Assert(pack.vy == (short)(short) -27741);
                Debug.Assert(pack.time_boot_ms == (uint)1277214634U);
                Debug.Assert(pack.hdg == (ushort)(ushort)8745);
                Debug.Assert(pack.vz == (short)(short)11235);
                Debug.Assert(pack.lat == (int)1770059850);
                Debug.Assert(pack.vx == (short)(short) -19867);
                Debug.Assert(pack.relative_alt == (int) -943568755);
                Debug.Assert(pack.alt == (int)417997898);
            };
            DemoDevice.GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.hdg = (ushort)(ushort)8745;
            p33.alt = (int)417997898;
            p33.lat = (int)1770059850;
            p33.lon = (int)1264653583;
            p33.vy = (short)(short) -27741;
            p33.relative_alt = (int) -943568755;
            p33.time_boot_ms = (uint)1277214634U;
            p33.vz = (short)(short)11235;
            p33.vx = (short)(short) -19867;
            LoopBackDemoChannel.instance.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan2_scaled == (short)(short) -20869);
                Debug.Assert(pack.chan3_scaled == (short)(short)10504);
                Debug.Assert(pack.chan1_scaled == (short)(short) -1063);
                Debug.Assert(pack.chan8_scaled == (short)(short) -8361);
                Debug.Assert(pack.rssi == (byte)(byte)141);
                Debug.Assert(pack.time_boot_ms == (uint)3932677171U);
                Debug.Assert(pack.chan4_scaled == (short)(short) -16552);
                Debug.Assert(pack.port == (byte)(byte)254);
                Debug.Assert(pack.chan7_scaled == (short)(short) -12975);
                Debug.Assert(pack.chan5_scaled == (short)(short) -4980);
                Debug.Assert(pack.chan6_scaled == (short)(short) -27826);
            };
            DemoDevice.RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.new_RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.rssi = (byte)(byte)141;
            p34.chan1_scaled = (short)(short) -1063;
            p34.time_boot_ms = (uint)3932677171U;
            p34.port = (byte)(byte)254;
            p34.chan7_scaled = (short)(short) -12975;
            p34.chan2_scaled = (short)(short) -20869;
            p34.chan3_scaled = (short)(short)10504;
            p34.chan6_scaled = (short)(short) -27826;
            p34.chan8_scaled = (short)(short) -8361;
            p34.chan4_scaled = (short)(short) -16552;
            p34.chan5_scaled = (short)(short) -4980;
            LoopBackDemoChannel.instance.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)31997);
                Debug.Assert(pack.port == (byte)(byte)22);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)15983);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)46292);
                Debug.Assert(pack.rssi == (byte)(byte)173);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)21238);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)21786);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)20646);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)9186);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)32285);
                Debug.Assert(pack.time_boot_ms == (uint)2204428875U);
            };
            DemoDevice.RC_CHANNELS_RAW p35 = LoopBackDemoChannel.new_RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.chan7_raw = (ushort)(ushort)15983;
            p35.chan4_raw = (ushort)(ushort)21238;
            p35.rssi = (byte)(byte)173;
            p35.chan6_raw = (ushort)(ushort)20646;
            p35.chan3_raw = (ushort)(ushort)31997;
            p35.port = (byte)(byte)22;
            p35.chan2_raw = (ushort)(ushort)32285;
            p35.time_boot_ms = (uint)2204428875U;
            p35.chan8_raw = (ushort)(ushort)9186;
            p35.chan1_raw = (ushort)(ushort)21786;
            p35.chan5_raw = (ushort)(ushort)46292;
            LoopBackDemoChannel.instance.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)15297);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)42126);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)24743);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)9581);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)19270);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)45875);
                Debug.Assert(pack.time_usec == (uint)1022546988U);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)17837);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)4384);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)55623);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)38009);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)24171);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)5271);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)59988);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)18565);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)1850);
                Debug.Assert(pack.port == (byte)(byte)156);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)11908);
            };
            DemoDevice.SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.new_SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo2_raw = (ushort)(ushort)59988;
            p36.servo5_raw = (ushort)(ushort)17837;
            p36.servo11_raw_SET((ushort)(ushort)9581, PH) ;
            p36.servo13_raw_SET((ushort)(ushort)19270, PH) ;
            p36.servo12_raw_SET((ushort)(ushort)5271, PH) ;
            p36.servo1_raw = (ushort)(ushort)18565;
            p36.servo6_raw = (ushort)(ushort)24743;
            p36.time_usec = (uint)1022546988U;
            p36.servo10_raw_SET((ushort)(ushort)4384, PH) ;
            p36.port = (byte)(byte)156;
            p36.servo9_raw_SET((ushort)(ushort)1850, PH) ;
            p36.servo16_raw_SET((ushort)(ushort)45875, PH) ;
            p36.servo8_raw = (ushort)(ushort)38009;
            p36.servo4_raw = (ushort)(ushort)15297;
            p36.servo7_raw = (ushort)(ushort)42126;
            p36.servo14_raw_SET((ushort)(ushort)55623, PH) ;
            p36.servo3_raw = (ushort)(ushort)11908;
            p36.servo15_raw_SET((ushort)(ushort)24171, PH) ;
            LoopBackDemoChannel.instance.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_index == (short)(short)28560);
                Debug.Assert(pack.end_index == (short)(short)9106);
                Debug.Assert(pack.target_system == (byte)(byte)193);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)250);
            };
            DemoDevice.MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.new_MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.start_index = (short)(short)28560;
            p37.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p37.target_component = (byte)(byte)250;
            p37.end_index = (short)(short)9106;
            p37.target_system = (byte)(byte)193;
            LoopBackDemoChannel.instance.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_system == (byte)(byte)46);
                Debug.Assert(pack.end_index == (short)(short)23526);
                Debug.Assert(pack.start_index == (short)(short) -15058);
                Debug.Assert(pack.target_component == (byte)(byte)149);
            };
            DemoDevice.MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.new_MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.end_index = (short)(short)23526;
            p38.target_system = (byte)(byte)46;
            p38.target_component = (byte)(byte)149;
            p38.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p38.start_index = (short)(short) -15058;
            LoopBackDemoChannel.instance.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)153);
                Debug.Assert(pack.seq == (ushort)(ushort)4236);
                Debug.Assert(pack.param2 == (float) -2.958227E38F);
                Debug.Assert(pack.target_system == (byte)(byte)255);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT);
                Debug.Assert(pack.current == (byte)(byte)46);
                Debug.Assert(pack.param1 == (float) -1.1232312E38F);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.autocontinue == (byte)(byte)66);
                Debug.Assert(pack.x == (float)7.3964487E37F);
                Debug.Assert(pack.y == (float) -3.6534628E37F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT);
                Debug.Assert(pack.z == (float) -2.5440835E38F);
                Debug.Assert(pack.param3 == (float)1.623726E38F);
                Debug.Assert(pack.param4 == (float) -3.0145453E38F);
            };
            DemoDevice.MISSION_ITEM p39 = LoopBackDemoChannel.new_MISSION_ITEM();
            PH.setPack(p39);
            p39.x = (float)7.3964487E37F;
            p39.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p39.param2 = (float) -2.958227E38F;
            p39.y = (float) -3.6534628E37F;
            p39.autocontinue = (byte)(byte)66;
            p39.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p39.param4 = (float) -3.0145453E38F;
            p39.z = (float) -2.5440835E38F;
            p39.target_component = (byte)(byte)153;
            p39.seq = (ushort)(ushort)4236;
            p39.target_system = (byte)(byte)255;
            p39.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT;
            p39.param1 = (float) -1.1232312E38F;
            p39.param3 = (float)1.623726E38F;
            p39.current = (byte)(byte)46;
            LoopBackDemoChannel.instance.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)239);
                Debug.Assert(pack.target_component == (byte)(byte)38);
                Debug.Assert(pack.seq == (ushort)(ushort)10437);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            };
            DemoDevice.MISSION_REQUEST p40 = LoopBackDemoChannel.new_MISSION_REQUEST();
            PH.setPack(p40);
            p40.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p40.target_system = (byte)(byte)239;
            p40.target_component = (byte)(byte)38;
            p40.seq = (ushort)(ushort)10437;
            LoopBackDemoChannel.instance.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)156);
                Debug.Assert(pack.seq == (ushort)(ushort)52193);
                Debug.Assert(pack.target_system == (byte)(byte)223);
            };
            DemoDevice.MISSION_SET_CURRENT p41 = LoopBackDemoChannel.new_MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_component = (byte)(byte)156;
            p41.seq = (ushort)(ushort)52193;
            p41.target_system = (byte)(byte)223;
            LoopBackDemoChannel.instance.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)28205);
            };
            DemoDevice.MISSION_CURRENT p42 = LoopBackDemoChannel.new_MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)28205;
            LoopBackDemoChannel.instance.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)98);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_system == (byte)(byte)140);
            };
            DemoDevice.MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.new_MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p43.target_component = (byte)(byte)98;
            p43.target_system = (byte)(byte)140;
            LoopBackDemoChannel.instance.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)89);
                Debug.Assert(pack.target_component == (byte)(byte)30);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.count == (ushort)(ushort)49514);
            };
            DemoDevice.MISSION_COUNT p44 = LoopBackDemoChannel.new_MISSION_COUNT();
            PH.setPack(p44);
            p44.target_system = (byte)(byte)89;
            p44.count = (ushort)(ushort)49514;
            p44.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p44.target_component = (byte)(byte)30;
            LoopBackDemoChannel.instance.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)63);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_system == (byte)(byte)249);
            };
            DemoDevice.MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.new_MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_system = (byte)(byte)249;
            p45.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p45.target_component = (byte)(byte)63;
            LoopBackDemoChannel.instance.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)5681);
            };
            DemoDevice.MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.new_MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)5681;
            LoopBackDemoChannel.instance.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_INVALID);
                Debug.Assert(pack.target_system == (byte)(byte)229);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)152);
            };
            DemoDevice.MISSION_ACK p47 = LoopBackDemoChannel.new_MISSION_ACK();
            PH.setPack(p47);
            p47.target_component = (byte)(byte)152;
            p47.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p47.target_system = (byte)(byte)229;
            p47.type = (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_INVALID;
            LoopBackDemoChannel.instance.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.latitude == (int)626649423);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)7116345402622817341L);
                Debug.Assert(pack.longitude == (int) -1376208641);
                Debug.Assert(pack.altitude == (int) -420173321);
                Debug.Assert(pack.target_system == (byte)(byte)123);
            };
            DemoDevice.SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.new_SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.latitude = (int)626649423;
            p48.time_usec_SET((ulong)7116345402622817341L, PH) ;
            p48.target_system = (byte)(byte)123;
            p48.longitude = (int) -1376208641;
            p48.altitude = (int) -420173321;
            LoopBackDemoChannel.instance.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int) -1912213576);
                Debug.Assert(pack.latitude == (int) -1333984205);
                Debug.Assert(pack.longitude == (int)950119827);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)8101935962298300898L);
            };
            DemoDevice.GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.new_GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.time_usec_SET((ulong)8101935962298300898L, PH) ;
            p49.altitude = (int) -1912213576;
            p49.latitude = (int) -1333984205;
            p49.longitude = (int)950119827;
            LoopBackDemoChannel.instance.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)182);
                Debug.Assert(pack.scale == (float) -9.644739E37F);
                Debug.Assert(pack.param_value_min == (float) -2.2351323E38F);
                Debug.Assert(pack.param_value_max == (float) -2.1000537E38F);
                Debug.Assert(pack.target_system == (byte)(byte)113);
                Debug.Assert(pack.target_component == (byte)(byte)201);
                Debug.Assert(pack.param_id_LEN(ph) == 15);
                Debug.Assert(pack.param_id_TRY(ph).Equals("qfBcyjipiVicbkj"));
                Debug.Assert(pack.param_value0 == (float) -3.3499718E38F);
                Debug.Assert(pack.param_index == (short)(short)18395);
            };
            DemoDevice.PARAM_MAP_RC p50 = LoopBackDemoChannel.new_PARAM_MAP_RC();
            PH.setPack(p50);
            p50.scale = (float) -9.644739E37F;
            p50.param_value_min = (float) -2.2351323E38F;
            p50.target_component = (byte)(byte)201;
            p50.param_value0 = (float) -3.3499718E38F;
            p50.param_index = (short)(short)18395;
            p50.param_value_max = (float) -2.1000537E38F;
            p50.param_id_SET("qfBcyjipiVicbkj", PH) ;
            p50.target_system = (byte)(byte)113;
            p50.parameter_rc_channel_index = (byte)(byte)182;
            LoopBackDemoChannel.instance.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)227);
                Debug.Assert(pack.target_component == (byte)(byte)84);
                Debug.Assert(pack.seq == (ushort)(ushort)6327);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            };
            DemoDevice.MISSION_REQUEST_INT p51 = LoopBackDemoChannel.new_MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.target_component = (byte)(byte)84;
            p51.seq = (ushort)(ushort)6327;
            p51.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p51.target_system = (byte)(byte)227;
            LoopBackDemoChannel.instance.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p1x == (float)6.469991E37F);
                Debug.Assert(pack.target_component == (byte)(byte)162);
                Debug.Assert(pack.target_system == (byte)(byte)96);
                Debug.Assert(pack.p1y == (float) -1.4042533E38F);
                Debug.Assert(pack.p2x == (float) -1.1430704E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
                Debug.Assert(pack.p1z == (float)2.6540926E38F);
                Debug.Assert(pack.p2y == (float)1.382094E38F);
                Debug.Assert(pack.p2z == (float)1.3112777E38F);
            };
            DemoDevice.SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.new_SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p54.p2z = (float)1.3112777E38F;
            p54.target_system = (byte)(byte)96;
            p54.target_component = (byte)(byte)162;
            p54.p2y = (float)1.382094E38F;
            p54.p1y = (float) -1.4042533E38F;
            p54.p2x = (float) -1.1430704E38F;
            p54.p1z = (float)2.6540926E38F;
            p54.p1x = (float)6.469991E37F;
            LoopBackDemoChannel.instance.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p1z == (float)1.6533442E38F);
                Debug.Assert(pack.p1x == (float)2.2982722E38F);
                Debug.Assert(pack.p1y == (float) -8.765188E37F);
                Debug.Assert(pack.p2y == (float) -2.0334318E38F);
                Debug.Assert(pack.p2x == (float) -1.829277E38F);
                Debug.Assert(pack.p2z == (float) -2.4996082E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            };
            DemoDevice.SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.new_SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p2y = (float) -2.0334318E38F;
            p55.p1z = (float)1.6533442E38F;
            p55.p2x = (float) -1.829277E38F;
            p55.p1x = (float)2.2982722E38F;
            p55.p2z = (float) -2.4996082E38F;
            p55.p1y = (float) -8.765188E37F;
            p55.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            LoopBackDemoChannel.instance.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.3204611E38F, 2.2414332E37F, -2.2566193E38F, -2.3760557E38F}));
                Debug.Assert(pack.yawspeed == (float) -2.4422303E38F);
                Debug.Assert(pack.time_usec == (ulong)5476274073752555517L);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {1.7488816E38F, 2.0736492E38F, -1.6917979E38F, -2.985755E38F, 1.2393535E38F, 7.1576674E37F, 1.5829888E36F, 2.8644703E38F, 6.4848147E37F}));
                Debug.Assert(pack.pitchspeed == (float) -1.1041824E38F);
                Debug.Assert(pack.rollspeed == (float)1.2886551E37F);
            };
            DemoDevice.ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.time_usec = (ulong)5476274073752555517L;
            p61.q_SET(new float[] {1.3204611E38F, 2.2414332E37F, -2.2566193E38F, -2.3760557E38F}, 0) ;
            p61.pitchspeed = (float) -1.1041824E38F;
            p61.rollspeed = (float)1.2886551E37F;
            p61.yawspeed = (float) -2.4422303E38F;
            p61.covariance_SET(new float[] {1.7488816E38F, 2.0736492E38F, -1.6917979E38F, -2.985755E38F, 1.2393535E38F, 7.1576674E37F, 1.5829888E36F, 2.8644703E38F, 6.4848147E37F}, 0) ;
            LoopBackDemoChannel.instance.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_bearing == (short)(short) -19524);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)29770);
                Debug.Assert(pack.aspd_error == (float) -5.4000597E37F);
                Debug.Assert(pack.alt_error == (float)1.6956159E38F);
                Debug.Assert(pack.nav_bearing == (short)(short) -6413);
                Debug.Assert(pack.nav_roll == (float)2.6049429E38F);
                Debug.Assert(pack.nav_pitch == (float) -1.8338984E38F);
                Debug.Assert(pack.xtrack_error == (float)3.5324032E37F);
            };
            DemoDevice.NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.new_NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.wp_dist = (ushort)(ushort)29770;
            p62.nav_pitch = (float) -1.8338984E38F;
            p62.alt_error = (float)1.6956159E38F;
            p62.nav_bearing = (short)(short) -6413;
            p62.target_bearing = (short)(short) -19524;
            p62.nav_roll = (float)2.6049429E38F;
            p62.xtrack_error = (float)3.5324032E37F;
            p62.aspd_error = (float) -5.4000597E37F;
            LoopBackDemoChannel.instance.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)1252257770956524301L);
                Debug.Assert(pack.alt == (int)912050653);
                Debug.Assert(pack.lon == (int)1478117765);
                Debug.Assert(pack.vx == (float)2.7914774E38F);
                Debug.Assert(pack.lat == (int)840471843);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
                Debug.Assert(pack.relative_alt == (int) -285566429);
                Debug.Assert(pack.vz == (float) -6.6312065E37F);
                Debug.Assert(pack.vy == (float) -4.457632E37F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-2.6394777E38F, -1.8111092E38F, 2.6770556E38F, 2.7163568E38F, 1.86862E38F, 2.1339488E38F, 2.4857977E38F, -2.299255E38F, -2.771874E38F, 2.9437857E38F, 1.0651424E38F, 3.3339568E38F, -2.7210463E38F, 8.83596E37F, 8.70684E37F, 2.8025106E38F, 1.9708853E38F, 1.6142485E37F, -2.488137E38F, 1.3293564E37F, 1.4454229E38F, -1.9582307E38F, 9.077637E37F, 2.515092E37F, 2.3287222E38F, -1.823213E38F, 6.337949E37F, -2.0212593E38F, -9.346166E37F, 3.2239813E38F, -2.776422E37F, -1.3962898E38F, -3.255025E38F, 2.6530956E38F, -1.6333677E38F, 9.985652E37F}));
            };
            DemoDevice.GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS;
            p63.time_usec = (ulong)1252257770956524301L;
            p63.vz = (float) -6.6312065E37F;
            p63.lat = (int)840471843;
            p63.covariance_SET(new float[] {-2.6394777E38F, -1.8111092E38F, 2.6770556E38F, 2.7163568E38F, 1.86862E38F, 2.1339488E38F, 2.4857977E38F, -2.299255E38F, -2.771874E38F, 2.9437857E38F, 1.0651424E38F, 3.3339568E38F, -2.7210463E38F, 8.83596E37F, 8.70684E37F, 2.8025106E38F, 1.9708853E38F, 1.6142485E37F, -2.488137E38F, 1.3293564E37F, 1.4454229E38F, -1.9582307E38F, 9.077637E37F, 2.515092E37F, 2.3287222E38F, -1.823213E38F, 6.337949E37F, -2.0212593E38F, -9.346166E37F, 3.2239813E38F, -2.776422E37F, -1.3962898E38F, -3.255025E38F, 2.6530956E38F, -1.6333677E38F, 9.985652E37F}, 0) ;
            p63.lon = (int)1478117765;
            p63.vx = (float)2.7914774E38F;
            p63.vy = (float) -4.457632E37F;
            p63.relative_alt = (int) -285566429;
            p63.alt = (int)912050653;
            LoopBackDemoChannel.instance.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.az == (float)4.9956553E37F);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
                Debug.Assert(pack.vz == (float)3.4336905E37F);
                Debug.Assert(pack.time_usec == (ulong)8218797684573711807L);
                Debug.Assert(pack.y == (float) -2.8409263E38F);
                Debug.Assert(pack.vy == (float) -2.2233306E38F);
                Debug.Assert(pack.ax == (float) -2.55476E38F);
                Debug.Assert(pack.ay == (float)1.2489147E38F);
                Debug.Assert(pack.vx == (float)1.5139791E38F);
                Debug.Assert(pack.x == (float) -4.3690977E37F);
                Debug.Assert(pack.z == (float) -2.036717E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-1.1034409E38F, -1.3943238E38F, 2.8010207E36F, 2.1137157E38F, 2.9832691E37F, -2.6142639E38F, 7.0144893E37F, 6.292396E37F, 8.908315E37F, 1.4604405E38F, -2.8868609E38F, 2.270746E38F, 2.0626977E38F, 1.1192772E36F, -3.2611253E38F, 1.3738949E38F, 2.21905E38F, 6.13901E37F, -1.7792729E37F, -1.0854181E38F, -2.9515774E38F, 1.3821894E38F, 1.4537179E38F, -2.397805E38F, -3.2077756E38F, -1.2439209E38F, 2.3026847E38F, 3.5720736E37F, -2.4526866E37F, 1.4552157E38F, -1.3339346E38F, -2.9582554E37F, -2.5944279E38F, -2.698487E38F, 1.6465386E38F, -8.947042E37F, -8.549145E36F, 2.8756924E38F, 2.6719596E38F, 3.3243005E38F, 3.1200605E38F, -1.934026E38F, -1.6476731E38F, -8.0784E37F, -1.7492797E38F}));
            };
            DemoDevice.LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.time_usec = (ulong)8218797684573711807L;
            p64.vy = (float) -2.2233306E38F;
            p64.ay = (float)1.2489147E38F;
            p64.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION;
            p64.x = (float) -4.3690977E37F;
            p64.vx = (float)1.5139791E38F;
            p64.ax = (float) -2.55476E38F;
            p64.y = (float) -2.8409263E38F;
            p64.az = (float)4.9956553E37F;
            p64.covariance_SET(new float[] {-1.1034409E38F, -1.3943238E38F, 2.8010207E36F, 2.1137157E38F, 2.9832691E37F, -2.6142639E38F, 7.0144893E37F, 6.292396E37F, 8.908315E37F, 1.4604405E38F, -2.8868609E38F, 2.270746E38F, 2.0626977E38F, 1.1192772E36F, -3.2611253E38F, 1.3738949E38F, 2.21905E38F, 6.13901E37F, -1.7792729E37F, -1.0854181E38F, -2.9515774E38F, 1.3821894E38F, 1.4537179E38F, -2.397805E38F, -3.2077756E38F, -1.2439209E38F, 2.3026847E38F, 3.5720736E37F, -2.4526866E37F, 1.4552157E38F, -1.3339346E38F, -2.9582554E37F, -2.5944279E38F, -2.698487E38F, 1.6465386E38F, -8.947042E37F, -8.549145E36F, 2.8756924E38F, 2.6719596E38F, 3.3243005E38F, 3.1200605E38F, -1.934026E38F, -1.6476731E38F, -8.0784E37F, -1.7492797E38F}, 0) ;
            p64.z = (float) -2.036717E38F;
            p64.vz = (float)3.4336905E37F;
            LoopBackDemoChannel.instance.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)50418);
                Debug.Assert(pack.time_boot_ms == (uint)3158147087U);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)62148);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)14168);
                Debug.Assert(pack.chancount == (byte)(byte)122);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)14146);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)58496);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)48544);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)29522);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)12388);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)2917);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)721);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)43042);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)53156);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)45626);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)42028);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)56918);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)42173);
                Debug.Assert(pack.rssi == (byte)(byte)76);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)39864);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)15903);
            };
            DemoDevice.RC_CHANNELS p65 = LoopBackDemoChannel.new_RC_CHANNELS();
            PH.setPack(p65);
            p65.chan17_raw = (ushort)(ushort)56918;
            p65.chan11_raw = (ushort)(ushort)58496;
            p65.chan15_raw = (ushort)(ushort)48544;
            p65.chan4_raw = (ushort)(ushort)62148;
            p65.chan7_raw = (ushort)(ushort)14146;
            p65.chan8_raw = (ushort)(ushort)12388;
            p65.chan16_raw = (ushort)(ushort)43042;
            p65.chan13_raw = (ushort)(ushort)45626;
            p65.chan10_raw = (ushort)(ushort)50418;
            p65.time_boot_ms = (uint)3158147087U;
            p65.chan9_raw = (ushort)(ushort)15903;
            p65.chan1_raw = (ushort)(ushort)721;
            p65.rssi = (byte)(byte)76;
            p65.chan5_raw = (ushort)(ushort)29522;
            p65.chan3_raw = (ushort)(ushort)42028;
            p65.chan14_raw = (ushort)(ushort)53156;
            p65.chan2_raw = (ushort)(ushort)14168;
            p65.chancount = (byte)(byte)122;
            p65.chan18_raw = (ushort)(ushort)2917;
            p65.chan6_raw = (ushort)(ushort)39864;
            p65.chan12_raw = (ushort)(ushort)42173;
            LoopBackDemoChannel.instance.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_stop == (byte)(byte)93);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)8668);
                Debug.Assert(pack.target_system == (byte)(byte)236);
                Debug.Assert(pack.target_component == (byte)(byte)77);
                Debug.Assert(pack.req_stream_id == (byte)(byte)40);
            };
            DemoDevice.REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.new_REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.start_stop = (byte)(byte)93;
            p66.target_component = (byte)(byte)77;
            p66.req_message_rate = (ushort)(ushort)8668;
            p66.req_stream_id = (byte)(byte)40;
            p66.target_system = (byte)(byte)236;
            LoopBackDemoChannel.instance.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.stream_id == (byte)(byte)21);
                Debug.Assert(pack.on_off == (byte)(byte)221);
                Debug.Assert(pack.message_rate == (ushort)(ushort)28214);
            };
            DemoDevice.DATA_STREAM p67 = LoopBackDemoChannel.new_DATA_STREAM();
            PH.setPack(p67);
            p67.stream_id = (byte)(byte)21;
            p67.message_rate = (ushort)(ushort)28214;
            p67.on_off = (byte)(byte)221;
            LoopBackDemoChannel.instance.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.buttons == (ushort)(ushort)17572);
                Debug.Assert(pack.target == (byte)(byte)53);
                Debug.Assert(pack.y == (short)(short)15571);
                Debug.Assert(pack.x == (short)(short)11568);
                Debug.Assert(pack.r == (short)(short) -3348);
                Debug.Assert(pack.z == (short)(short) -8461);
            };
            DemoDevice.MANUAL_CONTROL p69 = LoopBackDemoChannel.new_MANUAL_CONTROL();
            PH.setPack(p69);
            p69.z = (short)(short) -8461;
            p69.y = (short)(short)15571;
            p69.buttons = (ushort)(ushort)17572;
            p69.x = (short)(short)11568;
            p69.r = (short)(short) -3348;
            p69.target = (byte)(byte)53;
            LoopBackDemoChannel.instance.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)36246);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)35607);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)56066);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)57759);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)38783);
                Debug.Assert(pack.target_system == (byte)(byte)159);
                Debug.Assert(pack.target_component == (byte)(byte)52);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)28531);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)3069);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)14472);
            };
            DemoDevice.RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.new_RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan3_raw = (ushort)(ushort)38783;
            p70.chan6_raw = (ushort)(ushort)57759;
            p70.chan7_raw = (ushort)(ushort)56066;
            p70.chan8_raw = (ushort)(ushort)14472;
            p70.target_system = (byte)(byte)159;
            p70.target_component = (byte)(byte)52;
            p70.chan5_raw = (ushort)(ushort)3069;
            p70.chan1_raw = (ushort)(ushort)35607;
            p70.chan4_raw = (ushort)(ushort)36246;
            p70.chan2_raw = (ushort)(ushort)28531;
            LoopBackDemoChannel.instance.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN);
                Debug.Assert(pack.y == (int)897269760);
                Debug.Assert(pack.seq == (ushort)(ushort)64506);
                Debug.Assert(pack.autocontinue == (byte)(byte)60);
                Debug.Assert(pack.current == (byte)(byte)16);
                Debug.Assert(pack.target_component == (byte)(byte)151);
                Debug.Assert(pack.z == (float)8.1696704E37F);
                Debug.Assert(pack.x == (int) -357423719);
                Debug.Assert(pack.param3 == (float) -4.960402E37F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.target_system == (byte)(byte)131);
                Debug.Assert(pack.param4 == (float)2.6290045E38F);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.param1 == (float) -1.4659921E38F);
                Debug.Assert(pack.param2 == (float) -5.131129E37F);
            };
            DemoDevice.MISSION_ITEM_INT p73 = LoopBackDemoChannel.new_MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.z = (float)8.1696704E37F;
            p73.autocontinue = (byte)(byte)60;
            p73.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p73.param4 = (float)2.6290045E38F;
            p73.seq = (ushort)(ushort)64506;
            p73.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p73.current = (byte)(byte)16;
            p73.command = (MAV_CMD)MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
            p73.param1 = (float) -1.4659921E38F;
            p73.target_system = (byte)(byte)131;
            p73.param3 = (float) -4.960402E37F;
            p73.param2 = (float) -5.131129E37F;
            p73.y = (int)897269760;
            p73.x = (int) -357423719;
            p73.target_component = (byte)(byte)151;
            LoopBackDemoChannel.instance.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.heading == (short)(short) -10942);
                Debug.Assert(pack.alt == (float)1.8876515E37F);
                Debug.Assert(pack.climb == (float)2.3370528E38F);
                Debug.Assert(pack.throttle == (ushort)(ushort)32740);
                Debug.Assert(pack.airspeed == (float) -4.6888033E37F);
                Debug.Assert(pack.groundspeed == (float) -9.184111E37F);
            };
            DemoDevice.VFR_HUD p74 = LoopBackDemoChannel.new_VFR_HUD();
            PH.setPack(p74);
            p74.airspeed = (float) -4.6888033E37F;
            p74.throttle = (ushort)(ushort)32740;
            p74.climb = (float)2.3370528E38F;
            p74.heading = (short)(short) -10942;
            p74.alt = (float)1.8876515E37F;
            p74.groundspeed = (float) -9.184111E37F;
            LoopBackDemoChannel.instance.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -3.9733313E36F);
                Debug.Assert(pack.autocontinue == (byte)(byte)12);
                Debug.Assert(pack.param4 == (float) -2.275737E38F);
                Debug.Assert(pack.param1 == (float)4.649259E35F);
                Debug.Assert(pack.target_system == (byte)(byte)46);
                Debug.Assert(pack.current == (byte)(byte)210);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.y == (int) -1476192114);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE);
                Debug.Assert(pack.x == (int)1210193798);
                Debug.Assert(pack.param3 == (float) -2.10847E37F);
                Debug.Assert(pack.param2 == (float)6.262265E37F);
                Debug.Assert(pack.target_component == (byte)(byte)20);
            };
            DemoDevice.COMMAND_INT p75 = LoopBackDemoChannel.new_COMMAND_INT();
            PH.setPack(p75);
            p75.target_component = (byte)(byte)20;
            p75.y = (int) -1476192114;
            p75.param1 = (float)4.649259E35F;
            p75.param3 = (float) -2.10847E37F;
            p75.param4 = (float) -2.275737E38F;
            p75.x = (int)1210193798;
            p75.current = (byte)(byte)210;
            p75.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE;
            p75.target_system = (byte)(byte)46;
            p75.autocontinue = (byte)(byte)12;
            p75.param2 = (float)6.262265E37F;
            p75.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p75.z = (float) -3.9733313E36F;
            LoopBackDemoChannel.instance.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param2 == (float)3.2898249E38F);
                Debug.Assert(pack.target_component == (byte)(byte)215);
                Debug.Assert(pack.param4 == (float) -1.1419506E38F);
                Debug.Assert(pack.param3 == (float)3.1297206E38F);
                Debug.Assert(pack.param6 == (float) -1.9806832E38F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_SPATIAL_USER_1);
                Debug.Assert(pack.param7 == (float) -1.8093169E38F);
                Debug.Assert(pack.target_system == (byte)(byte)117);
                Debug.Assert(pack.confirmation == (byte)(byte)214);
                Debug.Assert(pack.param5 == (float) -1.0843204E38F);
                Debug.Assert(pack.param1 == (float) -2.24574E38F);
            };
            DemoDevice.COMMAND_LONG p76 = LoopBackDemoChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.target_system = (byte)(byte)117;
            p76.confirmation = (byte)(byte)214;
            p76.param6 = (float) -1.9806832E38F;
            p76.param4 = (float) -1.1419506E38F;
            p76.param2 = (float)3.2898249E38F;
            p76.param3 = (float)3.1297206E38F;
            p76.target_component = (byte)(byte)215;
            p76.command = (MAV_CMD)MAV_CMD.MAV_CMD_SPATIAL_USER_1;
            p76.param7 = (float) -1.8093169E38F;
            p76.param1 = (float) -2.24574E38F;
            p76.param5 = (float) -1.0843204E38F;
            LoopBackDemoChannel.instance.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)125);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)156);
                Debug.Assert(pack.result_param2_TRY(ph) == (int)1455652838);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_NAV_LAST);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)126);
                Debug.Assert(pack.result == (MAV_RESULT)MAV_RESULT.MAV_RESULT_FAILED);
            };
            DemoDevice.COMMAND_ACK p77 = LoopBackDemoChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.progress_SET((byte)(byte)126, PH) ;
            p77.command = (MAV_CMD)MAV_CMD.MAV_CMD_NAV_LAST;
            p77.target_system_SET((byte)(byte)125, PH) ;
            p77.result = (MAV_RESULT)MAV_RESULT.MAV_RESULT_FAILED;
            p77.target_component_SET((byte)(byte)156, PH) ;
            p77.result_param2_SET((int)1455652838, PH) ;
            LoopBackDemoChannel.instance.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float)9.734657E37F);
                Debug.Assert(pack.thrust == (float) -9.805066E36F);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)241);
                Debug.Assert(pack.mode_switch == (byte)(byte)251);
                Debug.Assert(pack.time_boot_ms == (uint)3585180651U);
                Debug.Assert(pack.yaw == (float) -1.3199075E38F);
                Debug.Assert(pack.roll == (float) -3.320375E38F);
            };
            DemoDevice.MANUAL_SETPOINT p81 = LoopBackDemoChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.pitch = (float)9.734657E37F;
            p81.mode_switch = (byte)(byte)251;
            p81.roll = (float) -3.320375E38F;
            p81.manual_override_switch = (byte)(byte)241;
            p81.thrust = (float) -9.805066E36F;
            p81.time_boot_ms = (uint)3585180651U;
            p81.yaw = (float) -1.3199075E38F;
            LoopBackDemoChannel.instance.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.body_roll_rate == (float) -3.3365521E38F);
                Debug.Assert(pack.body_pitch_rate == (float) -1.8076515E37F);
                Debug.Assert(pack.target_system == (byte)(byte)141);
                Debug.Assert(pack.time_boot_ms == (uint)1742479909U);
                Debug.Assert(pack.body_yaw_rate == (float)2.3036574E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-3.1005298E38F, 2.547104E38F, -4.3847375E37F, -1.8304382E38F}));
                Debug.Assert(pack.target_component == (byte)(byte)1);
                Debug.Assert(pack.type_mask == (byte)(byte)42);
                Debug.Assert(pack.thrust == (float) -1.9441582E38F);
            };
            DemoDevice.SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.target_component = (byte)(byte)1;
            p82.body_yaw_rate = (float)2.3036574E38F;
            p82.body_roll_rate = (float) -3.3365521E38F;
            p82.thrust = (float) -1.9441582E38F;
            p82.target_system = (byte)(byte)141;
            p82.body_pitch_rate = (float) -1.8076515E37F;
            p82.time_boot_ms = (uint)1742479909U;
            p82.q_SET(new float[] {-3.1005298E38F, 2.547104E38F, -4.3847375E37F, -1.8304382E38F}, 0) ;
            p82.type_mask = (byte)(byte)42;
            LoopBackDemoChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)795841553U);
                Debug.Assert(pack.thrust == (float) -7.224143E37F);
                Debug.Assert(pack.body_yaw_rate == (float) -1.8704452E38F);
                Debug.Assert(pack.body_roll_rate == (float)7.641953E36F);
                Debug.Assert(pack.body_pitch_rate == (float) -3.146263E38F);
                Debug.Assert(pack.type_mask == (byte)(byte)124);
                Debug.Assert(pack.q.SequenceEqual(new float[] {9.281026E37F, -2.5926676E38F, -1.4962004E38F, 6.2788337E37F}));
            };
            DemoDevice.ATTITUDE_TARGET p83 = LoopBackDemoChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.time_boot_ms = (uint)795841553U;
            p83.thrust = (float) -7.224143E37F;
            p83.type_mask = (byte)(byte)124;
            p83.body_pitch_rate = (float) -3.146263E38F;
            p83.q_SET(new float[] {9.281026E37F, -2.5926676E38F, -1.4962004E38F, 6.2788337E37F}, 0) ;
            p83.body_yaw_rate = (float) -1.8704452E38F;
            p83.body_roll_rate = (float)7.641953E36F;
            LoopBackDemoChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.afx == (float)3.1424799E38F);
                Debug.Assert(pack.y == (float) -2.1010483E38F);
                Debug.Assert(pack.target_system == (byte)(byte)58);
                Debug.Assert(pack.vx == (float) -6.6240144E37F);
                Debug.Assert(pack.afy == (float)7.293141E37F);
                Debug.Assert(pack.vy == (float) -2.6629128E38F);
                Debug.Assert(pack.x == (float)1.9747585E38F);
                Debug.Assert(pack.vz == (float) -1.463356E38F);
                Debug.Assert(pack.z == (float)2.1963111E38F);
                Debug.Assert(pack.target_component == (byte)(byte)163);
                Debug.Assert(pack.yaw == (float)1.2381114E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)38484);
                Debug.Assert(pack.time_boot_ms == (uint)3607903586U);
                Debug.Assert(pack.afz == (float) -3.1961311E38F);
                Debug.Assert(pack.yaw_rate == (float)2.1201286E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
            };
            DemoDevice.SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.y = (float) -2.1010483E38F;
            p84.afy = (float)7.293141E37F;
            p84.time_boot_ms = (uint)3607903586U;
            p84.vz = (float) -1.463356E38F;
            p84.vy = (float) -2.6629128E38F;
            p84.x = (float)1.9747585E38F;
            p84.target_component = (byte)(byte)163;
            p84.yaw = (float)1.2381114E38F;
            p84.afx = (float)3.1424799E38F;
            p84.vx = (float) -6.6240144E37F;
            p84.yaw_rate = (float)2.1201286E38F;
            p84.z = (float)2.1963111E38F;
            p84.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p84.target_system = (byte)(byte)58;
            p84.afz = (float) -3.1961311E38F;
            p84.type_mask = (ushort)(ushort)38484;
            LoopBackDemoChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3986739468U);
                Debug.Assert(pack.target_component == (byte)(byte)214);
                Debug.Assert(pack.lat_int == (int)1640288075);
                Debug.Assert(pack.vz == (float) -2.2557666E38F);
                Debug.Assert(pack.afx == (float)1.6815214E37F);
                Debug.Assert(pack.target_system == (byte)(byte)133);
                Debug.Assert(pack.vx == (float)9.865937E37F);
                Debug.Assert(pack.alt == (float)1.9123425E37F);
                Debug.Assert(pack.yaw_rate == (float) -3.36811E38F);
                Debug.Assert(pack.yaw == (float) -2.5287398E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)45095);
                Debug.Assert(pack.afy == (float) -1.1594269E38F);
                Debug.Assert(pack.vy == (float)1.5999246E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.lon_int == (int)2111144861);
                Debug.Assert(pack.afz == (float)4.7287094E37F);
            };
            DemoDevice.SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.target_system = (byte)(byte)133;
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p86.vz = (float) -2.2557666E38F;
            p86.target_component = (byte)(byte)214;
            p86.lat_int = (int)1640288075;
            p86.time_boot_ms = (uint)3986739468U;
            p86.lon_int = (int)2111144861;
            p86.yaw = (float) -2.5287398E38F;
            p86.afx = (float)1.6815214E37F;
            p86.afz = (float)4.7287094E37F;
            p86.vx = (float)9.865937E37F;
            p86.afy = (float) -1.1594269E38F;
            p86.type_mask = (ushort)(ushort)45095;
            p86.yaw_rate = (float) -3.36811E38F;
            p86.vy = (float)1.5999246E38F;
            p86.alt = (float)1.9123425E37F;
            LoopBackDemoChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw_rate == (float) -2.0255372E37F);
                Debug.Assert(pack.vz == (float) -2.0379917E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)62853);
                Debug.Assert(pack.afx == (float) -2.8771916E38F);
                Debug.Assert(pack.afy == (float) -2.6070264E37F);
                Debug.Assert(pack.vx == (float)1.249766E38F);
                Debug.Assert(pack.afz == (float)4.4103983E37F);
                Debug.Assert(pack.lat_int == (int)916735908);
                Debug.Assert(pack.yaw == (float)2.6742231E38F);
                Debug.Assert(pack.alt == (float) -1.8483804E38F);
                Debug.Assert(pack.vy == (float) -2.838325E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1280560803U);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.lon_int == (int) -933659216);
            };
            DemoDevice.POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.vx = (float)1.249766E38F;
            p87.type_mask = (ushort)(ushort)62853;
            p87.afz = (float)4.4103983E37F;
            p87.time_boot_ms = (uint)1280560803U;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p87.lon_int = (int) -933659216;
            p87.lat_int = (int)916735908;
            p87.vy = (float) -2.838325E38F;
            p87.vz = (float) -2.0379917E38F;
            p87.afx = (float) -2.8771916E38F;
            p87.afy = (float) -2.6070264E37F;
            p87.yaw_rate = (float) -2.0255372E37F;
            p87.yaw = (float)2.6742231E38F;
            p87.alt = (float) -1.8483804E38F;
            LoopBackDemoChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float) -1.2949707E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3720207653U);
                Debug.Assert(pack.pitch == (float)1.9524993E38F);
                Debug.Assert(pack.z == (float)1.1910269E38F);
                Debug.Assert(pack.yaw == (float)3.0220648E38F);
                Debug.Assert(pack.y == (float) -2.2542487E38F);
                Debug.Assert(pack.x == (float) -2.494697E38F);
            };
            DemoDevice.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.time_boot_ms = (uint)3720207653U;
            p89.z = (float)1.1910269E38F;
            p89.pitch = (float)1.9524993E38F;
            p89.y = (float) -2.2542487E38F;
            p89.yaw = (float)3.0220648E38F;
            p89.roll = (float) -1.2949707E38F;
            p89.x = (float) -2.494697E38F;
            LoopBackDemoChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitchspeed == (float)2.9295971E38F);
                Debug.Assert(pack.zacc == (short)(short)17844);
                Debug.Assert(pack.vx == (short)(short) -11492);
                Debug.Assert(pack.yawspeed == (float)1.34372E38F);
                Debug.Assert(pack.alt == (int)318164350);
                Debug.Assert(pack.lat == (int)563889478);
                Debug.Assert(pack.pitch == (float) -2.4908448E38F);
                Debug.Assert(pack.roll == (float)1.6401196E38F);
                Debug.Assert(pack.vy == (short)(short)7261);
                Debug.Assert(pack.xacc == (short)(short) -7500);
                Debug.Assert(pack.time_usec == (ulong)3422147173722234494L);
                Debug.Assert(pack.yaw == (float)9.360011E37F);
                Debug.Assert(pack.vz == (short)(short)4400);
                Debug.Assert(pack.lon == (int)995718985);
                Debug.Assert(pack.yacc == (short)(short) -20621);
                Debug.Assert(pack.rollspeed == (float)8.538894E37F);
            };
            DemoDevice.HIL_STATE p90 = LoopBackDemoChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.vx = (short)(short) -11492;
            p90.rollspeed = (float)8.538894E37F;
            p90.yawspeed = (float)1.34372E38F;
            p90.time_usec = (ulong)3422147173722234494L;
            p90.yacc = (short)(short) -20621;
            p90.alt = (int)318164350;
            p90.pitchspeed = (float)2.9295971E38F;
            p90.vz = (short)(short)4400;
            p90.lon = (int)995718985;
            p90.roll = (float)1.6401196E38F;
            p90.vy = (short)(short)7261;
            p90.lat = (int)563889478;
            p90.yaw = (float)9.360011E37F;
            p90.pitch = (float) -2.4908448E38F;
            p90.zacc = (short)(short)17844;
            p90.xacc = (short)(short) -7500;
            LoopBackDemoChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll_ailerons == (float)4.3085994E37F);
                Debug.Assert(pack.yaw_rudder == (float) -2.3313995E38F);
                Debug.Assert(pack.aux1 == (float) -1.3320802E38F);
                Debug.Assert(pack.nav_mode == (byte)(byte)161);
                Debug.Assert(pack.pitch_elevator == (float) -6.6494866E37F);
                Debug.Assert(pack.aux3 == (float) -1.5267918E38F);
                Debug.Assert(pack.throttle == (float)3.2803423E38F);
                Debug.Assert(pack.time_usec == (ulong)8613086477419953640L);
                Debug.Assert(pack.aux4 == (float)1.1446094E38F);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_AUTO_ARMED);
                Debug.Assert(pack.aux2 == (float) -2.1476354E38F);
            };
            DemoDevice.HIL_CONTROLS p91 = LoopBackDemoChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.aux4 = (float)1.1446094E38F;
            p91.time_usec = (ulong)8613086477419953640L;
            p91.aux1 = (float) -1.3320802E38F;
            p91.nav_mode = (byte)(byte)161;
            p91.aux2 = (float) -2.1476354E38F;
            p91.yaw_rudder = (float) -2.3313995E38F;
            p91.pitch_elevator = (float) -6.6494866E37F;
            p91.roll_ailerons = (float)4.3085994E37F;
            p91.aux3 = (float) -1.5267918E38F;
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_AUTO_ARMED;
            p91.throttle = (float)3.2803423E38F;
            LoopBackDemoChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)58003);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)58985);
                Debug.Assert(pack.time_usec == (ulong)4939247150946558186L);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)55712);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)54659);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)23036);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)33974);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)19288);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)9935);
                Debug.Assert(pack.rssi == (byte)(byte)39);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)8419);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)31254);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)16437);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)55466);
            };
            DemoDevice.HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan7_raw = (ushort)(ushort)8419;
            p92.chan2_raw = (ushort)(ushort)55712;
            p92.time_usec = (ulong)4939247150946558186L;
            p92.chan1_raw = (ushort)(ushort)16437;
            p92.chan3_raw = (ushort)(ushort)31254;
            p92.rssi = (byte)(byte)39;
            p92.chan11_raw = (ushort)(ushort)19288;
            p92.chan8_raw = (ushort)(ushort)54659;
            p92.chan9_raw = (ushort)(ushort)58003;
            p92.chan5_raw = (ushort)(ushort)33974;
            p92.chan4_raw = (ushort)(ushort)9935;
            p92.chan6_raw = (ushort)(ushort)58985;
            p92.chan12_raw = (ushort)(ushort)23036;
            p92.chan10_raw = (ushort)(ushort)55466;
            LoopBackDemoChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)7548168708985595865L);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {1.6489294E38F, 1.7458269E38F, -2.7315613E38F, -2.1043866E38F, 1.8758251E38F, 8.259072E37F, -3.307959E38F, -3.8793612E37F, 1.1561962E38F, -2.1482325E38F, 3.252701E38F, 1.0715619E38F, 2.8049508E38F, 9.435229E37F, -4.1098214E37F, 2.0394553E38F}));
                Debug.Assert(pack.flags == (ulong)6184759727413160002L);
            };
            DemoDevice.HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.flags = (ulong)6184759727413160002L;
            p93.time_usec = (ulong)7548168708985595865L;
            p93.controls_SET(new float[] {1.6489294E38F, 1.7458269E38F, -2.7315613E38F, -2.1043866E38F, 1.8758251E38F, 8.259072E37F, -3.307959E38F, -3.8793612E37F, 1.1561962E38F, -2.1482325E38F, 3.252701E38F, 1.0715619E38F, 2.8049508E38F, 9.435229E37F, -4.1098214E37F, 2.0394553E38F}, 0) ;
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_DISARMED;
            LoopBackDemoChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flow_y == (short)(short)26414);
                Debug.Assert(pack.ground_distance == (float) -2.7591746E38F);
                Debug.Assert(pack.flow_x == (short)(short) -24002);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float) -1.4378653E38F);
                Debug.Assert(pack.quality == (byte)(byte)34);
                Debug.Assert(pack.flow_comp_m_x == (float)1.0662956E37F);
                Debug.Assert(pack.flow_comp_m_y == (float) -1.6712008E38F);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float) -9.907809E37F);
                Debug.Assert(pack.sensor_id == (byte)(byte)102);
                Debug.Assert(pack.time_usec == (ulong)2263706107837162725L);
            };
            DemoDevice.OPTICAL_FLOW p100 = LoopBackDemoChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.flow_comp_m_x = (float)1.0662956E37F;
            p100.flow_x = (short)(short) -24002;
            p100.flow_comp_m_y = (float) -1.6712008E38F;
            p100.sensor_id = (byte)(byte)102;
            p100.time_usec = (ulong)2263706107837162725L;
            p100.flow_rate_y_SET((float) -1.4378653E38F, PH) ;
            p100.flow_rate_x_SET((float) -9.907809E37F, PH) ;
            p100.quality = (byte)(byte)34;
            p100.ground_distance = (float) -2.7591746E38F;
            p100.flow_y = (short)(short)26414;
            LoopBackDemoChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -2.627141E38F);
                Debug.Assert(pack.usec == (ulong)5394013690310875298L);
                Debug.Assert(pack.pitch == (float)3.133025E37F);
                Debug.Assert(pack.x == (float) -1.5762449E38F);
                Debug.Assert(pack.z == (float)2.2792976E38F);
                Debug.Assert(pack.roll == (float) -4.2107043E37F);
                Debug.Assert(pack.yaw == (float) -3.6033977E37F);
            };
            DemoDevice.GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.usec = (ulong)5394013690310875298L;
            p101.pitch = (float)3.133025E37F;
            p101.y = (float) -2.627141E38F;
            p101.yaw = (float) -3.6033977E37F;
            p101.roll = (float) -4.2107043E37F;
            p101.x = (float) -1.5762449E38F;
            p101.z = (float)2.2792976E38F;
            LoopBackDemoChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.usec == (ulong)6817812057893038588L);
                Debug.Assert(pack.y == (float)2.4219878E38F);
                Debug.Assert(pack.yaw == (float) -3.2282126E38F);
                Debug.Assert(pack.z == (float) -1.5841981E38F);
                Debug.Assert(pack.x == (float) -8.5861094E36F);
                Debug.Assert(pack.roll == (float)2.6898665E38F);
                Debug.Assert(pack.pitch == (float)2.9149161E38F);
            };
            DemoDevice.VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.yaw = (float) -3.2282126E38F;
            p102.y = (float)2.4219878E38F;
            p102.usec = (ulong)6817812057893038588L;
            p102.z = (float) -1.5841981E38F;
            p102.pitch = (float)2.9149161E38F;
            p102.x = (float) -8.5861094E36F;
            p102.roll = (float)2.6898665E38F;
            LoopBackDemoChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)1.2199959E38F);
                Debug.Assert(pack.usec == (ulong)2010013615738876119L);
                Debug.Assert(pack.y == (float)2.5418753E38F);
                Debug.Assert(pack.z == (float)9.931635E37F);
            };
            DemoDevice.VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.x = (float)1.2199959E38F;
            p103.usec = (ulong)2010013615738876119L;
            p103.z = (float)9.931635E37F;
            p103.y = (float)2.5418753E38F;
            LoopBackDemoChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float) -2.6835616E38F);
                Debug.Assert(pack.z == (float)3.1587565E38F);
                Debug.Assert(pack.y == (float)2.66676E38F);
                Debug.Assert(pack.usec == (ulong)38593687244864896L);
                Debug.Assert(pack.x == (float)1.6468645E38F);
                Debug.Assert(pack.roll == (float) -1.8630296E38F);
                Debug.Assert(pack.yaw == (float)8.143256E37F);
            };
            DemoDevice.VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.roll = (float) -1.8630296E38F;
            p104.z = (float)3.1587565E38F;
            p104.usec = (ulong)38593687244864896L;
            p104.yaw = (float)8.143256E37F;
            p104.pitch = (float) -2.6835616E38F;
            p104.y = (float)2.66676E38F;
            p104.x = (float)1.6468645E38F;
            LoopBackDemoChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.abs_pressure == (float)4.315842E37F);
                Debug.Assert(pack.xmag == (float)3.1688005E37F);
                Debug.Assert(pack.pressure_alt == (float)1.0830279E38F);
                Debug.Assert(pack.ymag == (float) -2.189447E38F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)31960);
                Debug.Assert(pack.xgyro == (float) -2.4516957E38F);
                Debug.Assert(pack.time_usec == (ulong)3015202372396191369L);
                Debug.Assert(pack.ygyro == (float) -2.5418583E38F);
                Debug.Assert(pack.yacc == (float)1.7195778E38F);
                Debug.Assert(pack.zacc == (float) -2.9319215E38F);
                Debug.Assert(pack.diff_pressure == (float)2.1023168E38F);
                Debug.Assert(pack.xacc == (float)1.5500447E38F);
                Debug.Assert(pack.zgyro == (float)2.228762E38F);
                Debug.Assert(pack.temperature == (float) -5.3167163E37F);
                Debug.Assert(pack.zmag == (float)2.9332745E38F);
            };
            DemoDevice.HIGHRES_IMU p105 = LoopBackDemoChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.pressure_alt = (float)1.0830279E38F;
            p105.fields_updated = (ushort)(ushort)31960;
            p105.xgyro = (float) -2.4516957E38F;
            p105.xmag = (float)3.1688005E37F;
            p105.xacc = (float)1.5500447E38F;
            p105.ymag = (float) -2.189447E38F;
            p105.temperature = (float) -5.3167163E37F;
            p105.yacc = (float)1.7195778E38F;
            p105.time_usec = (ulong)3015202372396191369L;
            p105.abs_pressure = (float)4.315842E37F;
            p105.diff_pressure = (float)2.1023168E38F;
            p105.zacc = (float) -2.9319215E38F;
            p105.zgyro = (float)2.228762E38F;
            p105.ygyro = (float) -2.5418583E38F;
            p105.zmag = (float)2.9332745E38F;
            LoopBackDemoChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_x == (float) -1.1339949E38F);
                Debug.Assert(pack.integrated_xgyro == (float)3.8972673E37F);
                Debug.Assert(pack.distance == (float)1.8716924E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)174);
                Debug.Assert(pack.integrated_ygyro == (float) -1.9922774E38F);
                Debug.Assert(pack.integrated_y == (float)5.2444166E37F);
                Debug.Assert(pack.time_delta_distance_us == (uint)2150425015U);
                Debug.Assert(pack.quality == (byte)(byte)173);
                Debug.Assert(pack.time_usec == (ulong)1580230647596729887L);
                Debug.Assert(pack.integrated_zgyro == (float)3.0710154E38F);
                Debug.Assert(pack.temperature == (short)(short)18358);
                Debug.Assert(pack.integration_time_us == (uint)1500452524U);
            };
            DemoDevice.OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.integrated_y = (float)5.2444166E37F;
            p106.quality = (byte)(byte)173;
            p106.time_delta_distance_us = (uint)2150425015U;
            p106.integrated_xgyro = (float)3.8972673E37F;
            p106.distance = (float)1.8716924E38F;
            p106.integrated_x = (float) -1.1339949E38F;
            p106.integrated_ygyro = (float) -1.9922774E38F;
            p106.time_usec = (ulong)1580230647596729887L;
            p106.sensor_id = (byte)(byte)174;
            p106.integration_time_us = (uint)1500452524U;
            p106.integrated_zgyro = (float)3.0710154E38F;
            p106.temperature = (short)(short)18358;
            LoopBackDemoChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xmag == (float)2.0825736E37F);
                Debug.Assert(pack.pressure_alt == (float) -7.320042E37F);
                Debug.Assert(pack.fields_updated == (uint)3332219946U);
                Debug.Assert(pack.zacc == (float) -2.5097504E38F);
                Debug.Assert(pack.yacc == (float)3.2708862E38F);
                Debug.Assert(pack.zgyro == (float) -8.038852E36F);
                Debug.Assert(pack.xgyro == (float)4.0768564E37F);
                Debug.Assert(pack.abs_pressure == (float)1.2350015E38F);
                Debug.Assert(pack.zmag == (float) -3.0898904E38F);
                Debug.Assert(pack.xacc == (float) -2.743672E37F);
                Debug.Assert(pack.ymag == (float)2.4081733E38F);
                Debug.Assert(pack.temperature == (float)1.2716428E38F);
                Debug.Assert(pack.ygyro == (float) -1.4076042E38F);
                Debug.Assert(pack.diff_pressure == (float)7.6254016E37F);
                Debug.Assert(pack.time_usec == (ulong)2267706265769794636L);
            };
            DemoDevice.HIL_SENSOR p107 = LoopBackDemoChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.time_usec = (ulong)2267706265769794636L;
            p107.fields_updated = (uint)3332219946U;
            p107.zacc = (float) -2.5097504E38F;
            p107.temperature = (float)1.2716428E38F;
            p107.xacc = (float) -2.743672E37F;
            p107.zgyro = (float) -8.038852E36F;
            p107.pressure_alt = (float) -7.320042E37F;
            p107.zmag = (float) -3.0898904E38F;
            p107.ymag = (float)2.4081733E38F;
            p107.diff_pressure = (float)7.6254016E37F;
            p107.abs_pressure = (float)1.2350015E38F;
            p107.ygyro = (float) -1.4076042E38F;
            p107.xgyro = (float)4.0768564E37F;
            p107.xmag = (float)2.0825736E37F;
            p107.yacc = (float)3.2708862E38F;
            LoopBackDemoChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zgyro == (float)1.1306862E38F);
                Debug.Assert(pack.std_dev_horz == (float)1.1724815E38F);
                Debug.Assert(pack.ve == (float)1.3119856E38F);
                Debug.Assert(pack.roll == (float) -9.938225E37F);
                Debug.Assert(pack.xacc == (float)2.4851828E38F);
                Debug.Assert(pack.std_dev_vert == (float) -2.3173547E38F);
                Debug.Assert(pack.yacc == (float)2.5895668E38F);
                Debug.Assert(pack.pitch == (float)3.2454366E38F);
                Debug.Assert(pack.q4 == (float) -2.352855E38F);
                Debug.Assert(pack.yaw == (float)1.5507568E38F);
                Debug.Assert(pack.vd == (float)3.2427604E38F);
                Debug.Assert(pack.q3 == (float)1.3638234E38F);
                Debug.Assert(pack.lon == (float)7.3537253E37F);
                Debug.Assert(pack.xgyro == (float) -3.075101E38F);
                Debug.Assert(pack.alt == (float)1.6468084E38F);
                Debug.Assert(pack.ygyro == (float)1.8743773E37F);
                Debug.Assert(pack.vn == (float) -2.8335277E38F);
                Debug.Assert(pack.lat == (float)1.3110116E38F);
                Debug.Assert(pack.q1 == (float) -3.0198287E38F);
                Debug.Assert(pack.q2 == (float) -1.34302E38F);
                Debug.Assert(pack.zacc == (float)1.8227664E38F);
            };
            DemoDevice.SIM_STATE p108 = LoopBackDemoChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.ve = (float)1.3119856E38F;
            p108.zacc = (float)1.8227664E38F;
            p108.xgyro = (float) -3.075101E38F;
            p108.q2 = (float) -1.34302E38F;
            p108.zgyro = (float)1.1306862E38F;
            p108.q4 = (float) -2.352855E38F;
            p108.yaw = (float)1.5507568E38F;
            p108.ygyro = (float)1.8743773E37F;
            p108.vd = (float)3.2427604E38F;
            p108.std_dev_vert = (float) -2.3173547E38F;
            p108.pitch = (float)3.2454366E38F;
            p108.std_dev_horz = (float)1.1724815E38F;
            p108.lon = (float)7.3537253E37F;
            p108.lat = (float)1.3110116E38F;
            p108.q1 = (float) -3.0198287E38F;
            p108.roll = (float) -9.938225E37F;
            p108.vn = (float) -2.8335277E38F;
            p108.alt = (float)1.6468084E38F;
            p108.q3 = (float)1.3638234E38F;
            p108.xacc = (float)2.4851828E38F;
            p108.yacc = (float)2.5895668E38F;
            LoopBackDemoChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.remrssi == (byte)(byte)107);
                Debug.Assert(pack.rssi == (byte)(byte)81);
                Debug.Assert(pack.remnoise == (byte)(byte)205);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)63217);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)52579);
                Debug.Assert(pack.noise == (byte)(byte)114);
                Debug.Assert(pack.txbuf == (byte)(byte)205);
            };
            DemoDevice.RADIO_STATUS p109 = LoopBackDemoChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.txbuf = (byte)(byte)205;
            p109.rssi = (byte)(byte)81;
            p109.remrssi = (byte)(byte)107;
            p109.rxerrors = (ushort)(ushort)63217;
            p109.fixed_ = (ushort)(ushort)52579;
            p109.remnoise = (byte)(byte)205;
            p109.noise = (byte)(byte)114;
            LoopBackDemoChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)67);
                Debug.Assert(pack.target_network == (byte)(byte)67);
                Debug.Assert(pack.target_system == (byte)(byte)83);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)132, (byte)239, (byte)51, (byte)50, (byte)131, (byte)94, (byte)80, (byte)115, (byte)51, (byte)226, (byte)225, (byte)134, (byte)187, (byte)3, (byte)209, (byte)89, (byte)72, (byte)117, (byte)146, (byte)120, (byte)200, (byte)196, (byte)58, (byte)130, (byte)145, (byte)62, (byte)217, (byte)26, (byte)51, (byte)168, (byte)195, (byte)135, (byte)128, (byte)31, (byte)143, (byte)134, (byte)51, (byte)97, (byte)105, (byte)101, (byte)5, (byte)116, (byte)86, (byte)195, (byte)156, (byte)128, (byte)63, (byte)83, (byte)56, (byte)249, (byte)135, (byte)254, (byte)10, (byte)19, (byte)219, (byte)114, (byte)60, (byte)24, (byte)222, (byte)24, (byte)13, (byte)19, (byte)47, (byte)182, (byte)44, (byte)41, (byte)6, (byte)18, (byte)69, (byte)51, (byte)244, (byte)169, (byte)237, (byte)187, (byte)134, (byte)139, (byte)0, (byte)156, (byte)46, (byte)45, (byte)113, (byte)252, (byte)202, (byte)161, (byte)45, (byte)76, (byte)94, (byte)71, (byte)230, (byte)198, (byte)68, (byte)0, (byte)161, (byte)170, (byte)224, (byte)38, (byte)174, (byte)157, (byte)130, (byte)175, (byte)220, (byte)94, (byte)181, (byte)58, (byte)59, (byte)253, (byte)217, (byte)164, (byte)249, (byte)246, (byte)7, (byte)28, (byte)231, (byte)200, (byte)139, (byte)137, (byte)176, (byte)134, (byte)208, (byte)69, (byte)248, (byte)180, (byte)182, (byte)214, (byte)65, (byte)241, (byte)101, (byte)239, (byte)32, (byte)178, (byte)146, (byte)18, (byte)215, (byte)47, (byte)240, (byte)186, (byte)27, (byte)21, (byte)236, (byte)144, (byte)28, (byte)88, (byte)158, (byte)96, (byte)75, (byte)16, (byte)75, (byte)80, (byte)57, (byte)15, (byte)66, (byte)218, (byte)39, (byte)130, (byte)40, (byte)75, (byte)194, (byte)39, (byte)202, (byte)209, (byte)220, (byte)199, (byte)21, (byte)192, (byte)184, (byte)73, (byte)91, (byte)81, (byte)53, (byte)67, (byte)89, (byte)177, (byte)232, (byte)49, (byte)90, (byte)244, (byte)43, (byte)245, (byte)159, (byte)208, (byte)164, (byte)143, (byte)207, (byte)78, (byte)231, (byte)230, (byte)244, (byte)230, (byte)118, (byte)195, (byte)161, (byte)39, (byte)91, (byte)68, (byte)55, (byte)250, (byte)11, (byte)83, (byte)1, (byte)226, (byte)61, (byte)137, (byte)43, (byte)10, (byte)251, (byte)11, (byte)186, (byte)48, (byte)107, (byte)244, (byte)205, (byte)205, (byte)59, (byte)189, (byte)83, (byte)164, (byte)198, (byte)38, (byte)138, (byte)52, (byte)129, (byte)224, (byte)16, (byte)2, (byte)146, (byte)187, (byte)235, (byte)240, (byte)229, (byte)223, (byte)198, (byte)65, (byte)100, (byte)176, (byte)52, (byte)86, (byte)7, (byte)53, (byte)35, (byte)76, (byte)64, (byte)41, (byte)103, (byte)39, (byte)75, (byte)50, (byte)85, (byte)42, (byte)214, (byte)77, (byte)45}));
            };
            DemoDevice.FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_system = (byte)(byte)83;
            p110.payload_SET(new byte[] {(byte)132, (byte)239, (byte)51, (byte)50, (byte)131, (byte)94, (byte)80, (byte)115, (byte)51, (byte)226, (byte)225, (byte)134, (byte)187, (byte)3, (byte)209, (byte)89, (byte)72, (byte)117, (byte)146, (byte)120, (byte)200, (byte)196, (byte)58, (byte)130, (byte)145, (byte)62, (byte)217, (byte)26, (byte)51, (byte)168, (byte)195, (byte)135, (byte)128, (byte)31, (byte)143, (byte)134, (byte)51, (byte)97, (byte)105, (byte)101, (byte)5, (byte)116, (byte)86, (byte)195, (byte)156, (byte)128, (byte)63, (byte)83, (byte)56, (byte)249, (byte)135, (byte)254, (byte)10, (byte)19, (byte)219, (byte)114, (byte)60, (byte)24, (byte)222, (byte)24, (byte)13, (byte)19, (byte)47, (byte)182, (byte)44, (byte)41, (byte)6, (byte)18, (byte)69, (byte)51, (byte)244, (byte)169, (byte)237, (byte)187, (byte)134, (byte)139, (byte)0, (byte)156, (byte)46, (byte)45, (byte)113, (byte)252, (byte)202, (byte)161, (byte)45, (byte)76, (byte)94, (byte)71, (byte)230, (byte)198, (byte)68, (byte)0, (byte)161, (byte)170, (byte)224, (byte)38, (byte)174, (byte)157, (byte)130, (byte)175, (byte)220, (byte)94, (byte)181, (byte)58, (byte)59, (byte)253, (byte)217, (byte)164, (byte)249, (byte)246, (byte)7, (byte)28, (byte)231, (byte)200, (byte)139, (byte)137, (byte)176, (byte)134, (byte)208, (byte)69, (byte)248, (byte)180, (byte)182, (byte)214, (byte)65, (byte)241, (byte)101, (byte)239, (byte)32, (byte)178, (byte)146, (byte)18, (byte)215, (byte)47, (byte)240, (byte)186, (byte)27, (byte)21, (byte)236, (byte)144, (byte)28, (byte)88, (byte)158, (byte)96, (byte)75, (byte)16, (byte)75, (byte)80, (byte)57, (byte)15, (byte)66, (byte)218, (byte)39, (byte)130, (byte)40, (byte)75, (byte)194, (byte)39, (byte)202, (byte)209, (byte)220, (byte)199, (byte)21, (byte)192, (byte)184, (byte)73, (byte)91, (byte)81, (byte)53, (byte)67, (byte)89, (byte)177, (byte)232, (byte)49, (byte)90, (byte)244, (byte)43, (byte)245, (byte)159, (byte)208, (byte)164, (byte)143, (byte)207, (byte)78, (byte)231, (byte)230, (byte)244, (byte)230, (byte)118, (byte)195, (byte)161, (byte)39, (byte)91, (byte)68, (byte)55, (byte)250, (byte)11, (byte)83, (byte)1, (byte)226, (byte)61, (byte)137, (byte)43, (byte)10, (byte)251, (byte)11, (byte)186, (byte)48, (byte)107, (byte)244, (byte)205, (byte)205, (byte)59, (byte)189, (byte)83, (byte)164, (byte)198, (byte)38, (byte)138, (byte)52, (byte)129, (byte)224, (byte)16, (byte)2, (byte)146, (byte)187, (byte)235, (byte)240, (byte)229, (byte)223, (byte)198, (byte)65, (byte)100, (byte)176, (byte)52, (byte)86, (byte)7, (byte)53, (byte)35, (byte)76, (byte)64, (byte)41, (byte)103, (byte)39, (byte)75, (byte)50, (byte)85, (byte)42, (byte)214, (byte)77, (byte)45}, 0) ;
            p110.target_network = (byte)(byte)67;
            p110.target_component = (byte)(byte)67;
            LoopBackDemoChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tc1 == (long) -7290212625726500997L);
                Debug.Assert(pack.ts1 == (long)5247347794158199819L);
            };
            DemoDevice.TIMESYNC p111 = LoopBackDemoChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long) -7290212625726500997L;
            p111.ts1 = (long)5247347794158199819L;
            LoopBackDemoChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (uint)1806190592U);
                Debug.Assert(pack.time_usec == (ulong)1139071301994918109L);
            };
            DemoDevice.CAMERA_TRIGGER p112 = LoopBackDemoChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.seq = (uint)1806190592U;
            p112.time_usec = (ulong)1139071301994918109L;
            LoopBackDemoChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.epv == (ushort)(ushort)3628);
                Debug.Assert(pack.time_usec == (ulong)7724561575181205480L);
                Debug.Assert(pack.eph == (ushort)(ushort)52475);
                Debug.Assert(pack.vn == (short)(short)14512);
                Debug.Assert(pack.ve == (short)(short) -20398);
                Debug.Assert(pack.fix_type == (byte)(byte)93);
                Debug.Assert(pack.vd == (short)(short)9416);
                Debug.Assert(pack.lon == (int)1279558542);
                Debug.Assert(pack.lat == (int)845683744);
                Debug.Assert(pack.alt == (int) -1671150014);
                Debug.Assert(pack.vel == (ushort)(ushort)37657);
                Debug.Assert(pack.cog == (ushort)(ushort)15473);
                Debug.Assert(pack.satellites_visible == (byte)(byte)185);
            };
            DemoDevice.HIL_GPS p113 = LoopBackDemoChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.vd = (short)(short)9416;
            p113.time_usec = (ulong)7724561575181205480L;
            p113.eph = (ushort)(ushort)52475;
            p113.ve = (short)(short) -20398;
            p113.lon = (int)1279558542;
            p113.vel = (ushort)(ushort)37657;
            p113.fix_type = (byte)(byte)93;
            p113.vn = (short)(short)14512;
            p113.alt = (int) -1671150014;
            p113.cog = (ushort)(ushort)15473;
            p113.satellites_visible = (byte)(byte)185;
            p113.epv = (ushort)(ushort)3628;
            p113.lat = (int)845683744;
            LoopBackDemoChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_delta_distance_us == (uint)2863167643U);
                Debug.Assert(pack.integration_time_us == (uint)1763576389U);
                Debug.Assert(pack.temperature == (short)(short) -25416);
                Debug.Assert(pack.integrated_y == (float)1.8929204E38F);
                Debug.Assert(pack.distance == (float) -2.1429944E38F);
                Debug.Assert(pack.quality == (byte)(byte)230);
                Debug.Assert(pack.integrated_ygyro == (float) -1.0152671E38F);
                Debug.Assert(pack.integrated_x == (float)1.1617593E37F);
                Debug.Assert(pack.sensor_id == (byte)(byte)252);
                Debug.Assert(pack.integrated_xgyro == (float) -1.7575212E37F);
                Debug.Assert(pack.time_usec == (ulong)1100716721211741832L);
                Debug.Assert(pack.integrated_zgyro == (float) -6.192338E37F);
            };
            DemoDevice.HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.integration_time_us = (uint)1763576389U;
            p114.quality = (byte)(byte)230;
            p114.time_usec = (ulong)1100716721211741832L;
            p114.time_delta_distance_us = (uint)2863167643U;
            p114.distance = (float) -2.1429944E38F;
            p114.sensor_id = (byte)(byte)252;
            p114.integrated_x = (float)1.1617593E37F;
            p114.integrated_xgyro = (float) -1.7575212E37F;
            p114.integrated_ygyro = (float) -1.0152671E38F;
            p114.integrated_y = (float)1.8929204E38F;
            p114.integrated_zgyro = (float) -6.192338E37F;
            p114.temperature = (short)(short) -25416;
            LoopBackDemoChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (short)(short) -11263);
                Debug.Assert(pack.lat == (int) -154983356);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {9.960295E37F, 2.673226E38F, -1.0089344E38F, 2.7977771E37F}));
                Debug.Assert(pack.yawspeed == (float) -3.322032E38F);
                Debug.Assert(pack.xacc == (short)(short)32041);
                Debug.Assert(pack.time_usec == (ulong)9176692696425769250L);
                Debug.Assert(pack.lon == (int) -1589980243);
                Debug.Assert(pack.rollspeed == (float) -2.6123636E38F);
                Debug.Assert(pack.vz == (short)(short)17753);
                Debug.Assert(pack.pitchspeed == (float)7.632298E37F);
                Debug.Assert(pack.vx == (short)(short)2980);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)27370);
                Debug.Assert(pack.yacc == (short)(short) -7800);
                Debug.Assert(pack.alt == (int)364315530);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)31988);
                Debug.Assert(pack.zacc == (short)(short) -3440);
            };
            DemoDevice.HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.true_airspeed = (ushort)(ushort)31988;
            p115.xacc = (short)(short)32041;
            p115.ind_airspeed = (ushort)(ushort)27370;
            p115.vx = (short)(short)2980;
            p115.lon = (int) -1589980243;
            p115.zacc = (short)(short) -3440;
            p115.rollspeed = (float) -2.6123636E38F;
            p115.alt = (int)364315530;
            p115.attitude_quaternion_SET(new float[] {9.960295E37F, 2.673226E38F, -1.0089344E38F, 2.7977771E37F}, 0) ;
            p115.vy = (short)(short) -11263;
            p115.pitchspeed = (float)7.632298E37F;
            p115.yacc = (short)(short) -7800;
            p115.vz = (short)(short)17753;
            p115.lat = (int) -154983356;
            p115.time_usec = (ulong)9176692696425769250L;
            p115.yawspeed = (float) -3.322032E38F;
            LoopBackDemoChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xmag == (short)(short)2562);
                Debug.Assert(pack.ygyro == (short)(short)18627);
                Debug.Assert(pack.zgyro == (short)(short) -2810);
                Debug.Assert(pack.zmag == (short)(short) -22377);
                Debug.Assert(pack.ymag == (short)(short) -299);
                Debug.Assert(pack.time_boot_ms == (uint)1664647133U);
                Debug.Assert(pack.zacc == (short)(short) -23271);
                Debug.Assert(pack.xacc == (short)(short)21239);
                Debug.Assert(pack.xgyro == (short)(short)14606);
                Debug.Assert(pack.yacc == (short)(short)7351);
            };
            DemoDevice.SCALED_IMU2 p116 = LoopBackDemoChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.zmag = (short)(short) -22377;
            p116.xacc = (short)(short)21239;
            p116.zgyro = (short)(short) -2810;
            p116.ygyro = (short)(short)18627;
            p116.yacc = (short)(short)7351;
            p116.time_boot_ms = (uint)1664647133U;
            p116.xgyro = (short)(short)14606;
            p116.ymag = (short)(short) -299;
            p116.zacc = (short)(short) -23271;
            p116.xmag = (short)(short)2562;
            LoopBackDemoChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start == (ushort)(ushort)30121);
                Debug.Assert(pack.target_component == (byte)(byte)28);
                Debug.Assert(pack.end == (ushort)(ushort)16275);
                Debug.Assert(pack.target_system == (byte)(byte)251);
            };
            DemoDevice.LOG_REQUEST_LIST p117 = LoopBackDemoChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_component = (byte)(byte)28;
            p117.end = (ushort)(ushort)16275;
            p117.start = (ushort)(ushort)30121;
            p117.target_system = (byte)(byte)251;
            LoopBackDemoChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.num_logs == (ushort)(ushort)15511);
                Debug.Assert(pack.id == (ushort)(ushort)23225);
                Debug.Assert(pack.time_utc == (uint)1430830198U);
                Debug.Assert(pack.size == (uint)1504238023U);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)51844);
            };
            DemoDevice.LOG_ENTRY p118 = LoopBackDemoChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.size = (uint)1504238023U;
            p118.time_utc = (uint)1430830198U;
            p118.num_logs = (ushort)(ushort)15511;
            p118.id = (ushort)(ushort)23225;
            p118.last_log_num = (ushort)(ushort)51844;
            LoopBackDemoChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (ushort)(ushort)17445);
                Debug.Assert(pack.target_system == (byte)(byte)139);
                Debug.Assert(pack.target_component == (byte)(byte)25);
                Debug.Assert(pack.count == (uint)3703318320U);
                Debug.Assert(pack.ofs == (uint)1726251501U);
            };
            DemoDevice.LOG_REQUEST_DATA p119 = LoopBackDemoChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_system = (byte)(byte)139;
            p119.count = (uint)3703318320U;
            p119.target_component = (byte)(byte)25;
            p119.id = (ushort)(ushort)17445;
            p119.ofs = (uint)1726251501U;
            LoopBackDemoChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (ushort)(ushort)21098);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)221, (byte)16, (byte)82, (byte)239, (byte)228, (byte)82, (byte)60, (byte)61, (byte)160, (byte)37, (byte)92, (byte)131, (byte)12, (byte)113, (byte)131, (byte)41, (byte)54, (byte)197, (byte)28, (byte)177, (byte)135, (byte)159, (byte)105, (byte)80, (byte)213, (byte)220, (byte)172, (byte)62, (byte)94, (byte)18, (byte)144, (byte)118, (byte)32, (byte)239, (byte)127, (byte)134, (byte)116, (byte)158, (byte)108, (byte)124, (byte)241, (byte)185, (byte)96, (byte)94, (byte)232, (byte)56, (byte)143, (byte)132, (byte)73, (byte)192, (byte)90, (byte)21, (byte)167, (byte)41, (byte)102, (byte)62, (byte)172, (byte)111, (byte)80, (byte)190, (byte)0, (byte)102, (byte)220, (byte)122, (byte)241, (byte)204, (byte)106, (byte)142, (byte)192, (byte)179, (byte)161, (byte)52, (byte)28, (byte)60, (byte)188, (byte)195, (byte)206, (byte)230, (byte)162, (byte)77, (byte)227, (byte)178, (byte)61, (byte)1, (byte)136, (byte)13, (byte)201, (byte)212, (byte)120, (byte)65}));
                Debug.Assert(pack.count == (byte)(byte)240);
                Debug.Assert(pack.ofs == (uint)1903147581U);
            };
            DemoDevice.LOG_DATA p120 = LoopBackDemoChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.ofs = (uint)1903147581U;
            p120.id = (ushort)(ushort)21098;
            p120.count = (byte)(byte)240;
            p120.data__SET(new byte[] {(byte)221, (byte)16, (byte)82, (byte)239, (byte)228, (byte)82, (byte)60, (byte)61, (byte)160, (byte)37, (byte)92, (byte)131, (byte)12, (byte)113, (byte)131, (byte)41, (byte)54, (byte)197, (byte)28, (byte)177, (byte)135, (byte)159, (byte)105, (byte)80, (byte)213, (byte)220, (byte)172, (byte)62, (byte)94, (byte)18, (byte)144, (byte)118, (byte)32, (byte)239, (byte)127, (byte)134, (byte)116, (byte)158, (byte)108, (byte)124, (byte)241, (byte)185, (byte)96, (byte)94, (byte)232, (byte)56, (byte)143, (byte)132, (byte)73, (byte)192, (byte)90, (byte)21, (byte)167, (byte)41, (byte)102, (byte)62, (byte)172, (byte)111, (byte)80, (byte)190, (byte)0, (byte)102, (byte)220, (byte)122, (byte)241, (byte)204, (byte)106, (byte)142, (byte)192, (byte)179, (byte)161, (byte)52, (byte)28, (byte)60, (byte)188, (byte)195, (byte)206, (byte)230, (byte)162, (byte)77, (byte)227, (byte)178, (byte)61, (byte)1, (byte)136, (byte)13, (byte)201, (byte)212, (byte)120, (byte)65}, 0) ;
            LoopBackDemoChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)118);
                Debug.Assert(pack.target_component == (byte)(byte)221);
            };
            DemoDevice.LOG_ERASE p121 = LoopBackDemoChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)118;
            p121.target_component = (byte)(byte)221;
            LoopBackDemoChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)8);
                Debug.Assert(pack.target_component == (byte)(byte)221);
            };
            DemoDevice.LOG_REQUEST_END p122 = LoopBackDemoChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_component = (byte)(byte)221;
            p122.target_system = (byte)(byte)8;
            LoopBackDemoChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)168);
                Debug.Assert(pack.target_component == (byte)(byte)187);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)243, (byte)36, (byte)198, (byte)99, (byte)41, (byte)190, (byte)205, (byte)6, (byte)72, (byte)178, (byte)192, (byte)15, (byte)158, (byte)175, (byte)36, (byte)68, (byte)4, (byte)47, (byte)51, (byte)173, (byte)115, (byte)120, (byte)249, (byte)149, (byte)46, (byte)217, (byte)34, (byte)167, (byte)250, (byte)106, (byte)76, (byte)137, (byte)76, (byte)167, (byte)211, (byte)221, (byte)19, (byte)116, (byte)222, (byte)163, (byte)242, (byte)15, (byte)5, (byte)233, (byte)53, (byte)35, (byte)100, (byte)249, (byte)182, (byte)65, (byte)90, (byte)42, (byte)149, (byte)107, (byte)15, (byte)190, (byte)204, (byte)87, (byte)87, (byte)48, (byte)124, (byte)62, (byte)73, (byte)29, (byte)195, (byte)45, (byte)102, (byte)194, (byte)194, (byte)150, (byte)239, (byte)106, (byte)110, (byte)195, (byte)134, (byte)145, (byte)112, (byte)116, (byte)221, (byte)14, (byte)28, (byte)37, (byte)147, (byte)189, (byte)142, (byte)206, (byte)47, (byte)71, (byte)255, (byte)109, (byte)102, (byte)189, (byte)10, (byte)172, (byte)93, (byte)154, (byte)10, (byte)36, (byte)2, (byte)218, (byte)249, (byte)135, (byte)35, (byte)178, (byte)249, (byte)162, (byte)222, (byte)13, (byte)216, (byte)16}));
                Debug.Assert(pack.len == (byte)(byte)188);
            };
            DemoDevice.GPS_INJECT_DATA p123 = LoopBackDemoChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_component = (byte)(byte)187;
            p123.data__SET(new byte[] {(byte)243, (byte)36, (byte)198, (byte)99, (byte)41, (byte)190, (byte)205, (byte)6, (byte)72, (byte)178, (byte)192, (byte)15, (byte)158, (byte)175, (byte)36, (byte)68, (byte)4, (byte)47, (byte)51, (byte)173, (byte)115, (byte)120, (byte)249, (byte)149, (byte)46, (byte)217, (byte)34, (byte)167, (byte)250, (byte)106, (byte)76, (byte)137, (byte)76, (byte)167, (byte)211, (byte)221, (byte)19, (byte)116, (byte)222, (byte)163, (byte)242, (byte)15, (byte)5, (byte)233, (byte)53, (byte)35, (byte)100, (byte)249, (byte)182, (byte)65, (byte)90, (byte)42, (byte)149, (byte)107, (byte)15, (byte)190, (byte)204, (byte)87, (byte)87, (byte)48, (byte)124, (byte)62, (byte)73, (byte)29, (byte)195, (byte)45, (byte)102, (byte)194, (byte)194, (byte)150, (byte)239, (byte)106, (byte)110, (byte)195, (byte)134, (byte)145, (byte)112, (byte)116, (byte)221, (byte)14, (byte)28, (byte)37, (byte)147, (byte)189, (byte)142, (byte)206, (byte)47, (byte)71, (byte)255, (byte)109, (byte)102, (byte)189, (byte)10, (byte)172, (byte)93, (byte)154, (byte)10, (byte)36, (byte)2, (byte)218, (byte)249, (byte)135, (byte)35, (byte)178, (byte)249, (byte)162, (byte)222, (byte)13, (byte)216, (byte)16}, 0) ;
            p123.len = (byte)(byte)188;
            p123.target_system = (byte)(byte)168;
            LoopBackDemoChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
                Debug.Assert(pack.epv == (ushort)(ushort)50851);
                Debug.Assert(pack.lat == (int)2091383605);
                Debug.Assert(pack.vel == (ushort)(ushort)20156);
                Debug.Assert(pack.dgps_numch == (byte)(byte)108);
                Debug.Assert(pack.eph == (ushort)(ushort)31342);
                Debug.Assert(pack.time_usec == (ulong)6305834954173973141L);
                Debug.Assert(pack.dgps_age == (uint)4169674191U);
                Debug.Assert(pack.satellites_visible == (byte)(byte)246);
                Debug.Assert(pack.lon == (int) -1132263677);
                Debug.Assert(pack.alt == (int)1582078357);
                Debug.Assert(pack.cog == (ushort)(ushort)47135);
            };
            DemoDevice.GPS2_RAW p124 = LoopBackDemoChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)6305834954173973141L;
            p124.eph = (ushort)(ushort)31342;
            p124.cog = (ushort)(ushort)47135;
            p124.satellites_visible = (byte)(byte)246;
            p124.epv = (ushort)(ushort)50851;
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p124.lon = (int) -1132263677;
            p124.alt = (int)1582078357;
            p124.dgps_numch = (byte)(byte)108;
            p124.dgps_age = (uint)4169674191U;
            p124.lat = (int)2091383605;
            p124.vel = (ushort)(ushort)20156;
            LoopBackDemoChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Vcc == (ushort)(ushort)26174);
                Debug.Assert(pack.Vservo == (ushort)(ushort)62419);
                Debug.Assert(pack.flags == (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID);
            };
            DemoDevice.POWER_STATUS p125 = LoopBackDemoChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID;
            p125.Vservo = (ushort)(ushort)62419;
            p125.Vcc = (ushort)(ushort)26174;
            LoopBackDemoChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baudrate == (uint)522155583U);
                Debug.Assert(pack.count == (byte)(byte)108);
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)142, (byte)229, (byte)172, (byte)120, (byte)190, (byte)225, (byte)206, (byte)99, (byte)87, (byte)107, (byte)37, (byte)95, (byte)153, (byte)203, (byte)214, (byte)63, (byte)177, (byte)140, (byte)109, (byte)22, (byte)50, (byte)238, (byte)180, (byte)103, (byte)189, (byte)221, (byte)222, (byte)99, (byte)184, (byte)245, (byte)215, (byte)63, (byte)221, (byte)151, (byte)159, (byte)4, (byte)204, (byte)1, (byte)93, (byte)43, (byte)135, (byte)73, (byte)92, (byte)182, (byte)128, (byte)156, (byte)228, (byte)25, (byte)32, (byte)73, (byte)225, (byte)204, (byte)135, (byte)249, (byte)228, (byte)87, (byte)115, (byte)18, (byte)160, (byte)133, (byte)240, (byte)58, (byte)190, (byte)180, (byte)135, (byte)87, (byte)56, (byte)140, (byte)87, (byte)249}));
                Debug.Assert(pack.timeout == (ushort)(ushort)28162);
                Debug.Assert(pack.device == (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL);
            };
            DemoDevice.SERIAL_CONTROL p126 = LoopBackDemoChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL;
            p126.baudrate = (uint)522155583U;
            p126.data__SET(new byte[] {(byte)142, (byte)229, (byte)172, (byte)120, (byte)190, (byte)225, (byte)206, (byte)99, (byte)87, (byte)107, (byte)37, (byte)95, (byte)153, (byte)203, (byte)214, (byte)63, (byte)177, (byte)140, (byte)109, (byte)22, (byte)50, (byte)238, (byte)180, (byte)103, (byte)189, (byte)221, (byte)222, (byte)99, (byte)184, (byte)245, (byte)215, (byte)63, (byte)221, (byte)151, (byte)159, (byte)4, (byte)204, (byte)1, (byte)93, (byte)43, (byte)135, (byte)73, (byte)92, (byte)182, (byte)128, (byte)156, (byte)228, (byte)25, (byte)32, (byte)73, (byte)225, (byte)204, (byte)135, (byte)249, (byte)228, (byte)87, (byte)115, (byte)18, (byte)160, (byte)133, (byte)240, (byte)58, (byte)190, (byte)180, (byte)135, (byte)87, (byte)56, (byte)140, (byte)87, (byte)249}, 0) ;
            p126.count = (byte)(byte)108;
            p126.timeout = (ushort)(ushort)28162;
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE;
            LoopBackDemoChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_last_baseline_ms == (uint)967177319U);
                Debug.Assert(pack.rtk_health == (byte)(byte)93);
                Debug.Assert(pack.iar_num_hypotheses == (int) -970320485);
                Debug.Assert(pack.nsats == (byte)(byte)110);
                Debug.Assert(pack.accuracy == (uint)2447604417U);
                Debug.Assert(pack.baseline_c_mm == (int) -863097586);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)166);
                Debug.Assert(pack.baseline_a_mm == (int) -1339666134);
                Debug.Assert(pack.rtk_rate == (byte)(byte)29);
                Debug.Assert(pack.tow == (uint)4015517494U);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)124);
                Debug.Assert(pack.wn == (ushort)(ushort)46084);
                Debug.Assert(pack.baseline_b_mm == (int) -1762929171);
            };
            DemoDevice.GPS_RTK p127 = LoopBackDemoChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.tow = (uint)4015517494U;
            p127.rtk_health = (byte)(byte)93;
            p127.wn = (ushort)(ushort)46084;
            p127.baseline_coords_type = (byte)(byte)166;
            p127.baseline_b_mm = (int) -1762929171;
            p127.rtk_rate = (byte)(byte)29;
            p127.time_last_baseline_ms = (uint)967177319U;
            p127.baseline_c_mm = (int) -863097586;
            p127.rtk_receiver_id = (byte)(byte)124;
            p127.baseline_a_mm = (int) -1339666134;
            p127.iar_num_hypotheses = (int) -970320485;
            p127.nsats = (byte)(byte)110;
            p127.accuracy = (uint)2447604417U;
            LoopBackDemoChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.accuracy == (uint)3159768966U);
                Debug.Assert(pack.baseline_c_mm == (int)145662885);
                Debug.Assert(pack.nsats == (byte)(byte)245);
                Debug.Assert(pack.rtk_health == (byte)(byte)172);
                Debug.Assert(pack.wn == (ushort)(ushort)14928);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)203);
                Debug.Assert(pack.iar_num_hypotheses == (int)1209997333);
                Debug.Assert(pack.tow == (uint)2930533958U);
                Debug.Assert(pack.baseline_b_mm == (int) -792210688);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)91);
                Debug.Assert(pack.time_last_baseline_ms == (uint)3435495560U);
                Debug.Assert(pack.baseline_a_mm == (int)174841408);
                Debug.Assert(pack.rtk_rate == (byte)(byte)119);
            };
            DemoDevice.GPS2_RTK p128 = LoopBackDemoChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.time_last_baseline_ms = (uint)3435495560U;
            p128.baseline_b_mm = (int) -792210688;
            p128.nsats = (byte)(byte)245;
            p128.rtk_rate = (byte)(byte)119;
            p128.tow = (uint)2930533958U;
            p128.wn = (ushort)(ushort)14928;
            p128.baseline_c_mm = (int)145662885;
            p128.baseline_coords_type = (byte)(byte)203;
            p128.rtk_health = (byte)(byte)172;
            p128.accuracy = (uint)3159768966U;
            p128.rtk_receiver_id = (byte)(byte)91;
            p128.iar_num_hypotheses = (int)1209997333;
            p128.baseline_a_mm = (int)174841408;
            LoopBackDemoChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (short)(short)15370);
                Debug.Assert(pack.xgyro == (short)(short)6331);
                Debug.Assert(pack.xmag == (short)(short) -15984);
                Debug.Assert(pack.zacc == (short)(short) -3763);
                Debug.Assert(pack.time_boot_ms == (uint)8712489U);
                Debug.Assert(pack.zgyro == (short)(short) -25417);
                Debug.Assert(pack.yacc == (short)(short)4530);
                Debug.Assert(pack.ygyro == (short)(short) -16611);
                Debug.Assert(pack.ymag == (short)(short) -6553);
                Debug.Assert(pack.zmag == (short)(short)5907);
            };
            DemoDevice.SCALED_IMU3 p129 = LoopBackDemoChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.zmag = (short)(short)5907;
            p129.time_boot_ms = (uint)8712489U;
            p129.xacc = (short)(short)15370;
            p129.xgyro = (short)(short)6331;
            p129.xmag = (short)(short) -15984;
            p129.ygyro = (short)(short) -16611;
            p129.yacc = (short)(short)4530;
            p129.zgyro = (short)(short) -25417;
            p129.ymag = (short)(short) -6553;
            p129.zacc = (short)(short) -3763;
            LoopBackDemoChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (byte)(byte)227);
                Debug.Assert(pack.width == (ushort)(ushort)30300);
                Debug.Assert(pack.jpg_quality == (byte)(byte)178);
                Debug.Assert(pack.packets == (ushort)(ushort)37377);
                Debug.Assert(pack.payload == (byte)(byte)84);
                Debug.Assert(pack.size == (uint)1008357934U);
                Debug.Assert(pack.height == (ushort)(ushort)49700);
            };
            DemoDevice.DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)227;
            p130.payload = (byte)(byte)84;
            p130.width = (ushort)(ushort)30300;
            p130.size = (uint)1008357934U;
            p130.jpg_quality = (byte)(byte)178;
            p130.packets = (ushort)(ushort)37377;
            p130.height = (ushort)(ushort)49700;
            LoopBackDemoChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seqnr == (ushort)(ushort)50876);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)14, (byte)186, (byte)173, (byte)135, (byte)30, (byte)64, (byte)136, (byte)215, (byte)212, (byte)131, (byte)109, (byte)24, (byte)15, (byte)177, (byte)189, (byte)64, (byte)58, (byte)41, (byte)68, (byte)79, (byte)39, (byte)51, (byte)191, (byte)15, (byte)184, (byte)175, (byte)166, (byte)128, (byte)255, (byte)107, (byte)250, (byte)229, (byte)169, (byte)27, (byte)231, (byte)208, (byte)103, (byte)154, (byte)250, (byte)178, (byte)43, (byte)92, (byte)12, (byte)211, (byte)224, (byte)125, (byte)214, (byte)85, (byte)233, (byte)174, (byte)118, (byte)156, (byte)19, (byte)80, (byte)241, (byte)79, (byte)30, (byte)66, (byte)73, (byte)216, (byte)224, (byte)137, (byte)68, (byte)189, (byte)19, (byte)54, (byte)144, (byte)184, (byte)32, (byte)22, (byte)26, (byte)40, (byte)148, (byte)105, (byte)23, (byte)8, (byte)73, (byte)189, (byte)140, (byte)88, (byte)46, (byte)78, (byte)75, (byte)167, (byte)216, (byte)74, (byte)231, (byte)74, (byte)202, (byte)16, (byte)65, (byte)101, (byte)193, (byte)108, (byte)203, (byte)157, (byte)122, (byte)72, (byte)130, (byte)98, (byte)161, (byte)218, (byte)165, (byte)21, (byte)7, (byte)128, (byte)76, (byte)139, (byte)244, (byte)180, (byte)225, (byte)11, (byte)67, (byte)10, (byte)103, (byte)145, (byte)51, (byte)44, (byte)20, (byte)253, (byte)18, (byte)162, (byte)82, (byte)210, (byte)83, (byte)61, (byte)13, (byte)138, (byte)96, (byte)51, (byte)244, (byte)118, (byte)90, (byte)93, (byte)88, (byte)108, (byte)226, (byte)87, (byte)35, (byte)242, (byte)101, (byte)156, (byte)92, (byte)8, (byte)37, (byte)78, (byte)161, (byte)44, (byte)255, (byte)9, (byte)123, (byte)46, (byte)110, (byte)144, (byte)147, (byte)19, (byte)24, (byte)77, (byte)171, (byte)248, (byte)216, (byte)196, (byte)253, (byte)124, (byte)63, (byte)111, (byte)101, (byte)116, (byte)170, (byte)9, (byte)118, (byte)254, (byte)61, (byte)230, (byte)36, (byte)247, (byte)194, (byte)91, (byte)57, (byte)196, (byte)112, (byte)120, (byte)122, (byte)115, (byte)38, (byte)32, (byte)80, (byte)135, (byte)55, (byte)161, (byte)216, (byte)19, (byte)136, (byte)110, (byte)136, (byte)164, (byte)226, (byte)225, (byte)230, (byte)91, (byte)77, (byte)50, (byte)197, (byte)180, (byte)79, (byte)222, (byte)229, (byte)83, (byte)127, (byte)190, (byte)157, (byte)185, (byte)73, (byte)131, (byte)246, (byte)185, (byte)176, (byte)19, (byte)248, (byte)227, (byte)33, (byte)77, (byte)126, (byte)171, (byte)30, (byte)83, (byte)20, (byte)179, (byte)6, (byte)176, (byte)241, (byte)8, (byte)237, (byte)162, (byte)254, (byte)159, (byte)64, (byte)226, (byte)151, (byte)46, (byte)8, (byte)181, (byte)210, (byte)46, (byte)215, (byte)70, (byte)210, (byte)188, (byte)100, (byte)153, (byte)212, (byte)175, (byte)61}));
            };
            DemoDevice.ENCAPSULATED_DATA p131 = LoopBackDemoChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.data__SET(new byte[] {(byte)14, (byte)186, (byte)173, (byte)135, (byte)30, (byte)64, (byte)136, (byte)215, (byte)212, (byte)131, (byte)109, (byte)24, (byte)15, (byte)177, (byte)189, (byte)64, (byte)58, (byte)41, (byte)68, (byte)79, (byte)39, (byte)51, (byte)191, (byte)15, (byte)184, (byte)175, (byte)166, (byte)128, (byte)255, (byte)107, (byte)250, (byte)229, (byte)169, (byte)27, (byte)231, (byte)208, (byte)103, (byte)154, (byte)250, (byte)178, (byte)43, (byte)92, (byte)12, (byte)211, (byte)224, (byte)125, (byte)214, (byte)85, (byte)233, (byte)174, (byte)118, (byte)156, (byte)19, (byte)80, (byte)241, (byte)79, (byte)30, (byte)66, (byte)73, (byte)216, (byte)224, (byte)137, (byte)68, (byte)189, (byte)19, (byte)54, (byte)144, (byte)184, (byte)32, (byte)22, (byte)26, (byte)40, (byte)148, (byte)105, (byte)23, (byte)8, (byte)73, (byte)189, (byte)140, (byte)88, (byte)46, (byte)78, (byte)75, (byte)167, (byte)216, (byte)74, (byte)231, (byte)74, (byte)202, (byte)16, (byte)65, (byte)101, (byte)193, (byte)108, (byte)203, (byte)157, (byte)122, (byte)72, (byte)130, (byte)98, (byte)161, (byte)218, (byte)165, (byte)21, (byte)7, (byte)128, (byte)76, (byte)139, (byte)244, (byte)180, (byte)225, (byte)11, (byte)67, (byte)10, (byte)103, (byte)145, (byte)51, (byte)44, (byte)20, (byte)253, (byte)18, (byte)162, (byte)82, (byte)210, (byte)83, (byte)61, (byte)13, (byte)138, (byte)96, (byte)51, (byte)244, (byte)118, (byte)90, (byte)93, (byte)88, (byte)108, (byte)226, (byte)87, (byte)35, (byte)242, (byte)101, (byte)156, (byte)92, (byte)8, (byte)37, (byte)78, (byte)161, (byte)44, (byte)255, (byte)9, (byte)123, (byte)46, (byte)110, (byte)144, (byte)147, (byte)19, (byte)24, (byte)77, (byte)171, (byte)248, (byte)216, (byte)196, (byte)253, (byte)124, (byte)63, (byte)111, (byte)101, (byte)116, (byte)170, (byte)9, (byte)118, (byte)254, (byte)61, (byte)230, (byte)36, (byte)247, (byte)194, (byte)91, (byte)57, (byte)196, (byte)112, (byte)120, (byte)122, (byte)115, (byte)38, (byte)32, (byte)80, (byte)135, (byte)55, (byte)161, (byte)216, (byte)19, (byte)136, (byte)110, (byte)136, (byte)164, (byte)226, (byte)225, (byte)230, (byte)91, (byte)77, (byte)50, (byte)197, (byte)180, (byte)79, (byte)222, (byte)229, (byte)83, (byte)127, (byte)190, (byte)157, (byte)185, (byte)73, (byte)131, (byte)246, (byte)185, (byte)176, (byte)19, (byte)248, (byte)227, (byte)33, (byte)77, (byte)126, (byte)171, (byte)30, (byte)83, (byte)20, (byte)179, (byte)6, (byte)176, (byte)241, (byte)8, (byte)237, (byte)162, (byte)254, (byte)159, (byte)64, (byte)226, (byte)151, (byte)46, (byte)8, (byte)181, (byte)210, (byte)46, (byte)215, (byte)70, (byte)210, (byte)188, (byte)100, (byte)153, (byte)212, (byte)175, (byte)61}, 0) ;
            p131.seqnr = (ushort)(ushort)50876;
            LoopBackDemoChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.max_distance == (ushort)(ushort)26075);
                Debug.Assert(pack.id == (byte)(byte)69);
                Debug.Assert(pack.orientation == (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_180);
                Debug.Assert(pack.min_distance == (ushort)(ushort)15084);
                Debug.Assert(pack.type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
                Debug.Assert(pack.current_distance == (ushort)(ushort)25034);
                Debug.Assert(pack.covariance == (byte)(byte)60);
                Debug.Assert(pack.time_boot_ms == (uint)3134286258U);
            };
            DemoDevice.DISTANCE_SENSOR p132 = LoopBackDemoChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)3134286258U;
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN;
            p132.covariance = (byte)(byte)60;
            p132.max_distance = (ushort)(ushort)26075;
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_180;
            p132.current_distance = (ushort)(ushort)25034;
            p132.id = (byte)(byte)69;
            p132.min_distance = (ushort)(ushort)15084;
            LoopBackDemoChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)3019);
                Debug.Assert(pack.lon == (int)1606035154);
                Debug.Assert(pack.lat == (int) -2001388880);
                Debug.Assert(pack.mask == (ulong)5029092606966334620L);
            };
            DemoDevice.TERRAIN_REQUEST p133 = LoopBackDemoChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.mask = (ulong)5029092606966334620L;
            p133.lat = (int) -2001388880;
            p133.grid_spacing = (ushort)(ushort)3019;
            p133.lon = (int)1606035154;
            LoopBackDemoChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gridbit == (byte)(byte)75);
                Debug.Assert(pack.lat == (int) -286820747);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)65423);
                Debug.Assert(pack.lon == (int) -706747338);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short) -29498, (short)3628, (short)25006, (short)21679, (short) -8180, (short) -6345, (short) -25463, (short)28629, (short)23713, (short) -30010, (short)135, (short)21277, (short)4467, (short)449, (short)24579, (short) -12770}));
            };
            DemoDevice.TERRAIN_DATA p134 = LoopBackDemoChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.grid_spacing = (ushort)(ushort)65423;
            p134.data__SET(new short[] {(short) -29498, (short)3628, (short)25006, (short)21679, (short) -8180, (short) -6345, (short) -25463, (short)28629, (short)23713, (short) -30010, (short)135, (short)21277, (short)4467, (short)449, (short)24579, (short) -12770}, 0) ;
            p134.gridbit = (byte)(byte)75;
            p134.lat = (int) -286820747;
            p134.lon = (int) -706747338;
            LoopBackDemoChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -979019742);
                Debug.Assert(pack.lon == (int)1654580750);
            };
            DemoDevice.TERRAIN_CHECK p135 = LoopBackDemoChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int) -979019742;
            p135.lon = (int)1654580750;
            LoopBackDemoChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.loaded == (ushort)(ushort)27302);
                Debug.Assert(pack.lat == (int) -1643308010);
                Debug.Assert(pack.current_height == (float)1.0640828E38F);
                Debug.Assert(pack.pending == (ushort)(ushort)17471);
                Debug.Assert(pack.lon == (int) -866726993);
                Debug.Assert(pack.spacing == (ushort)(ushort)32801);
                Debug.Assert(pack.terrain_height == (float) -2.4136903E38F);
            };
            DemoDevice.TERRAIN_REPORT p136 = LoopBackDemoChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.current_height = (float)1.0640828E38F;
            p136.spacing = (ushort)(ushort)32801;
            p136.pending = (ushort)(ushort)17471;
            p136.terrain_height = (float) -2.4136903E38F;
            p136.lat = (int) -1643308010;
            p136.lon = (int) -866726993;
            p136.loaded = (ushort)(ushort)27302;
            LoopBackDemoChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -30462);
                Debug.Assert(pack.press_diff == (float) -4.729282E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3563099778U);
                Debug.Assert(pack.press_abs == (float)4.8068723E37F);
            };
            DemoDevice.SCALED_PRESSURE2 p137 = LoopBackDemoChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)3563099778U;
            p137.press_abs = (float)4.8068723E37F;
            p137.press_diff = (float) -4.729282E37F;
            p137.temperature = (short)(short) -30462;
            LoopBackDemoChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.9256853E38F, -3.138602E38F, -1.02993E38F, -3.1250983E38F}));
                Debug.Assert(pack.x == (float)1.501665E37F);
                Debug.Assert(pack.y == (float)1.4226229E38F);
                Debug.Assert(pack.z == (float)9.238113E37F);
                Debug.Assert(pack.time_usec == (ulong)9051409550754846984L);
            };
            DemoDevice.ATT_POS_MOCAP p138 = LoopBackDemoChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.y = (float)1.4226229E38F;
            p138.z = (float)9.238113E37F;
            p138.time_usec = (ulong)9051409550754846984L;
            p138.x = (float)1.501665E37F;
            p138.q_SET(new float[] {2.9256853E38F, -3.138602E38F, -1.02993E38F, -3.1250983E38F}, 0) ;
            LoopBackDemoChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)4623954402874332570L);
                Debug.Assert(pack.target_system == (byte)(byte)250);
                Debug.Assert(pack.group_mlx == (byte)(byte)10);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-1.9306619E38F, 2.3393703E38F, 1.6469399E38F, 1.1197413E38F, -1.4576418E38F, -2.6940117E38F, -1.9552028E38F, 2.3601042E38F}));
                Debug.Assert(pack.target_component == (byte)(byte)175);
            };
            DemoDevice.SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)4623954402874332570L;
            p139.target_component = (byte)(byte)175;
            p139.target_system = (byte)(byte)250;
            p139.group_mlx = (byte)(byte)10;
            p139.controls_SET(new float[] {-1.9306619E38F, 2.3393703E38F, 1.6469399E38F, 1.1197413E38F, -1.4576418E38F, -2.6940117E38F, -1.9552028E38F, 2.3601042E38F}, 0) ;
            LoopBackDemoChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.group_mlx == (byte)(byte)137);
                Debug.Assert(pack.time_usec == (ulong)8225627752563700702L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {1.1502963E38F, -3.1143747E38F, 2.0401048E38F, 2.2490042E38F, -3.010513E38F, 1.8691699E38F, 1.1022844E37F, -1.6937158E38F}));
            };
            DemoDevice.ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.controls_SET(new float[] {1.1502963E38F, -3.1143747E38F, 2.0401048E38F, 2.2490042E38F, -3.010513E38F, 1.8691699E38F, 1.1022844E37F, -1.6937158E38F}, 0) ;
            p140.group_mlx = (byte)(byte)137;
            p140.time_usec = (ulong)8225627752563700702L;
            LoopBackDemoChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_monotonic == (float) -1.2338595E38F);
                Debug.Assert(pack.altitude_amsl == (float) -2.4854032E38F);
                Debug.Assert(pack.bottom_clearance == (float)3.2258605E38F);
                Debug.Assert(pack.altitude_relative == (float)2.1018055E38F);
                Debug.Assert(pack.time_usec == (ulong)2337254406327810090L);
                Debug.Assert(pack.altitude_terrain == (float)4.0895701E37F);
                Debug.Assert(pack.altitude_local == (float) -5.89763E36F);
            };
            DemoDevice.ALTITUDE p141 = LoopBackDemoChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.altitude_monotonic = (float) -1.2338595E38F;
            p141.bottom_clearance = (float)3.2258605E38F;
            p141.time_usec = (ulong)2337254406327810090L;
            p141.altitude_amsl = (float) -2.4854032E38F;
            p141.altitude_local = (float) -5.89763E36F;
            p141.altitude_relative = (float)2.1018055E38F;
            p141.altitude_terrain = (float)4.0895701E37F;
            LoopBackDemoChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.request_id == (byte)(byte)17);
                Debug.Assert(pack.transfer_type == (byte)(byte)238);
                Debug.Assert(pack.uri_type == (byte)(byte)176);
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)169, (byte)189, (byte)24, (byte)176, (byte)112, (byte)118, (byte)46, (byte)128, (byte)138, (byte)39, (byte)35, (byte)106, (byte)167, (byte)108, (byte)208, (byte)75, (byte)245, (byte)61, (byte)81, (byte)157, (byte)15, (byte)116, (byte)50, (byte)146, (byte)73, (byte)157, (byte)186, (byte)219, (byte)116, (byte)21, (byte)107, (byte)120, (byte)128, (byte)217, (byte)159, (byte)96, (byte)203, (byte)145, (byte)118, (byte)147, (byte)198, (byte)11, (byte)69, (byte)149, (byte)115, (byte)21, (byte)239, (byte)83, (byte)19, (byte)151, (byte)255, (byte)63, (byte)78, (byte)2, (byte)37, (byte)99, (byte)38, (byte)139, (byte)197, (byte)140, (byte)208, (byte)1, (byte)19, (byte)119, (byte)2, (byte)7, (byte)246, (byte)31, (byte)235, (byte)162, (byte)88, (byte)57, (byte)24, (byte)20, (byte)241, (byte)138, (byte)70, (byte)59, (byte)69, (byte)194, (byte)113, (byte)239, (byte)3, (byte)118, (byte)121, (byte)127, (byte)244, (byte)138, (byte)149, (byte)48, (byte)151, (byte)47, (byte)95, (byte)189, (byte)42, (byte)70, (byte)136, (byte)9, (byte)109, (byte)118, (byte)207, (byte)112, (byte)19, (byte)182, (byte)207, (byte)102, (byte)27, (byte)31, (byte)16, (byte)141, (byte)219, (byte)209, (byte)25, (byte)191, (byte)87, (byte)160, (byte)29, (byte)81, (byte)35, (byte)211}));
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)177, (byte)139, (byte)164, (byte)172, (byte)80, (byte)24, (byte)212, (byte)132, (byte)61, (byte)184, (byte)50, (byte)165, (byte)196, (byte)123, (byte)16, (byte)74, (byte)98, (byte)248, (byte)143, (byte)221, (byte)255, (byte)171, (byte)240, (byte)218, (byte)191, (byte)46, (byte)207, (byte)212, (byte)7, (byte)65, (byte)30, (byte)176, (byte)196, (byte)74, (byte)15, (byte)181, (byte)254, (byte)191, (byte)38, (byte)251, (byte)167, (byte)153, (byte)169, (byte)102, (byte)133, (byte)155, (byte)150, (byte)147, (byte)246, (byte)135, (byte)225, (byte)122, (byte)236, (byte)103, (byte)111, (byte)97, (byte)7, (byte)81, (byte)205, (byte)156, (byte)23, (byte)98, (byte)25, (byte)130, (byte)203, (byte)129, (byte)30, (byte)51, (byte)59, (byte)230, (byte)59, (byte)133, (byte)231, (byte)167, (byte)25, (byte)10, (byte)204, (byte)115, (byte)184, (byte)180, (byte)53, (byte)18, (byte)12, (byte)139, (byte)121, (byte)170, (byte)111, (byte)22, (byte)64, (byte)115, (byte)108, (byte)60, (byte)78, (byte)13, (byte)29, (byte)70, (byte)122, (byte)97, (byte)232, (byte)95, (byte)145, (byte)25, (byte)250, (byte)250, (byte)15, (byte)95, (byte)198, (byte)162, (byte)32, (byte)108, (byte)185, (byte)211, (byte)214, (byte)199, (byte)203, (byte)137, (byte)107, (byte)100, (byte)145, (byte)36}));
            };
            DemoDevice.RESOURCE_REQUEST p142 = LoopBackDemoChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)17;
            p142.uri_SET(new byte[] {(byte)169, (byte)189, (byte)24, (byte)176, (byte)112, (byte)118, (byte)46, (byte)128, (byte)138, (byte)39, (byte)35, (byte)106, (byte)167, (byte)108, (byte)208, (byte)75, (byte)245, (byte)61, (byte)81, (byte)157, (byte)15, (byte)116, (byte)50, (byte)146, (byte)73, (byte)157, (byte)186, (byte)219, (byte)116, (byte)21, (byte)107, (byte)120, (byte)128, (byte)217, (byte)159, (byte)96, (byte)203, (byte)145, (byte)118, (byte)147, (byte)198, (byte)11, (byte)69, (byte)149, (byte)115, (byte)21, (byte)239, (byte)83, (byte)19, (byte)151, (byte)255, (byte)63, (byte)78, (byte)2, (byte)37, (byte)99, (byte)38, (byte)139, (byte)197, (byte)140, (byte)208, (byte)1, (byte)19, (byte)119, (byte)2, (byte)7, (byte)246, (byte)31, (byte)235, (byte)162, (byte)88, (byte)57, (byte)24, (byte)20, (byte)241, (byte)138, (byte)70, (byte)59, (byte)69, (byte)194, (byte)113, (byte)239, (byte)3, (byte)118, (byte)121, (byte)127, (byte)244, (byte)138, (byte)149, (byte)48, (byte)151, (byte)47, (byte)95, (byte)189, (byte)42, (byte)70, (byte)136, (byte)9, (byte)109, (byte)118, (byte)207, (byte)112, (byte)19, (byte)182, (byte)207, (byte)102, (byte)27, (byte)31, (byte)16, (byte)141, (byte)219, (byte)209, (byte)25, (byte)191, (byte)87, (byte)160, (byte)29, (byte)81, (byte)35, (byte)211}, 0) ;
            p142.storage_SET(new byte[] {(byte)177, (byte)139, (byte)164, (byte)172, (byte)80, (byte)24, (byte)212, (byte)132, (byte)61, (byte)184, (byte)50, (byte)165, (byte)196, (byte)123, (byte)16, (byte)74, (byte)98, (byte)248, (byte)143, (byte)221, (byte)255, (byte)171, (byte)240, (byte)218, (byte)191, (byte)46, (byte)207, (byte)212, (byte)7, (byte)65, (byte)30, (byte)176, (byte)196, (byte)74, (byte)15, (byte)181, (byte)254, (byte)191, (byte)38, (byte)251, (byte)167, (byte)153, (byte)169, (byte)102, (byte)133, (byte)155, (byte)150, (byte)147, (byte)246, (byte)135, (byte)225, (byte)122, (byte)236, (byte)103, (byte)111, (byte)97, (byte)7, (byte)81, (byte)205, (byte)156, (byte)23, (byte)98, (byte)25, (byte)130, (byte)203, (byte)129, (byte)30, (byte)51, (byte)59, (byte)230, (byte)59, (byte)133, (byte)231, (byte)167, (byte)25, (byte)10, (byte)204, (byte)115, (byte)184, (byte)180, (byte)53, (byte)18, (byte)12, (byte)139, (byte)121, (byte)170, (byte)111, (byte)22, (byte)64, (byte)115, (byte)108, (byte)60, (byte)78, (byte)13, (byte)29, (byte)70, (byte)122, (byte)97, (byte)232, (byte)95, (byte)145, (byte)25, (byte)250, (byte)250, (byte)15, (byte)95, (byte)198, (byte)162, (byte)32, (byte)108, (byte)185, (byte)211, (byte)214, (byte)199, (byte)203, (byte)137, (byte)107, (byte)100, (byte)145, (byte)36}, 0) ;
            p142.transfer_type = (byte)(byte)238;
            p142.uri_type = (byte)(byte)176;
            LoopBackDemoChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff == (float)2.270287E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3316404984U);
                Debug.Assert(pack.press_abs == (float)4.2540463E37F);
                Debug.Assert(pack.temperature == (short)(short)5009);
            };
            DemoDevice.SCALED_PRESSURE3 p143 = LoopBackDemoChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.press_diff = (float)2.270287E38F;
            p143.time_boot_ms = (uint)3316404984U;
            p143.temperature = (short)(short)5009;
            p143.press_abs = (float)4.2540463E37F;
            LoopBackDemoChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.custom_state == (ulong)7844469677194671467L);
                Debug.Assert(pack.acc.SequenceEqual(new float[] {-1.6205979E38F, 2.8039198E38F, -2.9174587E38F}));
                Debug.Assert(pack.vel.SequenceEqual(new float[] {-7.0405346E36F, 1.1536528E38F, 3.3595968E38F}));
                Debug.Assert(pack.lat == (int) -584535134);
                Debug.Assert(pack.timestamp == (ulong)138819414235621534L);
                Debug.Assert(pack.rates.SequenceEqual(new float[] {-5.2276207E37F, 1.2162838E38F, 1.4867753E38F}));
                Debug.Assert(pack.est_capabilities == (byte)(byte)144);
                Debug.Assert(pack.alt == (float)1.5623131E38F);
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {-5.068084E37F, -1.4703176E38F, 3.296988E38F, 1.3646017E38F}));
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {-3.2493544E38F, -1.780974E38F, -2.6297136E38F}));
                Debug.Assert(pack.lon == (int) -96689352);
            };
            DemoDevice.FOLLOW_TARGET p144 = LoopBackDemoChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.acc_SET(new float[] {-1.6205979E38F, 2.8039198E38F, -2.9174587E38F}, 0) ;
            p144.lat = (int) -584535134;
            p144.rates_SET(new float[] {-5.2276207E37F, 1.2162838E38F, 1.4867753E38F}, 0) ;
            p144.timestamp = (ulong)138819414235621534L;
            p144.lon = (int) -96689352;
            p144.position_cov_SET(new float[] {-3.2493544E38F, -1.780974E38F, -2.6297136E38F}, 0) ;
            p144.vel_SET(new float[] {-7.0405346E36F, 1.1536528E38F, 3.3595968E38F}, 0) ;
            p144.alt = (float)1.5623131E38F;
            p144.est_capabilities = (byte)(byte)144;
            p144.custom_state = (ulong)7844469677194671467L;
            p144.attitude_q_SET(new float[] {-5.068084E37F, -1.4703176E38F, 3.296988E38F, 1.3646017E38F}, 0) ;
            LoopBackDemoChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {1.2785122E38F, 4.8624907E37F, -5.0409546E37F}));
                Debug.Assert(pack.y_acc == (float) -1.7467765E38F);
                Debug.Assert(pack.roll_rate == (float)6.857845E37F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {-3.034671E38F, 2.022101E38F, 1.415118E38F}));
                Debug.Assert(pack.airspeed == (float)2.5154108E38F);
                Debug.Assert(pack.time_usec == (ulong)4595853122627737403L);
                Debug.Assert(pack.yaw_rate == (float) -1.3444991E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-9.134121E37F, -8.646344E37F, 2.3747375E38F, -1.1978223E38F}));
                Debug.Assert(pack.z_vel == (float)2.0621422E38F);
                Debug.Assert(pack.x_acc == (float)1.237939E37F);
                Debug.Assert(pack.pitch_rate == (float) -3.0187935E38F);
                Debug.Assert(pack.y_pos == (float)2.1732473E37F);
                Debug.Assert(pack.y_vel == (float)1.819762E38F);
                Debug.Assert(pack.z_pos == (float) -2.1154571E37F);
                Debug.Assert(pack.x_pos == (float)8.696097E37F);
                Debug.Assert(pack.x_vel == (float)2.719891E38F);
                Debug.Assert(pack.z_acc == (float) -1.4586374E38F);
            };
            DemoDevice.CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.airspeed = (float)2.5154108E38F;
            p146.y_acc = (float) -1.7467765E38F;
            p146.z_vel = (float)2.0621422E38F;
            p146.roll_rate = (float)6.857845E37F;
            p146.x_vel = (float)2.719891E38F;
            p146.yaw_rate = (float) -1.3444991E38F;
            p146.vel_variance_SET(new float[] {-3.034671E38F, 2.022101E38F, 1.415118E38F}, 0) ;
            p146.pitch_rate = (float) -3.0187935E38F;
            p146.x_pos = (float)8.696097E37F;
            p146.y_vel = (float)1.819762E38F;
            p146.z_acc = (float) -1.4586374E38F;
            p146.x_acc = (float)1.237939E37F;
            p146.q_SET(new float[] {-9.134121E37F, -8.646344E37F, 2.3747375E38F, -1.1978223E38F}, 0) ;
            p146.pos_variance_SET(new float[] {1.2785122E38F, 4.8624907E37F, -5.0409546E37F}, 0) ;
            p146.y_pos = (float)2.1732473E37F;
            p146.z_pos = (float) -2.1154571E37F;
            p146.time_usec = (ulong)4595853122627737403L;
            LoopBackDemoChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.energy_consumed == (int)1500807317);
                Debug.Assert(pack.battery_function == (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)4);
                Debug.Assert(pack.current_consumed == (int) -720464036);
                Debug.Assert(pack.temperature == (short)(short) -25169);
                Debug.Assert(pack.current_battery == (short)(short)32607);
                Debug.Assert(pack.type == (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH);
                Debug.Assert(pack.id == (byte)(byte)159);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)24101, (ushort)10413, (ushort)32396, (ushort)37083, (ushort)1516, (ushort)13295, (ushort)33946, (ushort)31887, (ushort)60316, (ushort)53378}));
            };
            DemoDevice.BATTERY_STATUS p147 = LoopBackDemoChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH;
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN;
            p147.temperature = (short)(short) -25169;
            p147.battery_remaining = (sbyte)(sbyte)4;
            p147.id = (byte)(byte)159;
            p147.voltages_SET(new ushort[] {(ushort)24101, (ushort)10413, (ushort)32396, (ushort)37083, (ushort)1516, (ushort)13295, (ushort)33946, (ushort)31887, (ushort)60316, (ushort)53378}, 0) ;
            p147.energy_consumed = (int)1500807317;
            p147.current_consumed = (int) -720464036;
            p147.current_battery = (short)(short)32607;
            LoopBackDemoChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT);
                Debug.Assert(pack.flight_sw_version == (uint)905158066U);
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)203, (byte)54, (byte)77, (byte)86, (byte)98, (byte)234, (byte)104, (byte)39}));
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)37, (byte)149, (byte)180, (byte)158, (byte)42, (byte)65, (byte)56, (byte)39}));
                Debug.Assert(pack.uid == (ulong)651929659549129387L);
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)197, (byte)127, (byte)40, (byte)138, (byte)125, (byte)88, (byte)145, (byte)185}));
                Debug.Assert(pack.product_id == (ushort)(ushort)907);
                Debug.Assert(pack.os_sw_version == (uint)2062287537U);
                Debug.Assert(pack.middleware_sw_version == (uint)3407629863U);
                Debug.Assert(pack.vendor_id == (ushort)(ushort)2745);
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)85, (byte)76, (byte)21, (byte)160, (byte)200, (byte)80, (byte)105, (byte)32, (byte)170, (byte)101, (byte)213, (byte)91, (byte)228, (byte)150, (byte)149, (byte)168, (byte)92, (byte)156}));
                Debug.Assert(pack.board_version == (uint)1001536646U);
            };
            DemoDevice.AUTOPILOT_VERSION p148 = LoopBackDemoChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.flight_custom_version_SET(new byte[] {(byte)197, (byte)127, (byte)40, (byte)138, (byte)125, (byte)88, (byte)145, (byte)185}, 0) ;
            p148.middleware_sw_version = (uint)3407629863U;
            p148.product_id = (ushort)(ushort)907;
            p148.uid = (ulong)651929659549129387L;
            p148.uid2_SET(new byte[] {(byte)85, (byte)76, (byte)21, (byte)160, (byte)200, (byte)80, (byte)105, (byte)32, (byte)170, (byte)101, (byte)213, (byte)91, (byte)228, (byte)150, (byte)149, (byte)168, (byte)92, (byte)156}, 0, PH) ;
            p148.os_custom_version_SET(new byte[] {(byte)37, (byte)149, (byte)180, (byte)158, (byte)42, (byte)65, (byte)56, (byte)39}, 0) ;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT;
            p148.vendor_id = (ushort)(ushort)2745;
            p148.board_version = (uint)1001536646U;
            p148.middleware_custom_version_SET(new byte[] {(byte)203, (byte)54, (byte)77, (byte)86, (byte)98, (byte)234, (byte)104, (byte)39}, 0) ;
            p148.os_sw_version = (uint)2062287537U;
            p148.flight_sw_version = (uint)905158066U;
            LoopBackDemoChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y_TRY(ph) == (float)1.5964896E38F);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)2);
                Debug.Assert(pack.time_usec == (ulong)3245278604440132025L);
                Debug.Assert(pack.target_num == (byte)(byte)34);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.size_y == (float)2.669162E38F);
                Debug.Assert(pack.type == (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
                Debug.Assert(pack.z_TRY(ph) == (float) -3.1134704E38F);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {3.4056513E37F, 8.878777E37F, -1.8200092E38F, -1.3912868E38F}));
                Debug.Assert(pack.angle_y == (float)1.5322791E38F);
                Debug.Assert(pack.x_TRY(ph) == (float)2.3324765E38F);
                Debug.Assert(pack.size_x == (float)1.669618E37F);
                Debug.Assert(pack.angle_x == (float) -2.1706206E38F);
                Debug.Assert(pack.distance == (float) -7.7713127E37F);
            };
            DemoDevice.LANDING_TARGET p149 = LoopBackDemoChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.distance = (float) -7.7713127E37F;
            p149.x_SET((float)2.3324765E38F, PH) ;
            p149.angle_x = (float) -2.1706206E38F;
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON;
            p149.time_usec = (ulong)3245278604440132025L;
            p149.q_SET(new float[] {3.4056513E37F, 8.878777E37F, -1.8200092E38F, -1.3912868E38F}, 0, PH) ;
            p149.position_valid_SET((byte)(byte)2, PH) ;
            p149.size_x = (float)1.669618E37F;
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p149.target_num = (byte)(byte)34;
            p149.z_SET((float) -3.1134704E38F, PH) ;
            p149.size_y = (float)2.669162E38F;
            p149.y_SET((float)1.5964896E38F, PH) ;
            p149.angle_y = (float)1.5322791E38F;
            LoopBackDemoChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pos_vert_accuracy == (float)5.8588037E37F);
                Debug.Assert(pack.pos_vert_ratio == (float) -3.3231245E36F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE);
                Debug.Assert(pack.mag_ratio == (float) -3.078175E38F);
                Debug.Assert(pack.pos_horiz_accuracy == (float) -2.4415027E37F);
                Debug.Assert(pack.pos_horiz_ratio == (float)2.2205484E38F);
                Debug.Assert(pack.hagl_ratio == (float)1.0582154E38F);
                Debug.Assert(pack.vel_ratio == (float)3.1284392E38F);
                Debug.Assert(pack.tas_ratio == (float)3.1828114E38F);
                Debug.Assert(pack.time_usec == (ulong)7443239173319366570L);
            };
            DemoDevice.ESTIMATOR_STATUS p230 = LoopBackDemoChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.pos_vert_ratio = (float) -3.3231245E36F;
            p230.pos_horiz_ratio = (float)2.2205484E38F;
            p230.pos_vert_accuracy = (float)5.8588037E37F;
            p230.tas_ratio = (float)3.1828114E38F;
            p230.vel_ratio = (float)3.1284392E38F;
            p230.mag_ratio = (float) -3.078175E38F;
            p230.time_usec = (ulong)7443239173319366570L;
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE;
            p230.pos_horiz_accuracy = (float) -2.4415027E37F;
            p230.hagl_ratio = (float)1.0582154E38F;
            LoopBackDemoChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.horiz_accuracy == (float) -2.4192897E38F);
                Debug.Assert(pack.wind_x == (float) -1.7877386E38F);
                Debug.Assert(pack.var_vert == (float)1.6451168E37F);
                Debug.Assert(pack.time_usec == (ulong)3598102135730871475L);
                Debug.Assert(pack.wind_y == (float)1.2671821E38F);
                Debug.Assert(pack.wind_alt == (float)1.7320117E38F);
                Debug.Assert(pack.wind_z == (float) -1.4492218E38F);
                Debug.Assert(pack.vert_accuracy == (float)1.0020344E38F);
                Debug.Assert(pack.var_horiz == (float)3.2926344E38F);
            };
            DemoDevice.WIND_COV p231 = LoopBackDemoChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.var_horiz = (float)3.2926344E38F;
            p231.wind_alt = (float)1.7320117E38F;
            p231.vert_accuracy = (float)1.0020344E38F;
            p231.wind_z = (float) -1.4492218E38F;
            p231.time_usec = (ulong)3598102135730871475L;
            p231.horiz_accuracy = (float) -2.4192897E38F;
            p231.wind_y = (float)1.2671821E38F;
            p231.wind_x = (float) -1.7877386E38F;
            p231.var_vert = (float)1.6451168E37F;
            LoopBackDemoChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.speed_accuracy == (float)2.2867788E38F);
                Debug.Assert(pack.alt == (float) -2.8755333E38F);
                Debug.Assert(pack.vn == (float) -2.8059886E38F);
                Debug.Assert(pack.time_week_ms == (uint)4122427969U);
                Debug.Assert(pack.time_usec == (ulong)2351613734993714881L);
                Debug.Assert(pack.vd == (float)1.9115402E38F);
                Debug.Assert(pack.vert_accuracy == (float)9.086998E37F);
                Debug.Assert(pack.satellites_visible == (byte)(byte)241);
                Debug.Assert(pack.horiz_accuracy == (float) -8.3180753E37F);
                Debug.Assert(pack.time_week == (ushort)(ushort)61511);
                Debug.Assert(pack.ve == (float) -4.14356E37F);
                Debug.Assert(pack.gps_id == (byte)(byte)204);
                Debug.Assert(pack.lon == (int) -239645634);
                Debug.Assert(pack.hdop == (float)5.112951E37F);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP);
                Debug.Assert(pack.lat == (int)1035408088);
                Debug.Assert(pack.fix_type == (byte)(byte)18);
                Debug.Assert(pack.vdop == (float)9.228142E37F);
            };
            DemoDevice.GPS_INPUT p232 = LoopBackDemoChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.hdop = (float)5.112951E37F;
            p232.time_week = (ushort)(ushort)61511;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP;
            p232.lon = (int) -239645634;
            p232.vn = (float) -2.8059886E38F;
            p232.vdop = (float)9.228142E37F;
            p232.horiz_accuracy = (float) -8.3180753E37F;
            p232.gps_id = (byte)(byte)204;
            p232.alt = (float) -2.8755333E38F;
            p232.lat = (int)1035408088;
            p232.ve = (float) -4.14356E37F;
            p232.time_usec = (ulong)2351613734993714881L;
            p232.vert_accuracy = (float)9.086998E37F;
            p232.satellites_visible = (byte)(byte)241;
            p232.time_week_ms = (uint)4122427969U;
            p232.vd = (float)1.9115402E38F;
            p232.fix_type = (byte)(byte)18;
            p232.speed_accuracy = (float)2.2867788E38F;
            LoopBackDemoChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.len == (byte)(byte)109);
                Debug.Assert(pack.flags == (byte)(byte)111);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)158, (byte)181, (byte)0, (byte)254, (byte)245, (byte)166, (byte)45, (byte)206, (byte)234, (byte)19, (byte)155, (byte)159, (byte)2, (byte)52, (byte)106, (byte)10, (byte)72, (byte)193, (byte)125, (byte)68, (byte)169, (byte)197, (byte)134, (byte)85, (byte)171, (byte)82, (byte)143, (byte)184, (byte)39, (byte)65, (byte)218, (byte)75, (byte)127, (byte)13, (byte)85, (byte)112, (byte)39, (byte)72, (byte)59, (byte)128, (byte)97, (byte)183, (byte)200, (byte)54, (byte)195, (byte)183, (byte)206, (byte)119, (byte)64, (byte)181, (byte)172, (byte)9, (byte)209, (byte)72, (byte)228, (byte)177, (byte)129, (byte)37, (byte)143, (byte)183, (byte)188, (byte)157, (byte)149, (byte)235, (byte)12, (byte)233, (byte)47, (byte)111, (byte)194, (byte)107, (byte)207, (byte)250, (byte)26, (byte)252, (byte)144, (byte)52, (byte)46, (byte)32, (byte)65, (byte)30, (byte)224, (byte)25, (byte)102, (byte)132, (byte)93, (byte)137, (byte)232, (byte)25, (byte)78, (byte)58, (byte)195, (byte)248, (byte)111, (byte)129, (byte)34, (byte)173, (byte)126, (byte)134, (byte)224, (byte)89, (byte)217, (byte)239, (byte)82, (byte)184, (byte)172, (byte)231, (byte)44, (byte)158, (byte)180, (byte)48, (byte)14, (byte)168, (byte)251, (byte)5, (byte)232, (byte)217, (byte)47, (byte)245, (byte)166, (byte)115, (byte)127, (byte)209, (byte)82, (byte)224, (byte)51, (byte)236, (byte)20, (byte)44, (byte)67, (byte)156, (byte)20, (byte)135, (byte)209, (byte)136, (byte)15, (byte)207, (byte)9, (byte)72, (byte)107, (byte)176, (byte)116, (byte)28, (byte)162, (byte)196, (byte)194, (byte)10, (byte)185, (byte)140, (byte)141, (byte)228, (byte)178, (byte)84, (byte)57, (byte)218, (byte)219, (byte)62, (byte)88, (byte)47, (byte)49, (byte)219, (byte)0, (byte)130, (byte)8, (byte)46, (byte)195, (byte)205, (byte)1, (byte)54, (byte)122, (byte)197, (byte)42, (byte)2, (byte)254, (byte)5, (byte)148, (byte)83, (byte)52, (byte)165, (byte)113, (byte)178}));
            };
            DemoDevice.GPS_RTCM_DATA p233 = LoopBackDemoChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.data__SET(new byte[] {(byte)158, (byte)181, (byte)0, (byte)254, (byte)245, (byte)166, (byte)45, (byte)206, (byte)234, (byte)19, (byte)155, (byte)159, (byte)2, (byte)52, (byte)106, (byte)10, (byte)72, (byte)193, (byte)125, (byte)68, (byte)169, (byte)197, (byte)134, (byte)85, (byte)171, (byte)82, (byte)143, (byte)184, (byte)39, (byte)65, (byte)218, (byte)75, (byte)127, (byte)13, (byte)85, (byte)112, (byte)39, (byte)72, (byte)59, (byte)128, (byte)97, (byte)183, (byte)200, (byte)54, (byte)195, (byte)183, (byte)206, (byte)119, (byte)64, (byte)181, (byte)172, (byte)9, (byte)209, (byte)72, (byte)228, (byte)177, (byte)129, (byte)37, (byte)143, (byte)183, (byte)188, (byte)157, (byte)149, (byte)235, (byte)12, (byte)233, (byte)47, (byte)111, (byte)194, (byte)107, (byte)207, (byte)250, (byte)26, (byte)252, (byte)144, (byte)52, (byte)46, (byte)32, (byte)65, (byte)30, (byte)224, (byte)25, (byte)102, (byte)132, (byte)93, (byte)137, (byte)232, (byte)25, (byte)78, (byte)58, (byte)195, (byte)248, (byte)111, (byte)129, (byte)34, (byte)173, (byte)126, (byte)134, (byte)224, (byte)89, (byte)217, (byte)239, (byte)82, (byte)184, (byte)172, (byte)231, (byte)44, (byte)158, (byte)180, (byte)48, (byte)14, (byte)168, (byte)251, (byte)5, (byte)232, (byte)217, (byte)47, (byte)245, (byte)166, (byte)115, (byte)127, (byte)209, (byte)82, (byte)224, (byte)51, (byte)236, (byte)20, (byte)44, (byte)67, (byte)156, (byte)20, (byte)135, (byte)209, (byte)136, (byte)15, (byte)207, (byte)9, (byte)72, (byte)107, (byte)176, (byte)116, (byte)28, (byte)162, (byte)196, (byte)194, (byte)10, (byte)185, (byte)140, (byte)141, (byte)228, (byte)178, (byte)84, (byte)57, (byte)218, (byte)219, (byte)62, (byte)88, (byte)47, (byte)49, (byte)219, (byte)0, (byte)130, (byte)8, (byte)46, (byte)195, (byte)205, (byte)1, (byte)54, (byte)122, (byte)197, (byte)42, (byte)2, (byte)254, (byte)5, (byte)148, (byte)83, (byte)52, (byte)165, (byte)113, (byte)178}, 0) ;
            p233.flags = (byte)(byte)111;
            p233.len = (byte)(byte)109;
            LoopBackDemoChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);
                Debug.Assert(pack.heading == (ushort)(ushort)39032);
                Debug.Assert(pack.throttle == (sbyte)(sbyte) - 121);
                Debug.Assert(pack.heading_sp == (short)(short)19989);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)28424);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte)59);
                Debug.Assert(pack.custom_mode == (uint)3897142148U);
                Debug.Assert(pack.battery_remaining == (byte)(byte)252);
                Debug.Assert(pack.altitude_amsl == (short)(short) -9538);
                Debug.Assert(pack.gps_nsat == (byte)(byte)154);
                Debug.Assert(pack.airspeed == (byte)(byte)231);
                Debug.Assert(pack.failsafe == (byte)(byte)61);
                Debug.Assert(pack.pitch == (short)(short)20644);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)45);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte) - 6);
                Debug.Assert(pack.latitude == (int) -938424409);
                Debug.Assert(pack.longitude == (int)1005590279);
                Debug.Assert(pack.wp_num == (byte)(byte)221);
                Debug.Assert(pack.temperature == (sbyte)(sbyte)17);
                Debug.Assert(pack.altitude_sp == (short)(short)24375);
                Debug.Assert(pack.roll == (short)(short)10113);
                Debug.Assert(pack.groundspeed == (byte)(byte)55);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
                Debug.Assert(pack.gps_fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED);
            };
            DemoDevice.HIGH_LATENCY p234 = LoopBackDemoChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.longitude = (int)1005590279;
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
            p234.temperature_air = (sbyte)(sbyte)59;
            p234.heading = (ushort)(ushort)39032;
            p234.temperature = (sbyte)(sbyte)17;
            p234.throttle = (sbyte)(sbyte) - 121;
            p234.pitch = (short)(short)20644;
            p234.battery_remaining = (byte)(byte)252;
            p234.altitude_amsl = (short)(short) -9538;
            p234.altitude_sp = (short)(short)24375;
            p234.custom_mode = (uint)3897142148U;
            p234.heading_sp = (short)(short)19989;
            p234.climb_rate = (sbyte)(sbyte) - 6;
            p234.latitude = (int) -938424409;
            p234.roll = (short)(short)10113;
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED;
            p234.gps_nsat = (byte)(byte)154;
            p234.wp_distance = (ushort)(ushort)28424;
            p234.failsafe = (byte)(byte)61;
            p234.airspeed = (byte)(byte)231;
            p234.groundspeed = (byte)(byte)55;
            p234.airspeed_sp = (byte)(byte)45;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF;
            p234.wp_num = (byte)(byte)221;
            LoopBackDemoChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vibration_x == (float)4.1788591E37F);
                Debug.Assert(pack.vibration_z == (float)5.892588E37F);
                Debug.Assert(pack.time_usec == (ulong)36081073985630734L);
                Debug.Assert(pack.clipping_1 == (uint)4235191980U);
                Debug.Assert(pack.clipping_0 == (uint)1808692939U);
                Debug.Assert(pack.vibration_y == (float) -1.8021398E37F);
                Debug.Assert(pack.clipping_2 == (uint)2144187967U);
            };
            DemoDevice.VIBRATION p241 = LoopBackDemoChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)36081073985630734L;
            p241.clipping_1 = (uint)4235191980U;
            p241.clipping_2 = (uint)2144187967U;
            p241.vibration_y = (float) -1.8021398E37F;
            p241.vibration_z = (float)5.892588E37F;
            p241.clipping_0 = (uint)1808692939U;
            p241.vibration_x = (float)4.1788591E37F;
            LoopBackDemoChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int) -232667333);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)1133667616257573944L);
                Debug.Assert(pack.approach_y == (float) -1.2005051E38F);
                Debug.Assert(pack.approach_z == (float)1.1917521E38F);
                Debug.Assert(pack.latitude == (int)101084688);
                Debug.Assert(pack.approach_x == (float) -2.9881036E38F);
                Debug.Assert(pack.longitude == (int) -188651642);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.8525209E37F, 2.4619381E38F, 2.1139516E38F, -3.2881285E38F}));
                Debug.Assert(pack.y == (float) -2.5169811E38F);
                Debug.Assert(pack.z == (float)5.617584E37F);
                Debug.Assert(pack.x == (float) -3.1468743E38F);
            };
            DemoDevice.HOME_POSITION p242 = LoopBackDemoChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.longitude = (int) -188651642;
            p242.approach_z = (float)1.1917521E38F;
            p242.y = (float) -2.5169811E38F;
            p242.approach_y = (float) -1.2005051E38F;
            p242.approach_x = (float) -2.9881036E38F;
            p242.latitude = (int)101084688;
            p242.z = (float)5.617584E37F;
            p242.x = (float) -3.1468743E38F;
            p242.altitude = (int) -232667333;
            p242.time_usec_SET((ulong)1133667616257573944L, PH) ;
            p242.q_SET(new float[] {1.8525209E37F, 2.4619381E38F, 2.1139516E38F, -3.2881285E38F}, 0) ;
            LoopBackDemoChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)7903221302552517988L);
                Debug.Assert(pack.approach_y == (float) -3.317314E38F);
                Debug.Assert(pack.y == (float)3.356883E38F);
                Debug.Assert(pack.x == (float) -7.6249326E37F);
                Debug.Assert(pack.target_system == (byte)(byte)139);
                Debug.Assert(pack.z == (float)1.0591451E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.9225998E38F, 5.349688E37F, 2.1029932E38F, 2.9037093E38F}));
                Debug.Assert(pack.approach_x == (float)2.0376159E38F);
                Debug.Assert(pack.altitude == (int)2052186306);
                Debug.Assert(pack.longitude == (int)1788171243);
                Debug.Assert(pack.latitude == (int)1166322606);
                Debug.Assert(pack.approach_z == (float) -1.0708223E38F);
            };
            DemoDevice.SET_HOME_POSITION p243 = LoopBackDemoChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.longitude = (int)1788171243;
            p243.y = (float)3.356883E38F;
            p243.time_usec_SET((ulong)7903221302552517988L, PH) ;
            p243.approach_x = (float)2.0376159E38F;
            p243.x = (float) -7.6249326E37F;
            p243.target_system = (byte)(byte)139;
            p243.approach_y = (float) -3.317314E38F;
            p243.z = (float)1.0591451E38F;
            p243.q_SET(new float[] {1.9225998E38F, 5.349688E37F, 2.1029932E38F, 2.9037093E38F}, 0) ;
            p243.approach_z = (float) -1.0708223E38F;
            p243.altitude = (int)2052186306;
            p243.latitude = (int)1166322606;
            LoopBackDemoChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.interval_us == (int)1926127154);
                Debug.Assert(pack.message_id == (ushort)(ushort)51991);
            };
            DemoDevice.MESSAGE_INTERVAL p244 = LoopBackDemoChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.interval_us = (int)1926127154;
            p244.message_id = (ushort)(ushort)51991;
            LoopBackDemoChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vtol_state == (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_MC);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
            };
            DemoDevice.EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND;
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_MC;
            LoopBackDemoChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.squawk == (ushort)(ushort)48286);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)20314);
                Debug.Assert(pack.lon == (int) -991996770);
                Debug.Assert(pack.emitter_type == (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UAV);
                Debug.Assert(pack.ver_velocity == (short)(short) -8502);
                Debug.Assert(pack.altitude == (int)584193600);
                Debug.Assert(pack.heading == (ushort)(ushort)29940);
                Debug.Assert(pack.callsign_LEN(ph) == 8);
                Debug.Assert(pack.callsign_TRY(ph).Equals("dsskbquc"));
                Debug.Assert(pack.ICAO_address == (uint)2996582544U);
                Debug.Assert(pack.tslc == (byte)(byte)133);
                Debug.Assert(pack.flags == (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING);
                Debug.Assert(pack.lat == (int) -365783543);
                Debug.Assert(pack.altitude_type == (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
            };
            DemoDevice.ADSB_VEHICLE p246 = LoopBackDemoChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.hor_velocity = (ushort)(ushort)20314;
            p246.ver_velocity = (short)(short) -8502;
            p246.callsign_SET("dsskbquc", PH) ;
            p246.lon = (int) -991996770;
            p246.lat = (int) -365783543;
            p246.squawk = (ushort)(ushort)48286;
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING;
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC;
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UAV;
            p246.heading = (ushort)(ushort)29940;
            p246.altitude = (int)584193600;
            p246.tslc = (byte)(byte)133;
            p246.ICAO_address = (uint)2996582544U;
            LoopBackDemoChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.action == (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR);
                Debug.Assert(pack.altitude_minimum_delta == (float) -2.4248363E38F);
                Debug.Assert(pack.src_ == (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
                Debug.Assert(pack.threat_level == (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
                Debug.Assert(pack.time_to_minimum_delta == (float)3.4009956E38F);
                Debug.Assert(pack.horizontal_minimum_delta == (float)2.0273917E38F);
                Debug.Assert(pack.id == (uint)3746402654U);
            };
            DemoDevice.COLLISION p247 = LoopBackDemoChannel.new_COLLISION();
            PH.setPack(p247);
            p247.id = (uint)3746402654U;
            p247.horizontal_minimum_delta = (float)2.0273917E38F;
            p247.time_to_minimum_delta = (float)3.4009956E38F;
            p247.altitude_minimum_delta = (float) -2.4248363E38F;
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE;
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            LoopBackDemoChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)7);
                Debug.Assert(pack.message_type == (ushort)(ushort)61623);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)181, (byte)112, (byte)33, (byte)50, (byte)165, (byte)203, (byte)180, (byte)37, (byte)146, (byte)137, (byte)39, (byte)136, (byte)214, (byte)16, (byte)178, (byte)155, (byte)42, (byte)46, (byte)65, (byte)27, (byte)151, (byte)169, (byte)72, (byte)168, (byte)178, (byte)20, (byte)196, (byte)174, (byte)174, (byte)41, (byte)24, (byte)38, (byte)75, (byte)232, (byte)84, (byte)66, (byte)128, (byte)207, (byte)92, (byte)187, (byte)160, (byte)212, (byte)18, (byte)219, (byte)29, (byte)200, (byte)207, (byte)103, (byte)36, (byte)224, (byte)143, (byte)167, (byte)230, (byte)77, (byte)98, (byte)131, (byte)118, (byte)46, (byte)1, (byte)30, (byte)200, (byte)242, (byte)4, (byte)77, (byte)102, (byte)85, (byte)31, (byte)129, (byte)86, (byte)117, (byte)108, (byte)150, (byte)127, (byte)196, (byte)66, (byte)5, (byte)45, (byte)252, (byte)171, (byte)65, (byte)101, (byte)8, (byte)167, (byte)215, (byte)114, (byte)225, (byte)155, (byte)175, (byte)48, (byte)221, (byte)7, (byte)71, (byte)113, (byte)187, (byte)106, (byte)1, (byte)166, (byte)109, (byte)148, (byte)186, (byte)124, (byte)78, (byte)244, (byte)86, (byte)253, (byte)145, (byte)26, (byte)49, (byte)187, (byte)166, (byte)18, (byte)73, (byte)166, (byte)135, (byte)107, (byte)66, (byte)53, (byte)82, (byte)149, (byte)193, (byte)0, (byte)56, (byte)57, (byte)105, (byte)146, (byte)43, (byte)182, (byte)167, (byte)232, (byte)248, (byte)69, (byte)79, (byte)111, (byte)87, (byte)55, (byte)174, (byte)210, (byte)80, (byte)91, (byte)34, (byte)224, (byte)142, (byte)55, (byte)192, (byte)119, (byte)13, (byte)113, (byte)54, (byte)188, (byte)24, (byte)254, (byte)42, (byte)141, (byte)46, (byte)20, (byte)200, (byte)218, (byte)81, (byte)77, (byte)170, (byte)156, (byte)214, (byte)172, (byte)57, (byte)177, (byte)220, (byte)48, (byte)121, (byte)109, (byte)86, (byte)194, (byte)219, (byte)134, (byte)243, (byte)77, (byte)192, (byte)230, (byte)219, (byte)231, (byte)124, (byte)142, (byte)185, (byte)71, (byte)8, (byte)243, (byte)90, (byte)105, (byte)168, (byte)35, (byte)5, (byte)159, (byte)42, (byte)224, (byte)178, (byte)237, (byte)15, (byte)39, (byte)222, (byte)230, (byte)92, (byte)140, (byte)67, (byte)182, (byte)89, (byte)221, (byte)175, (byte)17, (byte)55, (byte)111, (byte)213, (byte)64, (byte)103, (byte)21, (byte)181, (byte)58, (byte)190, (byte)53, (byte)138, (byte)162, (byte)240, (byte)193, (byte)111, (byte)124, (byte)106, (byte)150, (byte)155, (byte)35, (byte)182, (byte)64, (byte)243, (byte)180, (byte)168, (byte)72, (byte)97, (byte)249, (byte)58, (byte)145, (byte)85, (byte)143, (byte)246, (byte)241, (byte)177, (byte)230, (byte)15, (byte)94, (byte)88, (byte)26, (byte)163, (byte)157}));
                Debug.Assert(pack.target_system == (byte)(byte)102);
                Debug.Assert(pack.target_network == (byte)(byte)79);
            };
            DemoDevice.V2_EXTENSION p248 = LoopBackDemoChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.payload_SET(new byte[] {(byte)181, (byte)112, (byte)33, (byte)50, (byte)165, (byte)203, (byte)180, (byte)37, (byte)146, (byte)137, (byte)39, (byte)136, (byte)214, (byte)16, (byte)178, (byte)155, (byte)42, (byte)46, (byte)65, (byte)27, (byte)151, (byte)169, (byte)72, (byte)168, (byte)178, (byte)20, (byte)196, (byte)174, (byte)174, (byte)41, (byte)24, (byte)38, (byte)75, (byte)232, (byte)84, (byte)66, (byte)128, (byte)207, (byte)92, (byte)187, (byte)160, (byte)212, (byte)18, (byte)219, (byte)29, (byte)200, (byte)207, (byte)103, (byte)36, (byte)224, (byte)143, (byte)167, (byte)230, (byte)77, (byte)98, (byte)131, (byte)118, (byte)46, (byte)1, (byte)30, (byte)200, (byte)242, (byte)4, (byte)77, (byte)102, (byte)85, (byte)31, (byte)129, (byte)86, (byte)117, (byte)108, (byte)150, (byte)127, (byte)196, (byte)66, (byte)5, (byte)45, (byte)252, (byte)171, (byte)65, (byte)101, (byte)8, (byte)167, (byte)215, (byte)114, (byte)225, (byte)155, (byte)175, (byte)48, (byte)221, (byte)7, (byte)71, (byte)113, (byte)187, (byte)106, (byte)1, (byte)166, (byte)109, (byte)148, (byte)186, (byte)124, (byte)78, (byte)244, (byte)86, (byte)253, (byte)145, (byte)26, (byte)49, (byte)187, (byte)166, (byte)18, (byte)73, (byte)166, (byte)135, (byte)107, (byte)66, (byte)53, (byte)82, (byte)149, (byte)193, (byte)0, (byte)56, (byte)57, (byte)105, (byte)146, (byte)43, (byte)182, (byte)167, (byte)232, (byte)248, (byte)69, (byte)79, (byte)111, (byte)87, (byte)55, (byte)174, (byte)210, (byte)80, (byte)91, (byte)34, (byte)224, (byte)142, (byte)55, (byte)192, (byte)119, (byte)13, (byte)113, (byte)54, (byte)188, (byte)24, (byte)254, (byte)42, (byte)141, (byte)46, (byte)20, (byte)200, (byte)218, (byte)81, (byte)77, (byte)170, (byte)156, (byte)214, (byte)172, (byte)57, (byte)177, (byte)220, (byte)48, (byte)121, (byte)109, (byte)86, (byte)194, (byte)219, (byte)134, (byte)243, (byte)77, (byte)192, (byte)230, (byte)219, (byte)231, (byte)124, (byte)142, (byte)185, (byte)71, (byte)8, (byte)243, (byte)90, (byte)105, (byte)168, (byte)35, (byte)5, (byte)159, (byte)42, (byte)224, (byte)178, (byte)237, (byte)15, (byte)39, (byte)222, (byte)230, (byte)92, (byte)140, (byte)67, (byte)182, (byte)89, (byte)221, (byte)175, (byte)17, (byte)55, (byte)111, (byte)213, (byte)64, (byte)103, (byte)21, (byte)181, (byte)58, (byte)190, (byte)53, (byte)138, (byte)162, (byte)240, (byte)193, (byte)111, (byte)124, (byte)106, (byte)150, (byte)155, (byte)35, (byte)182, (byte)64, (byte)243, (byte)180, (byte)168, (byte)72, (byte)97, (byte)249, (byte)58, (byte)145, (byte)85, (byte)143, (byte)246, (byte)241, (byte)177, (byte)230, (byte)15, (byte)94, (byte)88, (byte)26, (byte)163, (byte)157}, 0) ;
            p248.target_component = (byte)(byte)7;
            p248.target_network = (byte)(byte)79;
            p248.target_system = (byte)(byte)102;
            p248.message_type = (ushort)(ushort)61623;
            LoopBackDemoChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte)50, (sbyte) - 3, (sbyte) - 106, (sbyte) - 108, (sbyte)9, (sbyte)75, (sbyte) - 40, (sbyte) - 116, (sbyte) - 12, (sbyte) - 112, (sbyte)19, (sbyte) - 16, (sbyte) - 122, (sbyte)17, (sbyte) - 80, (sbyte)16, (sbyte)41, (sbyte) - 84, (sbyte)22, (sbyte) - 115, (sbyte) - 102, (sbyte) - 76, (sbyte) - 5, (sbyte)64, (sbyte) - 51, (sbyte)45, (sbyte) - 77, (sbyte) - 76, (sbyte) - 101, (sbyte) - 108, (sbyte) - 44, (sbyte) - 103}));
                Debug.Assert(pack.address == (ushort)(ushort)61531);
                Debug.Assert(pack.type == (byte)(byte)165);
                Debug.Assert(pack.ver == (byte)(byte)197);
            };
            DemoDevice.MEMORY_VECT p249 = LoopBackDemoChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.ver = (byte)(byte)197;
            p249.value_SET(new sbyte[] {(sbyte)50, (sbyte) - 3, (sbyte) - 106, (sbyte) - 108, (sbyte)9, (sbyte)75, (sbyte) - 40, (sbyte) - 116, (sbyte) - 12, (sbyte) - 112, (sbyte)19, (sbyte) - 16, (sbyte) - 122, (sbyte)17, (sbyte) - 80, (sbyte)16, (sbyte)41, (sbyte) - 84, (sbyte)22, (sbyte) - 115, (sbyte) - 102, (sbyte) - 76, (sbyte) - 5, (sbyte)64, (sbyte) - 51, (sbyte)45, (sbyte) - 77, (sbyte) - 76, (sbyte) - 101, (sbyte) - 108, (sbyte) - 44, (sbyte) - 103}, 0) ;
            p249.address = (ushort)(ushort)61531;
            p249.type = (byte)(byte)165;
            LoopBackDemoChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)2.8543321E38F);
                Debug.Assert(pack.name_LEN(ph) == 7);
                Debug.Assert(pack.name_TRY(ph).Equals("hunadgw"));
                Debug.Assert(pack.y == (float)1.1097709E38F);
                Debug.Assert(pack.time_usec == (ulong)453311538861601689L);
                Debug.Assert(pack.x == (float)3.3526843E38F);
            };
            DemoDevice.DEBUG_VECT p250 = LoopBackDemoChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.z = (float)2.8543321E38F;
            p250.x = (float)3.3526843E38F;
            p250.name_SET("hunadgw", PH) ;
            p250.time_usec = (ulong)453311538861601689L;
            p250.y = (float)1.1097709E38F;
            LoopBackDemoChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 6);
                Debug.Assert(pack.name_TRY(ph).Equals("qwjfDo"));
                Debug.Assert(pack.value == (float) -2.5595606E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1169639565U);
            };
            DemoDevice.NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.value = (float) -2.5595606E38F;
            p251.name_SET("qwjfDo", PH) ;
            p251.time_boot_ms = (uint)1169639565U;
            LoopBackDemoChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 5);
                Debug.Assert(pack.name_TRY(ph).Equals("wIPcc"));
                Debug.Assert(pack.time_boot_ms == (uint)1762270969U);
                Debug.Assert(pack.value == (int)683131183);
            };
            DemoDevice.NAMED_VALUE_INT p252 = LoopBackDemoChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.value = (int)683131183;
            p252.name_SET("wIPcc", PH) ;
            p252.time_boot_ms = (uint)1762270969U;
            LoopBackDemoChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.severity == (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_EMERGENCY);
                Debug.Assert(pack.text_LEN(ph) == 23);
                Debug.Assert(pack.text_TRY(ph).Equals("pPstrlvLnInplixlssxwqnD"));
            };
            DemoDevice.STATUSTEXT p253 = LoopBackDemoChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_EMERGENCY;
            p253.text_SET("pPstrlvLnInplixlssxwqnD", PH) ;
            LoopBackDemoChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (float)3.2864185E38F);
                Debug.Assert(pack.ind == (byte)(byte)66);
                Debug.Assert(pack.time_boot_ms == (uint)3901097310U);
            };
            DemoDevice.DEBUG p254 = LoopBackDemoChannel.new_DEBUG();
            PH.setPack(p254);
            p254.ind = (byte)(byte)66;
            p254.time_boot_ms = (uint)3901097310U;
            p254.value = (float)3.2864185E38F;
            LoopBackDemoChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)40);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)80, (byte)91, (byte)235, (byte)53, (byte)10, (byte)158, (byte)63, (byte)236, (byte)137, (byte)203, (byte)223, (byte)153, (byte)238, (byte)252, (byte)16, (byte)1, (byte)177, (byte)1, (byte)57, (byte)78, (byte)166, (byte)229, (byte)208, (byte)81, (byte)51, (byte)156, (byte)96, (byte)77, (byte)78, (byte)23, (byte)83, (byte)139}));
                Debug.Assert(pack.initial_timestamp == (ulong)7699168199519626876L);
                Debug.Assert(pack.target_component == (byte)(byte)89);
            };
            DemoDevice.SETUP_SIGNING p256 = LoopBackDemoChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.secret_key_SET(new byte[] {(byte)80, (byte)91, (byte)235, (byte)53, (byte)10, (byte)158, (byte)63, (byte)236, (byte)137, (byte)203, (byte)223, (byte)153, (byte)238, (byte)252, (byte)16, (byte)1, (byte)177, (byte)1, (byte)57, (byte)78, (byte)166, (byte)229, (byte)208, (byte)81, (byte)51, (byte)156, (byte)96, (byte)77, (byte)78, (byte)23, (byte)83, (byte)139}, 0) ;
            p256.initial_timestamp = (ulong)7699168199519626876L;
            p256.target_component = (byte)(byte)89;
            p256.target_system = (byte)(byte)40;
            LoopBackDemoChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.state == (byte)(byte)188);
                Debug.Assert(pack.last_change_ms == (uint)1113310971U);
                Debug.Assert(pack.time_boot_ms == (uint)917035520U);
            };
            DemoDevice.BUTTON_CHANGE p257 = LoopBackDemoChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)917035520U;
            p257.last_change_ms = (uint)1113310971U;
            p257.state = (byte)(byte)188;
            LoopBackDemoChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)92);
                Debug.Assert(pack.tune_LEN(ph) == 26);
                Debug.Assert(pack.tune_TRY(ph).Equals("amnhhkphntQowmygMjLctmbvrd"));
                Debug.Assert(pack.target_system == (byte)(byte)218);
            };
            DemoDevice.PLAY_TUNE p258 = LoopBackDemoChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_component = (byte)(byte)92;
            p258.tune_SET("amnhhkphntQowmygMjLctmbvrd", PH) ;
            p258.target_system = (byte)(byte)218;
            LoopBackDemoChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE);
                Debug.Assert(pack.lens_id == (byte)(byte)169);
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 67);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("pIawlrpactfzvjZafigdlpswupbxmzoGbIlocxjlwollftyckxqgqoevRjayYbGezlk"));
                Debug.Assert(pack.firmware_version == (uint)2549405029U);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)65494);
                Debug.Assert(pack.sensor_size_h == (float)1.2022026E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1204667033U);
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)214, (byte)120, (byte)124, (byte)18, (byte)33, (byte)101, (byte)60, (byte)254, (byte)29, (byte)157, (byte)176, (byte)152, (byte)184, (byte)184, (byte)194, (byte)80, (byte)130, (byte)198, (byte)227, (byte)124, (byte)48, (byte)99, (byte)215, (byte)176, (byte)248, (byte)25, (byte)54, (byte)138, (byte)236, (byte)238, (byte)237, (byte)218}));
                Debug.Assert(pack.focal_length == (float)1.5700875E38F);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)13342);
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)61513);
                Debug.Assert(pack.sensor_size_v == (float)2.71712E38F);
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)250, (byte)87, (byte)18, (byte)161, (byte)181, (byte)164, (byte)59, (byte)58, (byte)196, (byte)29, (byte)93, (byte)44, (byte)45, (byte)94, (byte)37, (byte)97, (byte)91, (byte)31, (byte)111, (byte)116, (byte)159, (byte)117, (byte)120, (byte)198, (byte)75, (byte)202, (byte)95, (byte)88, (byte)86, (byte)60, (byte)12, (byte)37}));
            };
            DemoDevice.CAMERA_INFORMATION p259 = LoopBackDemoChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.cam_definition_version = (ushort)(ushort)61513;
            p259.model_name_SET(new byte[] {(byte)250, (byte)87, (byte)18, (byte)161, (byte)181, (byte)164, (byte)59, (byte)58, (byte)196, (byte)29, (byte)93, (byte)44, (byte)45, (byte)94, (byte)37, (byte)97, (byte)91, (byte)31, (byte)111, (byte)116, (byte)159, (byte)117, (byte)120, (byte)198, (byte)75, (byte)202, (byte)95, (byte)88, (byte)86, (byte)60, (byte)12, (byte)37}, 0) ;
            p259.sensor_size_h = (float)1.2022026E38F;
            p259.sensor_size_v = (float)2.71712E38F;
            p259.cam_definition_uri_SET("pIawlrpactfzvjZafigdlpswupbxmzoGbIlocxjlwollftyckxqgqoevRjayYbGezlk", PH) ;
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE;
            p259.resolution_v = (ushort)(ushort)13342;
            p259.time_boot_ms = (uint)1204667033U;
            p259.lens_id = (byte)(byte)169;
            p259.resolution_h = (ushort)(ushort)65494;
            p259.firmware_version = (uint)2549405029U;
            p259.vendor_name_SET(new byte[] {(byte)214, (byte)120, (byte)124, (byte)18, (byte)33, (byte)101, (byte)60, (byte)254, (byte)29, (byte)157, (byte)176, (byte)152, (byte)184, (byte)184, (byte)194, (byte)80, (byte)130, (byte)198, (byte)227, (byte)124, (byte)48, (byte)99, (byte)215, (byte)176, (byte)248, (byte)25, (byte)54, (byte)138, (byte)236, (byte)238, (byte)237, (byte)218}, 0) ;
            p259.focal_length = (float)1.5700875E38F;
            LoopBackDemoChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1347644399U);
                Debug.Assert(pack.mode_id == (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE);
            };
            DemoDevice.CAMERA_SETTINGS p260 = LoopBackDemoChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)1347644399U;
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE;
            LoopBackDemoChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.available_capacity == (float)5.826617E37F);
                Debug.Assert(pack.used_capacity == (float)1.4052852E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3963647721U);
                Debug.Assert(pack.storage_count == (byte)(byte)174);
                Debug.Assert(pack.storage_id == (byte)(byte)34);
                Debug.Assert(pack.write_speed == (float)2.2350725E38F);
                Debug.Assert(pack.read_speed == (float)1.6557225E38F);
                Debug.Assert(pack.status == (byte)(byte)240);
                Debug.Assert(pack.total_capacity == (float)9.351186E37F);
            };
            DemoDevice.STORAGE_INFORMATION p261 = LoopBackDemoChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.available_capacity = (float)5.826617E37F;
            p261.write_speed = (float)2.2350725E38F;
            p261.time_boot_ms = (uint)3963647721U;
            p261.used_capacity = (float)1.4052852E38F;
            p261.read_speed = (float)1.6557225E38F;
            p261.storage_id = (byte)(byte)34;
            p261.total_capacity = (float)9.351186E37F;
            p261.status = (byte)(byte)240;
            p261.storage_count = (byte)(byte)174;
            LoopBackDemoChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.recording_time_ms == (uint)2073538262U);
                Debug.Assert(pack.time_boot_ms == (uint)1455925621U);
                Debug.Assert(pack.image_interval == (float) -9.675058E37F);
                Debug.Assert(pack.image_status == (byte)(byte)67);
                Debug.Assert(pack.available_capacity == (float) -2.00838E38F);
                Debug.Assert(pack.video_status == (byte)(byte)37);
            };
            DemoDevice.CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)1455925621U;
            p262.video_status = (byte)(byte)37;
            p262.recording_time_ms = (uint)2073538262U;
            p262.image_interval = (float) -9.675058E37F;
            p262.available_capacity = (float) -2.00838E38F;
            p262.image_status = (byte)(byte)67;
            LoopBackDemoChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.camera_id == (byte)(byte)183);
                Debug.Assert(pack.image_index == (int)1199219312);
                Debug.Assert(pack.capture_result == (sbyte)(sbyte)6);
                Debug.Assert(pack.relative_alt == (int) -1148680213);
                Debug.Assert(pack.alt == (int) -1172851171);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-8.937817E36F, -3.3506963E38F, -2.1750042E37F, -3.0879167E38F}));
                Debug.Assert(pack.time_utc == (ulong)7946529316078754463L);
                Debug.Assert(pack.time_boot_ms == (uint)1042389182U);
                Debug.Assert(pack.lon == (int)999367752);
                Debug.Assert(pack.file_url_LEN(ph) == 12);
                Debug.Assert(pack.file_url_TRY(ph).Equals("dmwsxnwvwoky"));
                Debug.Assert(pack.lat == (int)339577802);
            };
            DemoDevice.CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.time_boot_ms = (uint)1042389182U;
            p263.capture_result = (sbyte)(sbyte)6;
            p263.file_url_SET("dmwsxnwvwoky", PH) ;
            p263.relative_alt = (int) -1148680213;
            p263.q_SET(new float[] {-8.937817E36F, -3.3506963E38F, -2.1750042E37F, -3.0879167E38F}, 0) ;
            p263.time_utc = (ulong)7946529316078754463L;
            p263.alt = (int) -1172851171;
            p263.image_index = (int)1199219312;
            p263.camera_id = (byte)(byte)183;
            p263.lat = (int)339577802;
            p263.lon = (int)999367752;
            LoopBackDemoChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)247148345U);
                Debug.Assert(pack.flight_uuid == (ulong)5836302803507479711L);
                Debug.Assert(pack.arming_time_utc == (ulong)2262123773083996260L);
                Debug.Assert(pack.takeoff_time_utc == (ulong)4785964122979058825L);
            };
            DemoDevice.FLIGHT_INFORMATION p264 = LoopBackDemoChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.takeoff_time_utc = (ulong)4785964122979058825L;
            p264.time_boot_ms = (uint)247148345U;
            p264.arming_time_utc = (ulong)2262123773083996260L;
            p264.flight_uuid = (ulong)5836302803507479711L;
            LoopBackDemoChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float) -3.269663E38F);
                Debug.Assert(pack.yaw == (float) -8.645704E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3020365872U);
                Debug.Assert(pack.pitch == (float) -1.3309866E37F);
            };
            DemoDevice.MOUNT_ORIENTATION p265 = LoopBackDemoChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.yaw = (float) -8.645704E37F;
            p265.time_boot_ms = (uint)3020365872U;
            p265.roll = (float) -3.269663E38F;
            p265.pitch = (float) -1.3309866E37F;
            LoopBackDemoChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.length == (byte)(byte)187);
                Debug.Assert(pack.first_message_offset == (byte)(byte)9);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)55, (byte)27, (byte)101, (byte)26, (byte)127, (byte)248, (byte)199, (byte)33, (byte)204, (byte)154, (byte)216, (byte)46, (byte)228, (byte)200, (byte)31, (byte)72, (byte)239, (byte)102, (byte)84, (byte)133, (byte)181, (byte)60, (byte)229, (byte)231, (byte)111, (byte)9, (byte)98, (byte)133, (byte)83, (byte)217, (byte)77, (byte)202, (byte)149, (byte)141, (byte)69, (byte)189, (byte)215, (byte)100, (byte)75, (byte)37, (byte)133, (byte)168, (byte)18, (byte)251, (byte)69, (byte)138, (byte)28, (byte)239, (byte)212, (byte)68, (byte)97, (byte)156, (byte)202, (byte)135, (byte)58, (byte)71, (byte)65, (byte)96, (byte)39, (byte)10, (byte)116, (byte)13, (byte)119, (byte)126, (byte)53, (byte)12, (byte)245, (byte)228, (byte)70, (byte)109, (byte)248, (byte)177, (byte)139, (byte)38, (byte)28, (byte)90, (byte)39, (byte)192, (byte)61, (byte)129, (byte)187, (byte)245, (byte)109, (byte)120, (byte)119, (byte)184, (byte)149, (byte)134, (byte)191, (byte)136, (byte)9, (byte)109, (byte)87, (byte)204, (byte)26, (byte)212, (byte)216, (byte)124, (byte)29, (byte)187, (byte)42, (byte)215, (byte)197, (byte)110, (byte)221, (byte)56, (byte)116, (byte)201, (byte)3, (byte)135, (byte)81, (byte)68, (byte)15, (byte)51, (byte)92, (byte)109, (byte)168, (byte)64, (byte)149, (byte)143, (byte)37, (byte)124, (byte)222, (byte)58, (byte)141, (byte)92, (byte)239, (byte)74, (byte)221, (byte)1, (byte)58, (byte)69, (byte)171, (byte)166, (byte)236, (byte)87, (byte)81, (byte)75, (byte)76, (byte)224, (byte)33, (byte)146, (byte)124, (byte)112, (byte)173, (byte)99, (byte)81, (byte)91, (byte)181, (byte)64, (byte)102, (byte)212, (byte)127, (byte)23, (byte)214, (byte)134, (byte)213, (byte)84, (byte)54, (byte)222, (byte)61, (byte)77, (byte)211, (byte)188, (byte)207, (byte)37, (byte)189, (byte)186, (byte)47, (byte)186, (byte)34, (byte)100, (byte)208, (byte)234, (byte)57, (byte)215, (byte)87, (byte)81, (byte)108, (byte)36, (byte)112, (byte)93, (byte)54, (byte)222, (byte)97, (byte)27, (byte)121, (byte)90, (byte)8, (byte)215, (byte)36, (byte)57, (byte)182, (byte)231, (byte)255, (byte)29, (byte)56, (byte)248, (byte)54, (byte)44, (byte)168, (byte)71, (byte)91, (byte)198, (byte)101, (byte)18, (byte)75, (byte)123, (byte)255, (byte)222, (byte)160, (byte)221, (byte)82, (byte)164, (byte)42, (byte)50, (byte)64, (byte)221, (byte)166, (byte)253, (byte)96, (byte)200, (byte)96, (byte)226, (byte)89, (byte)166, (byte)223, (byte)150, (byte)236, (byte)80, (byte)45, (byte)42, (byte)87, (byte)106, (byte)3, (byte)53, (byte)108, (byte)198, (byte)157, (byte)73, (byte)12, (byte)204, (byte)99, (byte)53, (byte)124, (byte)65, (byte)162, (byte)86, (byte)24}));
                Debug.Assert(pack.target_component == (byte)(byte)195);
                Debug.Assert(pack.sequence == (ushort)(ushort)12390);
                Debug.Assert(pack.target_system == (byte)(byte)129);
            };
            DemoDevice.LOGGING_DATA p266 = LoopBackDemoChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_component = (byte)(byte)195;
            p266.first_message_offset = (byte)(byte)9;
            p266.data__SET(new byte[] {(byte)55, (byte)27, (byte)101, (byte)26, (byte)127, (byte)248, (byte)199, (byte)33, (byte)204, (byte)154, (byte)216, (byte)46, (byte)228, (byte)200, (byte)31, (byte)72, (byte)239, (byte)102, (byte)84, (byte)133, (byte)181, (byte)60, (byte)229, (byte)231, (byte)111, (byte)9, (byte)98, (byte)133, (byte)83, (byte)217, (byte)77, (byte)202, (byte)149, (byte)141, (byte)69, (byte)189, (byte)215, (byte)100, (byte)75, (byte)37, (byte)133, (byte)168, (byte)18, (byte)251, (byte)69, (byte)138, (byte)28, (byte)239, (byte)212, (byte)68, (byte)97, (byte)156, (byte)202, (byte)135, (byte)58, (byte)71, (byte)65, (byte)96, (byte)39, (byte)10, (byte)116, (byte)13, (byte)119, (byte)126, (byte)53, (byte)12, (byte)245, (byte)228, (byte)70, (byte)109, (byte)248, (byte)177, (byte)139, (byte)38, (byte)28, (byte)90, (byte)39, (byte)192, (byte)61, (byte)129, (byte)187, (byte)245, (byte)109, (byte)120, (byte)119, (byte)184, (byte)149, (byte)134, (byte)191, (byte)136, (byte)9, (byte)109, (byte)87, (byte)204, (byte)26, (byte)212, (byte)216, (byte)124, (byte)29, (byte)187, (byte)42, (byte)215, (byte)197, (byte)110, (byte)221, (byte)56, (byte)116, (byte)201, (byte)3, (byte)135, (byte)81, (byte)68, (byte)15, (byte)51, (byte)92, (byte)109, (byte)168, (byte)64, (byte)149, (byte)143, (byte)37, (byte)124, (byte)222, (byte)58, (byte)141, (byte)92, (byte)239, (byte)74, (byte)221, (byte)1, (byte)58, (byte)69, (byte)171, (byte)166, (byte)236, (byte)87, (byte)81, (byte)75, (byte)76, (byte)224, (byte)33, (byte)146, (byte)124, (byte)112, (byte)173, (byte)99, (byte)81, (byte)91, (byte)181, (byte)64, (byte)102, (byte)212, (byte)127, (byte)23, (byte)214, (byte)134, (byte)213, (byte)84, (byte)54, (byte)222, (byte)61, (byte)77, (byte)211, (byte)188, (byte)207, (byte)37, (byte)189, (byte)186, (byte)47, (byte)186, (byte)34, (byte)100, (byte)208, (byte)234, (byte)57, (byte)215, (byte)87, (byte)81, (byte)108, (byte)36, (byte)112, (byte)93, (byte)54, (byte)222, (byte)97, (byte)27, (byte)121, (byte)90, (byte)8, (byte)215, (byte)36, (byte)57, (byte)182, (byte)231, (byte)255, (byte)29, (byte)56, (byte)248, (byte)54, (byte)44, (byte)168, (byte)71, (byte)91, (byte)198, (byte)101, (byte)18, (byte)75, (byte)123, (byte)255, (byte)222, (byte)160, (byte)221, (byte)82, (byte)164, (byte)42, (byte)50, (byte)64, (byte)221, (byte)166, (byte)253, (byte)96, (byte)200, (byte)96, (byte)226, (byte)89, (byte)166, (byte)223, (byte)150, (byte)236, (byte)80, (byte)45, (byte)42, (byte)87, (byte)106, (byte)3, (byte)53, (byte)108, (byte)198, (byte)157, (byte)73, (byte)12, (byte)204, (byte)99, (byte)53, (byte)124, (byte)65, (byte)162, (byte)86, (byte)24}, 0) ;
            p266.target_system = (byte)(byte)129;
            p266.length = (byte)(byte)187;
            p266.sequence = (ushort)(ushort)12390;
            LoopBackDemoChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)244);
                Debug.Assert(pack.first_message_offset == (byte)(byte)108);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)229, (byte)91, (byte)21, (byte)182, (byte)153, (byte)83, (byte)121, (byte)3, (byte)190, (byte)52, (byte)93, (byte)28, (byte)62, (byte)21, (byte)66, (byte)160, (byte)224, (byte)25, (byte)95, (byte)174, (byte)26, (byte)218, (byte)44, (byte)226, (byte)119, (byte)113, (byte)51, (byte)50, (byte)81, (byte)156, (byte)249, (byte)175, (byte)76, (byte)211, (byte)134, (byte)184, (byte)141, (byte)145, (byte)117, (byte)196, (byte)140, (byte)166, (byte)207, (byte)212, (byte)67, (byte)132, (byte)101, (byte)99, (byte)235, (byte)110, (byte)205, (byte)90, (byte)40, (byte)172, (byte)168, (byte)98, (byte)95, (byte)201, (byte)87, (byte)102, (byte)72, (byte)241, (byte)28, (byte)13, (byte)226, (byte)41, (byte)86, (byte)233, (byte)55, (byte)206, (byte)127, (byte)128, (byte)11, (byte)217, (byte)24, (byte)30, (byte)106, (byte)121, (byte)196, (byte)124, (byte)248, (byte)17, (byte)55, (byte)131, (byte)127, (byte)44, (byte)157, (byte)169, (byte)91, (byte)13, (byte)144, (byte)102, (byte)171, (byte)79, (byte)141, (byte)241, (byte)23, (byte)187, (byte)60, (byte)171, (byte)196, (byte)84, (byte)44, (byte)237, (byte)172, (byte)27, (byte)20, (byte)84, (byte)126, (byte)180, (byte)169, (byte)139, (byte)162, (byte)42, (byte)253, (byte)151, (byte)239, (byte)32, (byte)17, (byte)89, (byte)235, (byte)2, (byte)51, (byte)33, (byte)138, (byte)93, (byte)8, (byte)102, (byte)49, (byte)17, (byte)226, (byte)157, (byte)49, (byte)242, (byte)105, (byte)109, (byte)0, (byte)8, (byte)119, (byte)6, (byte)69, (byte)220, (byte)84, (byte)161, (byte)43, (byte)161, (byte)62, (byte)252, (byte)47, (byte)142, (byte)152, (byte)226, (byte)236, (byte)157, (byte)152, (byte)177, (byte)215, (byte)14, (byte)239, (byte)217, (byte)224, (byte)16, (byte)108, (byte)251, (byte)229, (byte)244, (byte)211, (byte)239, (byte)145, (byte)48, (byte)12, (byte)54, (byte)163, (byte)189, (byte)138, (byte)68, (byte)93, (byte)235, (byte)187, (byte)174, (byte)173, (byte)127, (byte)192, (byte)52, (byte)6, (byte)191, (byte)188, (byte)110, (byte)216, (byte)192, (byte)170, (byte)0, (byte)95, (byte)92, (byte)2, (byte)153, (byte)169, (byte)254, (byte)247, (byte)154, (byte)251, (byte)202, (byte)143, (byte)65, (byte)95, (byte)110, (byte)43, (byte)138, (byte)245, (byte)247, (byte)214, (byte)129, (byte)193, (byte)141, (byte)32, (byte)84, (byte)127, (byte)187, (byte)184, (byte)113, (byte)0, (byte)137, (byte)105, (byte)246, (byte)202, (byte)225, (byte)69, (byte)231, (byte)112, (byte)178, (byte)95, (byte)118, (byte)58, (byte)194, (byte)46, (byte)140, (byte)233, (byte)80, (byte)30, (byte)37, (byte)191, (byte)81, (byte)111, (byte)125, (byte)118, (byte)181, (byte)150, (byte)180, (byte)112}));
                Debug.Assert(pack.sequence == (ushort)(ushort)60251);
                Debug.Assert(pack.target_system == (byte)(byte)56);
                Debug.Assert(pack.length == (byte)(byte)255);
            };
            DemoDevice.LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_component = (byte)(byte)244;
            p267.data__SET(new byte[] {(byte)229, (byte)91, (byte)21, (byte)182, (byte)153, (byte)83, (byte)121, (byte)3, (byte)190, (byte)52, (byte)93, (byte)28, (byte)62, (byte)21, (byte)66, (byte)160, (byte)224, (byte)25, (byte)95, (byte)174, (byte)26, (byte)218, (byte)44, (byte)226, (byte)119, (byte)113, (byte)51, (byte)50, (byte)81, (byte)156, (byte)249, (byte)175, (byte)76, (byte)211, (byte)134, (byte)184, (byte)141, (byte)145, (byte)117, (byte)196, (byte)140, (byte)166, (byte)207, (byte)212, (byte)67, (byte)132, (byte)101, (byte)99, (byte)235, (byte)110, (byte)205, (byte)90, (byte)40, (byte)172, (byte)168, (byte)98, (byte)95, (byte)201, (byte)87, (byte)102, (byte)72, (byte)241, (byte)28, (byte)13, (byte)226, (byte)41, (byte)86, (byte)233, (byte)55, (byte)206, (byte)127, (byte)128, (byte)11, (byte)217, (byte)24, (byte)30, (byte)106, (byte)121, (byte)196, (byte)124, (byte)248, (byte)17, (byte)55, (byte)131, (byte)127, (byte)44, (byte)157, (byte)169, (byte)91, (byte)13, (byte)144, (byte)102, (byte)171, (byte)79, (byte)141, (byte)241, (byte)23, (byte)187, (byte)60, (byte)171, (byte)196, (byte)84, (byte)44, (byte)237, (byte)172, (byte)27, (byte)20, (byte)84, (byte)126, (byte)180, (byte)169, (byte)139, (byte)162, (byte)42, (byte)253, (byte)151, (byte)239, (byte)32, (byte)17, (byte)89, (byte)235, (byte)2, (byte)51, (byte)33, (byte)138, (byte)93, (byte)8, (byte)102, (byte)49, (byte)17, (byte)226, (byte)157, (byte)49, (byte)242, (byte)105, (byte)109, (byte)0, (byte)8, (byte)119, (byte)6, (byte)69, (byte)220, (byte)84, (byte)161, (byte)43, (byte)161, (byte)62, (byte)252, (byte)47, (byte)142, (byte)152, (byte)226, (byte)236, (byte)157, (byte)152, (byte)177, (byte)215, (byte)14, (byte)239, (byte)217, (byte)224, (byte)16, (byte)108, (byte)251, (byte)229, (byte)244, (byte)211, (byte)239, (byte)145, (byte)48, (byte)12, (byte)54, (byte)163, (byte)189, (byte)138, (byte)68, (byte)93, (byte)235, (byte)187, (byte)174, (byte)173, (byte)127, (byte)192, (byte)52, (byte)6, (byte)191, (byte)188, (byte)110, (byte)216, (byte)192, (byte)170, (byte)0, (byte)95, (byte)92, (byte)2, (byte)153, (byte)169, (byte)254, (byte)247, (byte)154, (byte)251, (byte)202, (byte)143, (byte)65, (byte)95, (byte)110, (byte)43, (byte)138, (byte)245, (byte)247, (byte)214, (byte)129, (byte)193, (byte)141, (byte)32, (byte)84, (byte)127, (byte)187, (byte)184, (byte)113, (byte)0, (byte)137, (byte)105, (byte)246, (byte)202, (byte)225, (byte)69, (byte)231, (byte)112, (byte)178, (byte)95, (byte)118, (byte)58, (byte)194, (byte)46, (byte)140, (byte)233, (byte)80, (byte)30, (byte)37, (byte)191, (byte)81, (byte)111, (byte)125, (byte)118, (byte)181, (byte)150, (byte)180, (byte)112}, 0) ;
            p267.first_message_offset = (byte)(byte)108;
            p267.sequence = (ushort)(ushort)60251;
            p267.length = (byte)(byte)255;
            p267.target_system = (byte)(byte)56;
            LoopBackDemoChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sequence == (ushort)(ushort)48829);
                Debug.Assert(pack.target_system == (byte)(byte)115);
                Debug.Assert(pack.target_component == (byte)(byte)211);
            };
            DemoDevice.LOGGING_ACK p268 = LoopBackDemoChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.sequence = (ushort)(ushort)48829;
            p268.target_component = (byte)(byte)211;
            p268.target_system = (byte)(byte)115;
            LoopBackDemoChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.status == (byte)(byte)213);
                Debug.Assert(pack.uri_LEN(ph) == 165);
                Debug.Assert(pack.uri_TRY(ph).Equals("vxyxawSxeqtZaaiiugzduzrieifjevzddkpuddbazzfcegjdicvtsuphqfpzgygoowxxwkfdhkbgzxvmoqduomuqvhfTreFpciwqjDuamrxxavxPjkclaeezNnjyqFmcskocwzmewLvdnetdjadguybsynupKidnbsdee"));
                Debug.Assert(pack.camera_id == (byte)(byte)143);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)52339);
                Debug.Assert(pack.framerate == (float) -2.0457327E38F);
                Debug.Assert(pack.bitrate == (uint)2250072841U);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)11384);
                Debug.Assert(pack.rotation == (ushort)(ushort)49083);
            };
            DemoDevice.VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.resolution_h = (ushort)(ushort)52339;
            p269.bitrate = (uint)2250072841U;
            p269.resolution_v = (ushort)(ushort)11384;
            p269.uri_SET("vxyxawSxeqtZaaiiugzduzrieifjevzddkpuddbazzfcegjdicvtsuphqfpzgygoowxxwkfdhkbgzxvmoqduomuqvhfTreFpciwqjDuamrxxavxPjkclaeezNnjyqFmcskocwzmewLvdnetdjadguybsynupKidnbsdee", PH) ;
            p269.rotation = (ushort)(ushort)49083;
            p269.camera_id = (byte)(byte)143;
            p269.framerate = (float) -2.0457327E38F;
            p269.status = (byte)(byte)213;
            LoopBackDemoChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)39);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)32539);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)37963);
                Debug.Assert(pack.rotation == (ushort)(ushort)19619);
                Debug.Assert(pack.bitrate == (uint)3589387196U);
                Debug.Assert(pack.target_component == (byte)(byte)172);
                Debug.Assert(pack.camera_id == (byte)(byte)79);
                Debug.Assert(pack.uri_LEN(ph) == 146);
                Debug.Assert(pack.uri_TRY(ph).Equals("wwzzkxgovfWFmOcCroLcxWdSxuatjyrazjzoqqnOtujwAdgsffjlaqJlckfxdwHoqmtjizcpfpnGzczgrHwilizrplwvvyaxhnejrbdlidyyfjHxajmnnjadzWZlmoxscgastjkpazaxnampMn"));
                Debug.Assert(pack.framerate == (float) -2.720386E38F);
            };
            DemoDevice.SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.resolution_v = (ushort)(ushort)37963;
            p270.target_system = (byte)(byte)39;
            p270.bitrate = (uint)3589387196U;
            p270.uri_SET("wwzzkxgovfWFmOcCroLcxWdSxuatjyrazjzoqqnOtujwAdgsffjlaqJlckfxdwHoqmtjizcpfpnGzczgrHwilizrplwvvyaxhnejrbdlidyyfjHxajmnnjadzWZlmoxscgastjkpazaxnampMn", PH) ;
            p270.resolution_h = (ushort)(ushort)32539;
            p270.target_component = (byte)(byte)172;
            p270.rotation = (ushort)(ushort)19619;
            p270.framerate = (float) -2.720386E38F;
            p270.camera_id = (byte)(byte)79;
            LoopBackDemoChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.password_LEN(ph) == 48);
                Debug.Assert(pack.password_TRY(ph).Equals("cpfnbGkzcqgwvpxvfbbvpOxYlujFumruedaignbmdduuepCu"));
                Debug.Assert(pack.ssid_LEN(ph) == 24);
                Debug.Assert(pack.ssid_TRY(ph).Equals("cpnbrrbTiajebXrssbzvbppc"));
            };
            DemoDevice.WIFI_CONFIG_AP p299 = LoopBackDemoChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("cpnbrrbTiajebXrssbzvbppc", PH) ;
            p299.password_SET("cpfnbGkzcqgwvpxvfbbvpOxYlujFumruedaignbmdduuepCu", PH) ;
            LoopBackDemoChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)71, (byte)56, (byte)26, (byte)134, (byte)7, (byte)7, (byte)147, (byte)25}));
                Debug.Assert(pack.min_version == (ushort)(ushort)42872);
                Debug.Assert(pack.max_version == (ushort)(ushort)33606);
                Debug.Assert(pack.version == (ushort)(ushort)15730);
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)65, (byte)193, (byte)14, (byte)6, (byte)179, (byte)255, (byte)140, (byte)142}));
            };
            DemoDevice.PROTOCOL_VERSION p300 = LoopBackDemoChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.min_version = (ushort)(ushort)42872;
            p300.max_version = (ushort)(ushort)33606;
            p300.version = (ushort)(ushort)15730;
            p300.library_version_hash_SET(new byte[] {(byte)71, (byte)56, (byte)26, (byte)134, (byte)7, (byte)7, (byte)147, (byte)25}, 0) ;
            p300.spec_version_hash_SET(new byte[] {(byte)65, (byte)193, (byte)14, (byte)6, (byte)179, (byte)255, (byte)140, (byte)142}, 0) ;
            LoopBackDemoChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)1332464419708002800L);
                Debug.Assert(pack.mode == (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)8105);
                Debug.Assert(pack.health == (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING);
                Debug.Assert(pack.uptime_sec == (uint)3166680546U);
                Debug.Assert(pack.sub_mode == (byte)(byte)39);
            };
            DemoDevice.UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE;
            p310.uptime_sec = (uint)3166680546U;
            p310.sub_mode = (byte)(byte)39;
            p310.time_usec = (ulong)1332464419708002800L;
            p310.vendor_specific_status_code = (ushort)(ushort)8105;
            LoopBackDemoChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sw_vcs_commit == (uint)1224221777U);
                Debug.Assert(pack.hw_version_minor == (byte)(byte)170);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)191, (byte)95, (byte)140, (byte)204, (byte)22, (byte)252, (byte)216, (byte)236, (byte)254, (byte)200, (byte)73, (byte)235, (byte)201, (byte)239, (byte)102, (byte)121}));
                Debug.Assert(pack.uptime_sec == (uint)1186599126U);
                Debug.Assert(pack.hw_version_major == (byte)(byte)156);
                Debug.Assert(pack.sw_version_major == (byte)(byte)134);
                Debug.Assert(pack.time_usec == (ulong)2071431207764973516L);
                Debug.Assert(pack.name_LEN(ph) == 13);
                Debug.Assert(pack.name_TRY(ph).Equals("ppcukvjbLhvuX"));
                Debug.Assert(pack.sw_version_minor == (byte)(byte)248);
            };
            DemoDevice.UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.hw_version_major = (byte)(byte)156;
            p311.hw_version_minor = (byte)(byte)170;
            p311.name_SET("ppcukvjbLhvuX", PH) ;
            p311.sw_vcs_commit = (uint)1224221777U;
            p311.hw_unique_id_SET(new byte[] {(byte)191, (byte)95, (byte)140, (byte)204, (byte)22, (byte)252, (byte)216, (byte)236, (byte)254, (byte)200, (byte)73, (byte)235, (byte)201, (byte)239, (byte)102, (byte)121}, 0) ;
            p311.uptime_sec = (uint)1186599126U;
            p311.time_usec = (ulong)2071431207764973516L;
            p311.sw_version_major = (byte)(byte)134;
            p311.sw_version_minor = (byte)(byte)248;
            LoopBackDemoChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (short)(short) -9386);
                Debug.Assert(pack.target_system == (byte)(byte)153);
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("qpndt"));
                Debug.Assert(pack.target_component == (byte)(byte)101);
            };
            DemoDevice.PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)153;
            p320.param_id_SET("qpndt", PH) ;
            p320.target_component = (byte)(byte)101;
            p320.param_index = (short)(short) -9386;
            LoopBackDemoChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)42);
                Debug.Assert(pack.target_component == (byte)(byte)238);
            };
            DemoDevice.PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_component = (byte)(byte)238;
            p321.target_system = (byte)(byte)42;
            LoopBackDemoChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 1);
                Debug.Assert(pack.param_id_TRY(ph).Equals("a"));
                Debug.Assert(pack.param_count == (ushort)(ushort)65411);
                Debug.Assert(pack.param_index == (ushort)(ushort)58588);
                Debug.Assert(pack.param_value_LEN(ph) == 11);
                Debug.Assert(pack.param_value_TRY(ph).Equals("bssnJekpvvt"));
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
            };
            DemoDevice.PARAM_EXT_VALUE p322 = LoopBackDemoChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32;
            p322.param_count = (ushort)(ushort)65411;
            p322.param_id_SET("a", PH) ;
            p322.param_value_SET("bssnJekpvvt", PH) ;
            p322.param_index = (ushort)(ushort)58588;
            LoopBackDemoChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 16);
                Debug.Assert(pack.param_id_TRY(ph).Equals("oxawujpdUszOcoib"));
                Debug.Assert(pack.param_value_LEN(ph) == 61);
                Debug.Assert(pack.param_value_TRY(ph).Equals("ewycogatcbaHqgFUunvtdobtpklwwrvrFmdmrNiqcevOxpsdkfmtoaGcOsmwx"));
                Debug.Assert(pack.target_system == (byte)(byte)32);
                Debug.Assert(pack.target_component == (byte)(byte)6);
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
            };
            DemoDevice.PARAM_EXT_SET p323 = LoopBackDemoChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)32;
            p323.target_component = (byte)(byte)6;
            p323.param_id_SET("oxawujpdUszOcoib", PH) ;
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32;
            p323.param_value_SET("ewycogatcbaHqgFUunvtdobtpklwwrvrFmdmrNiqcevOxpsdkfmtoaGcOsmwx", PH) ;
            LoopBackDemoChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_result == (PARAM_ACK)PARAM_ACK.PARAM_ACK_IN_PROGRESS);
                Debug.Assert(pack.param_value_LEN(ph) == 121);
                Debug.Assert(pack.param_value_TRY(ph).Equals("pFaegyiXcejquxzhxcnknfjodyctixveyoaetcudYmqlemulkxadOapjaxfFkjveijzYuUvthvmegmbbnstuKahyhqebesDomXfcfownvsfenjhnbxiwZybpa"));
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("bzetf"));
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
            };
            DemoDevice.PARAM_EXT_ACK p324 = LoopBackDemoChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_IN_PROGRESS;
            p324.param_id_SET("bzetf", PH) ;
            p324.param_value_SET("pFaegyiXcejquxzhxcnknfjodyctixveyoaetcudYmqlemulkxadOapjaxfFkjveijzYuUvthvmegmbbnstuKahyhqebesDomXfcfownvsfenjhnbxiwZybpa", PH) ;
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16;
            LoopBackDemoChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.max_distance == (ushort)(ushort)44156);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)65206, (ushort)17527, (ushort)54380, (ushort)33022, (ushort)61785, (ushort)1971, (ushort)9317, (ushort)30591, (ushort)25785, (ushort)27939, (ushort)42316, (ushort)7060, (ushort)43530, (ushort)17269, (ushort)63597, (ushort)10852, (ushort)16824, (ushort)46225, (ushort)22964, (ushort)43711, (ushort)7590, (ushort)32531, (ushort)65435, (ushort)4843, (ushort)4082, (ushort)44880, (ushort)14416, (ushort)6187, (ushort)50968, (ushort)15046, (ushort)8002, (ushort)17040, (ushort)33244, (ushort)37953, (ushort)64034, (ushort)64005, (ushort)59699, (ushort)52492, (ushort)42431, (ushort)12980, (ushort)49852, (ushort)1241, (ushort)64497, (ushort)3157, (ushort)4425, (ushort)50224, (ushort)3957, (ushort)59758, (ushort)59963, (ushort)17012, (ushort)22810, (ushort)62932, (ushort)47762, (ushort)58007, (ushort)19251, (ushort)57909, (ushort)53977, (ushort)22071, (ushort)59240, (ushort)1233, (ushort)60517, (ushort)20344, (ushort)12547, (ushort)10056, (ushort)60592, (ushort)24908, (ushort)30187, (ushort)32815, (ushort)39546, (ushort)45150, (ushort)32767, (ushort)38852}));
                Debug.Assert(pack.time_usec == (ulong)4116540411815774274L);
                Debug.Assert(pack.sensor_type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
                Debug.Assert(pack.increment == (byte)(byte)116);
                Debug.Assert(pack.min_distance == (ushort)(ushort)36735);
            };
            DemoDevice.OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND;
            p330.min_distance = (ushort)(ushort)36735;
            p330.time_usec = (ulong)4116540411815774274L;
            p330.distances_SET(new ushort[] {(ushort)65206, (ushort)17527, (ushort)54380, (ushort)33022, (ushort)61785, (ushort)1971, (ushort)9317, (ushort)30591, (ushort)25785, (ushort)27939, (ushort)42316, (ushort)7060, (ushort)43530, (ushort)17269, (ushort)63597, (ushort)10852, (ushort)16824, (ushort)46225, (ushort)22964, (ushort)43711, (ushort)7590, (ushort)32531, (ushort)65435, (ushort)4843, (ushort)4082, (ushort)44880, (ushort)14416, (ushort)6187, (ushort)50968, (ushort)15046, (ushort)8002, (ushort)17040, (ushort)33244, (ushort)37953, (ushort)64034, (ushort)64005, (ushort)59699, (ushort)52492, (ushort)42431, (ushort)12980, (ushort)49852, (ushort)1241, (ushort)64497, (ushort)3157, (ushort)4425, (ushort)50224, (ushort)3957, (ushort)59758, (ushort)59963, (ushort)17012, (ushort)22810, (ushort)62932, (ushort)47762, (ushort)58007, (ushort)19251, (ushort)57909, (ushort)53977, (ushort)22071, (ushort)59240, (ushort)1233, (ushort)60517, (ushort)20344, (ushort)12547, (ushort)10056, (ushort)60592, (ushort)24908, (ushort)30187, (ushort)32815, (ushort)39546, (ushort)45150, (ushort)32767, (ushort)38852}, 0) ;
            p330.max_distance = (ushort)(ushort)44156;
            p330.increment = (byte)(byte)116;
            LoopBackDemoChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
        }
    }
}