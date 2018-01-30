
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
                Debug.Assert(pack.system_status == (MAV_STATE)MAV_STATE.MAV_STATE_CALIBRATING);
                Debug.Assert(pack.type == (MAV_TYPE)MAV_TYPE.MAV_TYPE_ROCKET);
                Debug.Assert(pack.mavlink_version == (byte)(byte)65);
                Debug.Assert(pack.custom_mode == (uint)2885716531U);
                Debug.Assert(pack.autopilot == (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_AEROB);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED);
            };
            DemoDevice.HEARTBEAT p0 = LoopBackDemoChannel.new_HEARTBEAT();
            PH.setPack(p0);
            p0.type = (MAV_TYPE)MAV_TYPE.MAV_TYPE_ROCKET;
            p0.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED;
            p0.custom_mode = (uint)2885716531U;
            p0.system_status = (MAV_STATE)MAV_STATE.MAV_STATE_CALIBRATING;
            p0.mavlink_version = (byte)(byte)65;
            p0.autopilot = (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_AEROB;
            LoopBackDemoChannel.instance.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)60222);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)75);
                Debug.Assert(pack.load == (ushort)(ushort)28025);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)36092);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)54593);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)33799);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)21046);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH);
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)5479);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)45015);
                Debug.Assert(pack.current_battery == (short)(short) -25176);
            };
            DemoDevice.SYS_STATUS p1 = LoopBackDemoChannel.new_SYS_STATUS();
            PH.setPack(p1);
            p1.voltage_battery = (ushort)(ushort)45015;
            p1.drop_rate_comm = (ushort)(ushort)5479;
            p1.errors_comm = (ushort)(ushort)36092;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH;
            p1.errors_count3 = (ushort)(ushort)21046;
            p1.load = (ushort)(ushort)28025;
            p1.errors_count2 = (ushort)(ushort)33799;
            p1.errors_count1 = (ushort)(ushort)60222;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
            p1.battery_remaining = (sbyte)(sbyte)75;
            p1.errors_count4 = (ushort)(ushort)54593;
            p1.current_battery = (short)(short) -25176;
            LoopBackDemoChannel.instance.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_unix_usec == (ulong)3189077256325802863L);
                Debug.Assert(pack.time_boot_ms == (uint)3712192240U);
            };
            DemoDevice.SYSTEM_TIME p2 = LoopBackDemoChannel.new_SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_boot_ms = (uint)3712192240U;
            p2.time_unix_usec = (ulong)3189077256325802863L;
            LoopBackDemoChannel.instance.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)2.1974376E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2449673624U);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_NED);
                Debug.Assert(pack.afy == (float) -2.6374591E37F);
                Debug.Assert(pack.z == (float)3.0292752E38F);
                Debug.Assert(pack.vy == (float) -6.946768E37F);
                Debug.Assert(pack.yaw_rate == (float)3.3661957E37F);
                Debug.Assert(pack.x == (float)2.4181964E38F);
                Debug.Assert(pack.vx == (float)3.0429001E38F);
                Debug.Assert(pack.vz == (float) -2.919698E38F);
                Debug.Assert(pack.afz == (float) -2.2492326E38F);
                Debug.Assert(pack.y == (float) -6.796179E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)31757);
                Debug.Assert(pack.afx == (float)1.6635588E38F);
            };
            DemoDevice.POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.y = (float) -6.796179E37F;
            p3.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_NED;
            p3.afy = (float) -2.6374591E37F;
            p3.vx = (float)3.0429001E38F;
            p3.yaw = (float)2.1974376E38F;
            p3.type_mask = (ushort)(ushort)31757;
            p3.afz = (float) -2.2492326E38F;
            p3.time_boot_ms = (uint)2449673624U;
            p3.vz = (float) -2.919698E38F;
            p3.z = (float)3.0292752E38F;
            p3.vy = (float) -6.946768E37F;
            p3.afx = (float)1.6635588E38F;
            p3.yaw_rate = (float)3.3661957E37F;
            p3.x = (float)2.4181964E38F;
            LoopBackDemoChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (uint)2554365100U);
                Debug.Assert(pack.target_component == (byte)(byte)119);
                Debug.Assert(pack.target_system == (byte)(byte)150);
                Debug.Assert(pack.time_usec == (ulong)3675871137173712526L);
            };
            DemoDevice.PING p4 = LoopBackDemoChannel.new_PING();
            PH.setPack(p4);
            p4.seq = (uint)2554365100U;
            p4.time_usec = (ulong)3675871137173712526L;
            p4.target_system = (byte)(byte)150;
            p4.target_component = (byte)(byte)119;
            LoopBackDemoChannel.instance.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.control_request == (byte)(byte)42);
                Debug.Assert(pack.target_system == (byte)(byte)95);
                Debug.Assert(pack.passkey_LEN(ph) == 16);
                Debug.Assert(pack.passkey_TRY(ph).Equals("OhhRrVkJKmqxkeqZ"));
                Debug.Assert(pack.version == (byte)(byte)110);
            };
            DemoDevice.CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.version = (byte)(byte)110;
            p5.target_system = (byte)(byte)95;
            p5.passkey_SET("OhhRrVkJKmqxkeqZ", PH) ;
            p5.control_request = (byte)(byte)42;
            LoopBackDemoChannel.instance.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ack == (byte)(byte)97);
                Debug.Assert(pack.gcs_system_id == (byte)(byte)181);
                Debug.Assert(pack.control_request == (byte)(byte)167);
            };
            DemoDevice.CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.ack = (byte)(byte)97;
            p6.control_request = (byte)(byte)167;
            p6.gcs_system_id = (byte)(byte)181;
            LoopBackDemoChannel.instance.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 23);
                Debug.Assert(pack.key_TRY(ph).Equals("kLubhwrzmPHwuJqkaakdshe"));
            };
            DemoDevice.AUTH_KEY p7 = LoopBackDemoChannel.new_AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("kLubhwrzmPHwuJqkaakdshe", PH) ;
            LoopBackDemoChannel.instance.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.custom_mode == (uint)2915988349U);
                Debug.Assert(pack.base_mode == (MAV_MODE)MAV_MODE.MAV_MODE_MANUAL_DISARMED);
                Debug.Assert(pack.target_system == (byte)(byte)71);
            };
            DemoDevice.SET_MODE p11 = LoopBackDemoChannel.new_SET_MODE();
            PH.setPack(p11);
            p11.base_mode = (MAV_MODE)MAV_MODE.MAV_MODE_MANUAL_DISARMED;
            p11.custom_mode = (uint)2915988349U;
            p11.target_system = (byte)(byte)71;
            LoopBackDemoChannel.instance.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (short)(short) -27863);
                Debug.Assert(pack.param_id_LEN(ph) == 13);
                Debug.Assert(pack.param_id_TRY(ph).Equals("fqjtPrakopxbb"));
                Debug.Assert(pack.target_system == (byte)(byte)83);
                Debug.Assert(pack.target_component == (byte)(byte)253);
            };
            DemoDevice.PARAM_REQUEST_READ p20 = LoopBackDemoChannel.new_PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.param_index = (short)(short) -27863;
            p20.param_id_SET("fqjtPrakopxbb", PH) ;
            p20.target_system = (byte)(byte)83;
            p20.target_component = (byte)(byte)253;
            LoopBackDemoChannel.instance.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)199);
                Debug.Assert(pack.target_component == (byte)(byte)239);
            };
            DemoDevice.PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.new_PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_system = (byte)(byte)199;
            p21.target_component = (byte)(byte)239;
            LoopBackDemoChannel.instance.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value == (float) -1.7507113E38F);
                Debug.Assert(pack.param_index == (ushort)(ushort)14653);
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("dlubt"));
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16);
                Debug.Assert(pack.param_count == (ushort)(ushort)61694);
            };
            DemoDevice.PARAM_VALUE p22 = LoopBackDemoChannel.new_PARAM_VALUE();
            PH.setPack(p22);
            p22.param_value = (float) -1.7507113E38F;
            p22.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16;
            p22.param_index = (ushort)(ushort)14653;
            p22.param_count = (ushort)(ushort)61694;
            p22.param_id_SET("dlubt", PH) ;
            LoopBackDemoChannel.instance.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)197);
                Debug.Assert(pack.target_component == (byte)(byte)223);
                Debug.Assert(pack.param_value == (float) -1.3003077E38F);
                Debug.Assert(pack.param_id_LEN(ph) == 11);
                Debug.Assert(pack.param_id_TRY(ph).Equals("gjcozcljcem"));
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16);
            };
            DemoDevice.PARAM_SET p23 = LoopBackDemoChannel.new_PARAM_SET();
            PH.setPack(p23);
            p23.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16;
            p23.param_id_SET("gjcozcljcem", PH) ;
            p23.target_component = (byte)(byte)223;
            p23.target_system = (byte)(byte)197;
            p23.param_value = (float) -1.3003077E38F;
            LoopBackDemoChannel.instance.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)1061462338U);
                Debug.Assert(pack.eph == (ushort)(ushort)56372);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)1347074097U);
                Debug.Assert(pack.alt == (int)1149161229);
                Debug.Assert(pack.epv == (ushort)(ushort)35682);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED);
                Debug.Assert(pack.time_usec == (ulong)922970407369733967L);
                Debug.Assert(pack.lon == (int) -148911102);
                Debug.Assert(pack.vel == (ushort)(ushort)34548);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)2614455747U);
                Debug.Assert(pack.lat == (int)565382190);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int) -925785877);
                Debug.Assert(pack.satellites_visible == (byte)(byte)117);
                Debug.Assert(pack.cog == (ushort)(ushort)21985);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)3289417500U);
            };
            DemoDevice.GPS_RAW_INT p24 = LoopBackDemoChannel.new_GPS_RAW_INT();
            PH.setPack(p24);
            p24.lon = (int) -148911102;
            p24.cog = (ushort)(ushort)21985;
            p24.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED;
            p24.vel_acc_SET((uint)3289417500U, PH) ;
            p24.alt_ellipsoid_SET((int) -925785877, PH) ;
            p24.vel = (ushort)(ushort)34548;
            p24.satellites_visible = (byte)(byte)117;
            p24.v_acc_SET((uint)1061462338U, PH) ;
            p24.epv = (ushort)(ushort)35682;
            p24.alt = (int)1149161229;
            p24.h_acc_SET((uint)1347074097U, PH) ;
            p24.time_usec = (ulong)922970407369733967L;
            p24.hdg_acc_SET((uint)2614455747U, PH) ;
            p24.lat = (int)565382190;
            p24.eph = (ushort)(ushort)56372;
            LoopBackDemoChannel.instance.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)217, (byte)54, (byte)151, (byte)150, (byte)98, (byte)95, (byte)150, (byte)43, (byte)172, (byte)87, (byte)209, (byte)3, (byte)242, (byte)43, (byte)91, (byte)162, (byte)28, (byte)109, (byte)44, (byte)217}));
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)47, (byte)120, (byte)188, (byte)202, (byte)199, (byte)219, (byte)3, (byte)24, (byte)107, (byte)14, (byte)73, (byte)77, (byte)122, (byte)168, (byte)48, (byte)185, (byte)111, (byte)113, (byte)231, (byte)215}));
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)109, (byte)182, (byte)46, (byte)150, (byte)237, (byte)1, (byte)99, (byte)241, (byte)110, (byte)139, (byte)41, (byte)74, (byte)159, (byte)98, (byte)232, (byte)157, (byte)15, (byte)224, (byte)180, (byte)77}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)112);
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)115, (byte)19, (byte)150, (byte)135, (byte)100, (byte)253, (byte)82, (byte)208, (byte)157, (byte)238, (byte)204, (byte)6, (byte)136, (byte)32, (byte)223, (byte)230, (byte)240, (byte)252, (byte)64, (byte)148}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)132, (byte)69, (byte)41, (byte)59, (byte)84, (byte)59, (byte)61, (byte)6, (byte)106, (byte)46, (byte)14, (byte)123, (byte)35, (byte)235, (byte)162, (byte)226, (byte)148, (byte)235, (byte)153, (byte)168}));
            };
            DemoDevice.GPS_STATUS p25 = LoopBackDemoChannel.new_GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_used_SET(new byte[] {(byte)217, (byte)54, (byte)151, (byte)150, (byte)98, (byte)95, (byte)150, (byte)43, (byte)172, (byte)87, (byte)209, (byte)3, (byte)242, (byte)43, (byte)91, (byte)162, (byte)28, (byte)109, (byte)44, (byte)217}, 0) ;
            p25.satellites_visible = (byte)(byte)112;
            p25.satellite_snr_SET(new byte[] {(byte)109, (byte)182, (byte)46, (byte)150, (byte)237, (byte)1, (byte)99, (byte)241, (byte)110, (byte)139, (byte)41, (byte)74, (byte)159, (byte)98, (byte)232, (byte)157, (byte)15, (byte)224, (byte)180, (byte)77}, 0) ;
            p25.satellite_prn_SET(new byte[] {(byte)47, (byte)120, (byte)188, (byte)202, (byte)199, (byte)219, (byte)3, (byte)24, (byte)107, (byte)14, (byte)73, (byte)77, (byte)122, (byte)168, (byte)48, (byte)185, (byte)111, (byte)113, (byte)231, (byte)215}, 0) ;
            p25.satellite_azimuth_SET(new byte[] {(byte)132, (byte)69, (byte)41, (byte)59, (byte)84, (byte)59, (byte)61, (byte)6, (byte)106, (byte)46, (byte)14, (byte)123, (byte)35, (byte)235, (byte)162, (byte)226, (byte)148, (byte)235, (byte)153, (byte)168}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)115, (byte)19, (byte)150, (byte)135, (byte)100, (byte)253, (byte)82, (byte)208, (byte)157, (byte)238, (byte)204, (byte)6, (byte)136, (byte)32, (byte)223, (byte)230, (byte)240, (byte)252, (byte)64, (byte)148}, 0) ;
            LoopBackDemoChannel.instance.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)96394448U);
                Debug.Assert(pack.xacc == (short)(short)29287);
                Debug.Assert(pack.ygyro == (short)(short) -13580);
                Debug.Assert(pack.yacc == (short)(short)2717);
                Debug.Assert(pack.xgyro == (short)(short) -4627);
                Debug.Assert(pack.zmag == (short)(short) -29455);
                Debug.Assert(pack.xmag == (short)(short) -28104);
                Debug.Assert(pack.ymag == (short)(short) -16210);
                Debug.Assert(pack.zgyro == (short)(short)22154);
                Debug.Assert(pack.zacc == (short)(short) -17306);
            };
            DemoDevice.SCALED_IMU p26 = LoopBackDemoChannel.new_SCALED_IMU();
            PH.setPack(p26);
            p26.ymag = (short)(short) -16210;
            p26.zgyro = (short)(short)22154;
            p26.ygyro = (short)(short) -13580;
            p26.zmag = (short)(short) -29455;
            p26.xacc = (short)(short)29287;
            p26.xmag = (short)(short) -28104;
            p26.yacc = (short)(short)2717;
            p26.zacc = (short)(short) -17306;
            p26.time_boot_ms = (uint)96394448U;
            p26.xgyro = (short)(short) -4627;
            LoopBackDemoChannel.instance.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (short)(short) -1396);
                Debug.Assert(pack.xmag == (short)(short)3171);
                Debug.Assert(pack.time_usec == (ulong)5927740107283173283L);
                Debug.Assert(pack.xgyro == (short)(short) -17579);
                Debug.Assert(pack.zgyro == (short)(short)30576);
                Debug.Assert(pack.ygyro == (short)(short)27500);
                Debug.Assert(pack.zacc == (short)(short)16082);
                Debug.Assert(pack.xacc == (short)(short) -10947);
                Debug.Assert(pack.zmag == (short)(short) -23095);
                Debug.Assert(pack.ymag == (short)(short)0);
            };
            DemoDevice.RAW_IMU p27 = LoopBackDemoChannel.new_RAW_IMU();
            PH.setPack(p27);
            p27.zgyro = (short)(short)30576;
            p27.zmag = (short)(short) -23095;
            p27.ygyro = (short)(short)27500;
            p27.xacc = (short)(short) -10947;
            p27.yacc = (short)(short) -1396;
            p27.zacc = (short)(short)16082;
            p27.ymag = (short)(short)0;
            p27.xmag = (short)(short)3171;
            p27.time_usec = (ulong)5927740107283173283L;
            p27.xgyro = (short)(short) -17579;
            LoopBackDemoChannel.instance.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)4131550303904474299L);
                Debug.Assert(pack.press_abs == (short)(short)13030);
                Debug.Assert(pack.press_diff1 == (short)(short) -19913);
                Debug.Assert(pack.press_diff2 == (short)(short) -17459);
                Debug.Assert(pack.temperature == (short)(short)5147);
            };
            DemoDevice.RAW_PRESSURE p28 = LoopBackDemoChannel.new_RAW_PRESSURE();
            PH.setPack(p28);
            p28.time_usec = (ulong)4131550303904474299L;
            p28.press_diff2 = (short)(short) -17459;
            p28.press_diff1 = (short)(short) -19913;
            p28.temperature = (short)(short)5147;
            p28.press_abs = (short)(short)13030;
            LoopBackDemoChannel.instance.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -25332);
                Debug.Assert(pack.time_boot_ms == (uint)1021509803U);
                Debug.Assert(pack.press_abs == (float) -4.4214398E36F);
                Debug.Assert(pack.press_diff == (float) -2.2405548E38F);
            };
            DemoDevice.SCALED_PRESSURE p29 = LoopBackDemoChannel.new_SCALED_PRESSURE();
            PH.setPack(p29);
            p29.time_boot_ms = (uint)1021509803U;
            p29.press_abs = (float) -4.4214398E36F;
            p29.temperature = (short)(short) -25332;
            p29.press_diff = (float) -2.2405548E38F;
            LoopBackDemoChannel.instance.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float) -1.3334935E38F);
                Debug.Assert(pack.pitch == (float) -1.908941E38F);
                Debug.Assert(pack.roll == (float) -1.3489822E38F);
                Debug.Assert(pack.pitchspeed == (float) -6.645875E36F);
                Debug.Assert(pack.rollspeed == (float)4.336518E37F);
                Debug.Assert(pack.yawspeed == (float)1.2915233E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1948895708U);
            };
            DemoDevice.ATTITUDE p30 = LoopBackDemoChannel.new_ATTITUDE();
            PH.setPack(p30);
            p30.time_boot_ms = (uint)1948895708U;
            p30.pitchspeed = (float) -6.645875E36F;
            p30.yawspeed = (float)1.2915233E38F;
            p30.roll = (float) -1.3489822E38F;
            p30.rollspeed = (float)4.336518E37F;
            p30.yaw = (float) -1.3334935E38F;
            p30.pitch = (float) -1.908941E38F;
            LoopBackDemoChannel.instance.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q1 == (float)1.9970953E38F);
                Debug.Assert(pack.q3 == (float)1.9005065E37F);
                Debug.Assert(pack.yawspeed == (float) -1.7822088E38F);
                Debug.Assert(pack.rollspeed == (float) -1.9700872E38F);
                Debug.Assert(pack.q4 == (float) -7.5407803E37F);
                Debug.Assert(pack.pitchspeed == (float)2.8617817E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3207518669U);
                Debug.Assert(pack.q2 == (float) -1.6004706E38F);
            };
            DemoDevice.ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.q2 = (float) -1.6004706E38F;
            p31.yawspeed = (float) -1.7822088E38F;
            p31.pitchspeed = (float)2.8617817E38F;
            p31.rollspeed = (float) -1.9700872E38F;
            p31.q4 = (float) -7.5407803E37F;
            p31.time_boot_ms = (uint)3207518669U;
            p31.q1 = (float)1.9970953E38F;
            p31.q3 = (float)1.9005065E37F;
            LoopBackDemoChannel.instance.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float)2.6597664E38F);
                Debug.Assert(pack.x == (float) -3.0006075E38F);
                Debug.Assert(pack.vz == (float) -1.5501938E38F);
                Debug.Assert(pack.vx == (float) -3.3106342E38F);
                Debug.Assert(pack.y == (float) -7.227363E37F);
                Debug.Assert(pack.z == (float) -1.1122698E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2397798233U);
            };
            DemoDevice.LOCAL_POSITION_NED p32 = LoopBackDemoChannel.new_LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.vx = (float) -3.3106342E38F;
            p32.vz = (float) -1.5501938E38F;
            p32.y = (float) -7.227363E37F;
            p32.x = (float) -3.0006075E38F;
            p32.z = (float) -1.1122698E38F;
            p32.time_boot_ms = (uint)2397798233U;
            p32.vy = (float)2.6597664E38F;
            LoopBackDemoChannel.instance.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (short)(short) -25143);
                Debug.Assert(pack.time_boot_ms == (uint)2024388513U);
                Debug.Assert(pack.lat == (int) -113265022);
                Debug.Assert(pack.alt == (int)740387229);
                Debug.Assert(pack.vz == (short)(short) -28546);
                Debug.Assert(pack.hdg == (ushort)(ushort)13553);
                Debug.Assert(pack.vy == (short)(short) -17425);
                Debug.Assert(pack.relative_alt == (int)1180865313);
                Debug.Assert(pack.lon == (int)2087051982);
            };
            DemoDevice.GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.alt = (int)740387229;
            p33.vy = (short)(short) -17425;
            p33.vx = (short)(short) -25143;
            p33.lon = (int)2087051982;
            p33.relative_alt = (int)1180865313;
            p33.time_boot_ms = (uint)2024388513U;
            p33.vz = (short)(short) -28546;
            p33.hdg = (ushort)(ushort)13553;
            p33.lat = (int) -113265022;
            LoopBackDemoChannel.instance.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan8_scaled == (short)(short) -6695);
                Debug.Assert(pack.rssi == (byte)(byte)62);
                Debug.Assert(pack.chan5_scaled == (short)(short)14933);
                Debug.Assert(pack.chan1_scaled == (short)(short)1701);
                Debug.Assert(pack.chan4_scaled == (short)(short)14278);
                Debug.Assert(pack.chan7_scaled == (short)(short)23127);
                Debug.Assert(pack.chan3_scaled == (short)(short)21640);
                Debug.Assert(pack.chan2_scaled == (short)(short)28247);
                Debug.Assert(pack.time_boot_ms == (uint)3362963738U);
                Debug.Assert(pack.port == (byte)(byte)23);
                Debug.Assert(pack.chan6_scaled == (short)(short)723);
            };
            DemoDevice.RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.new_RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.chan6_scaled = (short)(short)723;
            p34.chan3_scaled = (short)(short)21640;
            p34.chan7_scaled = (short)(short)23127;
            p34.chan4_scaled = (short)(short)14278;
            p34.chan2_scaled = (short)(short)28247;
            p34.port = (byte)(byte)23;
            p34.time_boot_ms = (uint)3362963738U;
            p34.chan5_scaled = (short)(short)14933;
            p34.rssi = (byte)(byte)62;
            p34.chan8_scaled = (short)(short) -6695;
            p34.chan1_scaled = (short)(short)1701;
            LoopBackDemoChannel.instance.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2589821430U);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)40281);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)33761);
                Debug.Assert(pack.rssi == (byte)(byte)76);
                Debug.Assert(pack.port == (byte)(byte)78);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)35962);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)40390);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)58779);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)62110);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)53550);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)41077);
            };
            DemoDevice.RC_CHANNELS_RAW p35 = LoopBackDemoChannel.new_RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.chan4_raw = (ushort)(ushort)40390;
            p35.time_boot_ms = (uint)2589821430U;
            p35.port = (byte)(byte)78;
            p35.chan3_raw = (ushort)(ushort)33761;
            p35.chan1_raw = (ushort)(ushort)58779;
            p35.chan5_raw = (ushort)(ushort)53550;
            p35.chan7_raw = (ushort)(ushort)41077;
            p35.rssi = (byte)(byte)76;
            p35.chan2_raw = (ushort)(ushort)35962;
            p35.chan6_raw = (ushort)(ushort)40281;
            p35.chan8_raw = (ushort)(ushort)62110;
            LoopBackDemoChannel.instance.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)56161);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)62696);
                Debug.Assert(pack.time_usec == (uint)3699147489U);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)15225);
                Debug.Assert(pack.port == (byte)(byte)88);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)16422);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)19124);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)10832);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)6585);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)19301);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)20560);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)57256);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)18836);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)3659);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)21654);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)53912);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)1701);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)62044);
            };
            DemoDevice.SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.new_SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo13_raw_SET((ushort)(ushort)1701, PH) ;
            p36.servo4_raw = (ushort)(ushort)15225;
            p36.time_usec = (uint)3699147489U;
            p36.servo8_raw = (ushort)(ushort)3659;
            p36.servo14_raw_SET((ushort)(ushort)53912, PH) ;
            p36.servo9_raw_SET((ushort)(ushort)20560, PH) ;
            p36.servo7_raw = (ushort)(ushort)16422;
            p36.port = (byte)(byte)88;
            p36.servo5_raw = (ushort)(ushort)62044;
            p36.servo6_raw = (ushort)(ushort)10832;
            p36.servo2_raw = (ushort)(ushort)21654;
            p36.servo11_raw_SET((ushort)(ushort)19124, PH) ;
            p36.servo1_raw = (ushort)(ushort)19301;
            p36.servo16_raw_SET((ushort)(ushort)62696, PH) ;
            p36.servo15_raw_SET((ushort)(ushort)6585, PH) ;
            p36.servo3_raw = (ushort)(ushort)57256;
            p36.servo12_raw_SET((ushort)(ushort)18836, PH) ;
            p36.servo10_raw_SET((ushort)(ushort)56161, PH) ;
            LoopBackDemoChannel.instance.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.end_index == (short)(short) -3159);
                Debug.Assert(pack.target_component == (byte)(byte)224);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_system == (byte)(byte)237);
                Debug.Assert(pack.start_index == (short)(short) -7142);
            };
            DemoDevice.MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.new_MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.target_system = (byte)(byte)237;
            p37.target_component = (byte)(byte)224;
            p37.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p37.end_index = (short)(short) -3159;
            p37.start_index = (short)(short) -7142;
            LoopBackDemoChannel.instance.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)36);
                Debug.Assert(pack.end_index == (short)(short) -12257);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_system == (byte)(byte)170);
                Debug.Assert(pack.start_index == (short)(short) -18876);
            };
            DemoDevice.MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.new_MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.start_index = (short)(short) -18876;
            p38.end_index = (short)(short) -12257;
            p38.target_system = (byte)(byte)170;
            p38.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p38.target_component = (byte)(byte)36;
            LoopBackDemoChannel.instance.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current == (byte)(byte)73);
                Debug.Assert(pack.x == (float) -1.4710979E38F);
                Debug.Assert(pack.param2 == (float) -4.395292E37F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED);
                Debug.Assert(pack.seq == (ushort)(ushort)60110);
                Debug.Assert(pack.autocontinue == (byte)(byte)225);
                Debug.Assert(pack.param3 == (float) -1.7459577E38F);
                Debug.Assert(pack.y == (float) -2.524264E37F);
                Debug.Assert(pack.param4 == (float)3.8620408E37F);
                Debug.Assert(pack.param1 == (float) -3.8982596E37F);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_system == (byte)(byte)195);
                Debug.Assert(pack.target_component == (byte)(byte)80);
                Debug.Assert(pack.z == (float)1.7304958E38F);
            };
            DemoDevice.MISSION_ITEM p39 = LoopBackDemoChannel.new_MISSION_ITEM();
            PH.setPack(p39);
            p39.x = (float) -1.4710979E38F;
            p39.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p39.target_system = (byte)(byte)195;
            p39.command = (MAV_CMD)MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION;
            p39.param4 = (float)3.8620408E37F;
            p39.seq = (ushort)(ushort)60110;
            p39.param1 = (float) -3.8982596E37F;
            p39.y = (float) -2.524264E37F;
            p39.target_component = (byte)(byte)80;
            p39.current = (byte)(byte)73;
            p39.autocontinue = (byte)(byte)225;
            p39.param2 = (float) -4.395292E37F;
            p39.z = (float)1.7304958E38F;
            p39.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p39.param3 = (float) -1.7459577E38F;
            LoopBackDemoChannel.instance.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.seq == (ushort)(ushort)35782);
                Debug.Assert(pack.target_system == (byte)(byte)206);
                Debug.Assert(pack.target_component == (byte)(byte)26);
            };
            DemoDevice.MISSION_REQUEST p40 = LoopBackDemoChannel.new_MISSION_REQUEST();
            PH.setPack(p40);
            p40.target_component = (byte)(byte)26;
            p40.seq = (ushort)(ushort)35782;
            p40.target_system = (byte)(byte)206;
            p40.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            LoopBackDemoChannel.instance.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)62633);
                Debug.Assert(pack.target_system == (byte)(byte)82);
                Debug.Assert(pack.target_component == (byte)(byte)200);
            };
            DemoDevice.MISSION_SET_CURRENT p41 = LoopBackDemoChannel.new_MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.seq = (ushort)(ushort)62633;
            p41.target_component = (byte)(byte)200;
            p41.target_system = (byte)(byte)82;
            LoopBackDemoChannel.instance.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)60329);
            };
            DemoDevice.MISSION_CURRENT p42 = LoopBackDemoChannel.new_MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)60329;
            LoopBackDemoChannel.instance.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)135);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)161);
            };
            DemoDevice.MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.new_MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.target_component = (byte)(byte)161;
            p43.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p43.target_system = (byte)(byte)135;
            LoopBackDemoChannel.instance.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (ushort)(ushort)60079);
                Debug.Assert(pack.target_system == (byte)(byte)9);
                Debug.Assert(pack.target_component == (byte)(byte)110);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            };
            DemoDevice.MISSION_COUNT p44 = LoopBackDemoChannel.new_MISSION_COUNT();
            PH.setPack(p44);
            p44.target_system = (byte)(byte)9;
            p44.count = (ushort)(ushort)60079;
            p44.target_component = (byte)(byte)110;
            p44.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            LoopBackDemoChannel.instance.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)144);
                Debug.Assert(pack.target_component == (byte)(byte)31);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            };
            DemoDevice.MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.new_MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_component = (byte)(byte)31;
            p45.target_system = (byte)(byte)144;
            p45.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            LoopBackDemoChannel.instance.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)51238);
            };
            DemoDevice.MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.new_MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)51238;
            LoopBackDemoChannel.instance.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)216);
                Debug.Assert(pack.target_system == (byte)(byte)15);
                Debug.Assert(pack.type == (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED_FRAME);
            };
            DemoDevice.MISSION_ACK p47 = LoopBackDemoChannel.new_MISSION_ACK();
            PH.setPack(p47);
            p47.target_component = (byte)(byte)216;
            p47.target_system = (byte)(byte)15;
            p47.type = (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED_FRAME;
            p47.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            LoopBackDemoChannel.instance.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.latitude == (int)2122196540);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)881364319738279586L);
                Debug.Assert(pack.longitude == (int)960706227);
                Debug.Assert(pack.target_system == (byte)(byte)32);
                Debug.Assert(pack.altitude == (int) -907953532);
            };
            DemoDevice.SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.new_SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.time_usec_SET((ulong)881364319738279586L, PH) ;
            p48.altitude = (int) -907953532;
            p48.target_system = (byte)(byte)32;
            p48.longitude = (int)960706227;
            p48.latitude = (int)2122196540;
            LoopBackDemoChannel.instance.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.latitude == (int) -164890682);
                Debug.Assert(pack.altitude == (int) -1081850891);
                Debug.Assert(pack.longitude == (int)1730363380);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)6113163421679194689L);
            };
            DemoDevice.GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.new_GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.time_usec_SET((ulong)6113163421679194689L, PH) ;
            p49.altitude = (int) -1081850891;
            p49.latitude = (int) -164890682;
            p49.longitude = (int)1730363380;
            LoopBackDemoChannel.instance.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)127);
                Debug.Assert(pack.param_value0 == (float) -2.4852073E38F);
                Debug.Assert(pack.param_id_LEN(ph) == 15);
                Debug.Assert(pack.param_id_TRY(ph).Equals("BqghqnwpjteLeTv"));
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)11);
                Debug.Assert(pack.param_value_min == (float) -4.1394857E37F);
                Debug.Assert(pack.target_system == (byte)(byte)134);
                Debug.Assert(pack.param_value_max == (float) -3.809224E37F);
                Debug.Assert(pack.scale == (float)2.0401339E37F);
                Debug.Assert(pack.param_index == (short)(short) -10143);
            };
            DemoDevice.PARAM_MAP_RC p50 = LoopBackDemoChannel.new_PARAM_MAP_RC();
            PH.setPack(p50);
            p50.parameter_rc_channel_index = (byte)(byte)11;
            p50.param_index = (short)(short) -10143;
            p50.target_system = (byte)(byte)134;
            p50.scale = (float)2.0401339E37F;
            p50.param_value_min = (float) -4.1394857E37F;
            p50.target_component = (byte)(byte)127;
            p50.param_id_SET("BqghqnwpjteLeTv", PH) ;
            p50.param_value0 = (float) -2.4852073E38F;
            p50.param_value_max = (float) -3.809224E37F;
            LoopBackDemoChannel.instance.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)86);
                Debug.Assert(pack.seq == (ushort)(ushort)34619);
                Debug.Assert(pack.target_system == (byte)(byte)69);
            };
            DemoDevice.MISSION_REQUEST_INT p51 = LoopBackDemoChannel.new_MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.target_component = (byte)(byte)86;
            p51.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p51.seq = (ushort)(ushort)34619;
            p51.target_system = (byte)(byte)69;
            LoopBackDemoChannel.instance.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p1z == (float) -2.2022943E38F);
                Debug.Assert(pack.p2x == (float)1.4517025E38F);
                Debug.Assert(pack.p1y == (float)1.3007944E38F);
                Debug.Assert(pack.p1x == (float)1.5730714E38F);
                Debug.Assert(pack.p2z == (float)1.6398541E38F);
                Debug.Assert(pack.p2y == (float) -1.1450595E38F);
                Debug.Assert(pack.target_component == (byte)(byte)97);
                Debug.Assert(pack.target_system == (byte)(byte)83);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            };
            DemoDevice.SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.new_SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.target_component = (byte)(byte)97;
            p54.p2y = (float) -1.1450595E38F;
            p54.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p54.p2z = (float)1.6398541E38F;
            p54.p1x = (float)1.5730714E38F;
            p54.target_system = (byte)(byte)83;
            p54.p1y = (float)1.3007944E38F;
            p54.p2x = (float)1.4517025E38F;
            p54.p1z = (float) -2.2022943E38F;
            LoopBackDemoChannel.instance.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p2z == (float) -2.0687603E38F);
                Debug.Assert(pack.p2y == (float) -1.7973424E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
                Debug.Assert(pack.p1x == (float) -1.0587857E38F);
                Debug.Assert(pack.p1z == (float)7.426446E37F);
                Debug.Assert(pack.p2x == (float) -9.217029E37F);
                Debug.Assert(pack.p1y == (float)1.9744212E38F);
            };
            DemoDevice.SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.new_SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p1z = (float)7.426446E37F;
            p55.p1x = (float) -1.0587857E38F;
            p55.p2z = (float) -2.0687603E38F;
            p55.p2y = (float) -1.7973424E38F;
            p55.p2x = (float) -9.217029E37F;
            p55.p1y = (float)1.9744212E38F;
            p55.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            LoopBackDemoChannel.instance.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitchspeed == (float) -7.8666487E37F);
                Debug.Assert(pack.rollspeed == (float)2.6432856E38F);
                Debug.Assert(pack.time_usec == (ulong)5212363291501402134L);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {2.59972E37F, -2.652494E37F, 3.2597804E38F, 2.1647769E38F, -2.6805602E38F, 2.0978409E38F, -1.8760241E38F, 2.7853066E38F, 2.8265063E38F}));
                Debug.Assert(pack.yawspeed == (float)3.0326358E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.242826E38F, 3.0017982E38F, 1.0417427E38F, -1.5675611E38F}));
            };
            DemoDevice.ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.time_usec = (ulong)5212363291501402134L;
            p61.rollspeed = (float)2.6432856E38F;
            p61.yawspeed = (float)3.0326358E38F;
            p61.covariance_SET(new float[] {2.59972E37F, -2.652494E37F, 3.2597804E38F, 2.1647769E38F, -2.6805602E38F, 2.0978409E38F, -1.8760241E38F, 2.7853066E38F, 2.8265063E38F}, 0) ;
            p61.q_SET(new float[] {1.242826E38F, 3.0017982E38F, 1.0417427E38F, -1.5675611E38F}, 0) ;
            p61.pitchspeed = (float) -7.8666487E37F;
            LoopBackDemoChannel.instance.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.nav_bearing == (short)(short)23189);
                Debug.Assert(pack.nav_pitch == (float)2.6305393E38F);
                Debug.Assert(pack.alt_error == (float) -2.7090782E38F);
                Debug.Assert(pack.xtrack_error == (float)2.5804014E38F);
                Debug.Assert(pack.target_bearing == (short)(short)3868);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)61084);
                Debug.Assert(pack.aspd_error == (float) -1.5430565E38F);
                Debug.Assert(pack.nav_roll == (float) -4.36799E37F);
            };
            DemoDevice.NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.new_NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.alt_error = (float) -2.7090782E38F;
            p62.nav_pitch = (float)2.6305393E38F;
            p62.target_bearing = (short)(short)3868;
            p62.wp_dist = (ushort)(ushort)61084;
            p62.nav_roll = (float) -4.36799E37F;
            p62.xtrack_error = (float)2.5804014E38F;
            p62.nav_bearing = (short)(short)23189;
            p62.aspd_error = (float) -1.5430565E38F;
            LoopBackDemoChannel.instance.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vz == (float)1.7316334E38F);
                Debug.Assert(pack.vx == (float) -2.3650683E38F);
                Debug.Assert(pack.vy == (float)1.5428241E38F);
                Debug.Assert(pack.time_usec == (ulong)608529818532108460L);
                Debug.Assert(pack.relative_alt == (int) -811547351);
                Debug.Assert(pack.alt == (int)736639475);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
                Debug.Assert(pack.lat == (int)310419004);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {1.6248693E38F, -2.1966727E37F, -2.9651923E38F, 2.3255841E38F, 2.533751E38F, -2.814343E38F, -1.3450385E37F, -1.2491824E38F, -1.2380395E38F, 2.0465661E38F, -2.8821793E38F, 1.9153184E38F, 2.2297633E38F, 3.1755068E37F, -8.7079E37F, -2.2077257E38F, -2.060028E37F, 1.4857323E38F, 5.655592E37F, 1.6977501E38F, -1.5527146E38F, -1.1802451E38F, -2.7069823E38F, -3.4968603E37F, 2.114165E38F, 2.7528323E38F, -3.305452E38F, 2.8113535E38F, 1.3315329E38F, -2.7337127E38F, -1.2190411E38F, 2.1386988E38F, -1.8927575E38F, 1.9096075E38F, -2.6257486E38F, -2.996272E38F}));
                Debug.Assert(pack.lon == (int)894509874);
            };
            DemoDevice.GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.relative_alt = (int) -811547351;
            p63.time_usec = (ulong)608529818532108460L;
            p63.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS;
            p63.covariance_SET(new float[] {1.6248693E38F, -2.1966727E37F, -2.9651923E38F, 2.3255841E38F, 2.533751E38F, -2.814343E38F, -1.3450385E37F, -1.2491824E38F, -1.2380395E38F, 2.0465661E38F, -2.8821793E38F, 1.9153184E38F, 2.2297633E38F, 3.1755068E37F, -8.7079E37F, -2.2077257E38F, -2.060028E37F, 1.4857323E38F, 5.655592E37F, 1.6977501E38F, -1.5527146E38F, -1.1802451E38F, -2.7069823E38F, -3.4968603E37F, 2.114165E38F, 2.7528323E38F, -3.305452E38F, 2.8113535E38F, 1.3315329E38F, -2.7337127E38F, -1.2190411E38F, 2.1386988E38F, -1.8927575E38F, 1.9096075E38F, -2.6257486E38F, -2.996272E38F}, 0) ;
            p63.vy = (float)1.5428241E38F;
            p63.alt = (int)736639475;
            p63.lat = (int)310419004;
            p63.lon = (int)894509874;
            p63.vx = (float) -2.3650683E38F;
            p63.vz = (float)1.7316334E38F;
            LoopBackDemoChannel.instance.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -5.468449E37F);
                Debug.Assert(pack.az == (float)5.3418593E36F);
                Debug.Assert(pack.time_usec == (ulong)8483518935090290137L);
                Debug.Assert(pack.x == (float) -1.8284523E38F);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
                Debug.Assert(pack.vz == (float)2.747706E38F);
                Debug.Assert(pack.vy == (float) -2.605024E38F);
                Debug.Assert(pack.y == (float)2.4908294E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-1.8278631E38F, 1.3292887E38F, 2.0883436E38F, 3.0443426E38F, 9.453006E37F, 2.6073068E38F, -2.006068E38F, -2.2461675E38F, 1.0496373E38F, -1.4365214E38F, -1.0810581E38F, 2.3708618E38F, -1.631904E38F, 2.37773E38F, -2.1164494E38F, -6.2337306E37F, -3.6847634E37F, -2.6991723E38F, -1.0524083E38F, -3.1065804E38F, 7.8645175E37F, -2.9212922E37F, 1.1348167E38F, 2.7354784E38F, 2.436095E38F, 2.5545409E38F, 2.9236907E38F, -3.1052383E38F, -2.0042803E38F, -3.7452075E37F, 3.4541694E37F, 5.6132263E37F, -6.27995E37F, 1.6152469E38F, -5.114841E37F, -1.0364733E38F, 2.224657E38F, -1.5234911E38F, -8.0454995E37F, -1.3211807E38F, 1.4958792E38F, 4.8180676E37F, 2.7245817E38F, -8.285494E37F, 2.2044458E38F}));
                Debug.Assert(pack.ax == (float)9.629605E37F);
                Debug.Assert(pack.ay == (float) -1.6755898E38F);
                Debug.Assert(pack.vx == (float) -2.4611367E38F);
            };
            DemoDevice.LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.time_usec = (ulong)8483518935090290137L;
            p64.ay = (float) -1.6755898E38F;
            p64.z = (float) -5.468449E37F;
            p64.covariance_SET(new float[] {-1.8278631E38F, 1.3292887E38F, 2.0883436E38F, 3.0443426E38F, 9.453006E37F, 2.6073068E38F, -2.006068E38F, -2.2461675E38F, 1.0496373E38F, -1.4365214E38F, -1.0810581E38F, 2.3708618E38F, -1.631904E38F, 2.37773E38F, -2.1164494E38F, -6.2337306E37F, -3.6847634E37F, -2.6991723E38F, -1.0524083E38F, -3.1065804E38F, 7.8645175E37F, -2.9212922E37F, 1.1348167E38F, 2.7354784E38F, 2.436095E38F, 2.5545409E38F, 2.9236907E38F, -3.1052383E38F, -2.0042803E38F, -3.7452075E37F, 3.4541694E37F, 5.6132263E37F, -6.27995E37F, 1.6152469E38F, -5.114841E37F, -1.0364733E38F, 2.224657E38F, -1.5234911E38F, -8.0454995E37F, -1.3211807E38F, 1.4958792E38F, 4.8180676E37F, 2.7245817E38F, -8.285494E37F, 2.2044458E38F}, 0) ;
            p64.ax = (float)9.629605E37F;
            p64.x = (float) -1.8284523E38F;
            p64.y = (float)2.4908294E38F;
            p64.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO;
            p64.vy = (float) -2.605024E38F;
            p64.vx = (float) -2.4611367E38F;
            p64.az = (float)5.3418593E36F;
            p64.vz = (float)2.747706E38F;
            LoopBackDemoChannel.instance.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)14679);
                Debug.Assert(pack.chancount == (byte)(byte)191);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)5463);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)57405);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)21306);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)44624);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)63705);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)31087);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)55045);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)65104);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)59028);
                Debug.Assert(pack.time_boot_ms == (uint)1371073016U);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)62213);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)2550);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)17554);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)19530);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)13341);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)9566);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)960);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)31044);
                Debug.Assert(pack.rssi == (byte)(byte)162);
            };
            DemoDevice.RC_CHANNELS p65 = LoopBackDemoChannel.new_RC_CHANNELS();
            PH.setPack(p65);
            p65.chan1_raw = (ushort)(ushort)31087;
            p65.chan4_raw = (ushort)(ushort)55045;
            p65.chan6_raw = (ushort)(ushort)57405;
            p65.chan8_raw = (ushort)(ushort)19530;
            p65.chan15_raw = (ushort)(ushort)63705;
            p65.chan14_raw = (ushort)(ushort)960;
            p65.chan5_raw = (ushort)(ushort)2550;
            p65.chan18_raw = (ushort)(ushort)13341;
            p65.chan7_raw = (ushort)(ushort)14679;
            p65.chan10_raw = (ushort)(ushort)5463;
            p65.chancount = (byte)(byte)191;
            p65.chan9_raw = (ushort)(ushort)21306;
            p65.time_boot_ms = (uint)1371073016U;
            p65.rssi = (byte)(byte)162;
            p65.chan12_raw = (ushort)(ushort)65104;
            p65.chan16_raw = (ushort)(ushort)9566;
            p65.chan17_raw = (ushort)(ushort)62213;
            p65.chan13_raw = (ushort)(ushort)17554;
            p65.chan2_raw = (ushort)(ushort)59028;
            p65.chan3_raw = (ushort)(ushort)31044;
            p65.chan11_raw = (ushort)(ushort)44624;
            LoopBackDemoChannel.instance.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)126);
                Debug.Assert(pack.req_stream_id == (byte)(byte)148);
                Debug.Assert(pack.target_system == (byte)(byte)50);
                Debug.Assert(pack.start_stop == (byte)(byte)22);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)51478);
            };
            DemoDevice.REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.new_REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.target_component = (byte)(byte)126;
            p66.target_system = (byte)(byte)50;
            p66.start_stop = (byte)(byte)22;
            p66.req_message_rate = (ushort)(ushort)51478;
            p66.req_stream_id = (byte)(byte)148;
            LoopBackDemoChannel.instance.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_rate == (ushort)(ushort)47829);
                Debug.Assert(pack.stream_id == (byte)(byte)109);
                Debug.Assert(pack.on_off == (byte)(byte)205);
            };
            DemoDevice.DATA_STREAM p67 = LoopBackDemoChannel.new_DATA_STREAM();
            PH.setPack(p67);
            p67.message_rate = (ushort)(ushort)47829;
            p67.on_off = (byte)(byte)205;
            p67.stream_id = (byte)(byte)109;
            LoopBackDemoChannel.instance.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (short)(short) -11729);
                Debug.Assert(pack.target == (byte)(byte)117);
                Debug.Assert(pack.x == (short)(short) -19441);
                Debug.Assert(pack.z == (short)(short)28238);
                Debug.Assert(pack.r == (short)(short) -20464);
                Debug.Assert(pack.buttons == (ushort)(ushort)42556);
            };
            DemoDevice.MANUAL_CONTROL p69 = LoopBackDemoChannel.new_MANUAL_CONTROL();
            PH.setPack(p69);
            p69.buttons = (ushort)(ushort)42556;
            p69.x = (short)(short) -19441;
            p69.target = (byte)(byte)117;
            p69.y = (short)(short) -11729;
            p69.r = (short)(short) -20464;
            p69.z = (short)(short)28238;
            LoopBackDemoChannel.instance.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)18001);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)26329);
                Debug.Assert(pack.target_component == (byte)(byte)179);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)637);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)48213);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)49183);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)43917);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)1820);
                Debug.Assert(pack.target_system == (byte)(byte)32);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)22681);
            };
            DemoDevice.RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.new_RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan5_raw = (ushort)(ushort)637;
            p70.chan2_raw = (ushort)(ushort)43917;
            p70.chan6_raw = (ushort)(ushort)49183;
            p70.target_component = (byte)(byte)179;
            p70.target_system = (byte)(byte)32;
            p70.chan7_raw = (ushort)(ushort)22681;
            p70.chan3_raw = (ushort)(ushort)48213;
            p70.chan4_raw = (ushort)(ushort)18001;
            p70.chan8_raw = (ushort)(ushort)1820;
            p70.chan1_raw = (ushort)(ushort)26329;
            LoopBackDemoChannel.instance.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)69);
                Debug.Assert(pack.seq == (ushort)(ushort)46140);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.param1 == (float)1.4014832E38F);
                Debug.Assert(pack.param2 == (float) -1.9457798E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)67);
                Debug.Assert(pack.x == (int) -773577566);
                Debug.Assert(pack.param3 == (float)3.2170585E38F);
                Debug.Assert(pack.param4 == (float)2.7300413E38F);
                Debug.Assert(pack.target_system == (byte)(byte)6);
                Debug.Assert(pack.z == (float)1.2832564E38F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION);
                Debug.Assert(pack.current == (byte)(byte)125);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.y == (int) -2017153317);
            };
            DemoDevice.MISSION_ITEM_INT p73 = LoopBackDemoChannel.new_MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p73.target_component = (byte)(byte)69;
            p73.command = (MAV_CMD)MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION;
            p73.current = (byte)(byte)125;
            p73.param1 = (float)1.4014832E38F;
            p73.param3 = (float)3.2170585E38F;
            p73.target_system = (byte)(byte)6;
            p73.param4 = (float)2.7300413E38F;
            p73.x = (int) -773577566;
            p73.z = (float)1.2832564E38F;
            p73.autocontinue = (byte)(byte)67;
            p73.param2 = (float) -1.9457798E38F;
            p73.y = (int) -2017153317;
            p73.seq = (ushort)(ushort)46140;
            p73.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            LoopBackDemoChannel.instance.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.climb == (float)3.0675755E38F);
                Debug.Assert(pack.airspeed == (float) -2.7460133E38F);
                Debug.Assert(pack.heading == (short)(short) -30128);
                Debug.Assert(pack.alt == (float)2.3225982E38F);
                Debug.Assert(pack.groundspeed == (float)6.7481245E37F);
                Debug.Assert(pack.throttle == (ushort)(ushort)47617);
            };
            DemoDevice.VFR_HUD p74 = LoopBackDemoChannel.new_VFR_HUD();
            PH.setPack(p74);
            p74.airspeed = (float) -2.7460133E38F;
            p74.alt = (float)2.3225982E38F;
            p74.throttle = (ushort)(ushort)47617;
            p74.groundspeed = (float)6.7481245E37F;
            p74.heading = (short)(short) -30128;
            p74.climb = (float)3.0675755E38F;
            LoopBackDemoChannel.instance.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param3 == (float) -1.5614579E37F);
                Debug.Assert(pack.z == (float) -9.608216E37F);
                Debug.Assert(pack.param2 == (float) -1.9287284E38F);
                Debug.Assert(pack.param4 == (float) -2.0961874E38F);
                Debug.Assert(pack.x == (int) -1606691836);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION);
                Debug.Assert(pack.target_system == (byte)(byte)239);
                Debug.Assert(pack.y == (int)863144170);
                Debug.Assert(pack.autocontinue == (byte)(byte)206);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
                Debug.Assert(pack.target_component == (byte)(byte)28);
                Debug.Assert(pack.param1 == (float) -1.8502145E38F);
                Debug.Assert(pack.current == (byte)(byte)204);
            };
            DemoDevice.COMMAND_INT p75 = LoopBackDemoChannel.new_COMMAND_INT();
            PH.setPack(p75);
            p75.target_system = (byte)(byte)239;
            p75.target_component = (byte)(byte)28;
            p75.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p75.param2 = (float) -1.9287284E38F;
            p75.param3 = (float) -1.5614579E37F;
            p75.current = (byte)(byte)204;
            p75.param1 = (float) -1.8502145E38F;
            p75.z = (float) -9.608216E37F;
            p75.x = (int) -1606691836;
            p75.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION;
            p75.y = (int)863144170;
            p75.param4 = (float) -2.0961874E38F;
            p75.autocontinue = (byte)(byte)206;
            LoopBackDemoChannel.instance.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param3 == (float)1.7256832E38F);
                Debug.Assert(pack.param6 == (float)2.4762167E38F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_FOLLOW);
                Debug.Assert(pack.confirmation == (byte)(byte)160);
                Debug.Assert(pack.target_system == (byte)(byte)72);
                Debug.Assert(pack.param5 == (float)1.0512811E38F);
                Debug.Assert(pack.target_component == (byte)(byte)154);
                Debug.Assert(pack.param1 == (float) -1.6233353E38F);
                Debug.Assert(pack.param2 == (float) -1.7935713E38F);
                Debug.Assert(pack.param4 == (float) -2.7840883E38F);
                Debug.Assert(pack.param7 == (float)7.001115E37F);
            };
            DemoDevice.COMMAND_LONG p76 = LoopBackDemoChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.param1 = (float) -1.6233353E38F;
            p76.param6 = (float)2.4762167E38F;
            p76.param3 = (float)1.7256832E38F;
            p76.target_system = (byte)(byte)72;
            p76.param2 = (float) -1.7935713E38F;
            p76.param7 = (float)7.001115E37F;
            p76.param4 = (float) -2.7840883E38F;
            p76.target_component = (byte)(byte)154;
            p76.param5 = (float)1.0512811E38F;
            p76.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_FOLLOW;
            p76.confirmation = (byte)(byte)160;
            LoopBackDemoChannel.instance.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)76);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)232);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)61);
                Debug.Assert(pack.result == (MAV_RESULT)MAV_RESULT.MAV_RESULT_IN_PROGRESS);
                Debug.Assert(pack.result_param2_TRY(ph) == (int)813729603);
            };
            DemoDevice.COMMAND_ACK p77 = LoopBackDemoChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.result_param2_SET((int)813729603, PH) ;
            p77.target_system_SET((byte)(byte)61, PH) ;
            p77.command = (MAV_CMD)MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT;
            p77.progress_SET((byte)(byte)232, PH) ;
            p77.target_component_SET((byte)(byte)76, PH) ;
            p77.result = (MAV_RESULT)MAV_RESULT.MAV_RESULT_IN_PROGRESS;
            LoopBackDemoChannel.instance.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)3.3012703E38F);
                Debug.Assert(pack.mode_switch == (byte)(byte)207);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)231);
                Debug.Assert(pack.time_boot_ms == (uint)2492212898U);
                Debug.Assert(pack.roll == (float)1.6700641E38F);
                Debug.Assert(pack.thrust == (float) -8.086003E37F);
                Debug.Assert(pack.pitch == (float)1.7034486E38F);
            };
            DemoDevice.MANUAL_SETPOINT p81 = LoopBackDemoChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.yaw = (float)3.3012703E38F;
            p81.time_boot_ms = (uint)2492212898U;
            p81.mode_switch = (byte)(byte)207;
            p81.roll = (float)1.6700641E38F;
            p81.pitch = (float)1.7034486E38F;
            p81.manual_override_switch = (byte)(byte)231;
            p81.thrust = (float) -8.086003E37F;
            LoopBackDemoChannel.instance.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)188);
                Debug.Assert(pack.body_pitch_rate == (float) -1.8161198E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.2404432E38F, 2.594266E38F, -1.3067383E38F, -3.834032E37F}));
                Debug.Assert(pack.type_mask == (byte)(byte)107);
                Debug.Assert(pack.time_boot_ms == (uint)3436476337U);
                Debug.Assert(pack.target_system == (byte)(byte)75);
                Debug.Assert(pack.body_yaw_rate == (float)3.392912E38F);
                Debug.Assert(pack.thrust == (float)1.4998982E38F);
                Debug.Assert(pack.body_roll_rate == (float) -7.4656365E37F);
            };
            DemoDevice.SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.q_SET(new float[] {2.2404432E38F, 2.594266E38F, -1.3067383E38F, -3.834032E37F}, 0) ;
            p82.type_mask = (byte)(byte)107;
            p82.body_pitch_rate = (float) -1.8161198E38F;
            p82.thrust = (float)1.4998982E38F;
            p82.target_system = (byte)(byte)75;
            p82.target_component = (byte)(byte)188;
            p82.time_boot_ms = (uint)3436476337U;
            p82.body_roll_rate = (float) -7.4656365E37F;
            p82.body_yaw_rate = (float)3.392912E38F;
            LoopBackDemoChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.body_roll_rate == (float)5.0306085E37F);
                Debug.Assert(pack.time_boot_ms == (uint)2281491651U);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.143735E38F, 1.4280377E38F, 2.1122537E38F, -2.2492152E38F}));
                Debug.Assert(pack.thrust == (float) -1.94304E38F);
                Debug.Assert(pack.body_yaw_rate == (float) -1.165623E38F);
                Debug.Assert(pack.type_mask == (byte)(byte)236);
                Debug.Assert(pack.body_pitch_rate == (float)8.868132E37F);
            };
            DemoDevice.ATTITUDE_TARGET p83 = LoopBackDemoChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.thrust = (float) -1.94304E38F;
            p83.body_roll_rate = (float)5.0306085E37F;
            p83.time_boot_ms = (uint)2281491651U;
            p83.body_pitch_rate = (float)8.868132E37F;
            p83.type_mask = (byte)(byte)236;
            p83.body_yaw_rate = (float) -1.165623E38F;
            p83.q_SET(new float[] {-1.143735E38F, 1.4280377E38F, 2.1122537E38F, -2.2492152E38F}, 0) ;
            LoopBackDemoChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)69);
                Debug.Assert(pack.time_boot_ms == (uint)81191460U);
                Debug.Assert(pack.vz == (float)3.1562583E38F);
                Debug.Assert(pack.x == (float)9.95535E37F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.type_mask == (ushort)(ushort)15071);
                Debug.Assert(pack.afy == (float) -1.509365E38F);
                Debug.Assert(pack.afx == (float)3.2448184E38F);
                Debug.Assert(pack.y == (float)3.1979446E38F);
                Debug.Assert(pack.target_system == (byte)(byte)87);
                Debug.Assert(pack.vx == (float)8.2955405E37F);
                Debug.Assert(pack.vy == (float) -2.0224436E38F);
                Debug.Assert(pack.z == (float)1.7188876E38F);
                Debug.Assert(pack.afz == (float)2.9096236E38F);
                Debug.Assert(pack.yaw == (float) -2.2303448E38F);
                Debug.Assert(pack.yaw_rate == (float) -3.2083691E38F);
            };
            DemoDevice.SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.x = (float)9.95535E37F;
            p84.afx = (float)3.2448184E38F;
            p84.target_system = (byte)(byte)87;
            p84.target_component = (byte)(byte)69;
            p84.vx = (float)8.2955405E37F;
            p84.y = (float)3.1979446E38F;
            p84.time_boot_ms = (uint)81191460U;
            p84.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p84.afz = (float)2.9096236E38F;
            p84.vz = (float)3.1562583E38F;
            p84.vy = (float) -2.0224436E38F;
            p84.yaw = (float) -2.2303448E38F;
            p84.yaw_rate = (float) -3.2083691E38F;
            p84.afy = (float) -1.509365E38F;
            p84.z = (float)1.7188876E38F;
            p84.type_mask = (ushort)(ushort)15071;
            LoopBackDemoChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (float)1.7629172E38F);
                Debug.Assert(pack.vz == (float)2.6362324E38F);
                Debug.Assert(pack.vy == (float)1.2686953E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)49780);
                Debug.Assert(pack.target_component == (byte)(byte)154);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.afx == (float)2.4966547E38F);
                Debug.Assert(pack.lat_int == (int) -462132575);
                Debug.Assert(pack.afz == (float) -3.2077503E38F);
                Debug.Assert(pack.afy == (float) -4.547289E37F);
                Debug.Assert(pack.target_system == (byte)(byte)122);
                Debug.Assert(pack.vx == (float) -1.8653026E38F);
                Debug.Assert(pack.time_boot_ms == (uint)635549702U);
                Debug.Assert(pack.yaw_rate == (float)1.9915097E38F);
                Debug.Assert(pack.lon_int == (int)1107259647);
                Debug.Assert(pack.yaw == (float) -2.260244E38F);
            };
            DemoDevice.SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.type_mask = (ushort)(ushort)49780;
            p86.target_system = (byte)(byte)122;
            p86.yaw = (float) -2.260244E38F;
            p86.afy = (float) -4.547289E37F;
            p86.alt = (float)1.7629172E38F;
            p86.target_component = (byte)(byte)154;
            p86.vz = (float)2.6362324E38F;
            p86.time_boot_ms = (uint)635549702U;
            p86.yaw_rate = (float)1.9915097E38F;
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p86.afz = (float) -3.2077503E38F;
            p86.lat_int = (int) -462132575;
            p86.vx = (float) -1.8653026E38F;
            p86.afx = (float)2.4966547E38F;
            p86.lon_int = (int)1107259647;
            p86.vy = (float)1.2686953E38F;
            LoopBackDemoChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3045116685U);
                Debug.Assert(pack.lat_int == (int)601559493);
                Debug.Assert(pack.lon_int == (int) -1067162595);
                Debug.Assert(pack.type_mask == (ushort)(ushort)56882);
                Debug.Assert(pack.afy == (float) -1.6033825E38F);
                Debug.Assert(pack.alt == (float) -2.5369603E37F);
                Debug.Assert(pack.afx == (float)3.0586411E38F);
                Debug.Assert(pack.yaw == (float) -2.2042992E38F);
                Debug.Assert(pack.vy == (float) -2.7787332E37F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED);
                Debug.Assert(pack.afz == (float) -3.1147798E38F);
                Debug.Assert(pack.vz == (float)2.2243597E38F);
                Debug.Assert(pack.vx == (float)2.2779728E38F);
                Debug.Assert(pack.yaw_rate == (float) -1.454935E38F);
            };
            DemoDevice.POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.afz = (float) -3.1147798E38F;
            p87.lat_int = (int)601559493;
            p87.vx = (float)2.2779728E38F;
            p87.vy = (float) -2.7787332E37F;
            p87.afx = (float)3.0586411E38F;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p87.alt = (float) -2.5369603E37F;
            p87.lon_int = (int) -1067162595;
            p87.afy = (float) -1.6033825E38F;
            p87.yaw = (float) -2.2042992E38F;
            p87.vz = (float)2.2243597E38F;
            p87.time_boot_ms = (uint)3045116685U;
            p87.type_mask = (ushort)(ushort)56882;
            p87.yaw_rate = (float) -1.454935E38F;
            LoopBackDemoChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float) -4.341012E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3174684536U);
                Debug.Assert(pack.y == (float)2.9542818E38F);
                Debug.Assert(pack.roll == (float)2.2503723E38F);
                Debug.Assert(pack.z == (float) -2.1468929E38F);
                Debug.Assert(pack.x == (float) -2.5717237E38F);
                Debug.Assert(pack.yaw == (float) -3.3277072E38F);
            };
            DemoDevice.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.pitch = (float) -4.341012E37F;
            p89.time_boot_ms = (uint)3174684536U;
            p89.y = (float)2.9542818E38F;
            p89.z = (float) -2.1468929E38F;
            p89.yaw = (float) -3.3277072E38F;
            p89.x = (float) -2.5717237E38F;
            p89.roll = (float)2.2503723E38F;
            LoopBackDemoChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yawspeed == (float)6.0355857E37F);
                Debug.Assert(pack.zacc == (short)(short) -16688);
                Debug.Assert(pack.lon == (int) -1335362722);
                Debug.Assert(pack.roll == (float)1.0435664E38F);
                Debug.Assert(pack.lat == (int)704612063);
                Debug.Assert(pack.vx == (short)(short) -6918);
                Debug.Assert(pack.vy == (short)(short) -5826);
                Debug.Assert(pack.vz == (short)(short) -6871);
                Debug.Assert(pack.time_usec == (ulong)6373609091238576021L);
                Debug.Assert(pack.yacc == (short)(short)4971);
                Debug.Assert(pack.xacc == (short)(short)16908);
                Debug.Assert(pack.rollspeed == (float)2.9720043E37F);
                Debug.Assert(pack.pitch == (float) -8.518963E37F);
                Debug.Assert(pack.pitchspeed == (float)2.7056258E38F);
                Debug.Assert(pack.yaw == (float) -3.227731E38F);
                Debug.Assert(pack.alt == (int) -1578022482);
            };
            DemoDevice.HIL_STATE p90 = LoopBackDemoChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.yaw = (float) -3.227731E38F;
            p90.zacc = (short)(short) -16688;
            p90.yacc = (short)(short)4971;
            p90.lat = (int)704612063;
            p90.xacc = (short)(short)16908;
            p90.vz = (short)(short) -6871;
            p90.pitch = (float) -8.518963E37F;
            p90.lon = (int) -1335362722;
            p90.alt = (int) -1578022482;
            p90.vx = (short)(short) -6918;
            p90.rollspeed = (float)2.9720043E37F;
            p90.pitchspeed = (float)2.7056258E38F;
            p90.roll = (float)1.0435664E38F;
            p90.vy = (short)(short) -5826;
            p90.time_usec = (ulong)6373609091238576021L;
            p90.yawspeed = (float)6.0355857E37F;
            LoopBackDemoChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)783363391928318604L);
                Debug.Assert(pack.roll_ailerons == (float) -1.8494032E38F);
                Debug.Assert(pack.nav_mode == (byte)(byte)36);
                Debug.Assert(pack.throttle == (float)5.1383867E37F);
                Debug.Assert(pack.pitch_elevator == (float) -8.521054E37F);
                Debug.Assert(pack.aux3 == (float) -1.5913139E38F);
                Debug.Assert(pack.aux2 == (float) -8.705757E37F);
                Debug.Assert(pack.aux4 == (float)1.6962595E38F);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_ARMED);
                Debug.Assert(pack.yaw_rudder == (float)5.244831E37F);
                Debug.Assert(pack.aux1 == (float) -2.556657E38F);
            };
            DemoDevice.HIL_CONTROLS p91 = LoopBackDemoChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.pitch_elevator = (float) -8.521054E37F;
            p91.roll_ailerons = (float) -1.8494032E38F;
            p91.aux1 = (float) -2.556657E38F;
            p91.aux3 = (float) -1.5913139E38F;
            p91.nav_mode = (byte)(byte)36;
            p91.time_usec = (ulong)783363391928318604L;
            p91.throttle = (float)5.1383867E37F;
            p91.aux2 = (float) -8.705757E37F;
            p91.yaw_rudder = (float)5.244831E37F;
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_ARMED;
            p91.aux4 = (float)1.6962595E38F;
            LoopBackDemoChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)1536);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)37134);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)58088);
                Debug.Assert(pack.time_usec == (ulong)4314871878835308787L);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)59198);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)24614);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)14244);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)61190);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)18130);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)5878);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)36229);
                Debug.Assert(pack.rssi == (byte)(byte)98);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)42638);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)34859);
            };
            DemoDevice.HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan5_raw = (ushort)(ushort)59198;
            p92.chan6_raw = (ushort)(ushort)18130;
            p92.time_usec = (ulong)4314871878835308787L;
            p92.chan9_raw = (ushort)(ushort)24614;
            p92.chan4_raw = (ushort)(ushort)58088;
            p92.chan11_raw = (ushort)(ushort)37134;
            p92.chan3_raw = (ushort)(ushort)34859;
            p92.chan8_raw = (ushort)(ushort)36229;
            p92.chan10_raw = (ushort)(ushort)61190;
            p92.rssi = (byte)(byte)98;
            p92.chan12_raw = (ushort)(ushort)42638;
            p92.chan1_raw = (ushort)(ushort)5878;
            p92.chan2_raw = (ushort)(ushort)14244;
            p92.chan7_raw = (ushort)(ushort)1536;
            LoopBackDemoChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (ulong)9140995076291740417L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {2.8049242E38F, 2.9192636E38F, 2.2137895E38F, 2.2152184E38F, 3.3352648E38F, 2.7640738E38F, -5.7417357E37F, 5.580016E37F, -3.2253558E38F, -1.5213375E38F, 9.187621E37F, 3.5860144E37F, -2.3824862E38F, -4.029872E37F, -4.315594E37F, -5.1736193E37F}));
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_AUTO_ARMED);
                Debug.Assert(pack.time_usec == (ulong)8819607786034517317L);
            };
            DemoDevice.HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.time_usec = (ulong)8819607786034517317L;
            p93.controls_SET(new float[] {2.8049242E38F, 2.9192636E38F, 2.2137895E38F, 2.2152184E38F, 3.3352648E38F, 2.7640738E38F, -5.7417357E37F, 5.580016E37F, -3.2253558E38F, -1.5213375E38F, 9.187621E37F, 3.5860144E37F, -2.3824862E38F, -4.029872E37F, -4.315594E37F, -5.1736193E37F}, 0) ;
            p93.flags = (ulong)9140995076291740417L;
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_AUTO_ARMED;
            LoopBackDemoChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flow_y == (short)(short)30382);
                Debug.Assert(pack.time_usec == (ulong)874302980542717606L);
                Debug.Assert(pack.flow_comp_m_y == (float) -2.9578882E38F);
                Debug.Assert(pack.flow_x == (short)(short)26116);
                Debug.Assert(pack.ground_distance == (float) -1.9638654E38F);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float)1.1829354E38F);
                Debug.Assert(pack.flow_comp_m_x == (float)2.8107718E38F);
                Debug.Assert(pack.quality == (byte)(byte)119);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float)1.3671956E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)129);
            };
            DemoDevice.OPTICAL_FLOW p100 = LoopBackDemoChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.time_usec = (ulong)874302980542717606L;
            p100.flow_x = (short)(short)26116;
            p100.sensor_id = (byte)(byte)129;
            p100.flow_rate_x_SET((float)1.3671956E38F, PH) ;
            p100.ground_distance = (float) -1.9638654E38F;
            p100.flow_comp_m_y = (float) -2.9578882E38F;
            p100.flow_y = (short)(short)30382;
            p100.quality = (byte)(byte)119;
            p100.flow_rate_y_SET((float)1.1829354E38F, PH) ;
            p100.flow_comp_m_x = (float)2.8107718E38F;
            LoopBackDemoChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -4.1568722E37F);
                Debug.Assert(pack.x == (float) -5.1669413E37F);
                Debug.Assert(pack.y == (float)2.6634353E38F);
                Debug.Assert(pack.usec == (ulong)8428202591362601899L);
                Debug.Assert(pack.pitch == (float) -2.3851588E38F);
                Debug.Assert(pack.roll == (float)3.362013E38F);
                Debug.Assert(pack.yaw == (float)1.2318946E38F);
            };
            DemoDevice.GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.x = (float) -5.1669413E37F;
            p101.z = (float) -4.1568722E37F;
            p101.y = (float)2.6634353E38F;
            p101.roll = (float)3.362013E38F;
            p101.pitch = (float) -2.3851588E38F;
            p101.usec = (ulong)8428202591362601899L;
            p101.yaw = (float)1.2318946E38F;
            LoopBackDemoChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float) -2.568746E38F);
                Debug.Assert(pack.usec == (ulong)6273127634344512474L);
                Debug.Assert(pack.pitch == (float) -9.168474E37F);
                Debug.Assert(pack.yaw == (float)1.3991858E38F);
                Debug.Assert(pack.z == (float) -4.5845642E36F);
                Debug.Assert(pack.x == (float)1.1786531E38F);
                Debug.Assert(pack.y == (float)6.6207814E37F);
            };
            DemoDevice.VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.x = (float)1.1786531E38F;
            p102.yaw = (float)1.3991858E38F;
            p102.roll = (float) -2.568746E38F;
            p102.usec = (ulong)6273127634344512474L;
            p102.z = (float) -4.5845642E36F;
            p102.pitch = (float) -9.168474E37F;
            p102.y = (float)6.6207814E37F;
            LoopBackDemoChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.usec == (ulong)3769965848237830251L);
                Debug.Assert(pack.z == (float) -1.038695E38F);
                Debug.Assert(pack.y == (float)2.570737E38F);
                Debug.Assert(pack.x == (float) -2.9224524E38F);
            };
            DemoDevice.VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.y = (float)2.570737E38F;
            p103.z = (float) -1.038695E38F;
            p103.usec = (ulong)3769965848237830251L;
            p103.x = (float) -2.9224524E38F;
            LoopBackDemoChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -1.9355583E38F);
                Debug.Assert(pack.y == (float) -2.9766215E38F);
                Debug.Assert(pack.usec == (ulong)4783790229704283047L);
                Debug.Assert(pack.pitch == (float)2.2549448E38F);
                Debug.Assert(pack.yaw == (float)6.2575285E37F);
                Debug.Assert(pack.roll == (float)5.1370156E37F);
                Debug.Assert(pack.z == (float) -1.1292194E38F);
            };
            DemoDevice.VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.x = (float) -1.9355583E38F;
            p104.z = (float) -1.1292194E38F;
            p104.yaw = (float)6.2575285E37F;
            p104.usec = (ulong)4783790229704283047L;
            p104.pitch = (float)2.2549448E38F;
            p104.roll = (float)5.1370156E37F;
            p104.y = (float) -2.9766215E38F;
            LoopBackDemoChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (float) -2.3379412E38F);
                Debug.Assert(pack.temperature == (float) -2.7515983E38F);
                Debug.Assert(pack.diff_pressure == (float) -3.3117883E38F);
                Debug.Assert(pack.xacc == (float)6.705468E37F);
                Debug.Assert(pack.ygyro == (float)1.3809245E38F);
                Debug.Assert(pack.zmag == (float)1.8114985E38F);
                Debug.Assert(pack.time_usec == (ulong)8580888121391110674L);
                Debug.Assert(pack.abs_pressure == (float)1.764464E38F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)25716);
                Debug.Assert(pack.xmag == (float)3.1498245E38F);
                Debug.Assert(pack.pressure_alt == (float) -2.8321618E38F);
                Debug.Assert(pack.zgyro == (float)2.7132384E38F);
                Debug.Assert(pack.xgyro == (float)2.2184575E37F);
                Debug.Assert(pack.ymag == (float) -3.1337173E38F);
                Debug.Assert(pack.zacc == (float) -6.260473E37F);
            };
            DemoDevice.HIGHRES_IMU p105 = LoopBackDemoChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.zmag = (float)1.8114985E38F;
            p105.abs_pressure = (float)1.764464E38F;
            p105.yacc = (float) -2.3379412E38F;
            p105.zacc = (float) -6.260473E37F;
            p105.xmag = (float)3.1498245E38F;
            p105.pressure_alt = (float) -2.8321618E38F;
            p105.xacc = (float)6.705468E37F;
            p105.temperature = (float) -2.7515983E38F;
            p105.zgyro = (float)2.7132384E38F;
            p105.diff_pressure = (float) -3.3117883E38F;
            p105.fields_updated = (ushort)(ushort)25716;
            p105.ygyro = (float)1.3809245E38F;
            p105.ymag = (float) -3.1337173E38F;
            p105.xgyro = (float)2.2184575E37F;
            p105.time_usec = (ulong)8580888121391110674L;
            LoopBackDemoChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_delta_distance_us == (uint)2267689780U);
                Debug.Assert(pack.temperature == (short)(short) -31021);
                Debug.Assert(pack.integrated_zgyro == (float) -2.8342371E38F);
                Debug.Assert(pack.distance == (float) -2.2704271E37F);
                Debug.Assert(pack.quality == (byte)(byte)7);
                Debug.Assert(pack.integrated_xgyro == (float)3.8689693E37F);
                Debug.Assert(pack.integrated_ygyro == (float)2.7765148E38F);
                Debug.Assert(pack.time_usec == (ulong)5643659684687034048L);
                Debug.Assert(pack.integration_time_us == (uint)4153304675U);
                Debug.Assert(pack.integrated_y == (float) -4.814169E37F);
                Debug.Assert(pack.sensor_id == (byte)(byte)105);
                Debug.Assert(pack.integrated_x == (float) -2.105865E38F);
            };
            DemoDevice.OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_usec = (ulong)5643659684687034048L;
            p106.integrated_ygyro = (float)2.7765148E38F;
            p106.integrated_y = (float) -4.814169E37F;
            p106.time_delta_distance_us = (uint)2267689780U;
            p106.temperature = (short)(short) -31021;
            p106.integrated_zgyro = (float) -2.8342371E38F;
            p106.sensor_id = (byte)(byte)105;
            p106.integrated_xgyro = (float)3.8689693E37F;
            p106.integration_time_us = (uint)4153304675U;
            p106.distance = (float) -2.2704271E37F;
            p106.quality = (byte)(byte)7;
            p106.integrated_x = (float) -2.105865E38F;
            LoopBackDemoChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.abs_pressure == (float) -1.3660503E38F);
                Debug.Assert(pack.fields_updated == (uint)4194504373U);
                Debug.Assert(pack.xgyro == (float)2.273061E38F);
                Debug.Assert(pack.xmag == (float) -1.4168161E38F);
                Debug.Assert(pack.time_usec == (ulong)6911407903014275166L);
                Debug.Assert(pack.xacc == (float)2.2049464E38F);
                Debug.Assert(pack.zacc == (float)1.1267183E37F);
                Debug.Assert(pack.zgyro == (float) -5.7336476E37F);
                Debug.Assert(pack.diff_pressure == (float)8.185408E37F);
                Debug.Assert(pack.zmag == (float)1.5183397E38F);
                Debug.Assert(pack.yacc == (float)1.6121145E38F);
                Debug.Assert(pack.temperature == (float) -5.6154097E37F);
                Debug.Assert(pack.ymag == (float)3.369578E38F);
                Debug.Assert(pack.ygyro == (float) -3.236657E37F);
                Debug.Assert(pack.pressure_alt == (float) -1.3044649E38F);
            };
            DemoDevice.HIL_SENSOR p107 = LoopBackDemoChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.xacc = (float)2.2049464E38F;
            p107.diff_pressure = (float)8.185408E37F;
            p107.yacc = (float)1.6121145E38F;
            p107.zmag = (float)1.5183397E38F;
            p107.ymag = (float)3.369578E38F;
            p107.zacc = (float)1.1267183E37F;
            p107.time_usec = (ulong)6911407903014275166L;
            p107.ygyro = (float) -3.236657E37F;
            p107.xmag = (float) -1.4168161E38F;
            p107.zgyro = (float) -5.7336476E37F;
            p107.temperature = (float) -5.6154097E37F;
            p107.fields_updated = (uint)4194504373U;
            p107.pressure_alt = (float) -1.3044649E38F;
            p107.abs_pressure = (float) -1.3660503E38F;
            p107.xgyro = (float)2.273061E38F;
            LoopBackDemoChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q2 == (float) -2.6533298E38F);
                Debug.Assert(pack.pitch == (float)1.001971E38F);
                Debug.Assert(pack.roll == (float)7.49603E37F);
                Debug.Assert(pack.q1 == (float)2.1340314E38F);
                Debug.Assert(pack.lon == (float) -1.5398156E38F);
                Debug.Assert(pack.zacc == (float)2.215683E38F);
                Debug.Assert(pack.lat == (float) -1.6199036E37F);
                Debug.Assert(pack.yaw == (float) -2.1813896E38F);
                Debug.Assert(pack.std_dev_horz == (float) -4.6927847E37F);
                Debug.Assert(pack.ygyro == (float) -9.893259E37F);
                Debug.Assert(pack.std_dev_vert == (float) -9.273239E37F);
                Debug.Assert(pack.q3 == (float)3.0614715E38F);
                Debug.Assert(pack.yacc == (float)3.237317E38F);
                Debug.Assert(pack.vd == (float) -6.928606E37F);
                Debug.Assert(pack.xgyro == (float)1.9891432E38F);
                Debug.Assert(pack.q4 == (float)2.5636712E38F);
                Debug.Assert(pack.ve == (float)2.827674E38F);
                Debug.Assert(pack.vn == (float) -3.399964E38F);
                Debug.Assert(pack.alt == (float)2.9105276E38F);
                Debug.Assert(pack.xacc == (float) -9.423184E37F);
                Debug.Assert(pack.zgyro == (float) -2.4756511E37F);
            };
            DemoDevice.SIM_STATE p108 = LoopBackDemoChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.alt = (float)2.9105276E38F;
            p108.yaw = (float) -2.1813896E38F;
            p108.xgyro = (float)1.9891432E38F;
            p108.ygyro = (float) -9.893259E37F;
            p108.pitch = (float)1.001971E38F;
            p108.roll = (float)7.49603E37F;
            p108.lon = (float) -1.5398156E38F;
            p108.xacc = (float) -9.423184E37F;
            p108.q3 = (float)3.0614715E38F;
            p108.lat = (float) -1.6199036E37F;
            p108.yacc = (float)3.237317E38F;
            p108.ve = (float)2.827674E38F;
            p108.vn = (float) -3.399964E38F;
            p108.std_dev_vert = (float) -9.273239E37F;
            p108.zacc = (float)2.215683E38F;
            p108.std_dev_horz = (float) -4.6927847E37F;
            p108.zgyro = (float) -2.4756511E37F;
            p108.q4 = (float)2.5636712E38F;
            p108.q2 = (float) -2.6533298E38F;
            p108.vd = (float) -6.928606E37F;
            p108.q1 = (float)2.1340314E38F;
            LoopBackDemoChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fixed_ == (ushort)(ushort)23620);
                Debug.Assert(pack.txbuf == (byte)(byte)188);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)53092);
                Debug.Assert(pack.remnoise == (byte)(byte)2);
                Debug.Assert(pack.rssi == (byte)(byte)253);
                Debug.Assert(pack.noise == (byte)(byte)95);
                Debug.Assert(pack.remrssi == (byte)(byte)49);
            };
            DemoDevice.RADIO_STATUS p109 = LoopBackDemoChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.remnoise = (byte)(byte)2;
            p109.txbuf = (byte)(byte)188;
            p109.noise = (byte)(byte)95;
            p109.fixed_ = (ushort)(ushort)23620;
            p109.rssi = (byte)(byte)253;
            p109.remrssi = (byte)(byte)49;
            p109.rxerrors = (ushort)(ushort)53092;
            LoopBackDemoChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)99, (byte)241, (byte)237, (byte)3, (byte)138, (byte)238, (byte)79, (byte)201, (byte)107, (byte)107, (byte)184, (byte)153, (byte)90, (byte)202, (byte)86, (byte)128, (byte)18, (byte)240, (byte)233, (byte)82, (byte)253, (byte)53, (byte)196, (byte)215, (byte)201, (byte)217, (byte)233, (byte)58, (byte)22, (byte)210, (byte)193, (byte)49, (byte)235, (byte)55, (byte)201, (byte)206, (byte)177, (byte)177, (byte)115, (byte)180, (byte)220, (byte)74, (byte)10, (byte)162, (byte)247, (byte)82, (byte)52, (byte)32, (byte)106, (byte)241, (byte)56, (byte)158, (byte)178, (byte)191, (byte)12, (byte)83, (byte)242, (byte)70, (byte)23, (byte)99, (byte)72, (byte)84, (byte)229, (byte)45, (byte)124, (byte)63, (byte)87, (byte)165, (byte)207, (byte)127, (byte)19, (byte)34, (byte)39, (byte)82, (byte)69, (byte)226, (byte)60, (byte)171, (byte)144, (byte)173, (byte)248, (byte)162, (byte)207, (byte)62, (byte)90, (byte)127, (byte)165, (byte)195, (byte)95, (byte)160, (byte)27, (byte)51, (byte)155, (byte)39, (byte)243, (byte)55, (byte)28, (byte)208, (byte)155, (byte)54, (byte)172, (byte)26, (byte)209, (byte)24, (byte)55, (byte)205, (byte)204, (byte)53, (byte)171, (byte)34, (byte)75, (byte)199, (byte)110, (byte)50, (byte)21, (byte)103, (byte)5, (byte)81, (byte)115, (byte)166, (byte)205, (byte)168, (byte)77, (byte)173, (byte)246, (byte)196, (byte)57, (byte)237, (byte)163, (byte)208, (byte)108, (byte)101, (byte)65, (byte)233, (byte)198, (byte)242, (byte)253, (byte)231, (byte)74, (byte)171, (byte)131, (byte)121, (byte)131, (byte)202, (byte)148, (byte)5, (byte)143, (byte)79, (byte)155, (byte)172, (byte)223, (byte)21, (byte)24, (byte)183, (byte)137, (byte)77, (byte)82, (byte)111, (byte)101, (byte)167, (byte)163, (byte)81, (byte)130, (byte)88, (byte)130, (byte)178, (byte)193, (byte)52, (byte)146, (byte)79, (byte)33, (byte)19, (byte)205, (byte)229, (byte)41, (byte)55, (byte)19, (byte)255, (byte)76, (byte)126, (byte)5, (byte)138, (byte)120, (byte)71, (byte)64, (byte)225, (byte)46, (byte)84, (byte)176, (byte)248, (byte)122, (byte)92, (byte)161, (byte)167, (byte)99, (byte)225, (byte)191, (byte)177, (byte)141, (byte)64, (byte)52, (byte)176, (byte)129, (byte)134, (byte)119, (byte)175, (byte)50, (byte)211, (byte)55, (byte)121, (byte)163, (byte)132, (byte)174, (byte)116, (byte)247, (byte)175, (byte)235, (byte)67, (byte)147, (byte)180, (byte)154, (byte)70, (byte)2, (byte)23, (byte)28, (byte)186, (byte)17, (byte)91, (byte)50, (byte)139, (byte)94, (byte)148, (byte)210, (byte)92, (byte)7, (byte)200, (byte)6, (byte)160, (byte)26, (byte)190, (byte)170, (byte)97, (byte)5, (byte)119, (byte)252, (byte)199, (byte)232, (byte)115, (byte)78, (byte)145, (byte)166}));
                Debug.Assert(pack.target_network == (byte)(byte)250);
                Debug.Assert(pack.target_system == (byte)(byte)73);
                Debug.Assert(pack.target_component == (byte)(byte)233);
            };
            DemoDevice.FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_component = (byte)(byte)233;
            p110.target_network = (byte)(byte)250;
            p110.target_system = (byte)(byte)73;
            p110.payload_SET(new byte[] {(byte)99, (byte)241, (byte)237, (byte)3, (byte)138, (byte)238, (byte)79, (byte)201, (byte)107, (byte)107, (byte)184, (byte)153, (byte)90, (byte)202, (byte)86, (byte)128, (byte)18, (byte)240, (byte)233, (byte)82, (byte)253, (byte)53, (byte)196, (byte)215, (byte)201, (byte)217, (byte)233, (byte)58, (byte)22, (byte)210, (byte)193, (byte)49, (byte)235, (byte)55, (byte)201, (byte)206, (byte)177, (byte)177, (byte)115, (byte)180, (byte)220, (byte)74, (byte)10, (byte)162, (byte)247, (byte)82, (byte)52, (byte)32, (byte)106, (byte)241, (byte)56, (byte)158, (byte)178, (byte)191, (byte)12, (byte)83, (byte)242, (byte)70, (byte)23, (byte)99, (byte)72, (byte)84, (byte)229, (byte)45, (byte)124, (byte)63, (byte)87, (byte)165, (byte)207, (byte)127, (byte)19, (byte)34, (byte)39, (byte)82, (byte)69, (byte)226, (byte)60, (byte)171, (byte)144, (byte)173, (byte)248, (byte)162, (byte)207, (byte)62, (byte)90, (byte)127, (byte)165, (byte)195, (byte)95, (byte)160, (byte)27, (byte)51, (byte)155, (byte)39, (byte)243, (byte)55, (byte)28, (byte)208, (byte)155, (byte)54, (byte)172, (byte)26, (byte)209, (byte)24, (byte)55, (byte)205, (byte)204, (byte)53, (byte)171, (byte)34, (byte)75, (byte)199, (byte)110, (byte)50, (byte)21, (byte)103, (byte)5, (byte)81, (byte)115, (byte)166, (byte)205, (byte)168, (byte)77, (byte)173, (byte)246, (byte)196, (byte)57, (byte)237, (byte)163, (byte)208, (byte)108, (byte)101, (byte)65, (byte)233, (byte)198, (byte)242, (byte)253, (byte)231, (byte)74, (byte)171, (byte)131, (byte)121, (byte)131, (byte)202, (byte)148, (byte)5, (byte)143, (byte)79, (byte)155, (byte)172, (byte)223, (byte)21, (byte)24, (byte)183, (byte)137, (byte)77, (byte)82, (byte)111, (byte)101, (byte)167, (byte)163, (byte)81, (byte)130, (byte)88, (byte)130, (byte)178, (byte)193, (byte)52, (byte)146, (byte)79, (byte)33, (byte)19, (byte)205, (byte)229, (byte)41, (byte)55, (byte)19, (byte)255, (byte)76, (byte)126, (byte)5, (byte)138, (byte)120, (byte)71, (byte)64, (byte)225, (byte)46, (byte)84, (byte)176, (byte)248, (byte)122, (byte)92, (byte)161, (byte)167, (byte)99, (byte)225, (byte)191, (byte)177, (byte)141, (byte)64, (byte)52, (byte)176, (byte)129, (byte)134, (byte)119, (byte)175, (byte)50, (byte)211, (byte)55, (byte)121, (byte)163, (byte)132, (byte)174, (byte)116, (byte)247, (byte)175, (byte)235, (byte)67, (byte)147, (byte)180, (byte)154, (byte)70, (byte)2, (byte)23, (byte)28, (byte)186, (byte)17, (byte)91, (byte)50, (byte)139, (byte)94, (byte)148, (byte)210, (byte)92, (byte)7, (byte)200, (byte)6, (byte)160, (byte)26, (byte)190, (byte)170, (byte)97, (byte)5, (byte)119, (byte)252, (byte)199, (byte)232, (byte)115, (byte)78, (byte)145, (byte)166}, 0) ;
            LoopBackDemoChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ts1 == (long)2733348123575242232L);
                Debug.Assert(pack.tc1 == (long) -4468822196686457743L);
            };
            DemoDevice.TIMESYNC p111 = LoopBackDemoChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.ts1 = (long)2733348123575242232L;
            p111.tc1 = (long) -4468822196686457743L;
            LoopBackDemoChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)6900891184297830027L);
                Debug.Assert(pack.seq == (uint)3474272719U);
            };
            DemoDevice.CAMERA_TRIGGER p112 = LoopBackDemoChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)6900891184297830027L;
            p112.seq = (uint)3474272719U;
            LoopBackDemoChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (int) -1651953735);
                Debug.Assert(pack.lon == (int) -2141999004);
                Debug.Assert(pack.cog == (ushort)(ushort)55008);
                Debug.Assert(pack.vn == (short)(short)17752);
                Debug.Assert(pack.ve == (short)(short)13444);
                Debug.Assert(pack.fix_type == (byte)(byte)77);
                Debug.Assert(pack.lat == (int) -73361980);
                Debug.Assert(pack.eph == (ushort)(ushort)8557);
                Debug.Assert(pack.satellites_visible == (byte)(byte)47);
                Debug.Assert(pack.vd == (short)(short)14225);
                Debug.Assert(pack.epv == (ushort)(ushort)27085);
                Debug.Assert(pack.vel == (ushort)(ushort)22743);
                Debug.Assert(pack.time_usec == (ulong)4468802854637931065L);
            };
            DemoDevice.HIL_GPS p113 = LoopBackDemoChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.vd = (short)(short)14225;
            p113.lon = (int) -2141999004;
            p113.satellites_visible = (byte)(byte)47;
            p113.fix_type = (byte)(byte)77;
            p113.eph = (ushort)(ushort)8557;
            p113.time_usec = (ulong)4468802854637931065L;
            p113.vn = (short)(short)17752;
            p113.epv = (ushort)(ushort)27085;
            p113.lat = (int) -73361980;
            p113.alt = (int) -1651953735;
            p113.vel = (ushort)(ushort)22743;
            p113.ve = (short)(short)13444;
            p113.cog = (ushort)(ushort)55008;
            LoopBackDemoChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.quality == (byte)(byte)94);
                Debug.Assert(pack.integrated_ygyro == (float)2.3264242E38F);
                Debug.Assert(pack.time_usec == (ulong)8384657757271798972L);
                Debug.Assert(pack.temperature == (short)(short) -19343);
                Debug.Assert(pack.sensor_id == (byte)(byte)137);
                Debug.Assert(pack.integration_time_us == (uint)2375618105U);
                Debug.Assert(pack.integrated_zgyro == (float)3.077009E38F);
                Debug.Assert(pack.integrated_xgyro == (float) -8.347567E37F);
                Debug.Assert(pack.time_delta_distance_us == (uint)4021917732U);
                Debug.Assert(pack.distance == (float) -2.3754774E38F);
                Debug.Assert(pack.integrated_y == (float) -7.727209E37F);
                Debug.Assert(pack.integrated_x == (float)2.3894534E38F);
            };
            DemoDevice.HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.integrated_xgyro = (float) -8.347567E37F;
            p114.quality = (byte)(byte)94;
            p114.temperature = (short)(short) -19343;
            p114.time_usec = (ulong)8384657757271798972L;
            p114.distance = (float) -2.3754774E38F;
            p114.sensor_id = (byte)(byte)137;
            p114.time_delta_distance_us = (uint)4021917732U;
            p114.integration_time_us = (uint)2375618105U;
            p114.integrated_y = (float) -7.727209E37F;
            p114.integrated_x = (float)2.3894534E38F;
            p114.integrated_zgyro = (float)3.077009E38F;
            p114.integrated_ygyro = (float)2.3264242E38F;
            LoopBackDemoChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (short)(short)23703);
                Debug.Assert(pack.zacc == (short)(short) -10200);
                Debug.Assert(pack.vy == (short)(short) -14079);
                Debug.Assert(pack.time_usec == (ulong)5361543567605078297L);
                Debug.Assert(pack.vz == (short)(short)1292);
                Debug.Assert(pack.vx == (short)(short) -17760);
                Debug.Assert(pack.yawspeed == (float) -2.4112558E38F);
                Debug.Assert(pack.pitchspeed == (float)1.4830155E38F);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)39196);
                Debug.Assert(pack.alt == (int)170940441);
                Debug.Assert(pack.rollspeed == (float)4.0188304E37F);
                Debug.Assert(pack.yacc == (short)(short) -22394);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {-2.9837637E38F, 2.0908787E38F, 9.216305E37F, -5.795243E37F}));
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)10668);
                Debug.Assert(pack.lon == (int)2047350476);
                Debug.Assert(pack.lat == (int) -860537451);
            };
            DemoDevice.HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.lat = (int) -860537451;
            p115.rollspeed = (float)4.0188304E37F;
            p115.true_airspeed = (ushort)(ushort)10668;
            p115.ind_airspeed = (ushort)(ushort)39196;
            p115.lon = (int)2047350476;
            p115.zacc = (short)(short) -10200;
            p115.xacc = (short)(short)23703;
            p115.yawspeed = (float) -2.4112558E38F;
            p115.alt = (int)170940441;
            p115.vz = (short)(short)1292;
            p115.attitude_quaternion_SET(new float[] {-2.9837637E38F, 2.0908787E38F, 9.216305E37F, -5.795243E37F}, 0) ;
            p115.yacc = (short)(short) -22394;
            p115.vy = (short)(short) -14079;
            p115.pitchspeed = (float)1.4830155E38F;
            p115.time_usec = (ulong)5361543567605078297L;
            p115.vx = (short)(short) -17760;
            LoopBackDemoChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zacc == (short)(short)5282);
                Debug.Assert(pack.xgyro == (short)(short)13863);
                Debug.Assert(pack.yacc == (short)(short) -4833);
                Debug.Assert(pack.time_boot_ms == (uint)1883020354U);
                Debug.Assert(pack.xacc == (short)(short)6392);
                Debug.Assert(pack.zgyro == (short)(short)7821);
                Debug.Assert(pack.zmag == (short)(short) -18445);
                Debug.Assert(pack.xmag == (short)(short) -29811);
                Debug.Assert(pack.ymag == (short)(short)3517);
                Debug.Assert(pack.ygyro == (short)(short) -5497);
            };
            DemoDevice.SCALED_IMU2 p116 = LoopBackDemoChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.time_boot_ms = (uint)1883020354U;
            p116.xacc = (short)(short)6392;
            p116.zgyro = (short)(short)7821;
            p116.zacc = (short)(short)5282;
            p116.yacc = (short)(short) -4833;
            p116.zmag = (short)(short) -18445;
            p116.xmag = (short)(short) -29811;
            p116.ymag = (short)(short)3517;
            p116.xgyro = (short)(short)13863;
            p116.ygyro = (short)(short) -5497;
            LoopBackDemoChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)88);
                Debug.Assert(pack.end == (ushort)(ushort)57432);
                Debug.Assert(pack.start == (ushort)(ushort)59148);
                Debug.Assert(pack.target_system == (byte)(byte)215);
            };
            DemoDevice.LOG_REQUEST_LIST p117 = LoopBackDemoChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.end = (ushort)(ushort)57432;
            p117.start = (ushort)(ushort)59148;
            p117.target_component = (byte)(byte)88;
            p117.target_system = (byte)(byte)215;
            LoopBackDemoChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_utc == (uint)3248091305U);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)10070);
                Debug.Assert(pack.id == (ushort)(ushort)23551);
                Debug.Assert(pack.size == (uint)884703895U);
                Debug.Assert(pack.num_logs == (ushort)(ushort)45846);
            };
            DemoDevice.LOG_ENTRY p118 = LoopBackDemoChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.num_logs = (ushort)(ushort)45846;
            p118.time_utc = (uint)3248091305U;
            p118.size = (uint)884703895U;
            p118.last_log_num = (ushort)(ushort)10070;
            p118.id = (ushort)(ushort)23551;
            LoopBackDemoChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ofs == (uint)662219932U);
                Debug.Assert(pack.count == (uint)930695589U);
                Debug.Assert(pack.target_system == (byte)(byte)6);
                Debug.Assert(pack.target_component == (byte)(byte)106);
                Debug.Assert(pack.id == (ushort)(ushort)47801);
            };
            DemoDevice.LOG_REQUEST_DATA p119 = LoopBackDemoChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.ofs = (uint)662219932U;
            p119.target_system = (byte)(byte)6;
            p119.id = (ushort)(ushort)47801;
            p119.count = (uint)930695589U;
            p119.target_component = (byte)(byte)106;
            LoopBackDemoChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (byte)(byte)126);
                Debug.Assert(pack.id == (ushort)(ushort)51066);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)182, (byte)102, (byte)72, (byte)141, (byte)12, (byte)138, (byte)185, (byte)155, (byte)35, (byte)178, (byte)98, (byte)169, (byte)94, (byte)198, (byte)122, (byte)177, (byte)237, (byte)76, (byte)7, (byte)109, (byte)46, (byte)24, (byte)169, (byte)134, (byte)173, (byte)116, (byte)227, (byte)209, (byte)104, (byte)174, (byte)109, (byte)106, (byte)126, (byte)113, (byte)103, (byte)68, (byte)252, (byte)224, (byte)96, (byte)198, (byte)113, (byte)200, (byte)173, (byte)119, (byte)14, (byte)122, (byte)72, (byte)44, (byte)81, (byte)39, (byte)206, (byte)36, (byte)35, (byte)1, (byte)176, (byte)5, (byte)168, (byte)157, (byte)196, (byte)219, (byte)182, (byte)138, (byte)55, (byte)147, (byte)132, (byte)73, (byte)93, (byte)254, (byte)99, (byte)218, (byte)156, (byte)200, (byte)166, (byte)158, (byte)27, (byte)254, (byte)227, (byte)172, (byte)22, (byte)253, (byte)28, (byte)246, (byte)71, (byte)43, (byte)171, (byte)232, (byte)5, (byte)73, (byte)73, (byte)37}));
                Debug.Assert(pack.ofs == (uint)1791716878U);
            };
            DemoDevice.LOG_DATA p120 = LoopBackDemoChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.ofs = (uint)1791716878U;
            p120.id = (ushort)(ushort)51066;
            p120.data__SET(new byte[] {(byte)182, (byte)102, (byte)72, (byte)141, (byte)12, (byte)138, (byte)185, (byte)155, (byte)35, (byte)178, (byte)98, (byte)169, (byte)94, (byte)198, (byte)122, (byte)177, (byte)237, (byte)76, (byte)7, (byte)109, (byte)46, (byte)24, (byte)169, (byte)134, (byte)173, (byte)116, (byte)227, (byte)209, (byte)104, (byte)174, (byte)109, (byte)106, (byte)126, (byte)113, (byte)103, (byte)68, (byte)252, (byte)224, (byte)96, (byte)198, (byte)113, (byte)200, (byte)173, (byte)119, (byte)14, (byte)122, (byte)72, (byte)44, (byte)81, (byte)39, (byte)206, (byte)36, (byte)35, (byte)1, (byte)176, (byte)5, (byte)168, (byte)157, (byte)196, (byte)219, (byte)182, (byte)138, (byte)55, (byte)147, (byte)132, (byte)73, (byte)93, (byte)254, (byte)99, (byte)218, (byte)156, (byte)200, (byte)166, (byte)158, (byte)27, (byte)254, (byte)227, (byte)172, (byte)22, (byte)253, (byte)28, (byte)246, (byte)71, (byte)43, (byte)171, (byte)232, (byte)5, (byte)73, (byte)73, (byte)37}, 0) ;
            p120.count = (byte)(byte)126;
            LoopBackDemoChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)1);
                Debug.Assert(pack.target_component == (byte)(byte)172);
            };
            DemoDevice.LOG_ERASE p121 = LoopBackDemoChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_component = (byte)(byte)172;
            p121.target_system = (byte)(byte)1;
            LoopBackDemoChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)248);
                Debug.Assert(pack.target_component == (byte)(byte)1);
            };
            DemoDevice.LOG_REQUEST_END p122 = LoopBackDemoChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)248;
            p122.target_component = (byte)(byte)1;
            LoopBackDemoChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.len == (byte)(byte)173);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)168, (byte)255, (byte)35, (byte)95, (byte)222, (byte)78, (byte)32, (byte)152, (byte)59, (byte)75, (byte)85, (byte)224, (byte)101, (byte)120, (byte)246, (byte)190, (byte)136, (byte)36, (byte)176, (byte)224, (byte)68, (byte)146, (byte)170, (byte)47, (byte)184, (byte)83, (byte)73, (byte)9, (byte)121, (byte)62, (byte)253, (byte)223, (byte)153, (byte)139, (byte)57, (byte)9, (byte)10, (byte)74, (byte)79, (byte)23, (byte)26, (byte)127, (byte)35, (byte)111, (byte)107, (byte)234, (byte)239, (byte)168, (byte)206, (byte)152, (byte)223, (byte)231, (byte)88, (byte)124, (byte)239, (byte)216, (byte)222, (byte)26, (byte)202, (byte)196, (byte)121, (byte)16, (byte)55, (byte)45, (byte)82, (byte)58, (byte)195, (byte)47, (byte)103, (byte)25, (byte)85, (byte)194, (byte)164, (byte)49, (byte)251, (byte)117, (byte)84, (byte)71, (byte)235, (byte)29, (byte)248, (byte)95, (byte)244, (byte)231, (byte)94, (byte)157, (byte)13, (byte)9, (byte)220, (byte)253, (byte)142, (byte)222, (byte)142, (byte)209, (byte)185, (byte)142, (byte)255, (byte)181, (byte)225, (byte)220, (byte)106, (byte)250, (byte)214, (byte)72, (byte)20, (byte)170, (byte)31, (byte)77, (byte)242, (byte)165}));
                Debug.Assert(pack.target_component == (byte)(byte)109);
                Debug.Assert(pack.target_system == (byte)(byte)38);
            };
            DemoDevice.GPS_INJECT_DATA p123 = LoopBackDemoChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_component = (byte)(byte)109;
            p123.data__SET(new byte[] {(byte)168, (byte)255, (byte)35, (byte)95, (byte)222, (byte)78, (byte)32, (byte)152, (byte)59, (byte)75, (byte)85, (byte)224, (byte)101, (byte)120, (byte)246, (byte)190, (byte)136, (byte)36, (byte)176, (byte)224, (byte)68, (byte)146, (byte)170, (byte)47, (byte)184, (byte)83, (byte)73, (byte)9, (byte)121, (byte)62, (byte)253, (byte)223, (byte)153, (byte)139, (byte)57, (byte)9, (byte)10, (byte)74, (byte)79, (byte)23, (byte)26, (byte)127, (byte)35, (byte)111, (byte)107, (byte)234, (byte)239, (byte)168, (byte)206, (byte)152, (byte)223, (byte)231, (byte)88, (byte)124, (byte)239, (byte)216, (byte)222, (byte)26, (byte)202, (byte)196, (byte)121, (byte)16, (byte)55, (byte)45, (byte)82, (byte)58, (byte)195, (byte)47, (byte)103, (byte)25, (byte)85, (byte)194, (byte)164, (byte)49, (byte)251, (byte)117, (byte)84, (byte)71, (byte)235, (byte)29, (byte)248, (byte)95, (byte)244, (byte)231, (byte)94, (byte)157, (byte)13, (byte)9, (byte)220, (byte)253, (byte)142, (byte)222, (byte)142, (byte)209, (byte)185, (byte)142, (byte)255, (byte)181, (byte)225, (byte)220, (byte)106, (byte)250, (byte)214, (byte)72, (byte)20, (byte)170, (byte)31, (byte)77, (byte)242, (byte)165}, 0) ;
            p123.target_system = (byte)(byte)38;
            p123.len = (byte)(byte)173;
            LoopBackDemoChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.dgps_numch == (byte)(byte)204);
                Debug.Assert(pack.epv == (ushort)(ushort)24701);
                Debug.Assert(pack.cog == (ushort)(ushort)10224);
                Debug.Assert(pack.time_usec == (ulong)3825327768737911220L);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
                Debug.Assert(pack.dgps_age == (uint)3504557798U);
                Debug.Assert(pack.lon == (int) -1723103112);
                Debug.Assert(pack.alt == (int) -735131726);
                Debug.Assert(pack.satellites_visible == (byte)(byte)161);
                Debug.Assert(pack.eph == (ushort)(ushort)15222);
                Debug.Assert(pack.lat == (int)1413427018);
                Debug.Assert(pack.vel == (ushort)(ushort)33386);
            };
            DemoDevice.GPS2_RAW p124 = LoopBackDemoChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.epv = (ushort)(ushort)24701;
            p124.satellites_visible = (byte)(byte)161;
            p124.vel = (ushort)(ushort)33386;
            p124.cog = (ushort)(ushort)10224;
            p124.eph = (ushort)(ushort)15222;
            p124.lon = (int) -1723103112;
            p124.alt = (int) -735131726;
            p124.lat = (int)1413427018;
            p124.time_usec = (ulong)3825327768737911220L;
            p124.dgps_numch = (byte)(byte)204;
            p124.dgps_age = (uint)3504557798U;
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            LoopBackDemoChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID);
                Debug.Assert(pack.Vservo == (ushort)(ushort)47978);
                Debug.Assert(pack.Vcc == (ushort)(ushort)35595);
            };
            DemoDevice.POWER_STATUS p125 = LoopBackDemoChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID;
            p125.Vcc = (ushort)(ushort)35595;
            p125.Vservo = (ushort)(ushort)47978;
            LoopBackDemoChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.device == (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)100, (byte)171, (byte)161, (byte)192, (byte)74, (byte)215, (byte)99, (byte)16, (byte)63, (byte)66, (byte)155, (byte)161, (byte)58, (byte)247, (byte)116, (byte)29, (byte)172, (byte)149, (byte)242, (byte)240, (byte)112, (byte)204, (byte)119, (byte)231, (byte)52, (byte)58, (byte)162, (byte)104, (byte)133, (byte)73, (byte)103, (byte)147, (byte)4, (byte)0, (byte)99, (byte)10, (byte)214, (byte)127, (byte)9, (byte)64, (byte)203, (byte)211, (byte)54, (byte)41, (byte)75, (byte)176, (byte)91, (byte)14, (byte)139, (byte)165, (byte)130, (byte)199, (byte)68, (byte)137, (byte)84, (byte)108, (byte)13, (byte)133, (byte)81, (byte)210, (byte)63, (byte)129, (byte)234, (byte)169, (byte)167, (byte)48, (byte)91, (byte)196, (byte)178, (byte)173}));
                Debug.Assert(pack.count == (byte)(byte)245);
                Debug.Assert(pack.baudrate == (uint)3342914214U);
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE);
                Debug.Assert(pack.timeout == (ushort)(ushort)6975);
            };
            DemoDevice.SERIAL_CONTROL p126 = LoopBackDemoChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE;
            p126.count = (byte)(byte)245;
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1;
            p126.timeout = (ushort)(ushort)6975;
            p126.data__SET(new byte[] {(byte)100, (byte)171, (byte)161, (byte)192, (byte)74, (byte)215, (byte)99, (byte)16, (byte)63, (byte)66, (byte)155, (byte)161, (byte)58, (byte)247, (byte)116, (byte)29, (byte)172, (byte)149, (byte)242, (byte)240, (byte)112, (byte)204, (byte)119, (byte)231, (byte)52, (byte)58, (byte)162, (byte)104, (byte)133, (byte)73, (byte)103, (byte)147, (byte)4, (byte)0, (byte)99, (byte)10, (byte)214, (byte)127, (byte)9, (byte)64, (byte)203, (byte)211, (byte)54, (byte)41, (byte)75, (byte)176, (byte)91, (byte)14, (byte)139, (byte)165, (byte)130, (byte)199, (byte)68, (byte)137, (byte)84, (byte)108, (byte)13, (byte)133, (byte)81, (byte)210, (byte)63, (byte)129, (byte)234, (byte)169, (byte)167, (byte)48, (byte)91, (byte)196, (byte)178, (byte)173}, 0) ;
            p126.baudrate = (uint)3342914214U;
            LoopBackDemoChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rtk_health == (byte)(byte)45);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)63);
                Debug.Assert(pack.baseline_c_mm == (int) -1573515806);
                Debug.Assert(pack.baseline_b_mm == (int) -1665106314);
                Debug.Assert(pack.nsats == (byte)(byte)126);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)179);
                Debug.Assert(pack.time_last_baseline_ms == (uint)2807906239U);
                Debug.Assert(pack.tow == (uint)2125819173U);
                Debug.Assert(pack.rtk_rate == (byte)(byte)233);
                Debug.Assert(pack.wn == (ushort)(ushort)32414);
                Debug.Assert(pack.baseline_a_mm == (int) -1465297092);
                Debug.Assert(pack.iar_num_hypotheses == (int)1417241387);
                Debug.Assert(pack.accuracy == (uint)2765653097U);
            };
            DemoDevice.GPS_RTK p127 = LoopBackDemoChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.rtk_rate = (byte)(byte)233;
            p127.baseline_coords_type = (byte)(byte)63;
            p127.time_last_baseline_ms = (uint)2807906239U;
            p127.rtk_receiver_id = (byte)(byte)179;
            p127.tow = (uint)2125819173U;
            p127.nsats = (byte)(byte)126;
            p127.accuracy = (uint)2765653097U;
            p127.baseline_b_mm = (int) -1665106314;
            p127.rtk_health = (byte)(byte)45;
            p127.baseline_a_mm = (int) -1465297092;
            p127.iar_num_hypotheses = (int)1417241387;
            p127.baseline_c_mm = (int) -1573515806;
            p127.wn = (ushort)(ushort)32414;
            LoopBackDemoChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.wn == (ushort)(ushort)11173);
                Debug.Assert(pack.baseline_c_mm == (int)736197307);
                Debug.Assert(pack.rtk_rate == (byte)(byte)106);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)219);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)129);
                Debug.Assert(pack.accuracy == (uint)1846801221U);
                Debug.Assert(pack.time_last_baseline_ms == (uint)3818120531U);
                Debug.Assert(pack.iar_num_hypotheses == (int) -257974167);
                Debug.Assert(pack.baseline_b_mm == (int)1975535995);
                Debug.Assert(pack.baseline_a_mm == (int) -873989353);
                Debug.Assert(pack.tow == (uint)957994712U);
                Debug.Assert(pack.rtk_health == (byte)(byte)197);
                Debug.Assert(pack.nsats == (byte)(byte)191);
            };
            DemoDevice.GPS2_RTK p128 = LoopBackDemoChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.iar_num_hypotheses = (int) -257974167;
            p128.baseline_c_mm = (int)736197307;
            p128.rtk_health = (byte)(byte)197;
            p128.rtk_receiver_id = (byte)(byte)129;
            p128.rtk_rate = (byte)(byte)106;
            p128.tow = (uint)957994712U;
            p128.nsats = (byte)(byte)191;
            p128.baseline_coords_type = (byte)(byte)219;
            p128.baseline_a_mm = (int) -873989353;
            p128.time_last_baseline_ms = (uint)3818120531U;
            p128.baseline_b_mm = (int)1975535995;
            p128.wn = (ushort)(ushort)11173;
            p128.accuracy = (uint)1846801221U;
            LoopBackDemoChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (short)(short) -17164);
                Debug.Assert(pack.xgyro == (short)(short) -27538);
                Debug.Assert(pack.ymag == (short)(short)30498);
                Debug.Assert(pack.ygyro == (short)(short) -26824);
                Debug.Assert(pack.time_boot_ms == (uint)609704418U);
                Debug.Assert(pack.xmag == (short)(short) -12575);
                Debug.Assert(pack.zmag == (short)(short) -15175);
                Debug.Assert(pack.zgyro == (short)(short)22456);
                Debug.Assert(pack.xacc == (short)(short) -12112);
                Debug.Assert(pack.zacc == (short)(short) -3261);
            };
            DemoDevice.SCALED_IMU3 p129 = LoopBackDemoChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.yacc = (short)(short) -17164;
            p129.xacc = (short)(short) -12112;
            p129.xmag = (short)(short) -12575;
            p129.zacc = (short)(short) -3261;
            p129.time_boot_ms = (uint)609704418U;
            p129.xgyro = (short)(short) -27538;
            p129.zmag = (short)(short) -15175;
            p129.ymag = (short)(short)30498;
            p129.zgyro = (short)(short)22456;
            p129.ygyro = (short)(short) -26824;
            LoopBackDemoChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.height == (ushort)(ushort)65217);
                Debug.Assert(pack.packets == (ushort)(ushort)10944);
                Debug.Assert(pack.size == (uint)3396135875U);
                Debug.Assert(pack.type == (byte)(byte)255);
                Debug.Assert(pack.width == (ushort)(ushort)3797);
                Debug.Assert(pack.payload == (byte)(byte)7);
                Debug.Assert(pack.jpg_quality == (byte)(byte)68);
            };
            DemoDevice.DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.size = (uint)3396135875U;
            p130.width = (ushort)(ushort)3797;
            p130.packets = (ushort)(ushort)10944;
            p130.height = (ushort)(ushort)65217;
            p130.type = (byte)(byte)255;
            p130.payload = (byte)(byte)7;
            p130.jpg_quality = (byte)(byte)68;
            LoopBackDemoChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)108, (byte)2, (byte)236, (byte)109, (byte)65, (byte)70, (byte)2, (byte)159, (byte)237, (byte)174, (byte)179, (byte)109, (byte)236, (byte)25, (byte)197, (byte)38, (byte)177, (byte)209, (byte)199, (byte)229, (byte)75, (byte)127, (byte)42, (byte)226, (byte)53, (byte)236, (byte)110, (byte)51, (byte)163, (byte)41, (byte)244, (byte)4, (byte)217, (byte)115, (byte)47, (byte)236, (byte)177, (byte)172, (byte)207, (byte)232, (byte)175, (byte)128, (byte)31, (byte)205, (byte)195, (byte)76, (byte)153, (byte)45, (byte)19, (byte)87, (byte)254, (byte)95, (byte)26, (byte)125, (byte)235, (byte)221, (byte)126, (byte)200, (byte)16, (byte)51, (byte)178, (byte)54, (byte)198, (byte)174, (byte)158, (byte)50, (byte)49, (byte)59, (byte)123, (byte)166, (byte)118, (byte)80, (byte)52, (byte)21, (byte)57, (byte)132, (byte)89, (byte)82, (byte)169, (byte)108, (byte)4, (byte)137, (byte)92, (byte)241, (byte)74, (byte)162, (byte)86, (byte)157, (byte)246, (byte)201, (byte)107, (byte)206, (byte)21, (byte)210, (byte)107, (byte)61, (byte)63, (byte)47, (byte)112, (byte)22, (byte)15, (byte)194, (byte)137, (byte)144, (byte)25, (byte)178, (byte)238, (byte)136, (byte)219, (byte)121, (byte)224, (byte)136, (byte)5, (byte)117, (byte)78, (byte)242, (byte)147, (byte)177, (byte)50, (byte)76, (byte)133, (byte)85, (byte)142, (byte)251, (byte)22, (byte)53, (byte)202, (byte)116, (byte)97, (byte)239, (byte)60, (byte)111, (byte)223, (byte)215, (byte)41, (byte)110, (byte)219, (byte)155, (byte)19, (byte)192, (byte)113, (byte)141, (byte)217, (byte)62, (byte)73, (byte)138, (byte)63, (byte)86, (byte)30, (byte)180, (byte)153, (byte)1, (byte)233, (byte)215, (byte)177, (byte)216, (byte)169, (byte)134, (byte)148, (byte)222, (byte)106, (byte)223, (byte)216, (byte)28, (byte)78, (byte)149, (byte)1, (byte)130, (byte)165, (byte)13, (byte)175, (byte)195, (byte)247, (byte)226, (byte)12, (byte)28, (byte)85, (byte)1, (byte)241, (byte)135, (byte)127, (byte)66, (byte)187, (byte)181, (byte)206, (byte)65, (byte)215, (byte)154, (byte)70, (byte)187, (byte)159, (byte)110, (byte)185, (byte)117, (byte)220, (byte)165, (byte)12, (byte)73, (byte)178, (byte)8, (byte)39, (byte)78, (byte)186, (byte)10, (byte)45, (byte)225, (byte)77, (byte)217, (byte)151, (byte)137, (byte)123, (byte)149, (byte)16, (byte)80, (byte)126, (byte)4, (byte)102, (byte)215, (byte)180, (byte)49, (byte)50, (byte)142, (byte)245, (byte)43, (byte)109, (byte)154, (byte)204, (byte)47, (byte)35, (byte)187, (byte)114, (byte)4, (byte)224, (byte)13, (byte)64, (byte)159, (byte)146, (byte)244, (byte)114, (byte)91, (byte)240, (byte)50, (byte)13, (byte)9, (byte)89, (byte)225, (byte)79, (byte)105, (byte)119, (byte)60, (byte)240, (byte)194, (byte)66}));
                Debug.Assert(pack.seqnr == (ushort)(ushort)5437);
            };
            DemoDevice.ENCAPSULATED_DATA p131 = LoopBackDemoChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)5437;
            p131.data__SET(new byte[] {(byte)108, (byte)2, (byte)236, (byte)109, (byte)65, (byte)70, (byte)2, (byte)159, (byte)237, (byte)174, (byte)179, (byte)109, (byte)236, (byte)25, (byte)197, (byte)38, (byte)177, (byte)209, (byte)199, (byte)229, (byte)75, (byte)127, (byte)42, (byte)226, (byte)53, (byte)236, (byte)110, (byte)51, (byte)163, (byte)41, (byte)244, (byte)4, (byte)217, (byte)115, (byte)47, (byte)236, (byte)177, (byte)172, (byte)207, (byte)232, (byte)175, (byte)128, (byte)31, (byte)205, (byte)195, (byte)76, (byte)153, (byte)45, (byte)19, (byte)87, (byte)254, (byte)95, (byte)26, (byte)125, (byte)235, (byte)221, (byte)126, (byte)200, (byte)16, (byte)51, (byte)178, (byte)54, (byte)198, (byte)174, (byte)158, (byte)50, (byte)49, (byte)59, (byte)123, (byte)166, (byte)118, (byte)80, (byte)52, (byte)21, (byte)57, (byte)132, (byte)89, (byte)82, (byte)169, (byte)108, (byte)4, (byte)137, (byte)92, (byte)241, (byte)74, (byte)162, (byte)86, (byte)157, (byte)246, (byte)201, (byte)107, (byte)206, (byte)21, (byte)210, (byte)107, (byte)61, (byte)63, (byte)47, (byte)112, (byte)22, (byte)15, (byte)194, (byte)137, (byte)144, (byte)25, (byte)178, (byte)238, (byte)136, (byte)219, (byte)121, (byte)224, (byte)136, (byte)5, (byte)117, (byte)78, (byte)242, (byte)147, (byte)177, (byte)50, (byte)76, (byte)133, (byte)85, (byte)142, (byte)251, (byte)22, (byte)53, (byte)202, (byte)116, (byte)97, (byte)239, (byte)60, (byte)111, (byte)223, (byte)215, (byte)41, (byte)110, (byte)219, (byte)155, (byte)19, (byte)192, (byte)113, (byte)141, (byte)217, (byte)62, (byte)73, (byte)138, (byte)63, (byte)86, (byte)30, (byte)180, (byte)153, (byte)1, (byte)233, (byte)215, (byte)177, (byte)216, (byte)169, (byte)134, (byte)148, (byte)222, (byte)106, (byte)223, (byte)216, (byte)28, (byte)78, (byte)149, (byte)1, (byte)130, (byte)165, (byte)13, (byte)175, (byte)195, (byte)247, (byte)226, (byte)12, (byte)28, (byte)85, (byte)1, (byte)241, (byte)135, (byte)127, (byte)66, (byte)187, (byte)181, (byte)206, (byte)65, (byte)215, (byte)154, (byte)70, (byte)187, (byte)159, (byte)110, (byte)185, (byte)117, (byte)220, (byte)165, (byte)12, (byte)73, (byte)178, (byte)8, (byte)39, (byte)78, (byte)186, (byte)10, (byte)45, (byte)225, (byte)77, (byte)217, (byte)151, (byte)137, (byte)123, (byte)149, (byte)16, (byte)80, (byte)126, (byte)4, (byte)102, (byte)215, (byte)180, (byte)49, (byte)50, (byte)142, (byte)245, (byte)43, (byte)109, (byte)154, (byte)204, (byte)47, (byte)35, (byte)187, (byte)114, (byte)4, (byte)224, (byte)13, (byte)64, (byte)159, (byte)146, (byte)244, (byte)114, (byte)91, (byte)240, (byte)50, (byte)13, (byte)9, (byte)89, (byte)225, (byte)79, (byte)105, (byte)119, (byte)60, (byte)240, (byte)194, (byte)66}, 0) ;
            LoopBackDemoChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current_distance == (ushort)(ushort)24554);
                Debug.Assert(pack.type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
                Debug.Assert(pack.max_distance == (ushort)(ushort)63074);
                Debug.Assert(pack.orientation == (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_90);
                Debug.Assert(pack.covariance == (byte)(byte)145);
                Debug.Assert(pack.time_boot_ms == (uint)3143233512U);
                Debug.Assert(pack.min_distance == (ushort)(ushort)62806);
                Debug.Assert(pack.id == (byte)(byte)163);
            };
            DemoDevice.DISTANCE_SENSOR p132 = LoopBackDemoChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)3143233512U;
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_90;
            p132.id = (byte)(byte)163;
            p132.covariance = (byte)(byte)145;
            p132.min_distance = (ushort)(ushort)62806;
            p132.current_distance = (ushort)(ushort)24554;
            p132.max_distance = (ushort)(ushort)63074;
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER;
            LoopBackDemoChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)1077707850);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)58391);
                Debug.Assert(pack.mask == (ulong)2425831210081282246L);
                Debug.Assert(pack.lat == (int)1038247757);
            };
            DemoDevice.TERRAIN_REQUEST p133 = LoopBackDemoChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int)1038247757;
            p133.lon = (int)1077707850;
            p133.mask = (ulong)2425831210081282246L;
            p133.grid_spacing = (ushort)(ushort)58391;
            LoopBackDemoChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short) -1027, (short)23223, (short)28604, (short)15889, (short) -17218, (short)25600, (short)12229, (short)6689, (short) -9242, (short)70, (short)134, (short) -6252, (short)29948, (short)12026, (short)14138, (short)25442}));
                Debug.Assert(pack.lon == (int) -350968827);
                Debug.Assert(pack.lat == (int)1848647112);
                Debug.Assert(pack.gridbit == (byte)(byte)165);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)56356);
            };
            DemoDevice.TERRAIN_DATA p134 = LoopBackDemoChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lon = (int) -350968827;
            p134.lat = (int)1848647112;
            p134.grid_spacing = (ushort)(ushort)56356;
            p134.gridbit = (byte)(byte)165;
            p134.data__SET(new short[] {(short) -1027, (short)23223, (short)28604, (short)15889, (short) -17218, (short)25600, (short)12229, (short)6689, (short) -9242, (short)70, (short)134, (short) -6252, (short)29948, (short)12026, (short)14138, (short)25442}, 0) ;
            LoopBackDemoChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -1645426465);
                Debug.Assert(pack.lon == (int)1384158229);
            };
            DemoDevice.TERRAIN_CHECK p135 = LoopBackDemoChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int) -1645426465;
            p135.lon = (int)1384158229;
            LoopBackDemoChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.spacing == (ushort)(ushort)28903);
                Debug.Assert(pack.terrain_height == (float)2.8839501E38F);
                Debug.Assert(pack.lon == (int) -794830035);
                Debug.Assert(pack.current_height == (float)1.3353322E38F);
                Debug.Assert(pack.pending == (ushort)(ushort)38752);
                Debug.Assert(pack.loaded == (ushort)(ushort)28702);
                Debug.Assert(pack.lat == (int)438724484);
            };
            DemoDevice.TERRAIN_REPORT p136 = LoopBackDemoChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.terrain_height = (float)2.8839501E38F;
            p136.loaded = (ushort)(ushort)28702;
            p136.spacing = (ushort)(ushort)28903;
            p136.pending = (ushort)(ushort)38752;
            p136.lat = (int)438724484;
            p136.current_height = (float)1.3353322E38F;
            p136.lon = (int) -794830035;
            LoopBackDemoChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float)1.1476418E38F);
                Debug.Assert(pack.temperature == (short)(short) -15737);
                Debug.Assert(pack.press_diff == (float)2.78426E37F);
                Debug.Assert(pack.time_boot_ms == (uint)867390924U);
            };
            DemoDevice.SCALED_PRESSURE2 p137 = LoopBackDemoChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.press_diff = (float)2.78426E37F;
            p137.time_boot_ms = (uint)867390924U;
            p137.press_abs = (float)1.1476418E38F;
            p137.temperature = (short)(short) -15737;
            LoopBackDemoChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {5.440317E37F, -1.9916505E38F, 2.4618515E38F, -3.9960071E37F}));
                Debug.Assert(pack.time_usec == (ulong)1305691609213151843L);
                Debug.Assert(pack.x == (float)3.0257834E36F);
                Debug.Assert(pack.y == (float) -1.4724548E38F);
                Debug.Assert(pack.z == (float)1.8410812E38F);
            };
            DemoDevice.ATT_POS_MOCAP p138 = LoopBackDemoChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.y = (float) -1.4724548E38F;
            p138.time_usec = (ulong)1305691609213151843L;
            p138.x = (float)3.0257834E36F;
            p138.q_SET(new float[] {5.440317E37F, -1.9916505E38F, 2.4618515E38F, -3.9960071E37F}, 0) ;
            p138.z = (float)1.8410812E38F;
            LoopBackDemoChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)159);
                Debug.Assert(pack.time_usec == (ulong)6369828790636813061L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-2.735531E38F, 2.8425097E38F, -2.6276847E38F, -9.867366E37F, -5.809194E37F, -3.8191037E37F, 1.5858198E38F, 1.7064023E38F}));
                Debug.Assert(pack.target_system == (byte)(byte)45);
                Debug.Assert(pack.group_mlx == (byte)(byte)80);
            };
            DemoDevice.SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)6369828790636813061L;
            p139.target_system = (byte)(byte)45;
            p139.target_component = (byte)(byte)159;
            p139.group_mlx = (byte)(byte)80;
            p139.controls_SET(new float[] {-2.735531E38F, 2.8425097E38F, -2.6276847E38F, -9.867366E37F, -5.809194E37F, -3.8191037E37F, 1.5858198E38F, 1.7064023E38F}, 0) ;
            LoopBackDemoChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.group_mlx == (byte)(byte)146);
                Debug.Assert(pack.time_usec == (ulong)4458535166551212345L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-3.367021E37F, -2.383614E38F, -1.5621552E37F, 2.1612218E37F, -2.0063163E38F, 2.925382E38F, -2.4163303E38F, 2.8802188E38F}));
            };
            DemoDevice.ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.group_mlx = (byte)(byte)146;
            p140.controls_SET(new float[] {-3.367021E37F, -2.383614E38F, -1.5621552E37F, 2.1612218E37F, -2.0063163E38F, 2.925382E38F, -2.4163303E38F, 2.8802188E38F}, 0) ;
            p140.time_usec = (ulong)4458535166551212345L;
            LoopBackDemoChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_local == (float)2.6963228E38F);
                Debug.Assert(pack.altitude_relative == (float) -1.2978567E38F);
                Debug.Assert(pack.altitude_amsl == (float)9.604571E36F);
                Debug.Assert(pack.altitude_monotonic == (float)1.8324891E38F);
                Debug.Assert(pack.altitude_terrain == (float)3.3616942E38F);
                Debug.Assert(pack.bottom_clearance == (float)5.658023E37F);
                Debug.Assert(pack.time_usec == (ulong)3100238445536105410L);
            };
            DemoDevice.ALTITUDE p141 = LoopBackDemoChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.altitude_local = (float)2.6963228E38F;
            p141.altitude_monotonic = (float)1.8324891E38F;
            p141.altitude_amsl = (float)9.604571E36F;
            p141.altitude_relative = (float) -1.2978567E38F;
            p141.time_usec = (ulong)3100238445536105410L;
            p141.altitude_terrain = (float)3.3616942E38F;
            p141.bottom_clearance = (float)5.658023E37F;
            LoopBackDemoChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)225, (byte)47, (byte)244, (byte)70, (byte)37, (byte)224, (byte)255, (byte)96, (byte)39, (byte)243, (byte)103, (byte)191, (byte)106, (byte)242, (byte)37, (byte)132, (byte)245, (byte)148, (byte)103, (byte)245, (byte)145, (byte)90, (byte)166, (byte)127, (byte)210, (byte)242, (byte)111, (byte)80, (byte)156, (byte)37, (byte)165, (byte)214, (byte)163, (byte)173, (byte)1, (byte)174, (byte)106, (byte)136, (byte)231, (byte)10, (byte)253, (byte)39, (byte)106, (byte)112, (byte)212, (byte)95, (byte)182, (byte)25, (byte)84, (byte)38, (byte)156, (byte)94, (byte)122, (byte)33, (byte)205, (byte)215, (byte)26, (byte)139, (byte)121, (byte)39, (byte)201, (byte)23, (byte)93, (byte)216, (byte)191, (byte)47, (byte)6, (byte)154, (byte)165, (byte)80, (byte)129, (byte)156, (byte)215, (byte)206, (byte)206, (byte)209, (byte)81, (byte)135, (byte)252, (byte)81, (byte)15, (byte)14, (byte)102, (byte)175, (byte)30, (byte)77, (byte)3, (byte)26, (byte)129, (byte)30, (byte)64, (byte)87, (byte)167, (byte)0, (byte)119, (byte)27, (byte)139, (byte)203, (byte)241, (byte)173, (byte)41, (byte)178, (byte)177, (byte)132, (byte)56, (byte)29, (byte)198, (byte)211, (byte)174, (byte)142, (byte)0, (byte)166, (byte)134, (byte)227, (byte)83, (byte)254, (byte)97, (byte)231, (byte)224, (byte)185}));
                Debug.Assert(pack.uri_type == (byte)(byte)140);
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)31, (byte)190, (byte)120, (byte)10, (byte)101, (byte)190, (byte)174, (byte)181, (byte)230, (byte)82, (byte)97, (byte)89, (byte)52, (byte)31, (byte)15, (byte)65, (byte)156, (byte)237, (byte)229, (byte)51, (byte)149, (byte)84, (byte)136, (byte)150, (byte)243, (byte)139, (byte)138, (byte)189, (byte)87, (byte)247, (byte)112, (byte)46, (byte)182, (byte)130, (byte)220, (byte)159, (byte)57, (byte)80, (byte)64, (byte)123, (byte)89, (byte)22, (byte)17, (byte)120, (byte)231, (byte)70, (byte)91, (byte)192, (byte)142, (byte)245, (byte)219, (byte)38, (byte)174, (byte)88, (byte)92, (byte)150, (byte)45, (byte)207, (byte)158, (byte)198, (byte)105, (byte)63, (byte)112, (byte)30, (byte)20, (byte)188, (byte)234, (byte)123, (byte)24, (byte)185, (byte)137, (byte)212, (byte)125, (byte)168, (byte)59, (byte)24, (byte)79, (byte)183, (byte)196, (byte)226, (byte)179, (byte)124, (byte)119, (byte)17, (byte)107, (byte)115, (byte)89, (byte)230, (byte)210, (byte)139, (byte)104, (byte)227, (byte)251, (byte)51, (byte)66, (byte)8, (byte)78, (byte)196, (byte)97, (byte)185, (byte)115, (byte)46, (byte)34, (byte)221, (byte)123, (byte)210, (byte)201, (byte)132, (byte)14, (byte)171, (byte)190, (byte)241, (byte)169, (byte)14, (byte)131, (byte)210, (byte)76, (byte)243, (byte)138, (byte)93}));
                Debug.Assert(pack.transfer_type == (byte)(byte)166);
                Debug.Assert(pack.request_id == (byte)(byte)104);
            };
            DemoDevice.RESOURCE_REQUEST p142 = LoopBackDemoChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)104;
            p142.storage_SET(new byte[] {(byte)31, (byte)190, (byte)120, (byte)10, (byte)101, (byte)190, (byte)174, (byte)181, (byte)230, (byte)82, (byte)97, (byte)89, (byte)52, (byte)31, (byte)15, (byte)65, (byte)156, (byte)237, (byte)229, (byte)51, (byte)149, (byte)84, (byte)136, (byte)150, (byte)243, (byte)139, (byte)138, (byte)189, (byte)87, (byte)247, (byte)112, (byte)46, (byte)182, (byte)130, (byte)220, (byte)159, (byte)57, (byte)80, (byte)64, (byte)123, (byte)89, (byte)22, (byte)17, (byte)120, (byte)231, (byte)70, (byte)91, (byte)192, (byte)142, (byte)245, (byte)219, (byte)38, (byte)174, (byte)88, (byte)92, (byte)150, (byte)45, (byte)207, (byte)158, (byte)198, (byte)105, (byte)63, (byte)112, (byte)30, (byte)20, (byte)188, (byte)234, (byte)123, (byte)24, (byte)185, (byte)137, (byte)212, (byte)125, (byte)168, (byte)59, (byte)24, (byte)79, (byte)183, (byte)196, (byte)226, (byte)179, (byte)124, (byte)119, (byte)17, (byte)107, (byte)115, (byte)89, (byte)230, (byte)210, (byte)139, (byte)104, (byte)227, (byte)251, (byte)51, (byte)66, (byte)8, (byte)78, (byte)196, (byte)97, (byte)185, (byte)115, (byte)46, (byte)34, (byte)221, (byte)123, (byte)210, (byte)201, (byte)132, (byte)14, (byte)171, (byte)190, (byte)241, (byte)169, (byte)14, (byte)131, (byte)210, (byte)76, (byte)243, (byte)138, (byte)93}, 0) ;
            p142.transfer_type = (byte)(byte)166;
            p142.uri_type = (byte)(byte)140;
            p142.uri_SET(new byte[] {(byte)225, (byte)47, (byte)244, (byte)70, (byte)37, (byte)224, (byte)255, (byte)96, (byte)39, (byte)243, (byte)103, (byte)191, (byte)106, (byte)242, (byte)37, (byte)132, (byte)245, (byte)148, (byte)103, (byte)245, (byte)145, (byte)90, (byte)166, (byte)127, (byte)210, (byte)242, (byte)111, (byte)80, (byte)156, (byte)37, (byte)165, (byte)214, (byte)163, (byte)173, (byte)1, (byte)174, (byte)106, (byte)136, (byte)231, (byte)10, (byte)253, (byte)39, (byte)106, (byte)112, (byte)212, (byte)95, (byte)182, (byte)25, (byte)84, (byte)38, (byte)156, (byte)94, (byte)122, (byte)33, (byte)205, (byte)215, (byte)26, (byte)139, (byte)121, (byte)39, (byte)201, (byte)23, (byte)93, (byte)216, (byte)191, (byte)47, (byte)6, (byte)154, (byte)165, (byte)80, (byte)129, (byte)156, (byte)215, (byte)206, (byte)206, (byte)209, (byte)81, (byte)135, (byte)252, (byte)81, (byte)15, (byte)14, (byte)102, (byte)175, (byte)30, (byte)77, (byte)3, (byte)26, (byte)129, (byte)30, (byte)64, (byte)87, (byte)167, (byte)0, (byte)119, (byte)27, (byte)139, (byte)203, (byte)241, (byte)173, (byte)41, (byte)178, (byte)177, (byte)132, (byte)56, (byte)29, (byte)198, (byte)211, (byte)174, (byte)142, (byte)0, (byte)166, (byte)134, (byte)227, (byte)83, (byte)254, (byte)97, (byte)231, (byte)224, (byte)185}, 0) ;
            LoopBackDemoChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1705606475U);
                Debug.Assert(pack.temperature == (short)(short) -20175);
                Debug.Assert(pack.press_abs == (float)1.0063175E37F);
                Debug.Assert(pack.press_diff == (float) -8.770973E37F);
            };
            DemoDevice.SCALED_PRESSURE3 p143 = LoopBackDemoChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.temperature = (short)(short) -20175;
            p143.press_abs = (float)1.0063175E37F;
            p143.time_boot_ms = (uint)1705606475U;
            p143.press_diff = (float) -8.770973E37F;
            LoopBackDemoChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.acc.SequenceEqual(new float[] {-2.3094904E38F, 1.9164776E38F, -2.1642039E38F}));
                Debug.Assert(pack.vel.SequenceEqual(new float[] {-1.2171798E38F, -1.9575673E38F, -3.123978E38F}));
                Debug.Assert(pack.est_capabilities == (byte)(byte)148);
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {1.7211217E38F, 1.3847185E38F, -2.491136E38F, 3.3148968E38F}));
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {2.5628824E38F, -2.3335537E38F, -1.261838E37F}));
                Debug.Assert(pack.lat == (int)1210173727);
                Debug.Assert(pack.lon == (int) -2085246808);
                Debug.Assert(pack.alt == (float) -2.5180573E38F);
                Debug.Assert(pack.custom_state == (ulong)7784697104885900809L);
                Debug.Assert(pack.rates.SequenceEqual(new float[] {2.991199E38F, -2.3246473E38F, 2.134212E38F}));
                Debug.Assert(pack.timestamp == (ulong)8764471713183968052L);
            };
            DemoDevice.FOLLOW_TARGET p144 = LoopBackDemoChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.rates_SET(new float[] {2.991199E38F, -2.3246473E38F, 2.134212E38F}, 0) ;
            p144.attitude_q_SET(new float[] {1.7211217E38F, 1.3847185E38F, -2.491136E38F, 3.3148968E38F}, 0) ;
            p144.position_cov_SET(new float[] {2.5628824E38F, -2.3335537E38F, -1.261838E37F}, 0) ;
            p144.vel_SET(new float[] {-1.2171798E38F, -1.9575673E38F, -3.123978E38F}, 0) ;
            p144.lat = (int)1210173727;
            p144.alt = (float) -2.5180573E38F;
            p144.custom_state = (ulong)7784697104885900809L;
            p144.acc_SET(new float[] {-2.3094904E38F, 1.9164776E38F, -2.1642039E38F}, 0) ;
            p144.est_capabilities = (byte)(byte)148;
            p144.timestamp = (ulong)8764471713183968052L;
            p144.lon = (int) -2085246808;
            LoopBackDemoChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch_rate == (float)1.0083313E38F);
                Debug.Assert(pack.z_vel == (float)2.9024165E38F);
                Debug.Assert(pack.time_usec == (ulong)2335447289733323511L);
                Debug.Assert(pack.x_pos == (float) -1.3562086E38F);
                Debug.Assert(pack.x_acc == (float) -1.7225855E38F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {-2.4348051E38F, 3.0921671E38F, 2.3444947E37F}));
                Debug.Assert(pack.x_vel == (float) -6.9091623E37F);
                Debug.Assert(pack.yaw_rate == (float)1.8683553E38F);
                Debug.Assert(pack.z_pos == (float) -4.3222124E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-7.382738E37F, 4.9570214E37F, 2.18263E38F, -1.5908583E38F}));
                Debug.Assert(pack.roll_rate == (float)2.2778673E38F);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {-3.0488155E38F, 9.203605E37F, -2.7164998E38F}));
                Debug.Assert(pack.airspeed == (float) -1.3251861E38F);
                Debug.Assert(pack.y_pos == (float)2.8596792E38F);
                Debug.Assert(pack.z_acc == (float)9.518899E37F);
                Debug.Assert(pack.y_vel == (float)1.9983715E38F);
                Debug.Assert(pack.y_acc == (float) -2.1101174E38F);
            };
            DemoDevice.CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.yaw_rate = (float)1.8683553E38F;
            p146.airspeed = (float) -1.3251861E38F;
            p146.y_pos = (float)2.8596792E38F;
            p146.y_vel = (float)1.9983715E38F;
            p146.roll_rate = (float)2.2778673E38F;
            p146.x_acc = (float) -1.7225855E38F;
            p146.z_vel = (float)2.9024165E38F;
            p146.time_usec = (ulong)2335447289733323511L;
            p146.pitch_rate = (float)1.0083313E38F;
            p146.pos_variance_SET(new float[] {-3.0488155E38F, 9.203605E37F, -2.7164998E38F}, 0) ;
            p146.vel_variance_SET(new float[] {-2.4348051E38F, 3.0921671E38F, 2.3444947E37F}, 0) ;
            p146.z_pos = (float) -4.3222124E37F;
            p146.x_pos = (float) -1.3562086E38F;
            p146.q_SET(new float[] {-7.382738E37F, 4.9570214E37F, 2.18263E38F, -1.5908583E38F}, 0) ;
            p146.z_acc = (float)9.518899E37F;
            p146.y_acc = (float) -2.1101174E38F;
            p146.x_vel = (float) -6.9091623E37F;
            LoopBackDemoChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (byte)(byte)120);
                Debug.Assert(pack.energy_consumed == (int) -798426507);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)4165, (ushort)42970, (ushort)6192, (ushort)37232, (ushort)48124, (ushort)5349, (ushort)25376, (ushort)45073, (ushort)22116, (ushort)47228}));
                Debug.Assert(pack.current_battery == (short)(short) -20005);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 76);
                Debug.Assert(pack.current_consumed == (int)1821372930);
                Debug.Assert(pack.temperature == (short)(short)7878);
                Debug.Assert(pack.type == (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN);
                Debug.Assert(pack.battery_function == (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS);
            };
            DemoDevice.BATTERY_STATUS p147 = LoopBackDemoChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.id = (byte)(byte)120;
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN;
            p147.current_consumed = (int)1821372930;
            p147.temperature = (short)(short)7878;
            p147.battery_remaining = (sbyte)(sbyte) - 76;
            p147.current_battery = (short)(short) -20005;
            p147.voltages_SET(new ushort[] {(ushort)4165, (ushort)42970, (ushort)6192, (ushort)37232, (ushort)48124, (ushort)5349, (ushort)25376, (ushort)45073, (ushort)22116, (ushort)47228}, 0) ;
            p147.energy_consumed = (int) -798426507;
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS;
            LoopBackDemoChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)60, (byte)91, (byte)11, (byte)117, (byte)122, (byte)12, (byte)91, (byte)41}));
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT);
                Debug.Assert(pack.flight_sw_version == (uint)409009961U);
                Debug.Assert(pack.os_sw_version == (uint)3997818574U);
                Debug.Assert(pack.product_id == (ushort)(ushort)63886);
                Debug.Assert(pack.board_version == (uint)1322435531U);
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)162, (byte)158, (byte)211, (byte)0, (byte)253, (byte)213, (byte)174, (byte)73}));
                Debug.Assert(pack.vendor_id == (ushort)(ushort)11434);
                Debug.Assert(pack.uid == (ulong)5273201730863815962L);
                Debug.Assert(pack.middleware_sw_version == (uint)3863463448U);
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)71, (byte)39, (byte)55, (byte)88, (byte)7, (byte)100, (byte)42, (byte)129}));
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)139, (byte)7, (byte)185, (byte)132, (byte)40, (byte)195, (byte)216, (byte)142, (byte)194, (byte)195, (byte)239, (byte)235, (byte)176, (byte)99, (byte)249, (byte)4, (byte)129, (byte)235}));
            };
            DemoDevice.AUTOPILOT_VERSION p148 = LoopBackDemoChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.uid = (ulong)5273201730863815962L;
            p148.os_custom_version_SET(new byte[] {(byte)162, (byte)158, (byte)211, (byte)0, (byte)253, (byte)213, (byte)174, (byte)73}, 0) ;
            p148.middleware_custom_version_SET(new byte[] {(byte)60, (byte)91, (byte)11, (byte)117, (byte)122, (byte)12, (byte)91, (byte)41}, 0) ;
            p148.flight_custom_version_SET(new byte[] {(byte)71, (byte)39, (byte)55, (byte)88, (byte)7, (byte)100, (byte)42, (byte)129}, 0) ;
            p148.vendor_id = (ushort)(ushort)11434;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT;
            p148.product_id = (ushort)(ushort)63886;
            p148.middleware_sw_version = (uint)3863463448U;
            p148.os_sw_version = (uint)3997818574U;
            p148.board_version = (uint)1322435531U;
            p148.uid2_SET(new byte[] {(byte)139, (byte)7, (byte)185, (byte)132, (byte)40, (byte)195, (byte)216, (byte)142, (byte)194, (byte)195, (byte)239, (byte)235, (byte)176, (byte)99, (byte)249, (byte)4, (byte)129, (byte)235}, 0, PH) ;
            p148.flight_sw_version = (uint)409009961U;
            LoopBackDemoChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x_TRY(ph) == (float)1.2901299E38F);
                Debug.Assert(pack.target_num == (byte)(byte)114);
                Debug.Assert(pack.size_x == (float)1.5426473E38F);
                Debug.Assert(pack.type == (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
                Debug.Assert(pack.time_usec == (ulong)5426870885532640304L);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)44);
                Debug.Assert(pack.z_TRY(ph) == (float)2.2655707E38F);
                Debug.Assert(pack.angle_y == (float) -9.751537E37F);
                Debug.Assert(pack.size_y == (float)8.414168E37F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT);
                Debug.Assert(pack.angle_x == (float)6.2168344E37F);
                Debug.Assert(pack.distance == (float)3.9048374E37F);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {-2.2468036E38F, 3.1020913E38F, 7.5305975E37F, 3.6897582E37F}));
                Debug.Assert(pack.y_TRY(ph) == (float)2.3206938E38F);
            };
            DemoDevice.LANDING_TARGET p149 = LoopBackDemoChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.distance = (float)3.9048374E37F;
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER;
            p149.position_valid_SET((byte)(byte)44, PH) ;
            p149.angle_x = (float)6.2168344E37F;
            p149.size_x = (float)1.5426473E38F;
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p149.size_y = (float)8.414168E37F;
            p149.q_SET(new float[] {-2.2468036E38F, 3.1020913E38F, 7.5305975E37F, 3.6897582E37F}, 0, PH) ;
            p149.x_SET((float)1.2901299E38F, PH) ;
            p149.angle_y = (float) -9.751537E37F;
            p149.time_usec = (ulong)5426870885532640304L;
            p149.target_num = (byte)(byte)114;
            p149.y_SET((float)2.3206938E38F, PH) ;
            p149.z_SET((float)2.2655707E38F, PH) ;
            LoopBackDemoChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCRIPT_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)10944);
                Debug.Assert(pack.name_LEN(ph) == 50);
                Debug.Assert(pack.name_TRY(ph).Equals("cprikbwnUsWvtxsbidohuohyfQflvmwqcooqdnuiofupjantjs"));
                Debug.Assert(pack.target_system == (byte)(byte)16);
                Debug.Assert(pack.target_component == (byte)(byte)43);
            };
            DemoDevice.SCRIPT_ITEM p180 = LoopBackDemoChannel.new_SCRIPT_ITEM();
            PH.setPack(p180);
            p180.target_component = (byte)(byte)43;
            p180.seq = (ushort)(ushort)10944;
            p180.name_SET("cprikbwnUsWvtxsbidohuohyfQflvmwqcooqdnuiofupjantjs", PH) ;
            p180.target_system = (byte)(byte)16;
            LoopBackDemoChannel.instance.send(p180);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCRIPT_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)26325);
                Debug.Assert(pack.target_system == (byte)(byte)33);
                Debug.Assert(pack.target_component == (byte)(byte)86);
            };
            DemoDevice.SCRIPT_REQUEST p181 = LoopBackDemoChannel.new_SCRIPT_REQUEST();
            PH.setPack(p181);
            p181.target_system = (byte)(byte)33;
            p181.target_component = (byte)(byte)86;
            p181.seq = (ushort)(ushort)26325;
            LoopBackDemoChannel.instance.send(p181);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCRIPT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)41);
                Debug.Assert(pack.target_component == (byte)(byte)40);
            };
            DemoDevice.SCRIPT_REQUEST_LIST p182 = LoopBackDemoChannel.new_SCRIPT_REQUEST_LIST();
            PH.setPack(p182);
            p182.target_component = (byte)(byte)40;
            p182.target_system = (byte)(byte)41;
            LoopBackDemoChannel.instance.send(p182);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCRIPT_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (ushort)(ushort)39849);
                Debug.Assert(pack.target_system == (byte)(byte)130);
                Debug.Assert(pack.target_component == (byte)(byte)13);
            };
            DemoDevice.SCRIPT_COUNT p183 = LoopBackDemoChannel.new_SCRIPT_COUNT();
            PH.setPack(p183);
            p183.target_system = (byte)(byte)130;
            p183.count = (ushort)(ushort)39849;
            p183.target_component = (byte)(byte)13;
            LoopBackDemoChannel.instance.send(p183);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCRIPT_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)53155);
            };
            DemoDevice.SCRIPT_CURRENT p184 = LoopBackDemoChannel.new_SCRIPT_CURRENT();
            PH.setPack(p184);
            p184.seq = (ushort)(ushort)53155;
            LoopBackDemoChannel.instance.send(p184);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pos_vert_accuracy == (float)2.9826713E38F);
                Debug.Assert(pack.pos_horiz_ratio == (float) -3.1372578E38F);
                Debug.Assert(pack.mag_ratio == (float)7.3583335E37F);
                Debug.Assert(pack.pos_vert_ratio == (float)1.7640518E38F);
                Debug.Assert(pack.vel_ratio == (float)3.2480115E38F);
                Debug.Assert(pack.tas_ratio == (float) -2.1511605E38F);
                Debug.Assert(pack.pos_horiz_accuracy == (float)1.4232535E38F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL);
                Debug.Assert(pack.hagl_ratio == (float)2.5511513E38F);
                Debug.Assert(pack.time_usec == (ulong)1456853162310887865L);
            };
            DemoDevice.ESTIMATOR_STATUS p230 = LoopBackDemoChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.time_usec = (ulong)1456853162310887865L;
            p230.pos_horiz_ratio = (float) -3.1372578E38F;
            p230.tas_ratio = (float) -2.1511605E38F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL;
            p230.hagl_ratio = (float)2.5511513E38F;
            p230.pos_horiz_accuracy = (float)1.4232535E38F;
            p230.pos_vert_ratio = (float)1.7640518E38F;
            p230.pos_vert_accuracy = (float)2.9826713E38F;
            p230.mag_ratio = (float)7.3583335E37F;
            p230.vel_ratio = (float)3.2480115E38F;
            LoopBackDemoChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vert_accuracy == (float) -1.6601985E38F);
                Debug.Assert(pack.wind_alt == (float)4.0529044E37F);
                Debug.Assert(pack.wind_z == (float) -6.769709E37F);
                Debug.Assert(pack.horiz_accuracy == (float) -1.6227045E38F);
                Debug.Assert(pack.time_usec == (ulong)7747471218635990349L);
                Debug.Assert(pack.var_horiz == (float) -4.9343457E37F);
                Debug.Assert(pack.wind_y == (float) -3.3375046E38F);
                Debug.Assert(pack.wind_x == (float) -9.753837E37F);
                Debug.Assert(pack.var_vert == (float) -1.7634078E37F);
            };
            DemoDevice.WIND_COV p231 = LoopBackDemoChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.wind_alt = (float)4.0529044E37F;
            p231.wind_z = (float) -6.769709E37F;
            p231.var_horiz = (float) -4.9343457E37F;
            p231.var_vert = (float) -1.7634078E37F;
            p231.vert_accuracy = (float) -1.6601985E38F;
            p231.horiz_accuracy = (float) -1.6227045E38F;
            p231.time_usec = (ulong)7747471218635990349L;
            p231.wind_y = (float) -3.3375046E38F;
            p231.wind_x = (float) -9.753837E37F;
            LoopBackDemoChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)3986347706196174752L);
                Debug.Assert(pack.vn == (float)3.4009999E38F);
                Debug.Assert(pack.gps_id == (byte)(byte)56);
                Debug.Assert(pack.time_week == (ushort)(ushort)6504);
                Debug.Assert(pack.fix_type == (byte)(byte)245);
                Debug.Assert(pack.lon == (int) -2135488543);
                Debug.Assert(pack.horiz_accuracy == (float)2.227798E38F);
                Debug.Assert(pack.ve == (float) -3.4758277E37F);
                Debug.Assert(pack.vdop == (float)2.9595532E38F);
                Debug.Assert(pack.time_week_ms == (uint)2934512952U);
                Debug.Assert(pack.vert_accuracy == (float) -9.394091E37F);
                Debug.Assert(pack.speed_accuracy == (float)1.8515785E38F);
                Debug.Assert(pack.alt == (float)2.1002085E37F);
                Debug.Assert(pack.lat == (int)135567880);
                Debug.Assert(pack.satellites_visible == (byte)(byte)253);
                Debug.Assert(pack.vd == (float)1.7227449E38F);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP);
                Debug.Assert(pack.hdop == (float) -2.5584067E38F);
            };
            DemoDevice.GPS_INPUT p232 = LoopBackDemoChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.time_week_ms = (uint)2934512952U;
            p232.hdop = (float) -2.5584067E38F;
            p232.time_usec = (ulong)3986347706196174752L;
            p232.vn = (float)3.4009999E38F;
            p232.gps_id = (byte)(byte)56;
            p232.satellites_visible = (byte)(byte)253;
            p232.lon = (int) -2135488543;
            p232.vd = (float)1.7227449E38F;
            p232.alt = (float)2.1002085E37F;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP;
            p232.ve = (float) -3.4758277E37F;
            p232.speed_accuracy = (float)1.8515785E38F;
            p232.time_week = (ushort)(ushort)6504;
            p232.fix_type = (byte)(byte)245;
            p232.vert_accuracy = (float) -9.394091E37F;
            p232.vdop = (float)2.9595532E38F;
            p232.lat = (int)135567880;
            p232.horiz_accuracy = (float)2.227798E38F;
            LoopBackDemoChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.len == (byte)(byte)86);
                Debug.Assert(pack.flags == (byte)(byte)200);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)1, (byte)176, (byte)176, (byte)77, (byte)224, (byte)132, (byte)110, (byte)22, (byte)194, (byte)116, (byte)244, (byte)15, (byte)118, (byte)172, (byte)144, (byte)23, (byte)146, (byte)19, (byte)22, (byte)57, (byte)148, (byte)240, (byte)6, (byte)119, (byte)112, (byte)47, (byte)244, (byte)15, (byte)196, (byte)32, (byte)4, (byte)180, (byte)224, (byte)37, (byte)205, (byte)208, (byte)91, (byte)79, (byte)30, (byte)75, (byte)108, (byte)38, (byte)126, (byte)76, (byte)68, (byte)73, (byte)58, (byte)57, (byte)18, (byte)3, (byte)175, (byte)140, (byte)17, (byte)98, (byte)249, (byte)159, (byte)68, (byte)17, (byte)148, (byte)112, (byte)51, (byte)201, (byte)251, (byte)7, (byte)139, (byte)180, (byte)254, (byte)14, (byte)156, (byte)158, (byte)223, (byte)247, (byte)61, (byte)210, (byte)176, (byte)176, (byte)208, (byte)126, (byte)228, (byte)243, (byte)156, (byte)133, (byte)136, (byte)146, (byte)111, (byte)18, (byte)165, (byte)94, (byte)4, (byte)44, (byte)88, (byte)140, (byte)245, (byte)82, (byte)253, (byte)132, (byte)79, (byte)134, (byte)198, (byte)241, (byte)10, (byte)120, (byte)68, (byte)247, (byte)123, (byte)146, (byte)191, (byte)220, (byte)39, (byte)0, (byte)69, (byte)20, (byte)188, (byte)36, (byte)235, (byte)145, (byte)74, (byte)137, (byte)143, (byte)161, (byte)218, (byte)29, (byte)223, (byte)243, (byte)67, (byte)228, (byte)66, (byte)73, (byte)186, (byte)188, (byte)254, (byte)102, (byte)6, (byte)244, (byte)245, (byte)16, (byte)131, (byte)107, (byte)48, (byte)41, (byte)254, (byte)120, (byte)15, (byte)121, (byte)104, (byte)89, (byte)228, (byte)26, (byte)8, (byte)21, (byte)251, (byte)0, (byte)1, (byte)31, (byte)132, (byte)180, (byte)232, (byte)220, (byte)20, (byte)123, (byte)98, (byte)212, (byte)198, (byte)10, (byte)133, (byte)53, (byte)109, (byte)45, (byte)142, (byte)149, (byte)129, (byte)183, (byte)175, (byte)146, (byte)51, (byte)58, (byte)18, (byte)186, (byte)3, (byte)206}));
            };
            DemoDevice.GPS_RTCM_DATA p233 = LoopBackDemoChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.data__SET(new byte[] {(byte)1, (byte)176, (byte)176, (byte)77, (byte)224, (byte)132, (byte)110, (byte)22, (byte)194, (byte)116, (byte)244, (byte)15, (byte)118, (byte)172, (byte)144, (byte)23, (byte)146, (byte)19, (byte)22, (byte)57, (byte)148, (byte)240, (byte)6, (byte)119, (byte)112, (byte)47, (byte)244, (byte)15, (byte)196, (byte)32, (byte)4, (byte)180, (byte)224, (byte)37, (byte)205, (byte)208, (byte)91, (byte)79, (byte)30, (byte)75, (byte)108, (byte)38, (byte)126, (byte)76, (byte)68, (byte)73, (byte)58, (byte)57, (byte)18, (byte)3, (byte)175, (byte)140, (byte)17, (byte)98, (byte)249, (byte)159, (byte)68, (byte)17, (byte)148, (byte)112, (byte)51, (byte)201, (byte)251, (byte)7, (byte)139, (byte)180, (byte)254, (byte)14, (byte)156, (byte)158, (byte)223, (byte)247, (byte)61, (byte)210, (byte)176, (byte)176, (byte)208, (byte)126, (byte)228, (byte)243, (byte)156, (byte)133, (byte)136, (byte)146, (byte)111, (byte)18, (byte)165, (byte)94, (byte)4, (byte)44, (byte)88, (byte)140, (byte)245, (byte)82, (byte)253, (byte)132, (byte)79, (byte)134, (byte)198, (byte)241, (byte)10, (byte)120, (byte)68, (byte)247, (byte)123, (byte)146, (byte)191, (byte)220, (byte)39, (byte)0, (byte)69, (byte)20, (byte)188, (byte)36, (byte)235, (byte)145, (byte)74, (byte)137, (byte)143, (byte)161, (byte)218, (byte)29, (byte)223, (byte)243, (byte)67, (byte)228, (byte)66, (byte)73, (byte)186, (byte)188, (byte)254, (byte)102, (byte)6, (byte)244, (byte)245, (byte)16, (byte)131, (byte)107, (byte)48, (byte)41, (byte)254, (byte)120, (byte)15, (byte)121, (byte)104, (byte)89, (byte)228, (byte)26, (byte)8, (byte)21, (byte)251, (byte)0, (byte)1, (byte)31, (byte)132, (byte)180, (byte)232, (byte)220, (byte)20, (byte)123, (byte)98, (byte)212, (byte)198, (byte)10, (byte)133, (byte)53, (byte)109, (byte)45, (byte)142, (byte)149, (byte)129, (byte)183, (byte)175, (byte)146, (byte)51, (byte)58, (byte)18, (byte)186, (byte)3, (byte)206}, 0) ;
            p233.flags = (byte)(byte)200;
            p233.len = (byte)(byte)86;
            LoopBackDemoChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.battery_remaining == (byte)(byte)122);
                Debug.Assert(pack.altitude_sp == (short)(short)2683);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
                Debug.Assert(pack.temperature == (sbyte)(sbyte)43);
                Debug.Assert(pack.gps_fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
                Debug.Assert(pack.gps_nsat == (byte)(byte)80);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte)88);
                Debug.Assert(pack.custom_mode == (uint)2418295701U);
                Debug.Assert(pack.heading_sp == (short)(short)12844);
                Debug.Assert(pack.throttle == (sbyte)(sbyte)103);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte) - 60);
                Debug.Assert(pack.roll == (short)(short) -13708);
                Debug.Assert(pack.heading == (ushort)(ushort)35011);
                Debug.Assert(pack.failsafe == (byte)(byte)33);
                Debug.Assert(pack.longitude == (int)1881408902);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)156);
                Debug.Assert(pack.altitude_amsl == (short)(short) -27775);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)9141);
                Debug.Assert(pack.latitude == (int)1825600461);
                Debug.Assert(pack.wp_num == (byte)(byte)132);
                Debug.Assert(pack.groundspeed == (byte)(byte)206);
                Debug.Assert(pack.airspeed == (byte)(byte)27);
                Debug.Assert(pack.pitch == (short)(short) -12226);
            };
            DemoDevice.HIGH_LATENCY p234 = LoopBackDemoChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.heading_sp = (short)(short)12844;
            p234.failsafe = (byte)(byte)33;
            p234.latitude = (int)1825600461;
            p234.temperature = (sbyte)(sbyte)43;
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX;
            p234.heading = (ushort)(ushort)35011;
            p234.groundspeed = (byte)(byte)206;
            p234.airspeed = (byte)(byte)27;
            p234.throttle = (sbyte)(sbyte)103;
            p234.roll = (short)(short) -13708;
            p234.wp_num = (byte)(byte)132;
            p234.airspeed_sp = (byte)(byte)156;
            p234.altitude_sp = (short)(short)2683;
            p234.gps_nsat = (byte)(byte)80;
            p234.custom_mode = (uint)2418295701U;
            p234.battery_remaining = (byte)(byte)122;
            p234.temperature_air = (sbyte)(sbyte) - 60;
            p234.pitch = (short)(short) -12226;
            p234.altitude_amsl = (short)(short) -27775;
            p234.climb_rate = (sbyte)(sbyte)88;
            p234.longitude = (int)1881408902;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED;
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED;
            p234.wp_distance = (ushort)(ushort)9141;
            LoopBackDemoChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.clipping_0 == (uint)3212568748U);
                Debug.Assert(pack.vibration_x == (float)1.8060145E38F);
                Debug.Assert(pack.time_usec == (ulong)9040995409536432057L);
                Debug.Assert(pack.clipping_1 == (uint)2801272946U);
                Debug.Assert(pack.clipping_2 == (uint)1648208157U);
                Debug.Assert(pack.vibration_y == (float)1.3959462E38F);
                Debug.Assert(pack.vibration_z == (float)2.5102143E38F);
            };
            DemoDevice.VIBRATION p241 = LoopBackDemoChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.vibration_y = (float)1.3959462E38F;
            p241.vibration_z = (float)2.5102143E38F;
            p241.time_usec = (ulong)9040995409536432057L;
            p241.clipping_0 = (uint)3212568748U;
            p241.clipping_2 = (uint)1648208157U;
            p241.vibration_x = (float)1.8060145E38F;
            p241.clipping_1 = (uint)2801272946U;
            LoopBackDemoChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.longitude == (int) -880403820);
                Debug.Assert(pack.y == (float) -1.6987682E38F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)6034540791293686489L);
                Debug.Assert(pack.altitude == (int)640106339);
                Debug.Assert(pack.approach_y == (float) -2.070992E36F);
                Debug.Assert(pack.x == (float) -3.313667E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-6.00989E37F, 1.889284E38F, 2.9379527E38F, -1.6194577E38F}));
                Debug.Assert(pack.approach_z == (float)1.5331519E38F);
                Debug.Assert(pack.latitude == (int) -470355545);
                Debug.Assert(pack.approach_x == (float) -4.0635952E37F);
                Debug.Assert(pack.z == (float)1.9061496E38F);
            };
            DemoDevice.HOME_POSITION p242 = LoopBackDemoChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.time_usec_SET((ulong)6034540791293686489L, PH) ;
            p242.q_SET(new float[] {-6.00989E37F, 1.889284E38F, 2.9379527E38F, -1.6194577E38F}, 0) ;
            p242.x = (float) -3.313667E38F;
            p242.longitude = (int) -880403820;
            p242.altitude = (int)640106339;
            p242.approach_x = (float) -4.0635952E37F;
            p242.approach_y = (float) -2.070992E36F;
            p242.latitude = (int) -470355545;
            p242.y = (float) -1.6987682E38F;
            p242.z = (float)1.9061496E38F;
            p242.approach_z = (float)1.5331519E38F;
            LoopBackDemoChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)508090507076509625L);
                Debug.Assert(pack.altitude == (int)1231362752);
                Debug.Assert(pack.approach_x == (float) -1.2009337E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-3.3989911E38F, 2.8332705E38F, 3.3787752E38F, -2.6594985E37F}));
                Debug.Assert(pack.longitude == (int)838571571);
                Debug.Assert(pack.target_system == (byte)(byte)125);
                Debug.Assert(pack.approach_z == (float)3.2817823E38F);
                Debug.Assert(pack.approach_y == (float)2.2846662E38F);
                Debug.Assert(pack.latitude == (int) -1997130487);
                Debug.Assert(pack.x == (float) -2.269883E38F);
                Debug.Assert(pack.y == (float)5.0271433E37F);
                Debug.Assert(pack.z == (float) -1.0116094E38F);
            };
            DemoDevice.SET_HOME_POSITION p243 = LoopBackDemoChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.x = (float) -2.269883E38F;
            p243.q_SET(new float[] {-3.3989911E38F, 2.8332705E38F, 3.3787752E38F, -2.6594985E37F}, 0) ;
            p243.z = (float) -1.0116094E38F;
            p243.approach_x = (float) -1.2009337E38F;
            p243.time_usec_SET((ulong)508090507076509625L, PH) ;
            p243.target_system = (byte)(byte)125;
            p243.latitude = (int) -1997130487;
            p243.y = (float)5.0271433E37F;
            p243.altitude = (int)1231362752;
            p243.longitude = (int)838571571;
            p243.approach_y = (float)2.2846662E38F;
            p243.approach_z = (float)3.2817823E38F;
            LoopBackDemoChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.interval_us == (int) -1569513331);
                Debug.Assert(pack.message_id == (ushort)(ushort)58051);
            };
            DemoDevice.MESSAGE_INTERVAL p244 = LoopBackDemoChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)58051;
            p244.interval_us = (int) -1569513331;
            LoopBackDemoChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vtol_state == (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
            };
            DemoDevice.EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR;
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC;
            LoopBackDemoChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_SIMULATED);
                Debug.Assert(pack.heading == (ushort)(ushort)19504);
                Debug.Assert(pack.altitude_type == (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
                Debug.Assert(pack.ver_velocity == (short)(short) -31568);
                Debug.Assert(pack.lon == (int) -49219173);
                Debug.Assert(pack.tslc == (byte)(byte)66);
                Debug.Assert(pack.emitter_type == (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHTER_AIR);
                Debug.Assert(pack.ICAO_address == (uint)2432962204U);
                Debug.Assert(pack.altitude == (int)1523734892);
                Debug.Assert(pack.lat == (int) -1492221460);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)19341);
                Debug.Assert(pack.callsign_LEN(ph) == 7);
                Debug.Assert(pack.callsign_TRY(ph).Equals("mdvioaq"));
                Debug.Assert(pack.squawk == (ushort)(ushort)17750);
            };
            DemoDevice.ADSB_VEHICLE p246 = LoopBackDemoChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.altitude = (int)1523734892;
            p246.ver_velocity = (short)(short) -31568;
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.tslc = (byte)(byte)66;
            p246.ICAO_address = (uint)2432962204U;
            p246.lat = (int) -1492221460;
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_SIMULATED;
            p246.squawk = (ushort)(ushort)17750;
            p246.hor_velocity = (ushort)(ushort)19341;
            p246.lon = (int) -49219173;
            p246.heading = (ushort)(ushort)19504;
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHTER_AIR;
            p246.callsign_SET("mdvioaq", PH) ;
            LoopBackDemoChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (uint)2879700844U);
                Debug.Assert(pack.altitude_minimum_delta == (float)8.0536855E37F);
                Debug.Assert(pack.time_to_minimum_delta == (float) -1.1290913E38F);
                Debug.Assert(pack.action == (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER);
                Debug.Assert(pack.threat_level == (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
                Debug.Assert(pack.src_ == (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
                Debug.Assert(pack.horizontal_minimum_delta == (float)2.5565448E38F);
            };
            DemoDevice.COLLISION p247 = LoopBackDemoChannel.new_COLLISION();
            PH.setPack(p247);
            p247.id = (uint)2879700844U;
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE;
            p247.altitude_minimum_delta = (float)8.0536855E37F;
            p247.horizontal_minimum_delta = (float)2.5565448E38F;
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER;
            p247.time_to_minimum_delta = (float) -1.1290913E38F;
            LoopBackDemoChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)149);
                Debug.Assert(pack.message_type == (ushort)(ushort)7938);
                Debug.Assert(pack.target_network == (byte)(byte)99);
                Debug.Assert(pack.target_component == (byte)(byte)180);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)121, (byte)31, (byte)60, (byte)209, (byte)71, (byte)153, (byte)129, (byte)231, (byte)174, (byte)95, (byte)142, (byte)172, (byte)166, (byte)187, (byte)110, (byte)33, (byte)227, (byte)153, (byte)103, (byte)73, (byte)219, (byte)20, (byte)52, (byte)21, (byte)93, (byte)150, (byte)29, (byte)203, (byte)119, (byte)61, (byte)184, (byte)50, (byte)85, (byte)104, (byte)167, (byte)168, (byte)124, (byte)132, (byte)10, (byte)52, (byte)125, (byte)97, (byte)204, (byte)172, (byte)207, (byte)85, (byte)3, (byte)85, (byte)79, (byte)100, (byte)7, (byte)80, (byte)31, (byte)43, (byte)89, (byte)17, (byte)114, (byte)200, (byte)35, (byte)27, (byte)249, (byte)106, (byte)241, (byte)20, (byte)154, (byte)245, (byte)195, (byte)104, (byte)210, (byte)221, (byte)50, (byte)79, (byte)118, (byte)18, (byte)225, (byte)227, (byte)29, (byte)188, (byte)163, (byte)69, (byte)226, (byte)22, (byte)233, (byte)62, (byte)219, (byte)53, (byte)72, (byte)67, (byte)106, (byte)133, (byte)137, (byte)87, (byte)13, (byte)106, (byte)146, (byte)198, (byte)230, (byte)179, (byte)90, (byte)23, (byte)33, (byte)229, (byte)255, (byte)203, (byte)168, (byte)211, (byte)120, (byte)180, (byte)23, (byte)46, (byte)33, (byte)47, (byte)230, (byte)160, (byte)23, (byte)233, (byte)37, (byte)67, (byte)205, (byte)127, (byte)25, (byte)53, (byte)171, (byte)138, (byte)90, (byte)86, (byte)189, (byte)77, (byte)41, (byte)238, (byte)141, (byte)135, (byte)36, (byte)226, (byte)141, (byte)154, (byte)81, (byte)194, (byte)191, (byte)35, (byte)172, (byte)217, (byte)66, (byte)27, (byte)211, (byte)120, (byte)41, (byte)79, (byte)191, (byte)144, (byte)171, (byte)255, (byte)45, (byte)250, (byte)52, (byte)247, (byte)180, (byte)149, (byte)71, (byte)198, (byte)64, (byte)204, (byte)115, (byte)160, (byte)25, (byte)188, (byte)93, (byte)4, (byte)233, (byte)144, (byte)146, (byte)160, (byte)113, (byte)173, (byte)150, (byte)146, (byte)247, (byte)16, (byte)85, (byte)47, (byte)38, (byte)139, (byte)129, (byte)171, (byte)127, (byte)118, (byte)163, (byte)185, (byte)57, (byte)16, (byte)33, (byte)22, (byte)177, (byte)216, (byte)154, (byte)164, (byte)202, (byte)109, (byte)191, (byte)121, (byte)20, (byte)54, (byte)2, (byte)71, (byte)39, (byte)62, (byte)10, (byte)154, (byte)92, (byte)102, (byte)219, (byte)87, (byte)245, (byte)169, (byte)100, (byte)226, (byte)243, (byte)67, (byte)233, (byte)106, (byte)41, (byte)108, (byte)60, (byte)108, (byte)58, (byte)233, (byte)88, (byte)37, (byte)172, (byte)121, (byte)9, (byte)39, (byte)88, (byte)18, (byte)202, (byte)56, (byte)187, (byte)155, (byte)27, (byte)67, (byte)200, (byte)25, (byte)92, (byte)22, (byte)232, (byte)29, (byte)208, (byte)126, (byte)147}));
            };
            DemoDevice.V2_EXTENSION p248 = LoopBackDemoChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_system = (byte)(byte)149;
            p248.payload_SET(new byte[] {(byte)121, (byte)31, (byte)60, (byte)209, (byte)71, (byte)153, (byte)129, (byte)231, (byte)174, (byte)95, (byte)142, (byte)172, (byte)166, (byte)187, (byte)110, (byte)33, (byte)227, (byte)153, (byte)103, (byte)73, (byte)219, (byte)20, (byte)52, (byte)21, (byte)93, (byte)150, (byte)29, (byte)203, (byte)119, (byte)61, (byte)184, (byte)50, (byte)85, (byte)104, (byte)167, (byte)168, (byte)124, (byte)132, (byte)10, (byte)52, (byte)125, (byte)97, (byte)204, (byte)172, (byte)207, (byte)85, (byte)3, (byte)85, (byte)79, (byte)100, (byte)7, (byte)80, (byte)31, (byte)43, (byte)89, (byte)17, (byte)114, (byte)200, (byte)35, (byte)27, (byte)249, (byte)106, (byte)241, (byte)20, (byte)154, (byte)245, (byte)195, (byte)104, (byte)210, (byte)221, (byte)50, (byte)79, (byte)118, (byte)18, (byte)225, (byte)227, (byte)29, (byte)188, (byte)163, (byte)69, (byte)226, (byte)22, (byte)233, (byte)62, (byte)219, (byte)53, (byte)72, (byte)67, (byte)106, (byte)133, (byte)137, (byte)87, (byte)13, (byte)106, (byte)146, (byte)198, (byte)230, (byte)179, (byte)90, (byte)23, (byte)33, (byte)229, (byte)255, (byte)203, (byte)168, (byte)211, (byte)120, (byte)180, (byte)23, (byte)46, (byte)33, (byte)47, (byte)230, (byte)160, (byte)23, (byte)233, (byte)37, (byte)67, (byte)205, (byte)127, (byte)25, (byte)53, (byte)171, (byte)138, (byte)90, (byte)86, (byte)189, (byte)77, (byte)41, (byte)238, (byte)141, (byte)135, (byte)36, (byte)226, (byte)141, (byte)154, (byte)81, (byte)194, (byte)191, (byte)35, (byte)172, (byte)217, (byte)66, (byte)27, (byte)211, (byte)120, (byte)41, (byte)79, (byte)191, (byte)144, (byte)171, (byte)255, (byte)45, (byte)250, (byte)52, (byte)247, (byte)180, (byte)149, (byte)71, (byte)198, (byte)64, (byte)204, (byte)115, (byte)160, (byte)25, (byte)188, (byte)93, (byte)4, (byte)233, (byte)144, (byte)146, (byte)160, (byte)113, (byte)173, (byte)150, (byte)146, (byte)247, (byte)16, (byte)85, (byte)47, (byte)38, (byte)139, (byte)129, (byte)171, (byte)127, (byte)118, (byte)163, (byte)185, (byte)57, (byte)16, (byte)33, (byte)22, (byte)177, (byte)216, (byte)154, (byte)164, (byte)202, (byte)109, (byte)191, (byte)121, (byte)20, (byte)54, (byte)2, (byte)71, (byte)39, (byte)62, (byte)10, (byte)154, (byte)92, (byte)102, (byte)219, (byte)87, (byte)245, (byte)169, (byte)100, (byte)226, (byte)243, (byte)67, (byte)233, (byte)106, (byte)41, (byte)108, (byte)60, (byte)108, (byte)58, (byte)233, (byte)88, (byte)37, (byte)172, (byte)121, (byte)9, (byte)39, (byte)88, (byte)18, (byte)202, (byte)56, (byte)187, (byte)155, (byte)27, (byte)67, (byte)200, (byte)25, (byte)92, (byte)22, (byte)232, (byte)29, (byte)208, (byte)126, (byte)147}, 0) ;
            p248.target_network = (byte)(byte)99;
            p248.message_type = (ushort)(ushort)7938;
            p248.target_component = (byte)(byte)180;
            LoopBackDemoChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte)53, (sbyte) - 101, (sbyte) - 102, (sbyte)125, (sbyte)24, (sbyte)93, (sbyte) - 60, (sbyte) - 75, (sbyte) - 69, (sbyte) - 104, (sbyte)58, (sbyte)41, (sbyte)1, (sbyte) - 119, (sbyte) - 95, (sbyte)0, (sbyte) - 125, (sbyte) - 10, (sbyte) - 7, (sbyte) - 53, (sbyte)33, (sbyte) - 5, (sbyte) - 29, (sbyte)3, (sbyte)23, (sbyte)53, (sbyte) - 86, (sbyte)107, (sbyte) - 70, (sbyte)46, (sbyte) - 102, (sbyte) - 94}));
                Debug.Assert(pack.type == (byte)(byte)108);
                Debug.Assert(pack.address == (ushort)(ushort)64421);
                Debug.Assert(pack.ver == (byte)(byte)247);
            };
            DemoDevice.MEMORY_VECT p249 = LoopBackDemoChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.value_SET(new sbyte[] {(sbyte)53, (sbyte) - 101, (sbyte) - 102, (sbyte)125, (sbyte)24, (sbyte)93, (sbyte) - 60, (sbyte) - 75, (sbyte) - 69, (sbyte) - 104, (sbyte)58, (sbyte)41, (sbyte)1, (sbyte) - 119, (sbyte) - 95, (sbyte)0, (sbyte) - 125, (sbyte) - 10, (sbyte) - 7, (sbyte) - 53, (sbyte)33, (sbyte) - 5, (sbyte) - 29, (sbyte)3, (sbyte)23, (sbyte)53, (sbyte) - 86, (sbyte)107, (sbyte) - 70, (sbyte)46, (sbyte) - 102, (sbyte) - 94}, 0) ;
            p249.ver = (byte)(byte)247;
            p249.address = (ushort)(ushort)64421;
            p249.type = (byte)(byte)108;
            LoopBackDemoChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 7);
                Debug.Assert(pack.name_TRY(ph).Equals("xevgMih"));
                Debug.Assert(pack.z == (float) -1.7302964E38F);
                Debug.Assert(pack.y == (float)1.7036113E38F);
                Debug.Assert(pack.time_usec == (ulong)547301970848426723L);
                Debug.Assert(pack.x == (float)3.3319478E38F);
            };
            DemoDevice.DEBUG_VECT p250 = LoopBackDemoChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.y = (float)1.7036113E38F;
            p250.x = (float)3.3319478E38F;
            p250.name_SET("xevgMih", PH) ;
            p250.time_usec = (ulong)547301970848426723L;
            p250.z = (float) -1.7302964E38F;
            LoopBackDemoChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2443989281U);
                Debug.Assert(pack.value == (float) -2.8867952E38F);
                Debug.Assert(pack.name_LEN(ph) == 1);
                Debug.Assert(pack.name_TRY(ph).Equals("z"));
            };
            DemoDevice.NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.name_SET("z", PH) ;
            p251.value = (float) -2.8867952E38F;
            p251.time_boot_ms = (uint)2443989281U;
            LoopBackDemoChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (int) -2070839769);
                Debug.Assert(pack.name_LEN(ph) == 10);
                Debug.Assert(pack.name_TRY(ph).Equals("rjqonmlcpm"));
                Debug.Assert(pack.time_boot_ms == (uint)836695953U);
            };
            DemoDevice.NAMED_VALUE_INT p252 = LoopBackDemoChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.value = (int) -2070839769;
            p252.time_boot_ms = (uint)836695953U;
            p252.name_SET("rjqonmlcpm", PH) ;
            LoopBackDemoChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.severity == (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_INFO);
                Debug.Assert(pack.text_LEN(ph) == 17);
                Debug.Assert(pack.text_TRY(ph).Equals("qusZyqoxmxrriepor"));
            };
            DemoDevice.STATUSTEXT p253 = LoopBackDemoChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.text_SET("qusZyqoxmxrriepor", PH) ;
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_INFO;
            LoopBackDemoChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ind == (byte)(byte)81);
                Debug.Assert(pack.time_boot_ms == (uint)3182766809U);
                Debug.Assert(pack.value == (float)2.6343459E38F);
            };
            DemoDevice.DEBUG p254 = LoopBackDemoChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)3182766809U;
            p254.ind = (byte)(byte)81;
            p254.value = (float)2.6343459E38F;
            LoopBackDemoChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.initial_timestamp == (ulong)7110854564521091129L);
                Debug.Assert(pack.target_component == (byte)(byte)191);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)248, (byte)125, (byte)20, (byte)249, (byte)15, (byte)129, (byte)227, (byte)115, (byte)242, (byte)193, (byte)90, (byte)144, (byte)229, (byte)94, (byte)153, (byte)113, (byte)249, (byte)206, (byte)137, (byte)175, (byte)81, (byte)10, (byte)172, (byte)179, (byte)56, (byte)215, (byte)4, (byte)76, (byte)2, (byte)115, (byte)114, (byte)205}));
                Debug.Assert(pack.target_system == (byte)(byte)174);
            };
            DemoDevice.SETUP_SIGNING p256 = LoopBackDemoChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.initial_timestamp = (ulong)7110854564521091129L;
            p256.target_component = (byte)(byte)191;
            p256.target_system = (byte)(byte)174;
            p256.secret_key_SET(new byte[] {(byte)248, (byte)125, (byte)20, (byte)249, (byte)15, (byte)129, (byte)227, (byte)115, (byte)242, (byte)193, (byte)90, (byte)144, (byte)229, (byte)94, (byte)153, (byte)113, (byte)249, (byte)206, (byte)137, (byte)175, (byte)81, (byte)10, (byte)172, (byte)179, (byte)56, (byte)215, (byte)4, (byte)76, (byte)2, (byte)115, (byte)114, (byte)205}, 0) ;
            LoopBackDemoChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.last_change_ms == (uint)2410825933U);
                Debug.Assert(pack.state == (byte)(byte)205);
                Debug.Assert(pack.time_boot_ms == (uint)2472596358U);
            };
            DemoDevice.BUTTON_CHANGE p257 = LoopBackDemoChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)2472596358U;
            p257.last_change_ms = (uint)2410825933U;
            p257.state = (byte)(byte)205;
            LoopBackDemoChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tune_LEN(ph) == 14);
                Debug.Assert(pack.tune_TRY(ph).Equals("oyoxzjqowtsrva"));
                Debug.Assert(pack.target_component == (byte)(byte)7);
                Debug.Assert(pack.target_system == (byte)(byte)51);
            };
            DemoDevice.PLAY_TUNE p258 = LoopBackDemoChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)51;
            p258.target_component = (byte)(byte)7;
            p258.tune_SET("oyoxzjqowtsrva", PH) ;
            LoopBackDemoChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)56189);
                Debug.Assert(pack.lens_id == (byte)(byte)41);
                Debug.Assert(pack.time_boot_ms == (uint)1760867967U);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)42697);
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)86, (byte)165, (byte)41, (byte)34, (byte)138, (byte)152, (byte)102, (byte)180, (byte)235, (byte)87, (byte)216, (byte)130, (byte)102, (byte)0, (byte)164, (byte)198, (byte)181, (byte)180, (byte)53, (byte)32, (byte)96, (byte)124, (byte)139, (byte)135, (byte)147, (byte)221, (byte)253, (byte)115, (byte)43, (byte)150, (byte)206, (byte)177}));
                Debug.Assert(pack.focal_length == (float)7.167931E37F);
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE);
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)157, (byte)146, (byte)162, (byte)235, (byte)0, (byte)87, (byte)230, (byte)90, (byte)36, (byte)232, (byte)47, (byte)188, (byte)207, (byte)207, (byte)15, (byte)236, (byte)45, (byte)23, (byte)129, (byte)125, (byte)143, (byte)242, (byte)178, (byte)59, (byte)130, (byte)66, (byte)207, (byte)89, (byte)91, (byte)224, (byte)11, (byte)22}));
                Debug.Assert(pack.resolution_h == (ushort)(ushort)59537);
                Debug.Assert(pack.sensor_size_v == (float)3.187402E38F);
                Debug.Assert(pack.sensor_size_h == (float)4.116552E37F);
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 113);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("omtrpuijwwhfLoapyhoilYzhojozwdfqehLtmrLrmhNxajlbpcjrlrydeiwwhlLaofjqztxrbrjyxivlOafuxBipAaesgcxdevesvbUnspnynhyam"));
                Debug.Assert(pack.firmware_version == (uint)2752757131U);
            };
            DemoDevice.CAMERA_INFORMATION p259 = LoopBackDemoChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE;
            p259.lens_id = (byte)(byte)41;
            p259.vendor_name_SET(new byte[] {(byte)157, (byte)146, (byte)162, (byte)235, (byte)0, (byte)87, (byte)230, (byte)90, (byte)36, (byte)232, (byte)47, (byte)188, (byte)207, (byte)207, (byte)15, (byte)236, (byte)45, (byte)23, (byte)129, (byte)125, (byte)143, (byte)242, (byte)178, (byte)59, (byte)130, (byte)66, (byte)207, (byte)89, (byte)91, (byte)224, (byte)11, (byte)22}, 0) ;
            p259.sensor_size_v = (float)3.187402E38F;
            p259.cam_definition_uri_SET("omtrpuijwwhfLoapyhoilYzhojozwdfqehLtmrLrmhNxajlbpcjrlrydeiwwhlLaofjqztxrbrjyxivlOafuxBipAaesgcxdevesvbUnspnynhyam", PH) ;
            p259.time_boot_ms = (uint)1760867967U;
            p259.firmware_version = (uint)2752757131U;
            p259.focal_length = (float)7.167931E37F;
            p259.model_name_SET(new byte[] {(byte)86, (byte)165, (byte)41, (byte)34, (byte)138, (byte)152, (byte)102, (byte)180, (byte)235, (byte)87, (byte)216, (byte)130, (byte)102, (byte)0, (byte)164, (byte)198, (byte)181, (byte)180, (byte)53, (byte)32, (byte)96, (byte)124, (byte)139, (byte)135, (byte)147, (byte)221, (byte)253, (byte)115, (byte)43, (byte)150, (byte)206, (byte)177}, 0) ;
            p259.sensor_size_h = (float)4.116552E37F;
            p259.cam_definition_version = (ushort)(ushort)56189;
            p259.resolution_h = (ushort)(ushort)59537;
            p259.resolution_v = (ushort)(ushort)42697;
            LoopBackDemoChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode_id == (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE);
                Debug.Assert(pack.time_boot_ms == (uint)1302520587U);
            };
            DemoDevice.CAMERA_SETTINGS p260 = LoopBackDemoChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)1302520587U;
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE;
            LoopBackDemoChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.storage_id == (byte)(byte)253);
                Debug.Assert(pack.used_capacity == (float)1.768248E38F);
                Debug.Assert(pack.storage_count == (byte)(byte)14);
                Debug.Assert(pack.total_capacity == (float)2.7972481E38F);
                Debug.Assert(pack.write_speed == (float) -2.227408E38F);
                Debug.Assert(pack.status == (byte)(byte)193);
                Debug.Assert(pack.available_capacity == (float)3.2058083E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1100133461U);
                Debug.Assert(pack.read_speed == (float)1.0897625E38F);
            };
            DemoDevice.STORAGE_INFORMATION p261 = LoopBackDemoChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.storage_count = (byte)(byte)14;
            p261.used_capacity = (float)1.768248E38F;
            p261.total_capacity = (float)2.7972481E38F;
            p261.read_speed = (float)1.0897625E38F;
            p261.storage_id = (byte)(byte)253;
            p261.write_speed = (float) -2.227408E38F;
            p261.time_boot_ms = (uint)1100133461U;
            p261.status = (byte)(byte)193;
            p261.available_capacity = (float)3.2058083E38F;
            LoopBackDemoChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.available_capacity == (float) -1.7361704E38F);
                Debug.Assert(pack.image_interval == (float) -1.1153193E38F);
                Debug.Assert(pack.recording_time_ms == (uint)3378044401U);
                Debug.Assert(pack.image_status == (byte)(byte)159);
                Debug.Assert(pack.time_boot_ms == (uint)1446167740U);
                Debug.Assert(pack.video_status == (byte)(byte)93);
            };
            DemoDevice.CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.recording_time_ms = (uint)3378044401U;
            p262.available_capacity = (float) -1.7361704E38F;
            p262.video_status = (byte)(byte)93;
            p262.image_status = (byte)(byte)159;
            p262.image_interval = (float) -1.1153193E38F;
            p262.time_boot_ms = (uint)1446167740U;
            LoopBackDemoChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.9644313E38F, 2.1424478E38F, 3.3150365E38F, -2.8642675E38F}));
                Debug.Assert(pack.file_url_LEN(ph) == 36);
                Debug.Assert(pack.file_url_TRY(ph).Equals("hiqfAwlqIcdlgtbsOnhtbyzqkcfyYdmlbtft"));
                Debug.Assert(pack.image_index == (int) -1413805186);
                Debug.Assert(pack.time_boot_ms == (uint)1846647894U);
                Debug.Assert(pack.capture_result == (sbyte)(sbyte)83);
                Debug.Assert(pack.lon == (int)1331901605);
                Debug.Assert(pack.time_utc == (ulong)7327143779868399845L);
                Debug.Assert(pack.relative_alt == (int)1389662709);
                Debug.Assert(pack.lat == (int) -1525690612);
                Debug.Assert(pack.alt == (int)993853465);
                Debug.Assert(pack.camera_id == (byte)(byte)196);
            };
            DemoDevice.CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.q_SET(new float[] {-1.9644313E38F, 2.1424478E38F, 3.3150365E38F, -2.8642675E38F}, 0) ;
            p263.file_url_SET("hiqfAwlqIcdlgtbsOnhtbyzqkcfyYdmlbtft", PH) ;
            p263.image_index = (int) -1413805186;
            p263.lon = (int)1331901605;
            p263.time_boot_ms = (uint)1846647894U;
            p263.time_utc = (ulong)7327143779868399845L;
            p263.alt = (int)993853465;
            p263.lat = (int) -1525690612;
            p263.camera_id = (byte)(byte)196;
            p263.capture_result = (sbyte)(sbyte)83;
            p263.relative_alt = (int)1389662709;
            LoopBackDemoChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.arming_time_utc == (ulong)9206443144216514235L);
                Debug.Assert(pack.flight_uuid == (ulong)8585417803390833615L);
                Debug.Assert(pack.takeoff_time_utc == (ulong)4088807940179348805L);
                Debug.Assert(pack.time_boot_ms == (uint)2740787703U);
            };
            DemoDevice.FLIGHT_INFORMATION p264 = LoopBackDemoChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.flight_uuid = (ulong)8585417803390833615L;
            p264.arming_time_utc = (ulong)9206443144216514235L;
            p264.time_boot_ms = (uint)2740787703U;
            p264.takeoff_time_utc = (ulong)4088807940179348805L;
            LoopBackDemoChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)845898648U);
                Debug.Assert(pack.yaw == (float) -9.807228E37F);
                Debug.Assert(pack.roll == (float) -2.3481597E38F);
                Debug.Assert(pack.pitch == (float) -3.1333272E38F);
            };
            DemoDevice.MOUNT_ORIENTATION p265 = LoopBackDemoChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.pitch = (float) -3.1333272E38F;
            p265.time_boot_ms = (uint)845898648U;
            p265.yaw = (float) -9.807228E37F;
            p265.roll = (float) -2.3481597E38F;
            LoopBackDemoChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)25, (byte)89, (byte)127, (byte)7, (byte)193, (byte)146, (byte)156, (byte)173, (byte)211, (byte)74, (byte)17, (byte)90, (byte)78, (byte)110, (byte)177, (byte)218, (byte)37, (byte)94, (byte)203, (byte)229, (byte)131, (byte)170, (byte)93, (byte)29, (byte)164, (byte)46, (byte)83, (byte)204, (byte)106, (byte)219, (byte)34, (byte)57, (byte)235, (byte)12, (byte)154, (byte)255, (byte)75, (byte)111, (byte)253, (byte)244, (byte)62, (byte)0, (byte)44, (byte)179, (byte)191, (byte)45, (byte)210, (byte)39, (byte)243, (byte)67, (byte)182, (byte)138, (byte)212, (byte)35, (byte)177, (byte)197, (byte)147, (byte)225, (byte)143, (byte)210, (byte)70, (byte)96, (byte)13, (byte)16, (byte)24, (byte)40, (byte)67, (byte)149, (byte)221, (byte)80, (byte)166, (byte)14, (byte)184, (byte)176, (byte)165, (byte)35, (byte)3, (byte)208, (byte)106, (byte)222, (byte)83, (byte)151, (byte)0, (byte)111, (byte)20, (byte)152, (byte)232, (byte)18, (byte)249, (byte)34, (byte)232, (byte)226, (byte)112, (byte)178, (byte)146, (byte)94, (byte)85, (byte)195, (byte)219, (byte)215, (byte)36, (byte)181, (byte)124, (byte)66, (byte)148, (byte)102, (byte)79, (byte)165, (byte)112, (byte)106, (byte)63, (byte)213, (byte)105, (byte)165, (byte)70, (byte)49, (byte)164, (byte)224, (byte)111, (byte)141, (byte)26, (byte)173, (byte)251, (byte)5, (byte)62, (byte)55, (byte)240, (byte)47, (byte)233, (byte)134, (byte)93, (byte)158, (byte)247, (byte)115, (byte)171, (byte)156, (byte)42, (byte)80, (byte)120, (byte)68, (byte)175, (byte)136, (byte)156, (byte)250, (byte)151, (byte)7, (byte)131, (byte)178, (byte)134, (byte)44, (byte)245, (byte)136, (byte)38, (byte)84, (byte)134, (byte)59, (byte)199, (byte)47, (byte)21, (byte)248, (byte)109, (byte)188, (byte)211, (byte)172, (byte)113, (byte)90, (byte)216, (byte)112, (byte)177, (byte)5, (byte)155, (byte)9, (byte)216, (byte)230, (byte)135, (byte)153, (byte)203, (byte)238, (byte)98, (byte)184, (byte)156, (byte)235, (byte)73, (byte)30, (byte)217, (byte)166, (byte)75, (byte)108, (byte)14, (byte)151, (byte)225, (byte)134, (byte)47, (byte)55, (byte)78, (byte)119, (byte)109, (byte)4, (byte)48, (byte)210, (byte)121, (byte)11, (byte)120, (byte)212, (byte)160, (byte)88, (byte)186, (byte)44, (byte)234, (byte)194, (byte)122, (byte)131, (byte)28, (byte)61, (byte)172, (byte)133, (byte)13, (byte)92, (byte)90, (byte)188, (byte)141, (byte)1, (byte)204, (byte)140, (byte)119, (byte)173, (byte)44, (byte)59, (byte)198, (byte)154, (byte)119, (byte)50, (byte)237, (byte)120, (byte)119, (byte)28, (byte)155, (byte)14, (byte)38, (byte)235, (byte)172, (byte)86, (byte)182, (byte)81, (byte)28, (byte)251, (byte)198, (byte)23, (byte)18}));
                Debug.Assert(pack.target_system == (byte)(byte)197);
                Debug.Assert(pack.sequence == (ushort)(ushort)56528);
                Debug.Assert(pack.length == (byte)(byte)179);
                Debug.Assert(pack.first_message_offset == (byte)(byte)25);
                Debug.Assert(pack.target_component == (byte)(byte)64);
            };
            DemoDevice.LOGGING_DATA p266 = LoopBackDemoChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.data__SET(new byte[] {(byte)25, (byte)89, (byte)127, (byte)7, (byte)193, (byte)146, (byte)156, (byte)173, (byte)211, (byte)74, (byte)17, (byte)90, (byte)78, (byte)110, (byte)177, (byte)218, (byte)37, (byte)94, (byte)203, (byte)229, (byte)131, (byte)170, (byte)93, (byte)29, (byte)164, (byte)46, (byte)83, (byte)204, (byte)106, (byte)219, (byte)34, (byte)57, (byte)235, (byte)12, (byte)154, (byte)255, (byte)75, (byte)111, (byte)253, (byte)244, (byte)62, (byte)0, (byte)44, (byte)179, (byte)191, (byte)45, (byte)210, (byte)39, (byte)243, (byte)67, (byte)182, (byte)138, (byte)212, (byte)35, (byte)177, (byte)197, (byte)147, (byte)225, (byte)143, (byte)210, (byte)70, (byte)96, (byte)13, (byte)16, (byte)24, (byte)40, (byte)67, (byte)149, (byte)221, (byte)80, (byte)166, (byte)14, (byte)184, (byte)176, (byte)165, (byte)35, (byte)3, (byte)208, (byte)106, (byte)222, (byte)83, (byte)151, (byte)0, (byte)111, (byte)20, (byte)152, (byte)232, (byte)18, (byte)249, (byte)34, (byte)232, (byte)226, (byte)112, (byte)178, (byte)146, (byte)94, (byte)85, (byte)195, (byte)219, (byte)215, (byte)36, (byte)181, (byte)124, (byte)66, (byte)148, (byte)102, (byte)79, (byte)165, (byte)112, (byte)106, (byte)63, (byte)213, (byte)105, (byte)165, (byte)70, (byte)49, (byte)164, (byte)224, (byte)111, (byte)141, (byte)26, (byte)173, (byte)251, (byte)5, (byte)62, (byte)55, (byte)240, (byte)47, (byte)233, (byte)134, (byte)93, (byte)158, (byte)247, (byte)115, (byte)171, (byte)156, (byte)42, (byte)80, (byte)120, (byte)68, (byte)175, (byte)136, (byte)156, (byte)250, (byte)151, (byte)7, (byte)131, (byte)178, (byte)134, (byte)44, (byte)245, (byte)136, (byte)38, (byte)84, (byte)134, (byte)59, (byte)199, (byte)47, (byte)21, (byte)248, (byte)109, (byte)188, (byte)211, (byte)172, (byte)113, (byte)90, (byte)216, (byte)112, (byte)177, (byte)5, (byte)155, (byte)9, (byte)216, (byte)230, (byte)135, (byte)153, (byte)203, (byte)238, (byte)98, (byte)184, (byte)156, (byte)235, (byte)73, (byte)30, (byte)217, (byte)166, (byte)75, (byte)108, (byte)14, (byte)151, (byte)225, (byte)134, (byte)47, (byte)55, (byte)78, (byte)119, (byte)109, (byte)4, (byte)48, (byte)210, (byte)121, (byte)11, (byte)120, (byte)212, (byte)160, (byte)88, (byte)186, (byte)44, (byte)234, (byte)194, (byte)122, (byte)131, (byte)28, (byte)61, (byte)172, (byte)133, (byte)13, (byte)92, (byte)90, (byte)188, (byte)141, (byte)1, (byte)204, (byte)140, (byte)119, (byte)173, (byte)44, (byte)59, (byte)198, (byte)154, (byte)119, (byte)50, (byte)237, (byte)120, (byte)119, (byte)28, (byte)155, (byte)14, (byte)38, (byte)235, (byte)172, (byte)86, (byte)182, (byte)81, (byte)28, (byte)251, (byte)198, (byte)23, (byte)18}, 0) ;
            p266.sequence = (ushort)(ushort)56528;
            p266.target_system = (byte)(byte)197;
            p266.length = (byte)(byte)179;
            p266.target_component = (byte)(byte)64;
            p266.first_message_offset = (byte)(byte)25;
            LoopBackDemoChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)174);
                Debug.Assert(pack.sequence == (ushort)(ushort)64365);
                Debug.Assert(pack.length == (byte)(byte)70);
                Debug.Assert(pack.target_component == (byte)(byte)88);
                Debug.Assert(pack.first_message_offset == (byte)(byte)120);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)92, (byte)91, (byte)134, (byte)84, (byte)18, (byte)79, (byte)147, (byte)224, (byte)15, (byte)225, (byte)37, (byte)14, (byte)186, (byte)70, (byte)63, (byte)47, (byte)159, (byte)162, (byte)150, (byte)122, (byte)56, (byte)103, (byte)216, (byte)200, (byte)245, (byte)125, (byte)122, (byte)50, (byte)13, (byte)98, (byte)126, (byte)150, (byte)217, (byte)88, (byte)208, (byte)241, (byte)236, (byte)242, (byte)58, (byte)116, (byte)28, (byte)254, (byte)224, (byte)225, (byte)15, (byte)215, (byte)5, (byte)168, (byte)189, (byte)175, (byte)147, (byte)132, (byte)0, (byte)164, (byte)227, (byte)208, (byte)95, (byte)164, (byte)104, (byte)56, (byte)225, (byte)147, (byte)119, (byte)33, (byte)219, (byte)38, (byte)250, (byte)241, (byte)122, (byte)219, (byte)173, (byte)126, (byte)125, (byte)232, (byte)84, (byte)137, (byte)229, (byte)188, (byte)176, (byte)223, (byte)239, (byte)41, (byte)57, (byte)81, (byte)193, (byte)209, (byte)180, (byte)75, (byte)226, (byte)13, (byte)149, (byte)41, (byte)29, (byte)49, (byte)84, (byte)49, (byte)91, (byte)18, (byte)39, (byte)214, (byte)141, (byte)116, (byte)156, (byte)82, (byte)205, (byte)66, (byte)73, (byte)137, (byte)250, (byte)114, (byte)80, (byte)243, (byte)201, (byte)90, (byte)145, (byte)100, (byte)143, (byte)251, (byte)35, (byte)226, (byte)72, (byte)227, (byte)94, (byte)35, (byte)242, (byte)43, (byte)27, (byte)143, (byte)252, (byte)48, (byte)175, (byte)219, (byte)202, (byte)229, (byte)67, (byte)8, (byte)86, (byte)143, (byte)134, (byte)227, (byte)123, (byte)106, (byte)166, (byte)196, (byte)44, (byte)62, (byte)190, (byte)20, (byte)193, (byte)57, (byte)101, (byte)99, (byte)233, (byte)200, (byte)223, (byte)26, (byte)67, (byte)70, (byte)198, (byte)8, (byte)197, (byte)191, (byte)178, (byte)132, (byte)152, (byte)186, (byte)234, (byte)61, (byte)18, (byte)57, (byte)221, (byte)166, (byte)36, (byte)235, (byte)29, (byte)90, (byte)123, (byte)165, (byte)181, (byte)93, (byte)123, (byte)193, (byte)56, (byte)47, (byte)146, (byte)139, (byte)48, (byte)246, (byte)2, (byte)82, (byte)255, (byte)23, (byte)203, (byte)209, (byte)125, (byte)158, (byte)197, (byte)143, (byte)149, (byte)30, (byte)2, (byte)112, (byte)162, (byte)210, (byte)253, (byte)214, (byte)194, (byte)59, (byte)241, (byte)151, (byte)220, (byte)186, (byte)80, (byte)156, (byte)188, (byte)173, (byte)192, (byte)179, (byte)148, (byte)108, (byte)209, (byte)14, (byte)11, (byte)39, (byte)182, (byte)25, (byte)226, (byte)4, (byte)52, (byte)182, (byte)249, (byte)17, (byte)20, (byte)138, (byte)107, (byte)208, (byte)244, (byte)232, (byte)215, (byte)210, (byte)129, (byte)253, (byte)105, (byte)120, (byte)77, (byte)237, (byte)195, (byte)109, (byte)0}));
            };
            DemoDevice.LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.first_message_offset = (byte)(byte)120;
            p267.sequence = (ushort)(ushort)64365;
            p267.length = (byte)(byte)70;
            p267.data__SET(new byte[] {(byte)92, (byte)91, (byte)134, (byte)84, (byte)18, (byte)79, (byte)147, (byte)224, (byte)15, (byte)225, (byte)37, (byte)14, (byte)186, (byte)70, (byte)63, (byte)47, (byte)159, (byte)162, (byte)150, (byte)122, (byte)56, (byte)103, (byte)216, (byte)200, (byte)245, (byte)125, (byte)122, (byte)50, (byte)13, (byte)98, (byte)126, (byte)150, (byte)217, (byte)88, (byte)208, (byte)241, (byte)236, (byte)242, (byte)58, (byte)116, (byte)28, (byte)254, (byte)224, (byte)225, (byte)15, (byte)215, (byte)5, (byte)168, (byte)189, (byte)175, (byte)147, (byte)132, (byte)0, (byte)164, (byte)227, (byte)208, (byte)95, (byte)164, (byte)104, (byte)56, (byte)225, (byte)147, (byte)119, (byte)33, (byte)219, (byte)38, (byte)250, (byte)241, (byte)122, (byte)219, (byte)173, (byte)126, (byte)125, (byte)232, (byte)84, (byte)137, (byte)229, (byte)188, (byte)176, (byte)223, (byte)239, (byte)41, (byte)57, (byte)81, (byte)193, (byte)209, (byte)180, (byte)75, (byte)226, (byte)13, (byte)149, (byte)41, (byte)29, (byte)49, (byte)84, (byte)49, (byte)91, (byte)18, (byte)39, (byte)214, (byte)141, (byte)116, (byte)156, (byte)82, (byte)205, (byte)66, (byte)73, (byte)137, (byte)250, (byte)114, (byte)80, (byte)243, (byte)201, (byte)90, (byte)145, (byte)100, (byte)143, (byte)251, (byte)35, (byte)226, (byte)72, (byte)227, (byte)94, (byte)35, (byte)242, (byte)43, (byte)27, (byte)143, (byte)252, (byte)48, (byte)175, (byte)219, (byte)202, (byte)229, (byte)67, (byte)8, (byte)86, (byte)143, (byte)134, (byte)227, (byte)123, (byte)106, (byte)166, (byte)196, (byte)44, (byte)62, (byte)190, (byte)20, (byte)193, (byte)57, (byte)101, (byte)99, (byte)233, (byte)200, (byte)223, (byte)26, (byte)67, (byte)70, (byte)198, (byte)8, (byte)197, (byte)191, (byte)178, (byte)132, (byte)152, (byte)186, (byte)234, (byte)61, (byte)18, (byte)57, (byte)221, (byte)166, (byte)36, (byte)235, (byte)29, (byte)90, (byte)123, (byte)165, (byte)181, (byte)93, (byte)123, (byte)193, (byte)56, (byte)47, (byte)146, (byte)139, (byte)48, (byte)246, (byte)2, (byte)82, (byte)255, (byte)23, (byte)203, (byte)209, (byte)125, (byte)158, (byte)197, (byte)143, (byte)149, (byte)30, (byte)2, (byte)112, (byte)162, (byte)210, (byte)253, (byte)214, (byte)194, (byte)59, (byte)241, (byte)151, (byte)220, (byte)186, (byte)80, (byte)156, (byte)188, (byte)173, (byte)192, (byte)179, (byte)148, (byte)108, (byte)209, (byte)14, (byte)11, (byte)39, (byte)182, (byte)25, (byte)226, (byte)4, (byte)52, (byte)182, (byte)249, (byte)17, (byte)20, (byte)138, (byte)107, (byte)208, (byte)244, (byte)232, (byte)215, (byte)210, (byte)129, (byte)253, (byte)105, (byte)120, (byte)77, (byte)237, (byte)195, (byte)109, (byte)0}, 0) ;
            p267.target_system = (byte)(byte)174;
            p267.target_component = (byte)(byte)88;
            LoopBackDemoChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)38);
                Debug.Assert(pack.target_component == (byte)(byte)69);
                Debug.Assert(pack.sequence == (ushort)(ushort)43545);
            };
            DemoDevice.LOGGING_ACK p268 = LoopBackDemoChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.sequence = (ushort)(ushort)43545;
            p268.target_component = (byte)(byte)69;
            p268.target_system = (byte)(byte)38;
            LoopBackDemoChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.framerate == (float)2.3180336E38F);
                Debug.Assert(pack.camera_id == (byte)(byte)234);
                Debug.Assert(pack.status == (byte)(byte)101);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)30637);
                Debug.Assert(pack.uri_LEN(ph) == 36);
                Debug.Assert(pack.uri_TRY(ph).Equals("lkyhwcdogAktjrywqvcaicrpkhgzlnizGkpi"));
                Debug.Assert(pack.rotation == (ushort)(ushort)33372);
                Debug.Assert(pack.bitrate == (uint)3570746960U);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)39329);
            };
            DemoDevice.VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.rotation = (ushort)(ushort)33372;
            p269.bitrate = (uint)3570746960U;
            p269.camera_id = (byte)(byte)234;
            p269.uri_SET("lkyhwcdogAktjrywqvcaicrpkhgzlnizGkpi", PH) ;
            p269.framerate = (float)2.3180336E38F;
            p269.resolution_h = (ushort)(ushort)39329;
            p269.resolution_v = (ushort)(ushort)30637;
            p269.status = (byte)(byte)101;
            LoopBackDemoChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.bitrate == (uint)1721674568U);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)48616);
                Debug.Assert(pack.camera_id == (byte)(byte)143);
                Debug.Assert(pack.uri_LEN(ph) == 116);
                Debug.Assert(pack.uri_TRY(ph).Equals("gfrlntvxuBlxgqneuVltpyDgubgnqlcqdfpowafxitupenEisFgpluaNrgIsokkyrjauvyeqsrbjTioxopinpDHnqdwojNrqvqeoiRxscsqhoOxajugw"));
                Debug.Assert(pack.rotation == (ushort)(ushort)51530);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)45691);
                Debug.Assert(pack.framerate == (float) -1.650644E38F);
                Debug.Assert(pack.target_system == (byte)(byte)72);
                Debug.Assert(pack.target_component == (byte)(byte)142);
            };
            DemoDevice.SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.framerate = (float) -1.650644E38F;
            p270.target_system = (byte)(byte)72;
            p270.camera_id = (byte)(byte)143;
            p270.resolution_v = (ushort)(ushort)48616;
            p270.bitrate = (uint)1721674568U;
            p270.rotation = (ushort)(ushort)51530;
            p270.uri_SET("gfrlntvxuBlxgqneuVltpyDgubgnqlcqdfpowafxitupenEisFgpluaNrgIsokkyrjauvyeqsrbjTioxopinpDHnqdwojNrqvqeoiRxscsqhoOxajugw", PH) ;
            p270.target_component = (byte)(byte)142;
            p270.resolution_h = (ushort)(ushort)45691;
            LoopBackDemoChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ssid_LEN(ph) == 3);
                Debug.Assert(pack.ssid_TRY(ph).Equals("nuq"));
                Debug.Assert(pack.password_LEN(ph) == 9);
                Debug.Assert(pack.password_TRY(ph).Equals("mtrzjxxbn"));
            };
            DemoDevice.WIFI_CONFIG_AP p299 = LoopBackDemoChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("nuq", PH) ;
            p299.password_SET("mtrzjxxbn", PH) ;
            LoopBackDemoChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)144, (byte)131, (byte)186, (byte)66, (byte)65, (byte)231, (byte)58, (byte)209}));
                Debug.Assert(pack.min_version == (ushort)(ushort)25217);
                Debug.Assert(pack.version == (ushort)(ushort)31187);
                Debug.Assert(pack.max_version == (ushort)(ushort)38588);
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)126, (byte)131, (byte)48, (byte)80, (byte)106, (byte)181, (byte)229, (byte)59}));
            };
            DemoDevice.PROTOCOL_VERSION p300 = LoopBackDemoChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.spec_version_hash_SET(new byte[] {(byte)126, (byte)131, (byte)48, (byte)80, (byte)106, (byte)181, (byte)229, (byte)59}, 0) ;
            p300.library_version_hash_SET(new byte[] {(byte)144, (byte)131, (byte)186, (byte)66, (byte)65, (byte)231, (byte)58, (byte)209}, 0) ;
            p300.min_version = (ushort)(ushort)25217;
            p300.version = (ushort)(ushort)31187;
            p300.max_version = (ushort)(ushort)38588;
            LoopBackDemoChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.health == (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL);
                Debug.Assert(pack.sub_mode == (byte)(byte)253);
                Debug.Assert(pack.mode == (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE);
                Debug.Assert(pack.time_usec == (ulong)3110544497940192614L);
                Debug.Assert(pack.uptime_sec == (uint)2605491996U);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)7442);
            };
            DemoDevice.UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.uptime_sec = (uint)2605491996U;
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL;
            p310.vendor_specific_status_code = (ushort)(ushort)7442;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE;
            p310.sub_mode = (byte)(byte)253;
            p310.time_usec = (ulong)3110544497940192614L;
            LoopBackDemoChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hw_version_minor == (byte)(byte)239);
                Debug.Assert(pack.time_usec == (ulong)5488530688857707893L);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)40, (byte)26, (byte)98, (byte)155, (byte)101, (byte)1, (byte)252, (byte)125, (byte)164, (byte)75, (byte)153, (byte)0, (byte)112, (byte)144, (byte)12, (byte)161}));
                Debug.Assert(pack.sw_vcs_commit == (uint)4123353699U);
                Debug.Assert(pack.hw_version_major == (byte)(byte)68);
                Debug.Assert(pack.sw_version_major == (byte)(byte)61);
                Debug.Assert(pack.uptime_sec == (uint)3945063799U);
                Debug.Assert(pack.sw_version_minor == (byte)(byte)172);
                Debug.Assert(pack.name_LEN(ph) == 36);
                Debug.Assert(pack.name_TRY(ph).Equals("udaoTcnsymrvnxjphkuggozdjgefrvmmdsrg"));
            };
            DemoDevice.UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.hw_version_major = (byte)(byte)68;
            p311.sw_version_minor = (byte)(byte)172;
            p311.sw_version_major = (byte)(byte)61;
            p311.time_usec = (ulong)5488530688857707893L;
            p311.hw_version_minor = (byte)(byte)239;
            p311.name_SET("udaoTcnsymrvnxjphkuggozdjgefrvmmdsrg", PH) ;
            p311.uptime_sec = (uint)3945063799U;
            p311.hw_unique_id_SET(new byte[] {(byte)40, (byte)26, (byte)98, (byte)155, (byte)101, (byte)1, (byte)252, (byte)125, (byte)164, (byte)75, (byte)153, (byte)0, (byte)112, (byte)144, (byte)12, (byte)161}, 0) ;
            p311.sw_vcs_commit = (uint)4123353699U;
            LoopBackDemoChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)218);
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("pukcu"));
                Debug.Assert(pack.target_system == (byte)(byte)130);
                Debug.Assert(pack.param_index == (short)(short) -4113);
            };
            DemoDevice.PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.param_index = (short)(short) -4113;
            p320.target_system = (byte)(byte)130;
            p320.param_id_SET("pukcu", PH) ;
            p320.target_component = (byte)(byte)218;
            LoopBackDemoChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)39);
                Debug.Assert(pack.target_system == (byte)(byte)218);
            };
            DemoDevice.PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)218;
            p321.target_component = (byte)(byte)39;
            LoopBackDemoChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_LEN(ph) == 101);
                Debug.Assert(pack.param_value_TRY(ph).Equals("tlFsvpjfLombMztedgqcfewksoabceeHmrqmivauvfypKnvfvgrkygthixkzstrjMamcyiehgrjztpkkztxcivqjnzdfrouhzudkj"));
                Debug.Assert(pack.param_count == (ushort)(ushort)6460);
                Debug.Assert(pack.param_id_LEN(ph) == 8);
                Debug.Assert(pack.param_id_TRY(ph).Equals("JhMwrLic"));
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32);
                Debug.Assert(pack.param_index == (ushort)(ushort)13448);
            };
            DemoDevice.PARAM_EXT_VALUE p322 = LoopBackDemoChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_value_SET("tlFsvpjfLombMztedgqcfewksoabceeHmrqmivauvfypKnvfvgrkygthixkzstrjMamcyiehgrjztpkkztxcivqjnzdfrouhzudkj", PH) ;
            p322.param_id_SET("JhMwrLic", PH) ;
            p322.param_index = (ushort)(ushort)13448;
            p322.param_count = (ushort)(ushort)6460;
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32;
            LoopBackDemoChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)66);
                Debug.Assert(pack.param_id_LEN(ph) == 1);
                Debug.Assert(pack.param_id_TRY(ph).Equals("f"));
                Debug.Assert(pack.param_value_LEN(ph) == 96);
                Debug.Assert(pack.param_value_TRY(ph).Equals("tinjcGvthxlmyixaxjaGacrlapbmdidnchfXkcxuaaeNhhavsnmwlygnJarmgmymrrWgkcehuaofrwcWnwGdahtnzkisktqu"));
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
                Debug.Assert(pack.target_system == (byte)(byte)71);
            };
            DemoDevice.PARAM_EXT_SET p323 = LoopBackDemoChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_component = (byte)(byte)66;
            p323.param_id_SET("f", PH) ;
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32;
            p323.target_system = (byte)(byte)71;
            p323.param_value_SET("tinjcGvthxlmyixaxjaGacrlapbmdidnchfXkcxuaaeNhhavsnmwlygnJarmgmymrrWgkcehuaofrwcWnwGdahtnzkisktqu", PH) ;
            LoopBackDemoChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_LEN(ph) == 45);
                Debug.Assert(pack.param_value_TRY(ph).Equals("azdbogbjgqitsyyabcoRedkugSsatqpjrfayqpcvmwgpo"));
                Debug.Assert(pack.param_id_LEN(ph) == 15);
                Debug.Assert(pack.param_id_TRY(ph).Equals("nnsntJgtfztZooo"));
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM);
                Debug.Assert(pack.param_result == (PARAM_ACK)PARAM_ACK.PARAM_ACK_IN_PROGRESS);
            };
            DemoDevice.PARAM_EXT_ACK p324 = LoopBackDemoChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("nnsntJgtfztZooo", PH) ;
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_IN_PROGRESS;
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM;
            p324.param_value_SET("azdbogbjgqitsyyabcoRedkugSsatqpjrfayqpcvmwgpo", PH) ;
            LoopBackDemoChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.increment == (byte)(byte)52);
                Debug.Assert(pack.min_distance == (ushort)(ushort)19085);
                Debug.Assert(pack.max_distance == (ushort)(ushort)3926);
                Debug.Assert(pack.time_usec == (ulong)1882485339721383163L);
                Debug.Assert(pack.sensor_type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)23215, (ushort)45248, (ushort)21988, (ushort)41838, (ushort)64214, (ushort)60115, (ushort)13409, (ushort)47636, (ushort)22191, (ushort)52665, (ushort)31557, (ushort)24796, (ushort)16677, (ushort)56786, (ushort)41259, (ushort)15967, (ushort)17386, (ushort)15959, (ushort)25863, (ushort)3624, (ushort)58332, (ushort)31856, (ushort)52436, (ushort)314, (ushort)44710, (ushort)15517, (ushort)2003, (ushort)62525, (ushort)60904, (ushort)2984, (ushort)7320, (ushort)2213, (ushort)27385, (ushort)43603, (ushort)25741, (ushort)64815, (ushort)46182, (ushort)49349, (ushort)2179, (ushort)10960, (ushort)52741, (ushort)36760, (ushort)18283, (ushort)42853, (ushort)53958, (ushort)41305, (ushort)38463, (ushort)59730, (ushort)53766, (ushort)52416, (ushort)42234, (ushort)6857, (ushort)24564, (ushort)29639, (ushort)2776, (ushort)12296, (ushort)22216, (ushort)26297, (ushort)56304, (ushort)32857, (ushort)1941, (ushort)35777, (ushort)63763, (ushort)42356, (ushort)33809, (ushort)11672, (ushort)46366, (ushort)10651, (ushort)21030, (ushort)28428, (ushort)13289, (ushort)12857}));
            };
            DemoDevice.OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.distances_SET(new ushort[] {(ushort)23215, (ushort)45248, (ushort)21988, (ushort)41838, (ushort)64214, (ushort)60115, (ushort)13409, (ushort)47636, (ushort)22191, (ushort)52665, (ushort)31557, (ushort)24796, (ushort)16677, (ushort)56786, (ushort)41259, (ushort)15967, (ushort)17386, (ushort)15959, (ushort)25863, (ushort)3624, (ushort)58332, (ushort)31856, (ushort)52436, (ushort)314, (ushort)44710, (ushort)15517, (ushort)2003, (ushort)62525, (ushort)60904, (ushort)2984, (ushort)7320, (ushort)2213, (ushort)27385, (ushort)43603, (ushort)25741, (ushort)64815, (ushort)46182, (ushort)49349, (ushort)2179, (ushort)10960, (ushort)52741, (ushort)36760, (ushort)18283, (ushort)42853, (ushort)53958, (ushort)41305, (ushort)38463, (ushort)59730, (ushort)53766, (ushort)52416, (ushort)42234, (ushort)6857, (ushort)24564, (ushort)29639, (ushort)2776, (ushort)12296, (ushort)22216, (ushort)26297, (ushort)56304, (ushort)32857, (ushort)1941, (ushort)35777, (ushort)63763, (ushort)42356, (ushort)33809, (ushort)11672, (ushort)46366, (ushort)10651, (ushort)21030, (ushort)28428, (ushort)13289, (ushort)12857}, 0) ;
            p330.min_distance = (ushort)(ushort)19085;
            p330.increment = (byte)(byte)52;
            p330.max_distance = (ushort)(ushort)3926;
            p330.time_usec = (ulong)1882485339721383163L;
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER;
            LoopBackDemoChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
        }
    }
}