
using System;
using System.Collections.Generic;
using System.Threading;
using System.Diagnostics;
using org.unirail.BlackBox;
using Inside = org.unirail.BlackBox.Host.Pack.Meta.Field.Bounds.Inside;
using Field = org.unirail.BlackBox.Host.Pack.Meta.Field;
namespace org.noname
{
    public class DemoDevice : Host
    {
        public class HEARTBEAT : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal HEARTBEAT() : base(meta0, 0) { }
            internal HEARTBEAT(int bytes) : base(meta0, bytes) { }
            public uint custom_mode //A bitfield for use for autopilot-specific flags
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public byte base_mode //System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public byte mavlink_version //MAVLink versio
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  5);}
            }

            public MAV_TYPE type //Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM
            {
                get {  return (MAV_TYPE)(0 +  BitUtils.get_bits(data, 48, 5));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 5, data, 48);}
            }

            public MAV_AUTOPILOT autopilot //Autopilot type / class. defined in MAV_AUTOPILOT ENU
            {
                get {  return (MAV_AUTOPILOT)(0 +  BitUtils.get_bits(data, 53, 4));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 53);}
            }

            public MAV_STATE system_status //System status flag, see MAV_STATE ENU
            {
                get {  return (MAV_STATE)(0 +  BitUtils.get_bits(data, 57, 4));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 57);}
            }
            static readonly Meta meta0 = new Meta(0, 0, 1, 0, 8, 61);
        }

        public class LoopBackDemoChannel : Channel
        {
            public override bool CanRead { get { return true ; } }
            public override bool CanWrite { get { return true; } }
            static LoopBackDemoChannel() {pack_id_bytes = 0;}

            public static  LoopBackDemoChannel instance = new LoopBackDemoChannel();

            //interface-to-mark of sendable through this channel packs_Schs_Rchs
            public interface Sendable {}
            public static HEARTBEAT new_HEARTBEAT() {return new  HEARTBEAT();}

            public void send(Sendable pack) {lock(sendout_packs) {sendout_packs.Enqueue((Pack) pack); Monitor.PulseAll(sendout_packs);}}
            protected readonly Queue<Pack> sendout_packs = new Queue<Pack>();
            protected readonly Queue<Pack> received_packs = new Queue<Pack>();
            private readonly Inside ph    = new Inside();
            protected internal override Pack process(Pack pack, int id)
            {
                for(bool LOOP = false; ;)
                {
                    switch(id)
                    {
                        default:
                            Debug.Assert(false);
                            return null;
                        case 0:
                            if(pack == null) return new HEARTBEAT();
                            if(OnHEARTBEATReceive == null) return null;
                            ph.setPack(pack);
                            OnHEARTBEATReceive(this, ph, (HEARTBEAT) pack);
                            if(LOOP) break;
                            return null;
                        case Channel.PROCESS_CHANNEL_REQEST:
                            if(pack == null) return sendout_packs.Count == 0 ? null : sendout_packs.Dequeue();
                            lock(received_packs) {received_packs.Enqueue(pack); Monitor.PulseAll(received_packs); }
                            return null;
                        case Channel.PROCESS_HOST_REQEST:
                            if(pack == null) return received_packs.Count == 0 ? null : received_packs.Dequeue();
                            lock(sendout_packs) { sendout_packs.Enqueue(pack); Monitor.PulseAll(sendout_packs); }
                            return null;
                        case Channel.PROCESS_RECEIVED:
                            LOOP = true;
                            break;
                    }
                    if(received_packs.Count == 0) return null;
                    pack = received_packs.Dequeue();
                    id = pack.meta.id;
                }
            }
            public event HEARTBEATReceiveHandler OnHEARTBEATReceive;
            public delegate void HEARTBEATReceiveHandler(Channel src, Inside ph, HEARTBEAT pack);
        }
        public class LoopBackDemoChannel_ADV : Channel.Advanced
        {
            public override bool CanRead { get { return false ; } }
            public override bool CanWrite { get { return true; } }
            static LoopBackDemoChannel_ADV() {pack_id_bytes = 0;}

            public static  LoopBackDemoChannel_ADV instance = new LoopBackDemoChannel_ADV();

            //interface-to-mark of sendable through this channel packs_Schs_Rchs
            public interface Sendable {}
            public static HEARTBEAT new_HEARTBEAT() {return new  HEARTBEAT();}

            public void send(Sendable pack) {lock(sendout_packs) {sendout_packs.Enqueue((Pack) pack); Monitor.PulseAll(sendout_packs);}}
            protected readonly Queue<Pack> sendout_packs = new Queue<Pack>();
            protected internal override Pack process(Pack pack, int id)
            {
                switch(id)
                {
                    default:
                        Debug.Assert(false);
                        return null;
                    case Channel.PROCESS_CHANNEL_REQEST:
                        if(pack == null) return sendout_packs.Count == 0 ? null : sendout_packs.Dequeue();
                        return null;
                    case Channel.PROCESS_HOST_REQEST:
                        lock(sendout_packs) { sendout_packs.Enqueue(pack); Monitor.PulseAll(sendout_packs); }
                        return null;
                }
            }
        }
        ; /*
				                                              */

        public enum MAV_TYPE
        {
            MAV_TYPE_GENERIC = 0,
            MAV_TYPE_FIXED_WING = 1,
            MAV_TYPE_QUADROTOR = 2,
            MAV_TYPE_COAXIAL = 3,
            MAV_TYPE_HELICOPTER = 4,
            MAV_TYPE_ANTENNA_TRACKER = 5,
            MAV_TYPE_GCS = 6,
            MAV_TYPE_AIRSHIP = 7,
            MAV_TYPE_FREE_BALLOON = 8,
            MAV_TYPE_ROCKET = 9,
            MAV_TYPE_GROUND_ROVER = 10,
            MAV_TYPE_SURFACE_BOAT = 11,
            MAV_TYPE_SUBMARINE = 12,
            MAV_TYPE_HEXAROTOR = 13,
            MAV_TYPE_OCTOROTOR = 14,
            MAV_TYPE_TRICOPTER = 15,
            MAV_TYPE_FLAPPING_WING = 16
        }
        ; /*
				                                              */

        public enum MAV_AUTOPILOT
        {
            MAV_AUTOPILOT_GENERIC = 0,
            MAV_AUTOPILOT_PIXHAWK = 1,
            MAV_AUTOPILOT_SLUGS = 2,
            MAV_AUTOPILOT_ARDUPILOTMEGA = 3,
            MAV_AUTOPILOT_OPENPILOT = 4,
            MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY = 5,
            MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = 6,
            MAV_AUTOPILOT_GENERIC_MISSION_FULL = 7,
            MAV_AUTOPILOT_INVALID = 8,
            MAV_AUTOPILOT_PPZ = 9,
            MAV_AUTOPILOT_UDB = 10,
            MAV_AUTOPILOT_FP = 11
        }
        ; /*
				                                              */

        public enum MAV_STATE
        {
            MAV_STATE_UNINIT = 0,
            MAV_STATE_BOOT = 1,
            MAV_STATE_CALIBRATING = 2,
            MAV_STATE_STANDBY = 3,
            MAV_STATE_ACTIVE = 4,
            MAV_STATE_CRITICAL = 5,
            MAV_STATE_EMERGENCY = 6,
            MAV_STATE_POWEROFF = 7
        }


    }
}