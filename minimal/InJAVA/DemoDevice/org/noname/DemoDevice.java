package org.noname;
import java.util.*;
import org.unirail.BlackBox.Host;
import org.unirail.BlackBox.Host.Pack.Meta.Field;
import org.unirail.BlackBox.Host.Pack.Meta.Field.Bounds;
import static org.unirail.BlackBox.BitUtils.*;
import java.util.concurrent.ConcurrentLinkedQueue;


public class DemoDevice extends Host
{
    public static class HEARTBEAT extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        HEARTBEAT() { super(meta, 0); }
        HEARTBEAT(int bytes) { super(meta, bytes); }
        public long custom_mode_GET()//A bitfield for use for autopilot-specific flags
        {  return (get_bytes(data,  0, 4)); }
        public void custom_mode_SET(long  src) //A bitfield for use for autopilot-specific flags
        {  set_bytes((src) & -1L, 4, data,  0); }
        public char base_mode_GET()//System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public void base_mode_SET(char  src) //System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public char mavlink_version_GET()//MAVLink versio
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public void mavlink_version_SET(char  src) //MAVLink versio
        {  set_bytes((char)(src) & -1L, 1, data,  5); }
        public @MAV_TYPE int type_GET()//Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM
        {  return  0 + (int)get_bits(data, 48, 5); }
        public void type_SET(@MAV_TYPE int  src) //Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM
        {  set_bits(- 0 +   src, 5, data, 48); }
        public @MAV_AUTOPILOT int autopilot_GET()//Autopilot type / class. defined in MAV_AUTOPILOT ENU
        {  return  0 + (int)get_bits(data, 53, 4); }
        public void autopilot_SET(@MAV_AUTOPILOT int  src) //Autopilot type / class. defined in MAV_AUTOPILOT ENU
        {  set_bits(- 0 +   src, 4, data, 53); }
        public @MAV_STATE int system_status_GET()//System status flag, see MAV_STATE ENU
        {  return  0 + (int)get_bits(data, 57, 4); }
        public void system_status_SET(@MAV_STATE int  src) //System status flag, see MAV_STATE ENU
        {  set_bits(- 0 +   src, 4, data, 57); }
        static final Meta meta = new Meta(0, 0, 1, 0, 8, 61);
    }

    public static class LoopBackDemoChannel  extends Channel
    {


        public static  LoopBackDemoChannel instance = new LoopBackDemoChannel();

        public final java.io.InputStream inputStream = new  InputStream();
        //interface-to-mark of sendable through this channel packs_Schs_Rchs
        public interface Sendable {}
        public static HEARTBEAT new_HEARTBEAT() {return new  HEARTBEAT();}

        public void send(Sendable pack) { sendout_packs.add((Pack) pack);}
        protected final Queue<Pack> sendout_packs = new ConcurrentLinkedQueue<Pack>()
        {
            @Override public boolean add(Pack pack)
            {
                synchronized(this)
                {
                    boolean ret = super.add(pack);
                    this.notify();
                    return ret;
                }
            }
        };

        public boolean waitingSendoutPack()
        {
            try
            {
                synchronized(sendout_packs)
                {
                    while(sendout_packs.size() == 0) sendout_packs.wait();
                }
                return true;
            }
            catch(InterruptedException e) {}
            return false;
        }

        public final java.io.OutputStream outputStream = new  OutputStream();
        protected final Queue<Pack> received_packs = new ConcurrentLinkedQueue<Pack>()
        {
            @Override public boolean add(Pack pack)
            {
                synchronized(this)
                {
                    boolean ret = super.add(pack);
                    this.notify();
                    return ret;
                }
            }
        };

        public boolean waitingReceivedPack()
        {
            try
            {
                synchronized(received_packs)
                {
                    while(received_packs.size() == 0) received_packs.wait();
                    return true;
                }
            }
            catch(InterruptedException e) {}
            return false;
        }


        private final Bounds.Inside ph    = new Bounds.Inside();


        @Override protected Pack process(Pack pack, int id)
        {
            for(boolean LOOP = false; ;)
            {
                switch(id)
                {
                    default:
                        assert(false);
                        return null;
                    case 0:
                        if(pack == null) return new HEARTBEAT();
                        final HEARTBEAT pi0 = (HEARTBEAT) pack;
                        ph.setPack(pi0);
                        on_HEARTBEAT.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi0));
                        if(LOOP) break;
                        return null;
                    case Channel.PROCESS_CHANNEL_REQEST:
                        if(pack == null) return sendout_packs.poll();
                        received_packs.add(pack);
                        return null;
                    case Channel.PROCESS_HOST_REQEST:
                        if(pack == null) return received_packs.poll();
                        sendout_packs.add(pack);
                        return null;
                    case Channel.PROCESS_RECEIVED:
                        LOOP = true;
                }
                if(received_packs.isEmpty()) return null;
                pack = received_packs.remove();
                id = pack.meta.id;
            }
        }
        public final Collection<OnReceive.Handler<HEARTBEAT, LoopBackDemoChannel>> on_HEARTBEAT = new OnReceive<>();
    }

    public static class LoopBackDemoChannel_ADV  extends Channel
    {


        public static  LoopBackDemoChannel_ADV instance = new LoopBackDemoChannel_ADV();

        public final java.io.InputStream inputStream = new  AdvancedInputStream();
        //interface-to-mark of sendable through this channel packs_Schs_Rchs
        public interface Sendable {}
        public static HEARTBEAT new_HEARTBEAT() {return new  HEARTBEAT();}

        public void send(Sendable pack) { sendout_packs.add((Pack) pack);}
        protected final Queue<Pack> sendout_packs = new ConcurrentLinkedQueue<Pack>()
        {
            @Override public boolean add(Pack pack)
            {
                synchronized(this)
                {
                    boolean ret = super.add(pack);
                    this.notify();
                    return ret;
                }
            }
        };

        public boolean waitingSendoutPack()
        {
            try
            {
                synchronized(sendout_packs)
                {
                    while(sendout_packs.size() == 0) sendout_packs.wait();
                }
                return true;
            }
            catch(InterruptedException e) {}
            return false;
        }


        @Override protected Pack process(Pack pack, int id)
        {
            switch(id)
            {
                default:
                    assert(false);
                    return null;
                case Channel.PROCESS_CHANNEL_REQEST:
                    if(pack == null) return sendout_packs.poll();
                    return null;
                case Channel.PROCESS_HOST_REQEST:
                    sendout_packs.add(pack);
                    return null;
            }
        }

    }

    /*
                  */
    public @interface MAV_TYPE
    {
        int
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
        MAV_TYPE_FLAPPING_WING = 16;
    }
    /*
    				                                              */
    public @interface MAV_AUTOPILOT
    {
        int
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
        MAV_AUTOPILOT_FP = 11;
    }
    /*
    				                                              */
    public @interface MAV_STATE
    {
        int
        MAV_STATE_UNINIT = 0,
        MAV_STATE_BOOT = 1,
        MAV_STATE_CALIBRATING = 2,
        MAV_STATE_STANDBY = 3,
        MAV_STATE_ACTIVE = 4,
        MAV_STATE_CRITICAL = 5,
        MAV_STATE_EMERGENCY = 6,
        MAV_STATE_POWEROFF = 7;
    }



}