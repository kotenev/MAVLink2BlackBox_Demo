package org.noname;
import java.util.*;
import org.unirail.BlackBox.Host;
import org.unirail.BlackBox.Host.Pack.Meta.Field;
import org.unirail.BlackBox.Host.Pack.Meta.Field.Bounds;
import static org.unirail.BlackBox.BitUtils.*;
import java.util.concurrent.ConcurrentLinkedQueue;


public class DemoDevice extends Host
{
    public static class TEST_TYPES extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        TEST_TYPES() { super(meta, 0); }
        TEST_TYPES(int bytes) { super(meta, bytes); }
        public char c_GET()//cha
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void c_SET(char  src) //cha
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char u16_GET()//uint16_
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void u16_SET(char  src) //uint16_
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char[] u16_array_GET(char[]  dst_ch, int pos)  //char_arra
        {
            for(int BYTE = 4, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public char[] u16_array_GET()//char_arra
        {return u16_array_GET(new char[3], 0);} public void u16_array_SET(char[]  src, int pos)  //char_arra
        {
            for(int BYTE =  4, src_max = pos + 3; pos < src_max; pos++, BYTE += 2)
                set_bytes((char)(src[pos]) & -1L, 2, data,  BYTE);
        }
        public long u32_GET()//uint32_
        {  return (get_bytes(data,  10, 4)); }
        public void u32_SET(long  src) //uint32_
        {  set_bytes((src) & -1L, 4, data,  10); }
        public long[] u32_array_GET(long[]  dst_ch, int pos)  //long_arra
        {
            for(int BYTE = 14, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (get_bytes(data,  BYTE, 4));
            return dst_ch;
        }
        public long[] u32_array_GET()//long_arra
        {return u32_array_GET(new long[3], 0);} public void u32_array_SET(long[]  src, int pos)  //long_arra
        {
            for(int BYTE =  14, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                set_bytes((src[pos]) & -1L, 4, data,  BYTE);
        }
        public long u64_GET()//uint64_
        {  return (get_bytes(data,  26, 8)); }
        public void u64_SET(long  src) //uint64_
        {  set_bytes((src) & -1L, 8, data,  26); }
        public long[] u64_array_GET(long[]  dst_ch, int pos)  //long_arra
        {
            for(int BYTE = 34, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 8)
                dst_ch[pos] = (get_bytes(data,  BYTE, 8));
            return dst_ch;
        }
        public long[] u64_array_GET()//long_arra
        {return u64_array_GET(new long[3], 0);} public void u64_array_SET(long[]  src, int pos)  //long_arra
        {
            for(int BYTE =  34, src_max = pos + 3; pos < src_max; pos++, BYTE += 8)
                set_bytes((src[pos]) & -1L, 8, data,  BYTE);
        }
        public char u8_GET()//uint8_
        {  return (char)((char) get_bytes(data,  58, 1)); }
        public void u8_SET(char  src) //uint8_
        {  set_bytes((char)(src) & -1L, 1, data,  58); }
        public byte s8_GET()//int8_
        {  return (byte)((byte) get_bytes(data,  59, 1)); }
        public void s8_SET(byte  src) //int8_
        {  set_bytes((byte)(src) & -1L, 1, data,  59); }
        public short s16_GET()//int16_
        {  return (short)((short) get_bytes(data,  60, 2)); }
        public void s16_SET(short  src) //int16_
        {  set_bytes((short)(src) & -1L, 2, data,  60); }
        public int s32_GET()//int32_
        {  return (int)((int) get_bytes(data,  62, 4)); }
        public void s32_SET(int  src) //int32_
        {  set_bytes((int)(src) & -1L, 4, data,  62); }
        public long s64_GET()//int64_
        {  return (get_bytes(data,  66, 8)); }
        public void s64_SET(long  src) //int64_
        {  set_bytes((src) & -1L, 8, data,  66); }
        public float f_GET()//floa
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  74, 4))); }
        public void f_SET(float  src) //floa
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 74); }
        public double d_GET()//doubl
        {  return (double)(Double.longBitsToDouble(get_bytes(data, 78, 8))); }
        public void d_SET(double  src) //doubl
        {  set_bytes(Double.doubleToLongBits(src), 8, data,  78); }
        public char[] u8_array_GET(char[]  dst_ch, int pos)  //char_arra
        {
            for(int BYTE = 86, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] u8_array_GET()//char_arra
        {return u8_array_GET(new char[3], 0);} public void u8_array_SET(char[]  src, int pos)  //char_arra
        {
            for(int BYTE =  86, src_max = pos + 3; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public byte[] s8_array_GET(byte[]  dst_ch, int pos)  //byte_arra
        {
            for(int BYTE = 89, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (byte)((byte) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public byte[] s8_array_GET()//byte_arra
        {return s8_array_GET(new byte[3], 0);} public void s8_array_SET(byte[]  src, int pos)  //byte_arra
        {
            for(int BYTE =  89, src_max = pos + 3; pos < src_max; pos++, BYTE += 1)
                set_bytes((byte)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public short[] s16_array_GET(short[]  dst_ch, int pos)  //short_arra
        {
            for(int BYTE = 92, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (short)((short) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public short[] s16_array_GET()//short_arra
        {return s16_array_GET(new short[3], 0);} public void s16_array_SET(short[]  src, int pos)  //short_arra
        {
            for(int BYTE =  92, src_max = pos + 3; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        }
        public int[] s32_array_GET(int[]  dst_ch, int pos)  //int_arra
        {
            for(int BYTE = 98, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (int)((int) get_bytes(data,  BYTE, 4));
            return dst_ch;
        }
        public int[] s32_array_GET()//int_arra
        {return s32_array_GET(new int[3], 0);} public void s32_array_SET(int[]  src, int pos)  //int_arra
        {
            for(int BYTE =  98, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                set_bytes((int)(src[pos]) & -1L, 4, data,  BYTE);
        }
        public long[] s64_array_GET(long[]  dst_ch, int pos)  //long_arra
        {
            for(int BYTE = 110, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 8)
                dst_ch[pos] = (get_bytes(data,  BYTE, 8));
            return dst_ch;
        }
        public long[] s64_array_GET()//long_arra
        {return s64_array_GET(new long[3], 0);} public void s64_array_SET(long[]  src, int pos)  //long_arra
        {
            for(int BYTE =  110, src_max = pos + 3; pos < src_max; pos++, BYTE += 8)
                set_bytes((src[pos]) & -1L, 8, data,  BYTE);
        }
        public float[] f_array_GET(float[]  dst_ch, int pos)  //float_arra
        {
            for(int BYTE = 134, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] f_array_GET()//float_arra
        {return f_array_GET(new float[3], 0);} public void f_array_SET(float[]  src, int pos)  //float_arra
        {
            for(int BYTE =  134, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public double[] d_array_GET(double[]  dst_ch, int pos)  //double_arra
        {
            for(int BYTE = 146, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 8)
                dst_ch[pos] = (double)(Double.longBitsToDouble(get_bytes(data, BYTE, 8)));
            return dst_ch;
        }
        public double[] d_array_GET()//double_arra
        {return d_array_GET(new double[3], 0);} public void d_array_SET(double[]  src, int pos)  //double_arra
        {
            for(int BYTE =  146, src_max = pos + 3; pos < src_max; pos++, BYTE += 8)
                set_bytes(Double.doubleToLongBits(src[pos]), 8, data,  BYTE);
        }
        public String s_TRY(Bounds.Inside ph)//strin
//strin
        {
            if(ph.field_bit !=  1360 && !try_visit_field(ph, 1360)  ||  !try_visit_item(ph, 0)) return null;
            return new String(s_GET(ph, new char[ph.items], 0));
        }
        public char[] s_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //strin
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int s_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  1360 && !try_visit_field(ph, 1360)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public void s_SET(String src, Bounds.Inside ph) //strin
        {s_SET(src.toCharArray(), 0, src.length(), ph);} public void s_SET(char[]  src, int pos, int items, Bounds.Inside ph) //strin
        {
            if(ph.field_bit != 1360 && insert_field(ph, 1360, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(0, 5, 4, 4, 171, 1360, 0, _E);
    }

    public static class LoopBackDemoChannel  extends Channel
    {


        public static  LoopBackDemoChannel instance = new LoopBackDemoChannel();

        public final java.io.InputStream inputStream = new  InputStream();
        //interface-to-mark of sendable through this channel packs_Schs_Rchs
        public interface Sendable {}
        public static TEST_TYPES new_TEST_TYPES() {return new  TEST_TYPES();}

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
                        if(pack == null) return new TEST_TYPES(-1);
                        final TEST_TYPES pi0 = (TEST_TYPES) pack;
                        ph.setPack(pi0);
                        on_TEST_TYPES.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi0));
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
        public final Collection<OnReceive.Handler<TEST_TYPES, LoopBackDemoChannel>> on_TEST_TYPES = new OnReceive<>();
    }

    public static class LoopBackDemoChannel_ADV  extends Channel
    {


        public static  LoopBackDemoChannel_ADV instance = new LoopBackDemoChannel_ADV();

        public final java.io.InputStream inputStream = new  AdvancedInputStream();
        //interface-to-mark of sendable through this channel packs_Schs_Rchs
        public interface Sendable {}
        public static TEST_TYPES new_TEST_TYPES() {return new  TEST_TYPES();}

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



    private static final Field _E = new Field(5, true, -4, 2, 1, 0, 0, 0, 1);

}