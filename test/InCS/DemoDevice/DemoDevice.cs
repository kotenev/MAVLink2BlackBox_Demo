
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
        public class TEST_TYPES : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal TEST_TYPES() : base(meta0, 0) { }
            internal TEST_TYPES(int bytes) : base(meta0, bytes) { }
            public ushort c //cha
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort u16 //uint16_
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort[] u16_array //ushort_arra
            {
                get {return u16_array_GET(new ushort[3], 0);}
                set {u16_array_SET(value, 0)  ;}
            }
            public ushort[]u16_array_GET(ushort[] dst_ch, int pos)  //ushort_arra
            {
                for(int BYTE = 4, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }

            public void u16_array_SET(ushort[] src, int pos)  //ushort_arra
            {
                for(int BYTE =  4, src_max = pos + 3; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ulong)(src[pos]), 2, data,  BYTE);
            }


            public uint u32 //uint32_
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  10, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  10);}
            }

            public uint[] u32_array //uint_arra
            {
                get {return u32_array_GET(new uint[3], 0);}
                set {u32_array_SET(value, 0)  ;}
            }
            public uint[]u32_array_GET(uint[] dst_ch, int pos)  //uint_arra
            {
                for(int BYTE = 14, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (uint)((uint) BitUtils.get_bytes(data,  BYTE, 4));
                return dst_ch;
            }

            public void u32_array_SET(uint[] src, int pos)  //uint_arra
            {
                for(int BYTE =  14, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes((ulong)(src[pos]), 4, data,  BYTE);
            }


            public ulong u64 //uint64_
            {
                get {  return (BitUtils.get_bytes(data,  26, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  26);}
            }

            public ulong[] u64_array //ulong_arra
            {
                get {return u64_array_GET(new ulong[3], 0);}
                set {u64_array_SET(value, 0)  ;}
            }
            public ulong[]u64_array_GET(ulong[] dst_ch, int pos)  //ulong_arra
            {
                for(int BYTE = 34, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 8)
                    dst_ch[pos] = (BitUtils.get_bytes(data,  BYTE, 8));
                return dst_ch;
            }

            public void u64_array_SET(ulong[] src, int pos)  //ulong_arra
            {
                for(int BYTE =  34, src_max = pos + 3; pos < src_max; pos++, BYTE += 8)
                    BitUtils.set_bytes((ulong)(src[pos]), 8, data,  BYTE);
            }


            public byte u8 //uint8_
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  58, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  58);}
            }

            public sbyte s8 //int8_
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  59, 1));}
                set {  BitUtils.set_bytes((byte)(value), 1, data,  59);}
            }

            public short s16 //int16_
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  60, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  60);}
            }

            public int s32 //int32_
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  62, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  62);}
            }

            public long s64 //int64_
            {
                get {  return (long)((long) BitUtils.get_bytes(data,  66, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  66);}
            }

            public float f //floa
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  74, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 74);}
            }

            public double d //doubl
            {
                get {  return (double)(BitConverter.Int64BitsToDouble(BitUtils.get_bytes(data, 78, 8)));}
                set {  BitUtils.set_bytes(BitConverter.DoubleToInt64Bits(value), 8, data,  78);}
            }

            public byte[] u8_array //byte_arra
            {
                get {return u8_array_GET(new byte[3], 0);}
                set {u8_array_SET(value, 0)  ;}
            }
            public byte[]u8_array_GET(byte[] dst_ch, int pos)  //byte_arra
            {
                for(int BYTE = 86, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public void u8_array_SET(byte[] src, int pos)  //byte_arra
            {
                for(int BYTE =  86, src_max = pos + 3; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }


            public sbyte[] s8_array //sbyte_arra
            {
                get {return s8_array_GET(new sbyte[3], 0);}
                set {s8_array_SET(value, 0)  ;}
            }
            public sbyte[]s8_array_GET(sbyte[] dst_ch, int pos)  //sbyte_arra
            {
                for(int BYTE = 89, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (sbyte)((sbyte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public void s8_array_SET(sbyte[] src, int pos)  //sbyte_arra
            {
                for(int BYTE =  89, src_max = pos + 3; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((byte)(src[pos]), 1, data,  BYTE);
            }


            public short[] s16_array //short_arra
            {
                get {return s16_array_GET(new short[3], 0);}
                set {s16_array_SET(value, 0)  ;}
            }
            public short[]s16_array_GET(short[] dst_ch, int pos)  //short_arra
            {
                for(int BYTE = 92, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (short)((short) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }

            public void s16_array_SET(short[] src, int pos)  //short_arra
            {
                for(int BYTE =  92, src_max = pos + 3; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            }


            public int[] s32_array //int_arra
            {
                get {return s32_array_GET(new int[3], 0);}
                set {s32_array_SET(value, 0)  ;}
            }
            public int[]s32_array_GET(int[] dst_ch, int pos)  //int_arra
            {
                for(int BYTE = 98, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (int)((int) BitUtils.get_bytes(data,  BYTE, 4));
                return dst_ch;
            }

            public void s32_array_SET(int[] src, int pos)  //int_arra
            {
                for(int BYTE =  98, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes((uint)(src[pos]), 4, data,  BYTE);
            }


            public long[] s64_array //long_arra
            {
                get {return s64_array_GET(new long[3], 0);}
                set {s64_array_SET(value, 0)  ;}
            }
            public long[]s64_array_GET(long[] dst_ch, int pos)  //long_arra
            {
                for(int BYTE = 110, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 8)
                    dst_ch[pos] = (long)((long) BitUtils.get_bytes(data,  BYTE, 8));
                return dst_ch;
            }

            public void s64_array_SET(long[] src, int pos)  //long_arra
            {
                for(int BYTE =  110, src_max = pos + 3; pos < src_max; pos++, BYTE += 8)
                    BitUtils.set_bytes((ulong)(src[pos]), 8, data,  BYTE);
            }


            public float[] f_array //float_arra
            {
                get {return f_array_GET(new float[3], 0);}
                set {f_array_SET(value, 0)  ;}
            }
            public float[]f_array_GET(float[] dst_ch, int pos)  //float_arra
            {
                for(int BYTE = 134, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }

            public void f_array_SET(float[] src, int pos)  //float_arra
            {
                for(int BYTE =  134, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }


            public double[] d_array //double_arra
            {
                get {return d_array_GET(new double[3], 0);}
                set {d_array_SET(value, 0)  ;}
            }
            public double[]d_array_GET(double[] dst_ch, int pos)  //double_arra
            {
                for(int BYTE = 146, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 8)
                    dst_ch[pos] = (double)(BitConverter.Int64BitsToDouble(BitUtils.get_bytes(data, BYTE, 8)));
                return dst_ch;
            }

            public void d_array_SET(double[] src, int pos)  //double_arra
            {
                for(int BYTE =  146, src_max = pos + 3; pos < src_max; pos++, BYTE += 8)
                    BitUtils.set_bytes(BitConverter.DoubleToInt64Bits(src[pos]), 8, data,  BYTE);
            }

            public string s_TRY(Inside ph)//strin
            {
                if(ph.field_bit !=  1360 && !try_visit_field(ph, 1360)  ||  !try_visit_item(ph, 0)) return null;
                return new string(s_GET(ph, new char[ph.items], 0));
            }
            public char[]s_GET(Inside ph, char[] dst_ch, int pos) //strin
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int s_LEN(Inside ph)
            {
                return (ph.field_bit !=  1360 && !try_visit_field(ph, 1360)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public void s_SET(string src, Inside ph) //strin
            {s_SET(src.ToCharArray(), 0, src.Length, ph);} public void s_SET(char[] src, int pos, int items, Inside ph) //strin
            {
                if(ph.field_bit != 1360 && insert_field(ph, 1360, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta0 = new Meta(0, 5, 4, 4, 171, 1360, 0, _d);
        }

        public class LoopBackDemoChannel : Channel
        {
            public override bool CanRead { get { return true ; } }
            public override bool CanWrite { get { return true; } }
            static LoopBackDemoChannel() {pack_id_bytes = 0;}

            public static  LoopBackDemoChannel instance = new LoopBackDemoChannel();

            //interface-to-mark of sendable through this channel packs_Schs_Rchs
            public interface Sendable {}
            public static TEST_TYPES new_TEST_TYPES() {return new  TEST_TYPES();}

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
                            if(pack == null) return new TEST_TYPES(-1);
                            if(OnTEST_TYPESReceive == null) return null;
                            ph.setPack(pack);
                            OnTEST_TYPESReceive(this, ph, (TEST_TYPES) pack);
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
            public event TEST_TYPESReceiveHandler OnTEST_TYPESReceive;
            public delegate void TEST_TYPESReceiveHandler(Channel src, Inside ph, TEST_TYPES pack);
        }
        public class LoopBackDemoChannel_ADV : Channel.Advanced
        {
            public override bool CanRead { get { return false ; } }
            public override bool CanWrite { get { return true; } }
            static LoopBackDemoChannel_ADV() {pack_id_bytes = 0;}

            public static  LoopBackDemoChannel_ADV instance = new LoopBackDemoChannel_ADV();

            //interface-to-mark of sendable through this channel packs_Schs_Rchs
            public interface Sendable {}
            public static TEST_TYPES new_TEST_TYPES() {return new  TEST_TYPES();}

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

        static readonly Field _d = new Field(5, true, -4, 2, 1, 0, 0, 0, 1);

    }
}