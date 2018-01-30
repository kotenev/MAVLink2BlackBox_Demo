
#pragma once

#include "BlackBox/Host.h"
#include <uchar.h>
INLINER size_t strlen16(register const char16_t * string)
{
    if(!string) return 0;
    register size_t len = 0;
    while(string[len])len++;
    return len;
}
Pack * c_LoopBackDemoChannel_new_TEST_TYPES_0();
Pack * c_LoopBackDemoChannel_ADV_new_TEST_TYPES_0();
extern void c_LoopBackDemoChannel_on_TEST_TYPES_0(Bounds_Inside * bi, Pack * pack);

extern Channel c_LoopBackDemoChannel;
INLINER void c_LoopBackDemoChannel_send(Pack * pack) {c_LoopBackDemoChannel.process(pack, PROCESS_HOST_REQEST);}
INLINER int32_t  c_LoopBackDemoChannel_input_bytes(uint8_t* dst, int32_t bytes) { return input_bytes(&c_LoopBackDemoChannel, dst, bytes);}

//------------------------
INLINER void  c_LoopBackDemoChannel_output_bytes(uint8_t* src, int32_t bytes) {  output_bytes(&c_LoopBackDemoChannel, src, bytes);} //pass received from a network bytes to the channel
INLINER void c_LoopBackDemoChannel_process_received() { c_LoopBackDemoChannel.process(NULL, PROCESS_RECEIVED_PACKS);}//dispatch received packs to its handlers

extern Channel c_LoopBackDemoChannel_ADV;
INLINER void c_LoopBackDemoChannel_ADV_send(Pack * pack) {c_LoopBackDemoChannel_ADV.process(pack, PROCESS_HOST_REQEST);}
INLINER int32_t  c_LoopBackDemoChannel_ADV_input_bytes(uint8_t* dst, int32_t bytes) { return input_bytes_adv(&c_LoopBackDemoChannel_ADV, dst, bytes);}


INLINER uint16_t p0_c_GET(Pack * src)//cha
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER void p0_c_SET(uint16_t  src, Pack * dst)//cha
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER uint16_t p0_u16_GET(Pack * src)//uint16_
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER void p0_u16_SET(uint16_t  src, Pack * dst)//uint16_
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER uint16_t* p0_u16_array_GET(Pack * src, uint16_t*  dst, int32_t pos) //uint16_t_arra
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 4, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}

static const  uint32_t p0_u16_array_LEN = 3; //return array length

INLINER  uint16_t*  p0_u16_array_GET_(Pack * src) {return p0_u16_array_GET(src, malloc(3 * sizeof(uint16_t)), 0);}
INLINER void p0_u16_array_SET(uint16_t*  src, int32_t pos, Pack * dst) //uint16_t_arra
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  4, src_max = pos + 3; pos < src_max; pos++, BYTE += 2)
        set_bytes((src[pos]), 2, data,  BYTE);
}
INLINER uint32_t p0_u32_GET(Pack * src)//uint32_
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 4)));
}
INLINER void p0_u32_SET(uint32_t  src, Pack * dst)//uint32_
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  10);
}
INLINER uint32_t* p0_u32_array_GET(Pack * src, uint32_t*  dst, int32_t pos) //uint32_t_arra
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 14, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = ((get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p0_u32_array_LEN = 3; //return array length

INLINER  uint32_t*  p0_u32_array_GET_(Pack * src) {return p0_u32_array_GET(src, malloc(3 * sizeof(uint32_t)), 0);}
INLINER void p0_u32_array_SET(uint32_t*  src, int32_t pos, Pack * dst) //uint32_t_arra
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  14, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
        set_bytes((src[pos]), 4, data,  BYTE);
}
INLINER uint64_t p0_u64_GET(Pack * src)//uint64_
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  26, 8)));
}
INLINER void p0_u64_SET(uint64_t  src, Pack * dst)//uint64_
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  26);
}
INLINER uint64_t* p0_u64_array_GET(Pack * src, uint64_t*  dst, int32_t pos) //uint64_t_arra
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 34, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 8)
        dst[pos] = ((get_bytes(data,  BYTE, 8)));
    return dst;
}

static const  uint32_t p0_u64_array_LEN = 3; //return array length

INLINER  uint64_t*  p0_u64_array_GET_(Pack * src) {return p0_u64_array_GET(src, malloc(3 * sizeof(uint64_t)), 0);}
INLINER void p0_u64_array_SET(uint64_t*  src, int32_t pos, Pack * dst) //uint64_t_arra
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  34, src_max = pos + 3; pos < src_max; pos++, BYTE += 8)
        set_bytes((src[pos]), 8, data,  BYTE);
}
INLINER uint8_t p0_u8_GET(Pack * src)//uint8_
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  58, 1)));
}
INLINER void p0_u8_SET(uint8_t  src, Pack * dst)//uint8_
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  58);
}
INLINER int8_t p0_s8_GET(Pack * src)//int8_
{
    uint8_t * data = src->data;
    return ((int8_t)(get_bytes(data,  59, 1)));
}
INLINER void p0_s8_SET(int8_t  src, Pack * dst)//int8_
{
    uint8_t * data = dst->data;
    set_bytes((uint8_t)(src), 1, data,  59);
}
INLINER int16_t p0_s16_GET(Pack * src)//int16_
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  60, 2)));
}
INLINER void p0_s16_SET(int16_t  src, Pack * dst)//int16_
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  60);
}
INLINER int32_t p0_s32_GET(Pack * src)//int32_
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  62, 4)));
}
INLINER void p0_s32_SET(int32_t  src, Pack * dst)//int32_
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  62);
}
INLINER int64_t p0_s64_GET(Pack * src)//int64_
{
    uint8_t * data = src->data;
    return ((int64_t)(get_bytes(data,  66, 8)));
}
INLINER void p0_s64_SET(int64_t  src, Pack * dst)//int64_
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  66);
}
INLINER float p0_f_GET(Pack * src)//floa
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  74, 4)));
}
INLINER void p0_f_SET(float  src, Pack * dst)//floa
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  74);
}
INLINER double p0_d_GET(Pack * src)//doubl
{
    uint8_t * data = src->data;
    return (longBitsToDouble(get_bytes(data,  78, 8)));
}
INLINER void p0_d_SET(double  src, Pack * dst)//doubl
{
    uint8_t * data = dst->data;
    set_bytes(doubleToLongBits(src), 8, data,  78);
}
INLINER uint8_t* p0_u8_array_GET(Pack * src, uint8_t*  dst, int32_t pos) //uint8_t_arra
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 86, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p0_u8_array_LEN = 3; //return array length

INLINER  uint8_t*  p0_u8_array_GET_(Pack * src) {return p0_u8_array_GET(src, malloc(3 * sizeof(uint8_t)), 0);}
INLINER void p0_u8_array_SET(uint8_t*  src, int32_t pos, Pack * dst) //uint8_t_arra
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  86, src_max = pos + 3; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER int8_t* p0_s8_array_GET(Pack * src, int8_t*  dst, int32_t pos) //int8_t_arra
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 89, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((int8_t)(get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p0_s8_array_LEN = 3; //return array length

INLINER  int8_t*  p0_s8_array_GET_(Pack * src) {return p0_s8_array_GET(src, malloc(3 * sizeof(int8_t)), 0);}
INLINER void p0_s8_array_SET(int8_t*  src, int32_t pos, Pack * dst) //int8_t_arra
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  89, src_max = pos + 3; pos < src_max; pos++, BYTE += 1)
        set_bytes((uint8_t)(src[pos]), 1, data,  BYTE);
}
INLINER int16_t* p0_s16_array_GET(Pack * src, int16_t*  dst, int32_t pos) //int16_t_arra
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 92, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((int16_t)(get_bytes(data,  BYTE, 2)));
    return dst;
}

static const  uint32_t p0_s16_array_LEN = 3; //return array length

INLINER  int16_t*  p0_s16_array_GET_(Pack * src) {return p0_s16_array_GET(src, malloc(3 * sizeof(int16_t)), 0);}
INLINER void p0_s16_array_SET(int16_t*  src, int32_t pos, Pack * dst) //int16_t_arra
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  92, src_max = pos + 3; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER int32_t* p0_s32_array_GET(Pack * src, int32_t*  dst, int32_t pos) //int32_t_arra
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 98, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = ((int32_t)(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p0_s32_array_LEN = 3; //return array length

INLINER  int32_t*  p0_s32_array_GET_(Pack * src) {return p0_s32_array_GET(src, malloc(3 * sizeof(int32_t)), 0);}
INLINER void p0_s32_array_SET(int32_t*  src, int32_t pos, Pack * dst) //int32_t_arra
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  98, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
        set_bytes((uint32_t)(src[pos]), 4, data,  BYTE);
}
INLINER int64_t* p0_s64_array_GET(Pack * src, int64_t*  dst, int32_t pos) //int64_t_arra
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 110, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 8)
        dst[pos] = ((int64_t)(get_bytes(data,  BYTE, 8)));
    return dst;
}

static const  uint32_t p0_s64_array_LEN = 3; //return array length

INLINER  int64_t*  p0_s64_array_GET_(Pack * src) {return p0_s64_array_GET(src, malloc(3 * sizeof(int64_t)), 0);}
INLINER void p0_s64_array_SET(int64_t*  src, int32_t pos, Pack * dst) //int64_t_arra
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  110, src_max = pos + 3; pos < src_max; pos++, BYTE += 8)
        set_bytes((src[pos]), 8, data,  BYTE);
}
INLINER float* p0_f_array_GET(Pack * src, float*  dst, int32_t pos) //float_arra
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 134, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p0_f_array_LEN = 3; //return array length

INLINER  float*  p0_f_array_GET_(Pack * src) {return p0_f_array_GET(src, malloc(3 * sizeof(float)), 0);}
INLINER void p0_f_array_SET(float*  src, int32_t pos, Pack * dst) //float_arra
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  134, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER double* p0_d_array_GET(Pack * src, double*  dst, int32_t pos) //double_arra
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 146, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 8)
        dst[pos] = (longBitsToDouble(get_bytes(data,  BYTE, 8)));
    return dst;
}

static const  uint32_t p0_d_array_LEN = 3; //return array length

INLINER  double*  p0_d_array_GET_(Pack * src) {return p0_d_array_GET(src, malloc(3 * sizeof(double)), 0);}
INLINER void p0_d_array_SET(double*  src, int32_t pos, Pack * dst) //double_arra
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  146, src_max = pos + 3; pos < src_max; pos++, BYTE += 8)
        set_bytes(doubleToLongBits(src[pos]), 8, data,  BYTE);
}
INLINER char16_t * p0_s_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //strin
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p0_s_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  1360 && !try_visit_field(src, 1360)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p0_s_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  1360 && !try_visit_field(src, 1360)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p0_s_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER void p0_s_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //strin
{
    if(dst->base.field_bit != 1360 && insert_field(dst, 1360, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p0_s_SET_(char16_t*  src, Bounds_Inside * dst) {p0_s_SET(src, 0, strlen16(src), dst);}

