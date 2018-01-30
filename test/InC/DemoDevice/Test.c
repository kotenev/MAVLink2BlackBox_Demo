
#pragma once
#include "DemoDevice.h"
#include <string.h>
#include <stdio.h>
#include <assert.h>

INLINER int32_t Arrays_equals(uint8_t * L, uint8_t * R, int32_t bytes)
{
    for(int32_t i = 0; i < bytes; i++) if(L[i] != R[i]) return i;
    return -1;
}
static Pack * zero2empty(Pack * pack)
{
    pack = realloc(pack, sizeof(Pack) + pack->meta->packMinBytes);
    for(int32_t i = pack->meta->packMinBytes; -1 < --i;)pack->data[i] = 0; //clean bytes
    return pack;
}
void failure(Channel * ch, int32_t id, int32_t arg) {assert(false);}




void c_LoopBackDemoChannel_on_TEST_TYPES_0(Bounds_Inside * ph, Pack * pack)
{
    {
        double exemplary[] =  {6.513313752378887E307, -1.4418217467781236E308, -2.756464658589273E307} ;
        double*  sample = p0_d_array_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 24);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        int16_t exemplary[] =  {(int16_t)11149, (int16_t)13123, (int16_t)32356} ;
        int16_t*  sample = p0_s16_array_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p0_c_GET(pack) == (uint16_t)(uint16_t)12575);
    assert(p0_s32_GET(pack) == (int32_t) -609246627);
    assert(p0_u16_GET(pack) == (uint16_t)(uint16_t)25326);
    assert(p0_u8_GET(pack) == (uint8_t)(uint8_t)248);
    {
        uint64_t exemplary[] =  {5309132582570721661L, 7425048208630760714L, 4216449606896362227L} ;
        uint64_t*  sample = p0_u64_array_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 24);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {2.7083414E38F, 1.4175893E38F, 1.9511514E38F} ;
        float*  sample = p0_f_array_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p0_u32_GET(pack) == (uint32_t)2572392107L);
    assert(p0_s8_GET(pack) == (int8_t)(int8_t) -125);
    assert(p0_s_LEN(ph) == 9);
    {
        char16_t * exemplary = u"dAzieohqx";
        char16_t * sample = p0_s_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        int64_t exemplary[] =  {1195347618534627240L, -6036448290186581052L, 2639251285536695700L} ;
        int64_t*  sample = p0_s64_array_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 24);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)45, (uint8_t)52, (uint8_t)119} ;
        uint8_t*  sample = p0_u8_array_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 3);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint32_t exemplary[] =  {125735436L, 603512684L, 1783073488L} ;
        uint32_t*  sample = p0_u32_array_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p0_f_GET(pack) == (float)3.3530393E38F);
    {
        int8_t exemplary[] =  {(int8_t) -3, (int8_t) -63, (int8_t) -36} ;
        int8_t*  sample = p0_s8_array_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 3);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        int32_t exemplary[] =  {699592188, -932703711, -1444951554} ;
        int32_t*  sample = p0_s32_array_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p0_s64_GET(pack) == (int64_t)7856075271052316282L);
    assert(p0_u64_GET(pack) == (uint64_t)7400510159785866200L);
    assert(p0_s16_GET(pack) == (int16_t)(int16_t) -22548);
    assert(p0_d_GET(pack) == (double) -1.709187986831777E306);
    {
        uint16_t exemplary[] =  {(uint16_t)58353, (uint16_t)39681, (uint16_t)65257} ;
        uint16_t*  sample = p0_u16_array_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};



Pack * c_TEST_Channel_process(Pack * pack, int32_t id)
{
#define rb_size0 (5)
    static RBUF_INIT(Pack*, rb_size0) sendout_packs;
    static RBUF_INIT(Pack*, rb_size0) received_packs;
    static Bounds_Inside ph;
    for(bool LOOP = false;;)
    {
        switch(id) /*220*/
        {
            default:
                assert(0);
                return NULL;
            case PROCESS_CHANNEL_REQEST:
                if(pack == NULL) return (RBUF_ISEMPTY(sendout_packs)) ? NULL : RBUF_GET(sendout_packs);
                if(RBUF_ISFULL(received_packs)) return pack;
                RBUF_PUT(received_packs, pack)
                return NULL;
            case PROCESS_HOST_REQEST:
                if(pack == NULL) return (RBUF_ISEMPTY(received_packs)) ? NULL : RBUF_GET(received_packs);
                if(RBUF_ISFULL(sendout_packs)) return pack;
                RBUF_PUT(sendout_packs, pack)
                return NULL;
            case PROCESS_RECEIVED_PACKS:
                LOOP = true;
                goto next_pack;
        }
        free(pack); // dispose pack
        if(!LOOP) return NULL;
next_pack:
        if(RBUF_ISEMPTY(received_packs)) return NULL;
        pack = RBUF_GET(received_packs);
        id = pack->meta->id;
        setPack(pack, &ph);
    }
}
Channel c_TEST_Channel = {.process = c_TEST_Channel_process};


void c_TEST_Channel_process_received() { c_TEST_Channel_process(NULL, PROCESS_RECEIVED_PACKS); }
void c_TEST_Channel_send(Pack* pack) { c_TEST_Channel_process(pack, PROCESS_HOST_REQEST); }
int main()
{
    static uint8_t buff[512];
    static Bounds_Inside PH;
    {
        setPack(c_LoopBackDemoChannel_new_TEST_TYPES_0(), &PH);
        {
            int64_t s64_array[] =  {1195347618534627240L, -6036448290186581052L, 2639251285536695700L};
            p0_s64_array_SET(&s64_array, 0, PH.base.pack) ;
        }
        p0_s16_SET((int16_t)(int16_t) -22548, PH.base.pack) ;
        p0_s32_SET((int32_t) -609246627, PH.base.pack) ;
        {
            int32_t s32_array[] =  {699592188, -932703711, -1444951554};
            p0_s32_array_SET(&s32_array, 0, PH.base.pack) ;
        }
        {
            int16_t s16_array[] =  {(int16_t)11149, (int16_t)13123, (int16_t)32356};
            p0_s16_array_SET(&s16_array, 0, PH.base.pack) ;
        }
        p0_u64_SET((uint64_t)7400510159785866200L, PH.base.pack) ;
        p0_u16_SET((uint16_t)(uint16_t)25326, PH.base.pack) ;
        {
            double d_array[] =  {6.513313752378887E307, -1.4418217467781236E308, -2.756464658589273E307};
            p0_d_array_SET(&d_array, 0, PH.base.pack) ;
        }
        p0_s8_SET((int8_t)(int8_t) -125, PH.base.pack) ;
        p0_d_SET((double) -1.709187986831777E306, PH.base.pack) ;
        {
            float f_array[] =  {2.7083414E38F, 1.4175893E38F, 1.9511514E38F};
            p0_f_array_SET(&f_array, 0, PH.base.pack) ;
        }
        p0_u8_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
        p0_c_SET((uint16_t)(uint16_t)12575, PH.base.pack) ;
        {
            uint64_t u64_array[] =  {5309132582570721661L, 7425048208630760714L, 4216449606896362227L};
            p0_u64_array_SET(&u64_array, 0, PH.base.pack) ;
        }
        {
            uint32_t u32_array[] =  {125735436L, 603512684L, 1783073488L};
            p0_u32_array_SET(&u32_array, 0, PH.base.pack) ;
        }
        {
            uint16_t u16_array[] =  {(uint16_t)58353, (uint16_t)39681, (uint16_t)65257};
            p0_u16_array_SET(&u16_array, 0, PH.base.pack) ;
        }
        p0_f_SET((float)3.3530393E38F, PH.base.pack) ;
        p0_s64_SET((int64_t)7856075271052316282L, PH.base.pack) ;
        {
            char16_t* s = u"dAzieohqx";
            p0_s_SET_(s, &PH) ;
        }
        {
            uint8_t u8_array[] =  {(uint8_t)45, (uint8_t)52, (uint8_t)119};
            p0_u8_array_SET(&u8_array, 0, PH.base.pack) ;
        }
        p0_u32_SET((uint32_t)2572392107L, PH.base.pack) ;
        {
            int8_t s8_array[] =  {(int8_t) -3, (int8_t) -63, (int8_t) -36};
            p0_s8_array_SET(&s8_array, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_TEST_TYPES_0(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
}

