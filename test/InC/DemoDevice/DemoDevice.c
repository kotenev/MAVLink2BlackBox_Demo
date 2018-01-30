
#pragma once
#include "DemoDevice.h"
const int pack_id_bytes = 0;
static const Field _w = {5, true, -4, 2, 1, 0, 0, 0, 1, 1 };

static const Field * meta0fields[] = {&_w};
static const Meta meta0 = {0, 5, 4, 4, 0, 1360,  171, 1, (Field**)&meta0fields};

Pack * c_LoopBackDemoChannel_new_TEST_TYPES_0()
{
    Pack * pack = calloc(sizeof(Pack) + meta0.packMinBytes, 1);
    pack->meta = (Meta *)&meta0;
    return pack;
};
Pack * c_LoopBackDemoChannel_ADV_new_TEST_TYPES_0()
{
    Pack * pack = calloc(sizeof(Pack) + meta0.packMinBytes, 1);
    pack->meta = (Meta *)&meta0;
    return pack;
};
static Pack *  x__P(Pack* pack, int32_t id);
Channel c_LoopBackDemoChannel = {.process =  x__P }; //initialized channel instance


static Pack * x__P(Pack * pack, int32_t id)
{
#define rb_size0 (5)
    Meta * meta = NULL;
    static RBUF_INIT(Pack*, rb_size0) sendout_packs;
    static RBUF_INIT(Pack*, rb_size0) received_packs;
    static Bounds_Inside ph;
    for(bool LOOP = false;;)
    {
        switch(id)
        {
            default:
                //assert(0);
                return NULL;
            case 0:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta0;
                    goto new_pack;
                }
                c_LoopBackDemoChannel_on_TEST_TYPES_0(&ph, pack); //handle received pack
                break;
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
new_pack:
    pack = calloc(sizeof(Pack) + (meta->fields_count ? 0 : meta->packMinBytes), 1);
    pack->meta = meta;
    return pack;
}

static Pack *  x__w(Pack* pack, int32_t id);
Channel c_LoopBackDemoChannel_ADV = {.process =  x__w }; //initialized channel instance


static Pack * x__w(Pack * pack, int32_t id)
{
#define rb_size0 (5)
    Meta * meta = NULL;
    static RBUF_INIT(Pack*, rb_size0) sendout_packs;
    switch(id)
    {
        default:
            //assert(0);
            return NULL;
        case PROCESS_CHANNEL_REQEST:
            if(pack == NULL) return (RBUF_ISEMPTY(sendout_packs)) ? NULL : RBUF_GET(sendout_packs);
            return NULL;
        case PROCESS_HOST_REQEST:
            if(RBUF_ISFULL(sendout_packs)) return pack;
            RBUF_PUT(sendout_packs, pack)
            return NULL;
    }
}

