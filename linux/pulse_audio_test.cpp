#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <mutex>

#include <pulse/error.h>
#include <pulse/def.h>
#include <pulse/pulseaudio.h>
#include <pulse/simple.h>

typedef unsigned short  WORD;
typedef unsigned int    DWORD;
typedef unsigned int    UINT;
typedef long long REFERENCE_TIME;
typedef unsigned short USHORT;

static const REFERENCE_TIME MinimumPeriod = 30000;
static const REFERENCE_TIME DefaultPeriod = 100000;

static std::mutex pulse_mtx;

enum phys_device_bus_type {
    phys_device_bus_invalid = -1,
    phys_device_bus_pci,
    phys_device_bus_usb
};

typedef enum _EndpointFormFactor {
    RemoteNetworkDevice = 0,
    Speakers = 1,
    LineLevel = 2,
    Headphones = 3,
    Microphone = 4,
    Headset = 5,
    Handset = 6,
    UnknownDigitalPassthrough = 7,
    SPDIF = 8,
    DigitalAudioDisplayDevice = 9,
    UnknownFormFactor = 10,
    EndpointFormFactor_enum_count = 11
} EndpointFormFactor;

typedef struct _PhysDevice {
    std::string pulse_name;
    std::string friendly_name;
    phys_device_bus_type bus_type;
    USHORT vendor_id, product_id;
    EndpointFormFactor form;
    UINT index;
} PhysDevice;

enum data_flow {
    eCapture,
    eRender
};

typedef struct {
    data_flow dataflow;
    pa_sample_spec ss;
    pa_buffer_attr attri;
    std::string pulse_name;
} stream_params;

typedef struct {
    std::vector<PhysDevice> speakers;
    std::vector<PhysDevice> microphones;

} my_audio_ctx;

static UINT pulse_channel_map_dump(const pa_channel_map *map)
{
    int i;
    UINT mask = 0;

    for (i = 0; i < map->channels; ++i) {
        printf(" channel %s\n", pa_channel_position_to_string(map->map[i]));
    }

    return mask;
}

#define MAX_DEVICE_NAME_LEN 62

static std::string get_device_name(std::string desc, pa_proplist *proplist)
{
    /*
       Some broken apps (e.g. Split/Second with fmodex) can't handle names that
       are too long and crash even on native. If the device desc is too long,
       we'll attempt to incrementally build it to try to stay under the limit.
       ( + 1 is to check against truncated buffer after ntdll_umbstowcs )
    */

    /* For monitors of sinks; this does not seem to be localized in PA either */
    //static const WCHAR monitor_of[] = {'M','o','n','i','t','o','r',' ','o','f',' '};

    size_t len = desc.length();
    std::string prop;
    if (len > MAX_DEVICE_NAME_LEN && proplist) {
        auto prop_char = pa_proplist_gets(proplist, PA_PROP_DEVICE_CLASS);
        prop = prop_char ? prop_char : "";
        int monitor = 0;

        if (prop.compare("monitor") == 0) {
            monitor = 1;
        }

        prop_char = pa_proplist_gets(proplist, PA_PROP_DEVICE_PRODUCT_NAME);
        prop = prop_char ? prop_char : "";
        if (prop.empty()) {
            prop_char = pa_proplist_gets(proplist, "alsa.card_name");
            prop = prop_char ? prop_char : "";
        }
    }
    return prop;
}

static void fill_device_info(PhysDevice *dev, pa_proplist *p)
{
    const char *buffer;

    dev->bus_type = phys_device_bus_invalid;
    dev->vendor_id = 0;
    dev->product_id = 0;

    if (!p)
        return;

    if ((buffer = pa_proplist_gets(p, PA_PROP_DEVICE_BUS))) {
        std::string tmp = buffer;
        if (!tmp.compare("usb"))
            dev->bus_type = phys_device_bus_usb;
        else if (!tmp.compare("pci"))
            dev->bus_type = phys_device_bus_pci;
    }

    if ((buffer = pa_proplist_gets(p, PA_PROP_DEVICE_VENDOR_ID)))
        dev->vendor_id = strtol(buffer, NULL, 16);

    if ((buffer = pa_proplist_gets(p, PA_PROP_DEVICE_PRODUCT_ID)))
        dev->product_id = strtol(buffer, NULL, 16);
}

static void dump_attr(const pa_buffer_attr *attr)
{
    printf("maxlength: %u\n", attr->maxlength);
    printf("minreq: %u\n", attr->minreq);
    printf("fragsize: %u\n", attr->fragsize);
    printf("tlength: %u\n", attr->tlength);
    printf("prebuf: %u\n", attr->prebuf);
}

static int pulse_poll_cb(struct pollfd *ufds, unsigned long nfds, int timeout, void*userdata)
{
    return 0;
}

static void pulse_server_info_cb(pa_context *c, const pa_server_info *i, void *userdata)
{
    printf("default sink name = %s\n", i->default_sink_name);
}

static void pulse_context_state_cb(pa_context *c, void *userdata) 
{
    switch (pa_context_get_state(c)) 
    {
        default:
            printf("Unhandled state: %i\n", pa_context_get_state(c));
            return;

        case PA_CONTEXT_CONNECTING:
        case PA_CONTEXT_UNCONNECTED:
        case PA_CONTEXT_AUTHORIZING:
        case PA_CONTEXT_SETTING_NAME:
        case PA_CONTEXT_TERMINATED:
            printf("State change to %i\n", pa_context_get_state(c));
            return;

        case PA_CONTEXT_READY:
            printf("Ready\n");
            break;
        case PA_CONTEXT_FAILED:
            printf("Context failed: %s\n", pa_strerror(pa_context_errno(c)));
            break;
    }
}

static void pulse_stream_state_cb(pa_stream *s, void *user)
{
    pa_stream_state_t state = pa_stream_get_state(s);
    printf("Stream state changed to %i\n", state);
}

static void pulse_stream_underflow_cb(pa_stream *s, void *userdata)
{
    printf("%p: Underflow\n", userdata);
}

static void pulse_stream_started_cb(pa_stream *s, void *userdata)
{
    printf("%p: (Re)started playing\n", userdata);
}

static void pulse_attr_update_cb(pa_stream *s, void *user) {
    const pa_buffer_attr *attr = pa_stream_get_buffer_attr(s);
    printf("New attributes or device moved:\n");
    dump_attr(attr);
}

static void pulse_phys_info_cb(pa_context* c, const pa_sink_info* info, int eol, void* userdata) 
{
    if (info != nullptr) 
    {
        for (int i = 0; i < info->volume.channels; ++i) 
        {
            // 打印各个声道的音量
            std::cout << (info->volume.values[i]*1.0 / info->base_volume)*100 << std::endl;
        }
        std::cout << "mute:" << info->mute << std::endl;
    }
}

static void pulse_phys_speakers_cb(pa_context *c, const pa_sink_info *i, int eol, void *userdata)
{
    my_audio_ctx *ctx = (my_audio_ctx*)userdata;
    if (!i || !i->name || !i->name[0])
        return;
    printf("sink info\n");
    pulse_channel_map_dump(&i->channel_map);

    PhysDevice dev;
    dev.form = Speakers;
    dev.index = i->index;
    dev.friendly_name = get_device_name(i->description ? i->description : "", i->proplist);
    dev.pulse_name = i->name ? i->name : "";
    fill_device_info(&dev, i->proplist);
    
    printf("pulse_phys_speakers_cb name \n   %s\n", dev.pulse_name.c_str());
    printf("pulse_phys_speakers_cb friendly \n   %s\n", dev.friendly_name.c_str());

    ctx->speakers.push_back(std::move(dev));
}

static void pulse_phys_sources_cb(pa_context *c, const pa_source_info *i, int eol, void *userdata)
{
    if (!i || !i->name || !i->name[0])
        return;
    printf("source info\n");
    pulse_channel_map_dump(&i->channel_map);

    my_audio_ctx *ctx = (my_audio_ctx*)userdata;

    PhysDevice dev;
    dev.form = (i->monitor_of_sink == PA_INVALID_INDEX) ? Microphone : LineLevel;
    dev.index = i->index;
    dev.friendly_name = get_device_name(i->description ? i->description : "", i->proplist);
    dev.pulse_name = i->name ? i->name : "";
    fill_device_info(&dev, i->proplist);
    printf("pulse_phys_sources_cb name \n   %s\n", dev.pulse_name.c_str());
    printf("pulse_phys_sources_cb friendly \n   %s\n", dev.friendly_name.c_str());
    ctx->speakers.push_back(std::move(dev));
}

static void pulse_phys_subscribe_cb(pa_context *c, pa_subscription_event_type_t type, uint32_t idx, void *userdata)
{
    unsigned facility = type & PA_SUBSCRIPTION_EVENT_FACILITY_MASK;
    //type &= PA_SUBSCRIPTION_EVENT_TYPE_MASK;

    pa_operation *op = NULL;

    printf("%s\n", __FUNCTION__);

    switch (facility)
    {
        case PA_SUBSCRIPTION_EVENT_SINK:
            op = pa_context_get_sink_info_by_index(c, idx, pulse_phys_info_cb, userdata);
            break;

        default:
            assert(0); // Got event we aren't expecting.
            break;
    }
    
    if (op)
        pa_operation_unref(op);
}

static int pulse_audio_connect(pa_mainloop** pa_ml, pa_context** pa_ctx, const char* server, const char* description, void* userdata) 
{
    int ret;
    *pa_ml = NULL;
    *pa_ml = pa_mainloop_new();
    if (!(*pa_ml)) { return -1; }
 
    pa_mainloop_api* pa_mlapi = NULL;
    pa_mlapi = pa_mainloop_get_api(*pa_ml);
    if (!pa_mlapi) { return -1; }
 
    *pa_ctx = NULL;
    *pa_ctx = pa_context_new(pa_mlapi, description);
    if (!(*pa_ctx)) { return -1; }
    
    pa_context_set_state_callback(*pa_ctx, 
                                  pulse_context_state_cb,
                                  userdata);
    
    printf("libpulse protocol version: %u. API Version %u\n", 
        pa_context_get_protocol_version(*pa_ctx), PA_API_VERSION);

    if (pa_context_connect(*pa_ctx, server, PA_CONTEXT_NOFLAGS, NULL) < 0) 
    {
        return -1;
    }
 
    /* Wait for connection */
    while (pa_mainloop_iterate(*pa_ml, 1, &ret) >= 0) 
    {
        pa_context_state_t state = pa_context_get_state(*pa_ctx);

        if (state == PA_CONTEXT_FAILED || state == PA_CONTEXT_TERMINATED)
            return -2;

        if (state == PA_CONTEXT_READY)
            break;
    }

    if (pa_context_get_state(*pa_ctx) != PA_CONTEXT_READY)
        return -3;

    printf("Test-connected to server %s with protocol version: %i.\n",
        pa_context_get_server(*pa_ctx),
        pa_context_get_server_protocol_version(*pa_ctx));
    
    pa_context_get_server_info(*pa_ctx, pulse_server_info_cb, userdata);
    return 0;
}

static void pulse_audio_disconnect(pa_mainloop** pa_ml, pa_context** pa_ctx) {
    assert(pa_ml);
    assert(pa_ctx);
 
    if (*pa_ctx) {
        pa_context_set_state_callback(*pa_ctx, NULL, NULL);
        pa_context_disconnect(*pa_ctx);
        pa_context_unref(*pa_ctx);
    }
 
    if (*pa_ml) pa_mainloop_free(*pa_ml);
    *pa_ml = NULL;
    *pa_ctx = NULL;
}

static void pulse_mainloop_wait(pa_mainloop* pa_ml, pa_operation* pa_op)
{
    int ret;
    if (pa_op) 
    {
        while (pa_mainloop_iterate(pa_ml, 1, &ret) >= 0 &&
                pa_operation_get_state(pa_op) == PA_OPERATION_RUNNING)
        {}
        pa_operation_unref(pa_op);
    }
}

static void pulse_probe_settings(pa_mainloop* pulse_ml, pa_context *pulse_ctx, stream_params& params, my_audio_ctx *ctx) 
{
    pa_stream *stream;
    pa_channel_map map;
    pa_sample_spec ss; 
    pa_buffer_attr attr;
    pa_stream_flags sflags;
    int ret;
    unsigned int length = 0;
    const char* pulse_name = NULL;
    if(!params.pulse_name.empty())
    {
        pulse_name = params.pulse_name.c_str();
    }
    printf("%s\n", __FUNCTION__);

    pa_channel_map_init_auto(&map, 2, PA_CHANNEL_MAP_ALSA);
    ss.rate = 48000;
    ss.format = PA_SAMPLE_FLOAT32LE;
    ss.channels = map.channels;

    attr.maxlength = -1;
    attr.tlength = -1;
    attr.minreq = attr.fragsize = pa_frame_size(&ss);
    attr.prebuf = 0;

    sflags = (pa_stream_flags)(PA_STREAM_START_CORKED|
        PA_STREAM_FIX_RATE|
        PA_STREAM_FIX_CHANNELS|
        PA_STREAM_EARLY_REQUESTS), 
    
    stream = pa_stream_new(pulse_ctx, "format test stream", &ss, &map);
    if (stream)
        pa_stream_set_state_callback(stream, pulse_stream_state_cb, NULL);
    if (!stream)
        ret = -1;
    else if (params.dataflow == eRender)
        ret = pa_stream_connect_playback(stream, 
                pulse_name, 
                &attr,
                sflags,
                NULL, 
                NULL);
    else
        ret = pa_stream_connect_record(stream, 
                pulse_name, 
                &attr, 
                sflags);
    
    if (ret >= 0) 
    {
        while (pa_mainloop_iterate(pulse_ml, 1, &ret) >= 0 &&
                pa_stream_get_state(stream) == PA_STREAM_CREATING)
        {}
        if (pa_stream_get_state(stream) == PA_STREAM_READY) 
        {
            params.ss = *pa_stream_get_sample_spec(stream);
            params.attri = *pa_stream_get_buffer_attr(stream);
            map = *pa_stream_get_channel_map(stream);
            pa_stream_disconnect(stream);
            while (pa_mainloop_iterate(pulse_ml, 1, &ret) >= 0 &&
                    pa_stream_get_state(stream) == PA_STREAM_READY)
            {}
        }
    }

    if (stream)
        pa_stream_unref(stream);
}

static pa_stream* pulse_stream_connect(pa_mainloop* pa_ml, pa_context *pulse_ctx, stream_params& params, my_audio_ctx* ctx)
{
    int ret;
    char buffer[64];
    const char* pulse_name = NULL;
    pa_channel_map map;
    int flags = (PA_STREAM_START_CORKED | 
                PA_STREAM_START_UNMUTED | 
                PA_STREAM_ADJUST_LATENCY);

    auto pmap = pa_channel_map_init_auto(&map, params.ss.channels, PA_CHANNEL_MAP_ALSA);
    if(!pmap) 
    {
        fprintf(stderr, "pa_channel_map_init_auto failed\n");
        return NULL;
    }

    pa_stream* stream = pa_stream_new(pulse_ctx, buffer, &params.ss, &map);

    if (!stream) {
        fprintf(stderr, "pa_stream_new returned error %i\n", pa_context_errno(pulse_ctx));
        return NULL;
    }

    pa_stream_set_state_callback(stream, pulse_stream_state_cb, ctx);
    pa_stream_set_buffer_attr_callback(stream, pulse_attr_update_cb, ctx);
    pa_stream_set_moved_callback(stream, pulse_attr_update_cb, stream);

    dump_attr(&params.attri);

    /* If specific device was requested, use it exactly */
    pulse_name = params.pulse_name.c_str();
    if (!params.pulse_name.empty())
        flags |= PA_STREAM_DONT_MOVE;
    else
        pulse_name = NULL;  /* use default */

    if (params.dataflow == eRender)
        ret = pa_stream_connect_playback(stream, pulse_name, &params.attri, (pa_stream_flags_t)flags, NULL, NULL);
    else
        ret = pa_stream_connect_record(stream, pulse_name, &params.attri, (pa_stream_flags_t)flags);

    if (ret < 0) {
        pa_stream_unref(stream);
        stream = NULL;
        printf("Returns %i\n", ret);
        return NULL;
    }
    while (pa_mainloop_iterate(pa_ml, 1, &ret) >= 0 && 
        pa_stream_get_state(stream) == PA_STREAM_CREATING)
    {}
    if (pa_stream_get_state(stream) != PA_STREAM_READY)
    {
        pa_stream_disconnect(stream);
        pa_stream_unref(stream);
        stream = NULL;
        return NULL;
    }

    if (params.dataflow == eRender) {
        pa_stream_set_underflow_callback(stream, pulse_stream_underflow_cb, stream);
        pa_stream_set_started_callback(stream, pulse_stream_started_cb, stream);
    }
    params.ss = *pa_stream_get_sample_spec(stream);
    params.attri = *pa_stream_get_buffer_attr(stream);
    return stream;
}

int main(int argc, char** argv) 
{
    pa_mainloop* pa_ml = nullptr;
    pa_operation* pa_op = nullptr;
    pa_context* pa_ctx = nullptr;
    pa_stream* pa_stream = nullptr;
    my_audio_ctx ctx;
    stream_params params[2];

    int ret = pulse_audio_connect(&pa_ml, &pa_ctx, nullptr, "audio recorder", &ctx);
    std::shared_ptr<void> raii_connect(nullptr, [&](void*) {
        pulse_audio_disconnect(&pa_ml, &pa_ctx);
    });
    if (ret != 0 || pa_ctx == nullptr) { return 0; }

    params[0].dataflow = eRender;
    params[1].dataflow = eCapture;

    pulse_probe_settings(pa_ml, pa_ctx, params[0], &ctx);
    pulse_probe_settings(pa_ml, pa_ctx, params[1], &ctx);

    pa_op = pa_context_get_sink_info_list(pa_ctx, &pulse_phys_speakers_cb, &ctx);
    pulse_mainloop_wait(pa_ml, pa_op);

    pa_op = pa_context_get_source_info_list(pa_ctx, &pulse_phys_sources_cb, &ctx);
    pulse_mainloop_wait(pa_ml, pa_op);

    // Subscribe to sink events from the server. This is how we get
    // volume change notifications from the server.
    pa_context_set_subscribe_callback(pa_ctx, pulse_phys_subscribe_cb, &ctx);
    pa_context_subscribe(pa_ctx, PA_SUBSCRIPTION_MASK_SINK, NULL, NULL);

    int ch;
    while (ch = getchar())
    {
        if(ch == 'q')
            break;
        else if(ch == 'c')
        {
            stream_params capture = params[1];
            pa_stream = pulse_stream_connect(pa_ml, pa_ctx, capture, &ctx);
            if(pa_stream)
            {

            }
            break;
        }
    }
    
    return 0;
}