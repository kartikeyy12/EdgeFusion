#include "vision/gst_pipeline.hpp"

#include <cstdio>
#include <string>

namespace vision {

// ── Constructor ───────────────────────────────────────────────────────────
GstPipeline::GstPipeline(std::string_view pipeline_desc,
                         std::string_view appsink_name,
                         FrameCallback    callback)
    : frame_cb_{std::move(callback)}
{
    if (!gst_is_initialized()) {
        gst_init(nullptr, nullptr);
    }

    // 1. Parse the pipeline string into a live GstElement graph
    GError*      error{nullptr};
    GstElement*  raw = gst_parse_launch(
                           std::string{pipeline_desc}.c_str(), &error);

    if (error) {
        std::fprintf(stderr, "[GstPipeline] Parse error: %s\n", error->message);
        g_error_free(error);
        return;
    }
    pipeline_.reset(raw);

    // 2. Grab the appsink element by the name we gave it in the string
    GstElement* sink_elem = gst_bin_get_by_name(
                                GST_BIN(pipeline_.get()),
                                std::string{appsink_name}.c_str());
    if (!sink_elem) {
        std::fprintf(stderr, "[GstPipeline] appsink '%s' not found.\n",
                     std::string{appsink_name}.c_str());
        pipeline_.reset();
        return;
    }

    // gst_bin_get_by_name gives a +1 ref; drop it — pipeline_ owns lifetime
    gst_object_unref(sink_elem);
    appsink_raw_ = GST_APP_SINK(sink_elem);

    // 3. Configure appsink for real-time, zero-buffering behaviour
    //    emit-signals → fires our callback on every new frame
    //    sync=false   → don't wait for clock, deliver as fast as decoded
    //    max-buffers=1, drop=true → if we're slow, drop old frame, never block
    g_object_set(G_OBJECT(appsink_raw_),
        "emit-signals", TRUE,
        "sync",         FALSE,
        "max-buffers",  static_cast<guint>(1),
        "drop",         TRUE,
        nullptr
    );

    // 4. Connect the "new-sample" signal to our static trampoline
    //    'this' is passed as user_data so we can get back to the C++ instance
    g_signal_connect(appsink_raw_,
                     "new-sample",
                     G_CALLBACK(on_new_sample_trampoline),
                     this);
}

// ── Destructor ────────────────────────────────────────────────────────────
GstPipeline::~GstPipeline() {
    stop();
}

// ── start() ───────────────────────────────────────────────────────────────
bool GstPipeline::start() noexcept {
    if (!pipeline_) {
        std::fputs("[GstPipeline] Cannot start: pipeline not built.\n", stderr);
        return false;
    }

    GstStateChangeReturn ret =
        gst_element_set_state(pipeline_.get(), GST_STATE_PLAYING);

    if (ret == GST_STATE_CHANGE_FAILURE) {
        std::fputs("[GstPipeline] Failed to reach PLAYING state.\n", stderr);
        drain_bus();
        return false;
    }

    running_.store(true, std::memory_order_release);
    std::fputs("[GstPipeline] Pipeline started.\n", stdout);
    return true;
}

// ── stop() ────────────────────────────────────────────────────────────────
void GstPipeline::stop() noexcept {
    if (!pipeline_ || !running_.load(std::memory_order_acquire)) return;

    running_.store(false, std::memory_order_release);

    gst_element_send_event(pipeline_.get(), gst_event_new_eos());
    gst_element_set_state(pipeline_.get(), GST_STATE_NULL);
    drain_bus();

    std::fputs("[GstPipeline] Pipeline stopped.\n", stdout);
}

// ── Static trampoline ─────────────────────────────────────────────────────
GstFlowReturn
GstPipeline::on_new_sample_trampoline(GstAppSink* sink,
                                      gpointer    user_data) noexcept {
    return static_cast<GstPipeline*>(user_data)->on_new_sample(sink);
}

// ── on_new_sample — hot path, runs at 30Hz ────────────────────────────────
GstFlowReturn GstPipeline::on_new_sample(GstAppSink* sink) noexcept {
    if (!running_.load(std::memory_order_acquire))
        return GST_FLOW_EOS;

    // Pull the latest decoded frame
    UniqueGstSample sample{gst_app_sink_pull_sample(sink)};
    if (!sample) return GST_FLOW_ERROR;

    // Read the video format (width, height, stride) from the caps
    GstCaps*      caps  = gst_sample_get_caps(sample.get());
    GstVideoInfo  vinfo{};
    if (!gst_video_info_from_caps(&vinfo, caps)) return GST_FLOW_ERROR;

    // Map the buffer into CPU-readable memory (no copy for software decode)
    GstBuffer*     buf    = gst_sample_get_buffer(sample.get());
    GstVideoFrame  vframe{};
    if (!gst_video_frame_map(&vframe, &vinfo, buf, GST_MAP_READ))
        return GST_FLOW_ERROR;

    // Wrap GStreamer's memory directly into a cv::Mat — zero copy
    // The stride (bytes per row) accounts for any hardware padding
    cv::Mat frame(
        GST_VIDEO_INFO_HEIGHT(&vinfo),
        GST_VIDEO_INFO_WIDTH(&vinfo),
        CV_8UC3,
        GST_VIDEO_FRAME_PLANE_DATA(&vframe, 0),
        static_cast<std::size_t>(GST_VIDEO_FRAME_PLANE_STRIDE(&vframe, 0))
    );

    // PTS = presentation timestamp from the hardware decoder clock
    const std::int64_t pts_ns =
        static_cast<std::int64_t>(GST_BUFFER_PTS(buf));

    // Fire the callback — frame is only valid until gst_video_frame_unmap
    frame_cb_(frame, pts_ns);

    gst_video_frame_unmap(&vframe);
    return GST_FLOW_OK;
}

// ── drain_bus — print any GStreamer errors/warnings ───────────────────────
void GstPipeline::drain_bus() noexcept {
    GstBus* bus = gst_element_get_bus(pipeline_.get());
    if (!bus) return;

    GstMessage* msg{nullptr};
    while ((msg = gst_bus_pop(bus)) != nullptr) {
        if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
            GError* err{nullptr};
            gchar*  dbg{nullptr};
            gst_message_parse_error(msg, &err, &dbg);
            std::fprintf(stderr, "[GstBus] ERROR: %s\n", err->message);
            g_error_free(err);
            g_free(dbg);
        }
        gst_message_unref(msg);
    }
    gst_object_unref(bus);
}

} // namespace vision
