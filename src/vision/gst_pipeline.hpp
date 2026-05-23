#pragma once

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>
#include <opencv2/core/mat.hpp>

#include <atomic>
#include <functional>
#include <memory>
#include <string_view>

namespace vision {

// ── RAII deleters for GStreamer's ref-counted objects ─────────────────────
struct GstElementDeleter {
    void operator()(GstElement* e) const noexcept {
        if (e) gst_object_unref(e);
    }
};

struct GstSampleDeleter {
    void operator()(GstSample* s) const noexcept {
        if (s) gst_sample_unref(s);
    }
};

using UniqueGstElement = std::unique_ptr<GstElement, GstElementDeleter>;
using UniqueGstSample  = std::unique_ptr<GstSample,  GstSampleDeleter>;

// ── The callback your fusion engine will register ─────────────────────────
// frame  → zero-copy view into GStreamer memory, valid ONLY inside callback
// pts_ns → hardware timestamp in nanoseconds
using FrameCallback = std::function<void(const cv::Mat& frame,
                                         std::int64_t   pts_ns)>;

// ── GstPipeline — owns one full GStreamer decode pipeline ─────────────────
class GstPipeline {
public:
    explicit GstPipeline(std::string_view pipeline_desc,
                         std::string_view appsink_name,
                         FrameCallback    callback);

    // Non-copyable — owns hardware resources
    GstPipeline(const GstPipeline&)            = delete;
    GstPipeline& operator=(const GstPipeline&) = delete;

    ~GstPipeline();

    [[nodiscard]] bool start() noexcept;
    void               stop()  noexcept;

    [[nodiscard]] bool is_running() const noexcept {
        return running_.load(std::memory_order_acquire);
    }

private:
    // Static trampoline — required because GStreamer is a C API
    // and cannot hold a C++ member function pointer directly
    static GstFlowReturn on_new_sample_trampoline(GstAppSink* sink,
                                                  gpointer    user_data) noexcept;

    GstFlowReturn on_new_sample(GstAppSink* sink) noexcept;
    void          drain_bus() noexcept;

    UniqueGstElement  pipeline_;
    GstAppSink*       appsink_raw_{nullptr};
    FrameCallback     frame_cb_;
    std::atomic<bool> running_{false};
};

} // namespace vision
