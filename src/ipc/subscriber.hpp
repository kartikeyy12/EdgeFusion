#pragma once
#include <zmq.hpp>
#include <functional>
#include <span>
#include <string>
#include <string_view>
#include <vector>
#include <atomic>

namespace ipc {

using MessageCallback = std::function<void(std::string_view topic,
                                           std::span<const std::byte> data)>;

class Subscriber {
public:
    explicit Subscriber(zmq::context_t& ctx, std::string_view endpoint) {
        socket_ = zmq::socket_t{ctx, zmq::socket_type::sub};
        socket_.set(zmq::sockopt::linger,  0);
        socket_.set(zmq::sockopt::rcvhwm,  1);
        socket_.connect(std::string{endpoint});
        socket_.set(zmq::sockopt::subscribe, "");
    }

    zmq::socket_t& socket() { return socket_; }

private:
    zmq::socket_t socket_;
};

inline void poll_two(Subscriber& a, Subscriber& b,
                     MessageCallback cb,
                     const std::atomic<bool>& shutdown) {
    zmq::pollitem_t items[] = {
        {a.socket().handle(), 0, ZMQ_POLLIN, 0},
        {b.socket().handle(), 0, ZMQ_POLLIN, 0},
    };

    while (!shutdown.load(std::memory_order_acquire)) {
        zmq::poll(items, 2, std::chrono::milliseconds{100});

        for (int i = 0; i < 2; ++i) {
            if (!(items[i].revents & ZMQ_POLLIN)) continue;

            zmq::message_t topic_msg, data_msg;
            auto& sock = (i == 0) ? a.socket() : b.socket();
            (void)sock.recv(topic_msg, zmq::recv_flags::none);
            (void)sock.recv(data_msg,  zmq::recv_flags::none);

            std::string_view topic{
                static_cast<const char*>(topic_msg.data()), topic_msg.size()};
            std::span<const std::byte> data{
                static_cast<const std::byte*>(data_msg.data()), data_msg.size()};

            cb(topic, data);
        }
    }
}

} // namespace ipc
