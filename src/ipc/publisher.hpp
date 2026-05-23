#pragma once
#include <zmq.hpp>
#include <string_view>
#include <span>

namespace ipc {

class Publisher {
public:
    explicit Publisher(zmq::context_t& ctx, std::string_view endpoint) {
        socket_ = zmq::socket_t{ctx, zmq::socket_type::pub};
        socket_.set(zmq::sockopt::linger, 0);
        socket_.set(zmq::sockopt::sndhwm, 1);  // drop old, never buffer
        socket_.bind(std::string{endpoint});
    }

    // Send topic + raw bytes — zero extra copy
    void send(std::string_view topic, std::span<const std::byte> data) {
        zmq::message_t t_msg{topic.data(), topic.size()};
        zmq::message_t d_msg{data.data(), data.size()};
        socket_.send(t_msg, zmq::send_flags::sndmore);
        socket_.send(d_msg, zmq::send_flags::none);
    }

private:
    zmq::socket_t socket_;
};

} // namespace ipc
