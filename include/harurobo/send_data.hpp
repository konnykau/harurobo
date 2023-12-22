#pragma once

#include <can_utils.hpp>
#include <utility>
namespace can_utils{
    inline std::unique_ptr<can_plugins2::msg::Frame> shirasu_target(const uint16_t id, const float data)
    {
        can_plugins2::msg::Frame frame;
        frame.id = id;
        frame.is_rtr = false;
        frame.is_extended = false;
        frame.is_error = false;

        frame.dlc = sizeof(float);//4個
        frame.data.fill(0);

        can_pack<float>(frame.data, data);
        std::swap(frame.data[0],frame.data[3]);
        std::swap(frame.data[2],frame.data[1]);

        return make_unique<can_plugins2::msg::Frame>(frame);
    }
}