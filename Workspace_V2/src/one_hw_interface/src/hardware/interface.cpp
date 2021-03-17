//
// Created by one on 3/17/21.
//

#include "hardware/interface.h"

namespace hardware {
    interface::interface() {

    }

    bool interface::add_joint_(const joint& joint_info) {
        if (joints_.count(joint_info.name)) {
            ROS_ERROR("Joint %s already exists.", joint_info.name.c_str());
            return false;
        }

        // append list
        joints_[joint_info.name] = std::make_unique<hardware::joint>(joint_info);

        // add can
        if (can_adapter_)

        // set can mapping
        joint *ptr = joints_[joint_info.name].get();
        can2joint_[std::make_pair(joint_info.can_id)]

        return true;
    }

    size_t interface::add_can_(const std::string& can_interface) {
        // find existing
        for (size_t id = 0; id < can_adapter_.size(); id++) {
            if (can_adapter_[id]->interface_ == can_interface) {
                return id;
            }
        }

        // create new
        size_t new_if_id = can_adapter_.size();
        can_adapter_.push_back(std::make_unique<can::SocketCAN>(can_interface, [=](const can_frame &frame) {
            can_receive_callback_(new_if_id, frame);
        }));

        return new_if_id;
    }

    void interface::can_receive_callback_(const size_t can_if_id, const can_frame &frame) {
        // FIXME: Multi-threading here, called from another thread!

        // get joint
        std::map< pair<int, int>, joint* >::iterator it = can2joint_.find(std::make_pair(can_if_id, frame.can_id));
        if (it == can2joint_.end()) {
            ROS_WARN_THROTTLE(1., "Unknown CAN frame on %s ID %x",
                              can_adapter_[size_t(can_if_id)]->interface_.c_str(),
                              frame.can_id);
            return;
        }
        joint *joint_item =
    }
}