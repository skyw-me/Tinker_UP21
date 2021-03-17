//
// Created by one on 3/17/21.
//

#ifndef SRC_INTERFACE_H
#define SRC_INTERFACE_H

#include "hardware/socketcan.h"

#include <ros/ros.h>

#include <map>
#include <vector>

namespace hardware {
    struct joint {
        // name
        std::string name;

        // interface
        std::string can_interface;
        canid_t can_id;

        // status
        double pos;
        double vel;
        double torque;

        // params
        double param_act2vel;
        double param_act2pos;
        double param_torque2act;
    };

    class interface {
    public:
        interface();

    private:
        // joints
        std::map< std::string, std::unique_ptr<joint> > joints_;

        // can interface
        std::vector< std::unique_ptr<can::SocketCAN> > can_adapter_;
        std::map< std::pair<size_t, canid_t>, joint* > can2joint_;

        // add joint
        bool add_joint_(const joint& joint_info);

        // find can
        size_t add_can_(const std::string& can_interface);

        // callback
        void can_receive_callback_(const size_t can_if_id, const can_frame &frame);
    };
}

#endif //SRC_INTERFACE_H
