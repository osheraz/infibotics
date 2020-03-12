//
// Created by yossi on 11/30/18.
//

#ifndef INFI_SIM_V1_PARTSPAWNPLUGIN_H
#define INFI_SIM_V1_PARTSPAWNPLUGIN_H

#include "misc/text_colors.h"
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Vector3.hh>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"


#define DEFAULT_PART_NAME "spawned_part_001_brick"

namespace gazebo {
    class PartSpawn : public ModelPlugin {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
        // Called by the world update start event
        void OnUpdate(); /// Will be called on each simulation iteration.
        PartSpawn();
    private:
        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;
        // Pointer to the model
        physics::ModelPtr model;
        sdf::ElementPtr sdf;

        physics::WorldPtr world;
        std::string model_name, default_part_name;

        std::unique_ptr<ros::NodeHandle> rosNode;
        ros::Publisher status_pub;
        ros::ServiceServer main_srv;

        /// \brief A node used for transport
        transport::NodePtr node;

        /// \brief A subscriber to a named topic.
        transport::SubscriberPtr sub;
        transport::SubscriberPtr contactSub; //subscriber to contact updates


        bool spawn_service(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
        void OnContact(const ConstContactsPtr &_msg);
        void dummy_callback(const std_msgs::String::ConstPtr &data);
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(PartSpawn)
}

#endif //INFI_SIM_V1_PARTSPAWNPLUGIN_H
