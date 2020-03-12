//
// Created by yossi on 11/30/18.
//

#include "partSpawnPlugin.h"


namespace gazebo {
    void PartSpawn::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
        // Store the pointer to the model
        this->model = _parent;
        this->sdf = _sdf;
        world = this->model->GetWorld();
        model_name = model->GetName();

        if (sdf->HasElement("part_name")) {
            default_part_name = _sdf->GetElement("part_name")->GetValue()->GetAsString();
        }
        else {
            std::cout << YELLOW_TXT << "Model: " << model_name
                      << " no part_name element specified in SDF. Using default: " << DEFAULT_PART_NAME << NO_COLOR
                      << std::endl;
            default_part_name = DEFAULT_PART_NAME;
        }

        std::string ros_node_name("spawn_plugin_" + model_name);
        std::string topicName = "~/" + model_name + "/topic";

        std::cout << WHITE_TXT << "Loaded PartSpawn plugin for model: " << model_name << NO_COLOR << std::endl;
        // std::cout << WHITE_TXT << "Model: " << model_name << " loaded in world: " << world->GetName() <<  NO_COLOR << std::endl;
        // std::cout << WHITE_TXT << "sdf->GetAttributeCount(): " << this->sdf->GetAttributeCount() << NO_COLOR << std::endl;

        if (!ros::isInitialized()) { // TODO: Check is this condition needed.
            int argc = 0;
            char **argv = NULL;

            std::cout << WHITE_TXT << "Initializing node: " << ros_node_name << NO_COLOR << std::endl;
            ros::init(argc, argv, ros_node_name, ros::init_options::NoSigintHandler);
            ros::NodeHandle n;
        }
        else {
            // std::cout << YELLOW_TXT << "ros::isInitialized is true." << NO_COLOR << std::endl;
            // ros::NodeHandle n;
            // std::cout << YELLOW_TXT << "ros::NodeHandle getNamespace is: " << n.getNamespace() << NO_COLOR << std::endl;
        }
        // this->rosNode.reset(new ros::NodeHandle(ros_node_name));
        this->rosNode.reset(new ros::NodeHandle());
        // std::cout << YELLOW_TXT << "rosNode->getNamespace is: " << rosNode->getNamespace() << NO_COLOR << std::endl;
        status_pub = rosNode->advertise<std_msgs::String>(model_name + "/status", 10);
        main_srv = rosNode->advertiseService(model_name + "/spawn_part", &PartSpawn::spawn_service, this);


        // Listen to the update event. This event is broadcast every
        // simulation iteration.

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&PartSpawn::OnUpdate, this));
    }

    void PartSpawn::OnContact(const ConstContactsPtr &_msg) {
        std::cout << GREEN_TXT << "PartSpawn::OnContact got a message. " << NO_COLOR << std::endl;
    }

    void PartSpawn::dummy_callback(const std_msgs::String::ConstPtr &data) {
        std::cout << GREEN_TXT << "Got Message: " << data->data << NO_COLOR << std::endl;
    }

    bool PartSpawn::spawn_service(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
        std::cout << WHITE_TXT << "Model: " << model_name << " spawn_service is called." << NO_COLOR << std::endl;
        std::cout << WHITE_TXT << "Spawning model: " << default_part_name << NO_COLOR << std::endl;

        world->InsertModelFile("model://" + default_part_name);


        res.success = 1;
        res.message = "Desired model succesfully spawned";
        return true;
    }

    void PartSpawn::OnUpdate() {
        // std::cout << GREEN_TXT << "Update world. " << this->model->GetName()<< NO_COLOR << std::endl;
        // std_msgs::String msg;
        // msg.data = model->GetName();
        // status_pub.publish(msg);


        // Apply a small linear velocity to the model.
        // this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
    }

    PartSpawn::PartSpawn() {
        // std::cout << GREEN_TXT << "PartSpawn() called. " << NO_COLOR << std::endl;
        // std::cout << GREEN_TXT << "ros::isInitialized() " << ros::isInitialized() << NO_COLOR << std::endl;
    }
}


#ifdef SOMETHING_THAT_NEVER_SHOULD_BE_DEFINED
// this->node = transport::NodePtr(new transport::Node());
// this->node->Init(ros_node_name);
//
// transport::NodePtr node = transport::NodePtr(new transport::Node());
// node->Init(ros_node_name);
//
// std::cout << node->GetTopicNamespace() << std::endl;
// std::cout << this->node->GetTopicNamespace() << std::endl;



// std::cout << this->node->GetId() << std::endl;

/*if (!this->contactSub) {
    std::cout << "Subscribing contact manager to topic " << topicName << std::endl;
    this->contactSub = this->node->Subscribe(topicName, &PartSpawn::dummy_callback, this);
}*/


// if (!this->sub) {
//     std::cout<<"Subscribing to topic "<<topicName<<std::endl;
//     this->sub = this->node->Subscribe(topicName,&PartSpawn::dummy_callback, this, false);
// }

// Subscribe to the topic, and register a callback
// this->sub = this->node->Subscribe(topicName, &PartSpawn::dummy_callback, this);


/*
        /// Starting ros node

        if (1 || !ros::isInitialized()) { // TODO: Check is this condition needed.
            int argc = 0;
            char **argv = NULL;

            std::cout << WHITE_TXT << "Initializing node: " << ros_node_name << NO_COLOR << std::endl;
            ros::init(argc, argv, ros_node_name, ros::init_options::NoSigintHandler);
            ros::NodeHandle n;
        } else {
            std::cout << YELLOW_TXT << "ros::isInitialized is true." << NO_COLOR << std::endl;

        }
        this->rosNode.reset(new ros::NodeHandle(ros_node_name));
        status_pub = this->rosNode->advertise<std_msgs::String>(ros_node_name+"/status", 1);

std::cout << WHITE_TXT << ros_node_name << " in namespace: " << this->node->getNamespace() << NO_COLOR
          << std::endl;*/
}
#endif