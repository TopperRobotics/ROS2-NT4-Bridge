#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include "ntcore_cpp.h"
#include "json.hpp"

using json = nlohmann::json;

std::string roborioIPAddress;

struct Mapping {
    std::string ros_topic;
    std::string nt_topic;
    std::string type;
    std::vector<std::string> fields;
};

class RosNtBridge : public rclcpp::Node {
public:
    RosNtBridge(const std::string& config_path)
        : Node("ros_nt_bridge") {
        // ---- Load JSON Config ----
        std::ifstream file(config_path);
        if (!file.is_open()) {
            throw std::runtime_error("Could not open config file: " + config_path);
        }
        json j;
        file >> j;

        for (auto& m : j["ros_to_nt"]) {
            Mapping map;
            map.ros_topic = m["ros_topic"];
            map.nt_topic = m["nt_topic"];
            map.type = m["type"];
            if (m.contains("fields")) {
                for (auto& f : m["fields"]) map.fields.push_back(f);
            }
            ros_to_nt_.push_back(map);
        }
        for (auto& m : j["nt_to_ros"]) {
            Mapping map;
            map.ros_topic = m["ros_topic"];
            map.nt_topic = m["nt_topic"];
            map.type = m["type"];
            if (m.contains("fields")) {
                for (auto& f : m["fields"]) map.fields.push_back(f);
            }
            nt_to_ros_.push_back(map);
        }

        // ---- NT4 Setup ----
        inst_ = nt::GetDefaultInstance();
        nt::StartClient4(inst_, "ros2-nt-bridge");
        nt::SetServer(inst_, roborioIPAddress);  // Replace with roboRIO/NT server IP

        // ---- Setup ROS↔NT mappings ----
        setupRosToNt();
        setupNtToRos();

        // Poll NT regularly
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),  // 50 Hz
            [this]() { this->pollNT(); }
        );
    }

private:
    std::vector<Mapping> ros_to_nt_;
    std::vector<Mapping> nt_to_ros_;

    // ROS storage
    std::vector<rclcpp::SubscriptionBase::SharedPtr> ros_subs_;
    std::vector<rclcpp::PublisherBase::SharedPtr> ros_pubs_;
    rclcpp::TimerBase::SharedPtr timer_;

    // NT storage
    NT_Inst inst_;
    std::vector<NT_DoubleTopic> nt_double_topics_out_;
    std::vector<NT_DoubleTopic> nt_double_topics_in_;
    std::vector<NT_SubscriberDouble> nt_double_subs_;
    std::vector<Mapping> nt_in_mappings_;

    // -------------------------
    void setupRosToNt() {
        for (auto& m : ros_to_nt_) {
            if (m.type == "float64") {
                auto nt_topic = nt::GetDoubleTopic(inst_, m.nt_topic);
                nt_double_topics_out_.push_back(nt_topic);

                auto sub = this->create_subscription<std_msgs::msg::Float64>(
                    m.ros_topic, 10,
                    [nt_topic](const std_msgs::msg::Float64::SharedPtr msg) {
                        nt::SetDoubleTopic(nt_topic, msg->data);
                    }
                );
                ros_subs_.push_back(sub);

            }
            /*else if (m.type == "float32") {
                auto nt_topic = nt::GetDoubleTopic(inst_, m.nt_topic);
                nt_double_topics_out_.push_back(nt_topic);

                auto sub = this->create_subscription<std_msgs::msg::Float32>(
                    m.ros_topic, 10,
                    [nt_topic](const std_msgs::msg::Float32::SharedPtr msg) {
                        nt::SetDoubleTopic(nt_topic, static_cast<double>(msg->data));
                    }
                );
                ros_subs_.push_back(sub);

            }*/
            /*else if (m.type == "int32" || m.type == "uint32") {
                auto nt_topic = nt::GetDoubleTopic(inst_, m.nt_topic);
                nt_double_topics_out_.push_back(nt_topic);

                auto sub = this->create_subscription<std_msgs::msg::Int32>(
                    m.ros_topic, 10,
                    [nt_topic](const std_msgs::msg::Int32::SharedPtr msg) {
                        nt::SetDoubleTopic(nt_topic, static_cast<double>(msg->data));
                    }
                );
                ros_subs_.push_back(sub);

            }
            else if (m.type == "bool") {
                auto nt_topic = nt::GetBooleanTopic(inst_, m.nt_topic);
                auto sub = this->create_subscription<std_msgs::msg::Bool>(
                    m.ros_topic, 10,
                    [nt_topic](const std_msgs::msg::Bool::SharedPtr msg) {
                        nt::SetBooleanTopic(nt_topic, msg->data);
                    }
                );
                ros_subs_.push_back(sub);

            }
            else if (m.type == "string") {
                auto nt_topic = nt::GetStringTopic(inst_, m.nt_topic);
                auto sub = this->create_subscription<std_msgs::msg::String>(
                    m.ros_topic, 10,
                    [nt_topic](const std_msgs::msg::String::SharedPtr msg) {
                        nt::SetStringTopic(nt_topic, msg->data);
                    }
                );
                ros_subs_.push_back(sub);

            }
            else if (m.type == "geometry_msgs/Pose2D") {
                // Flatten Pose2D into x, y, theta
                std::vector<NT_DoubleTopic> field_topics;
                for (auto& f : m.fields) {
                    field_topics.push_back(
                        nt::GetDoubleTopic(inst_, m.nt_topic + "/" + f));
                }

                auto sub = this->create_subscription<geometry_msgs::msg::Pose2D>(
                    m.ros_topic, 10,
                    [field_topics](const geometry_msgs::msg::Pose2D::SharedPtr msg) {
                        nt::SetDoubleTopic(field_topics[0], msg->x);
                        nt::SetDoubleTopic(field_topics[1], msg->y);
                        nt::SetDoubleTopic(field_topics[2], msg->theta);
                    }
                );
                ros_subs_.push_back(sub);

            }*/
            else {
                RCLCPP_WARN(this->get_logger(), "Unsupported ROS->NT type: %s", m.type.c_str());
            }
        }
    }

    void setupNtToRos() {
        for (auto& m : nt_to_ros_) {
            if (m.type == "float64") {
                auto nt_topic = nt::GetDoubleTopic(inst_, m.nt_topic);
                auto sub = nt::Subscribe(nt_topic, 0.0);
                nt_double_topics_in_.push_back(nt_topic);
                nt_double_subs_.push_back(sub);
                nt_in_mappings_.push_back(m);

                auto pub = this->create_publisher<std_msgs::msg::Float64>(m.ros_topic, 10);
                ros_pubs_.push_back(pub);

            }
            /*else if (m.type == "float32") {
                auto nt_topic = nt::GetDoubleTopic(inst_, m.nt_topic);
                auto sub = nt::Subscribe(nt_topic, 0.0);
                nt_double_topics_in_.push_back(nt_topic);
                nt_double_subs_.push_back(sub);
                nt_in_mappings_.push_back(m);

                auto pub = this->create_publisher<std_msgs::msg::Float32>(m.ros_topic, 10);
                ros_pubs_.push_back(pub);

            }
            else if (m.type == "int32" || m.type == "uint32") {
                auto nt_topic = nt::GetDoubleTopic(inst_, m.nt_topic);
                auto sub = nt::Subscribe(nt_topic, 0.0);
                nt_double_topics_in_.push_back(nt_topic);
                nt_double_subs_.push_back(sub);
                nt_in_mappings_.push_back(m);

                auto pub = this->create_publisher<std_msgs::msg::Int32>(m.ros_topic, 10);
                ros_pubs_.push_back(pub);

            }
            else if (m.type == "bool") {
                auto nt_topic = nt::GetBooleanTopic(inst_, m.nt_topic);
                auto sub = nt::Subscribe(nt_topic, false);
                nt_in_mappings_.push_back(m);

                auto pub = this->create_publisher<std_msgs::msg::Bool>(m.ros_topic, 10);
                ros_pubs_.push_back(pub);

            }
            else if (m.type == "string") {
                auto nt_topic = nt::GetStringTopic(inst_, m.nt_topic);
                auto sub = nt::Subscribe(nt_topic, "");
                nt_in_mappings_.push_back(m);

                auto pub = this->create_publisher<std_msgs::msg::String>(m.ros_topic, 10);
                ros_pubs_.push_back(pub);

            }
            else if (m.type == "geometry_msgs/Pose2D") {
                std::vector<NT_DoubleTopic> field_topics;
                std::vector<NT_SubscriberDouble> field_subs;
                for (auto& f : m.fields) {
                    auto nt_topic = nt::GetDoubleTopic(inst_, m.nt_topic + "/" + f);
                    field_topics.push_back(nt_topic);
                    field_subs.push_back(nt::Subscribe(nt_topic, 0.0));
                }
                // Store mapping and subscribers
                nt_double_topics_in_.insert(nt_double_topics_in_.end(),
                    field_topics.begin(), field_topics.end());
                nt_double_subs_.insert(nt_double_subs_.end(),
                    field_subs.begin(), field_subs.end());
                nt_in_mappings_.push_back(m);

                auto pub = this->create_publisher<geometry_msgs::msg::Pose2D>(m.ros_topic, 10);
                ros_pubs_.push_back(pub);

            }*/
            else {
                RCLCPP_WARN(this->get_logger(), "Unsupported NT->ROS type: %s", m.type.c_str());
            }
        }
    }

    void pollNT() {
        size_t pub_index = 0;
        for (size_t i = 0; i < nt_in_mappings_.size(); i++) {
            const auto& m = nt_in_mappings_[i];

            if (m.type == "float64") {
                auto data = nt::ReadQueueValue(nt_double_subs_[i]);
                if (data) {
                    auto msg = std_msgs::msg::Float64();
                    msg.data = data->value;
                    auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Float64>>(ros_pubs_[pub_index]);
                    pub->publish(msg);
                }
                pub_index++;

            }
            /*else if (m.type == "float32") {
                auto data = nt::ReadQueueValue(nt_double_subs_[i]);
                if (data) {
                    auto msg = std_msgs::msg::Float32();
                    msg.data = static_cast<float>(data->value);
                    auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Float32>>(ros_pubs_[pub_index]);
                    pub->publish(msg);
                }
                pub_index++;

            }
            else if (m.type == "int32" || m.type == "uint32") {
                auto data = nt::ReadQueueValue(nt_double_subs_[i]);
                if (data) {
                    auto msg = std_msgs::msg::Int32();
                    msg.data = static_cast<int32_t>(data->value);
                    auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Int32>>(ros_pubs_[pub_index]);
                    pub->publish(msg);
                }
                pub_index++;

            }
            else if (m.type == "bool") {
                auto data = nt::ReadQueueValue(inst_, m.nt_topic);
                if (data) {
                    auto msg = std_msgs::msg::Bool();
                    msg.data = *data;
                    auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Bool>>(ros_pubs_[pub_index]);
                    pub->publish(msg);
                }
                pub_index++;

            }
            else if (m.type == "string") {
                auto data = nt::ReadQueueValue(inst_, m.nt_topic);
                if (data) {
                    auto msg = std_msgs::msg::String();
                    msg.data = *data;
                    auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::String>>(ros_pubs_[pub_index]);
                    pub->publish(msg);
                }
                pub_index++;

            }
            else if (m.type == "geometry_msgs/Pose2D") {
                if (i + 2 < nt_double_subs_.size()) {
                    auto d0 = nt::ReadQueueValue(nt_double_subs_[i]);
                    auto d1 = nt::ReadQueueValue(nt_double_subs_[i + 1]);
                    auto d2 = nt::ReadQueueValue(nt_double_subs_[i + 2]);

                    if (d0 && d1 && d2) {
                        auto msg = geometry_msgs::msg::Pose2D();
                        msg.x = d0->value;
                        msg.y = d1->value;
                        msg.theta = d2->value;

                        auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::Pose2D>>(ros_pubs_[pub_index]);
                        pub->publish(msg);
                    }
                    pub_index++;
                }
            }*/
           else {
                std::cout<<"here here here here here here pollnt else";
           }
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    if (argc < 2) {
        std::cerr << "Usage: ros_nt_bridge <config.json> <roboRIO IP>" << std::endl;
        return 1;
    }
    auto node = std::make_shared<RosNtBridge>(argv[1]);
    roborioIPAddress = argv[2];
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
