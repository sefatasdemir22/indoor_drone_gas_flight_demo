#include "gas_distribution_sim/gas_field_plugin.hpp"
#include <iostream>

namespace gazebo
{
  GasFieldPlugin::GasFieldPlugin() : ModelPlugin(), pub_counter_(0)
  {
  }

  GasFieldPlugin::~GasFieldPlugin()
  {
  }

  void GasFieldPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // ROS 2 Node'unu başlat
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    
    std::string node_name = "gas_field_plugin_node";
    this->ros_node_ = std::make_shared<rclcpp::Node>(node_name);

    this->gas_params_pub_ = this->ros_node_->create_publisher<std_msgs::msg::String>(
      "/gas_distribution/active_zones", 10);

    // Parametreleri oku
    if (_sdf->HasElement("config_file"))
    {
      this->config_file_path_ = _sdf->Get<std::string>("config_file");
      RCLCPP_INFO(this->ros_node_->get_logger(), "Config File: %s", this->config_file_path_.c_str());
    }

    if (_sdf->HasElement("scenario"))
    {
      this->active_scenario_name_ = _sdf->Get<std::string>("scenario");
      RCLCPP_INFO(this->ros_node_->get_logger(), "Active Scenario: %s", this->active_scenario_name_.c_str());
    }

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GasFieldPlugin::OnUpdate, this));

    RCLCPP_INFO(this->ros_node_->get_logger(), "GasFieldPlugin BASARIYLA YUKLENDI!");
  }

  void GasFieldPlugin::OnUpdate()
  {
    rclcpp::spin_some(this->ros_node_);

    this->pub_counter_++;
    if (this->pub_counter_ >= 100)
    {
      auto message = std_msgs::msg::String();
      message.data = "{'scenario': '" + this->active_scenario_name_ + "'}";
      
      this->gas_params_pub_->publish(message);
      this->pub_counter_ = 0;
    }
  }
} 

// DÜZELTME BURADA: Başına 'gazebo::' ekledik.
GZ_REGISTER_MODEL_PLUGIN(gazebo::GasFieldPlugin)
