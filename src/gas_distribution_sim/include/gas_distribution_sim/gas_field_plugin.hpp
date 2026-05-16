#ifndef GAS_FIELD_PLUGIN_HPP_
#define GAS_FIELD_PLUGIN_HPP_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp> 

namespace gazebo
{
  // Bu sınıf, Gazebo dünyasında görünmez bir 'yönetici' olarak çalışacak.
  class GasFieldPlugin : public ModelPlugin
  {
  public:
    GasFieldPlugin();
    virtual ~GasFieldPlugin();

    // Plugin yüklendiğinde çalışacak fonksiyon
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Her simülasyon adımında çalışacak fonksiyon
    virtual void OnUpdate();

  private:
    // ROS 2 Entegrasyonu
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gas_params_pub_;

    // Gazebo Olayı
    event::ConnectionPtr update_connection_;

    // Parametreleri tutacağımız yapı
    std::string config_file_path_;
    std::string active_scenario_name_;
    
    // Basit bir sayaç (sürekli log basmasın diye)
    int pub_counter_;
  };
}

#endif
