#include <functional>
#include <memory>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>

#include <gazebo/physics/physics.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace gazebo
{

class WheelEncoderPlugin : public ModelPlugin
{
public:

  WheelEncoderPlugin() = default;

  virtual ~WheelEncoderPlugin()
  {
    update_connection_.reset();
  }

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;

    if (!rclcpp::ok())
    {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }

    node_ = std::make_shared<rclcpp::Node>("wheel_encoder_plugin");

    left_pub_ =
      node_->create_publisher<std_msgs::msg::Float64>(
        "/VelocityEncL",
        10);

    right_pub_ =
      node_->create_publisher<std_msgs::msg::Float64>(
        "/VelocityEncR",
        10);

    left_joint_name_ = "wheel_left_joint";
    right_joint_name_ = "wheel_right_joint";

    if (sdf->HasElement("left_joint"))
    {
      left_joint_name_ =
        sdf->Get<std::string>("left_joint");
    }

    if (sdf->HasElement("right_joint"))
    {
      right_joint_name_ =
        sdf->Get<std::string>("right_joint");
    }

    left_joint_ =
      model_->GetJoint(left_joint_name_);

    right_joint_ =
      model_->GetJoint(right_joint_name_);

    if (!left_joint_)
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Could not find joint: %s",
        left_joint_name_.c_str());
      return;
    }

    if (!right_joint_)
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Could not find joint: %s",
        right_joint_name_.c_str());
      return;
    }

#if GAZEBO_MAJOR_VERSION >= 8
    last_publish_time_ = model_->GetWorld()->SimTime();
#else
    last_publish_time_ = model_->GetWorld()->GetSimTime();
#endif

    update_connection_ =
      event::Events::ConnectWorldUpdateBegin(
        std::bind(
          &WheelEncoderPlugin::OnUpdate,
          this));

    RCLCPP_INFO(
      node_->get_logger(),
      "Wheel Encoder Plugin Loaded");
  }

private:

  void OnUpdate()
  {
#if GAZEBO_MAJOR_VERSION >= 8
    gazebo::common::Time current_time =
      model_->GetWorld()->SimTime();
#else
    gazebo::common::Time current_time =
      model_->GetWorld()->GetSimTime();
#endif

    double dt =
      (current_time - last_publish_time_).Double();

    if (dt < 0.02)
    {
      return;
    }

    last_publish_time_ = current_time;

    std_msgs::msg::Float64 left_msg;
    std_msgs::msg::Float64 right_msg;

    // Velocidades angulares en rad/s
    left_msg.data = left_joint_->GetVelocity(0);
    right_msg.data = right_joint_->GetVelocity(0);

    left_pub_->publish(left_msg);
    right_pub_->publish(right_msg);

    rclcpp::spin_some(node_);
  }

private:

  physics::ModelPtr model_;

  physics::JointPtr left_joint_;
  physics::JointPtr right_joint_;

  std::string left_joint_name_;
  std::string right_joint_name_;

  gazebo::common::Time last_publish_time_;

  event::ConnectionPtr update_connection_;

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_pub_;
};

GZ_REGISTER_MODEL_PLUGIN(WheelEncoderPlugin)

} // namespace gazebo