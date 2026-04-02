#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace lidarbot_one {

class MicroRosHardware : public hardware_interface::SystemInterface {
public:

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
      return hardware_interface::CallbackReturn::ERROR;
    pos_ = {0.0, 0.0};
    vel_ = {0.0, 0.0};
    cmd_ = {0.0, 0.0};
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override
  {
    node_ = std::make_shared<rclcpp::Node>("hw_interface_node");

    cmd_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
      "joint_commands", 10);

    state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->position.size() >= 2) {
          pos_[0] = msg->position[0];
          pos_[1] = msg->position[1];
        }
        if (msg->velocity.size() >= 2) {
          vel_[0] = msg->velocity[0];
          vel_[1] = msg->velocity[1];
        }
      });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override
  {
    executor_->cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> interfaces;
    interfaces.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &pos_[0]);
    interfaces.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &vel_[0]);
    interfaces.emplace_back(info_.joints[1].name, hardware_interface::HW_IF_POSITION, &pos_[1]);
    interfaces.emplace_back(info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &vel_[1]);
    return interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &cmd_[0]);
    interfaces.emplace_back(info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &cmd_[1]);
    return interfaces;
  }

  hardware_interface::return_type read(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data = {static_cast<float>(cmd_[0]), static_cast<float>(cmd_[1])};
    cmd_pub_->publish(msg);
    return hardware_interface::return_type::OK;
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::array<double, 2> pos_, vel_, cmd_;
};

}  // namespace lidarbot_one

PLUGINLIB_EXPORT_CLASS(
  lidarbot_one::MicroRosHardware,
  hardware_interface::SystemInterface)