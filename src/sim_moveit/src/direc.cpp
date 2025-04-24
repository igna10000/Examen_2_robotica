#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>

int main(int argc, char * argv[])
{
  // Inicializar ROS y crear el Nodo
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Crear un logger de ROS
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Crear la interfaz MoveGroup de MoveIt
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // Definir las posiciones de las articulaciones
  std::vector<double> joint_positions = {1.3, 0.57, 0.0, 2.0, 1.0, 2.14, 0.5}; ///modificar el estado de cada joint

  // Establecer el objetivo de las articulaciones
  move_group_interface.setJointValueTarget(joint_positions);

  // Crear un plan hacia el objetivo de las articulaciones
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Ejecutar el plan
  if (success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Apagar ROS
  rclcpp::shutdown();
  return 0;
}

