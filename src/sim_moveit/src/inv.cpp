#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>
#include <iostream>

geometry_msgs::msg::Pose create_pose(double x, double y, double z, double w)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = w;
  return pose;
}

void add_obstacles()
{
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

  moveit_msgs::msg::CollisionObject obstacle1;
  obstacle1.id = "box1";
  obstacle1.header.frame_id = "panda_link0";
  shape_msgs::msg::SolidPrimitive primitive1;
  primitive1.type = primitive1.BOX;
  primitive1.dimensions = {0.2, 0.2, 0.2};
  geometry_msgs::msg::Pose box_pose1;
  box_pose1.orientation.w = 1.0;
  box_pose1.position.x = 0.5;
  box_pose1.position.y = 0.0;
  box_pose1.position.z = 0.1;
  obstacle1.primitives.push_back(primitive1);
  obstacle1.primitive_poses.push_back(box_pose1);
  obstacle1.operation = obstacle1.ADD;

  moveit_msgs::msg::CollisionObject obstacle2;
  obstacle2.id = "box2";
  obstacle2.header.frame_id = "panda_link0";
  shape_msgs::msg::SolidPrimitive primitive2;
  primitive2.type = primitive2.BOX;
  primitive2.dimensions = {0.2, 0.2, 0.2};
  geometry_msgs::msg::Pose box_pose2;
  box_pose2.orientation.w = 1.0;
  box_pose2.position.x = 0.2;
  box_pose2.position.y = -0.3;
  box_pose2.position.z = 0.1;
  obstacle2.primitives.push_back(primitive2);
  obstacle2.primitive_poses.push_back(box_pose2);
  obstacle2.operation = obstacle2.ADD;

  collision_objects.push_back(obstacle1);
  collision_objects.push_back(obstacle2);

  planning_scene_interface.addCollisionObjects(collision_objects);
}

void move_to_pose(const geometry_msgs::msg::Pose& target_pose, moveit::planning_interface::MoveGroupInterface& move_group_interface, rclcpp::Logger logger)
{
  move_group_interface.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    move_group_interface.execute(plan);
    RCLCPP_INFO(logger, "Successfully moved to target pose.");
  }
  else
  {
    RCLCPP_ERROR(logger, "Failed to plan path to target pose.");
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("panda_interactive_ik");
  auto logger = rclcpp::get_logger("panda_interactive_ik");

  moveit::planning_interface::MoveGroupInterface move_group_interface(node, "panda_arm");
  move_group_interface.setPlanningTime(10.0);

  add_obstacles();

  geometry_msgs::msg::Pose pose1 = create_pose(-0.58, 0.2, 0.5, 1.0); ///punto1
  geometry_msgs::msg::Pose pose2 = create_pose(0.6, 0.4, 0.1, 0.7);   ///punto2
  geometry_msgs::msg::Pose pose3 = create_pose(0.1, -0.6, 0.2, 0.0);  ///punto3

  while (rclcpp::ok())
  {
    std::cout << "\nMENU:\n";
    std::cout << "1: Mover a posición 1\n";
    std::cout << "2: Mover a posición 2\n";
    std::cout << "3: Mover a posición 3\n";
    std::cout << "4: Ingresar coordenadas deseadas\n";
    std::cout << "5: Salir\n";
    std::cout << "Seleccione una opción: ";

    int option;
    std::cin >> option;

    if (option == 1)
    {
      move_to_pose(pose1, move_group_interface, logger);
    }
    else if (option == 2)
    {
      move_to_pose(pose2, move_group_interface, logger);
    }
    else if (option == 3)
    {
      move_to_pose(pose3, move_group_interface, logger);
    }
    else if (option == 4)
    {
      double x, y, z, w;
      std::cout << "Ingrese x, y, z, w: ";
      std::cin >> x >> y >> z >> w;
      geometry_msgs::msg::Pose user_pose = create_pose(x, y, z, w);
      move_to_pose(user_pose, move_group_interface, logger);
    }
    else if (option == 5)
    {
      std::cout << "Saliendo del programa.\n";
      break;
    }
    else
    {
      std::cout << "Opción no válida. Intente nuevamente.\n";
    }
  }

  rclcpp::shutdown();
  return 0;
}

