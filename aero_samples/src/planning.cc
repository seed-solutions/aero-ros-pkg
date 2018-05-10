///
#include <aero_std/AeroMoveitInterface.hh>

///
#include <aero_std/AeroMoveGroupInterface.hh>

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "planning_sample_node");
  ros::NodeHandle nh, planner_nh;

  // init robot interface
  aero::interface::AeroMoveitInterface::Ptr robot(new aero::interface::AeroMoveitInterface(nh));
  ROS_INFO("reseting robot pose");
  robot->setPoseVariables(aero::pose::reset_manip);
  robot->sendModelAngles(3000);

  ROS_INFO("create move_group_interface");
  aero::interface::AeroMoveGroupInterface mgi("larm_with_waist");

  ROS_INFO_NAMED("tutorial", "Reference frame: %s", mgi.move_group->getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "Planner: %s", mgi.move_group->getDefaultPlannerId().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s", mgi.move_group->getEndEffectorLink().c_str());

  mgi.move_group->setPlannerId("BKPIECEkConfigDefault");
  mgi.move_group->setPlanningTime(10.0);

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = std::sqrt(0.5);
  target_pose1.orientation.y = - std::sqrt(0.5);
  target_pose1.position.x = 0.305;
  target_pose1.position.y = 0.375;
  target_pose1.position.z = 1.205;

  aero::Transform target_pose1_trans;
  tf::poseMsgToEigen(target_pose1, target_pose1_trans);
#if 0
  mgi.move_group->setPoseTarget(target_pose1);
#else
  mgi.move_group->setPoseTarget(target_pose1_trans);
#endif

  robot->waitInterpolation();
  {
    ROS_INFO("plan");
    aero::MoveGroupClass::Plan my_plan;
    bool success = (mgi.move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
      ROS_INFO("success plan");
      mgi.move_group->execute(my_plan);
    } else {
      ROS_INFO("failed plan");
    }
  }

  ROS_INFO("add object");
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = mgi.move_group->getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 0.4;
  primitive.dimensions[2] = 0.8;

  //Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  aero::Transform box_trans = aero::Translation(0.45, 0.0, 1.0) * aero::Quaternion::Identity();
  //box_pose.orientation.w = 1.0;
  //box_pose.position.x = 0.45;
  //box_pose.position.y = 0.0;
  //box_pose.position.z = 1.0;
  tf::poseEigenToMsg(box_trans, box_pose);

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  mgi.scene->addCollisionObjects(collision_objects);

  ros::Duration(1).sleep();

  geometry_msgs::Pose target_pose2;
  target_pose2.orientation.x = 0.5;
  target_pose2.orientation.y = 0.5;
  target_pose2.orientation.z = 0.5;
  target_pose2.orientation.w = -0.5;
  target_pose2.position.x = 0.631;
  target_pose2.position.y = 0.173;
  target_pose2.position.z = 1.16;
  aero::Transform target_pose2_trans;
  tf::poseMsgToEigen(target_pose2, target_pose2_trans);

  geometry_msgs::Pose target_pose3;
  target_pose3.orientation.x = 0;
  target_pose3.orientation.y = 0;
  target_pose3.orientation.z = - std::sqrt(0.5);
  target_pose3.orientation.w = std::sqrt(0.5);
  target_pose3.position.x = 0.601;
  target_pose3.position.y = 0.093;
  target_pose3.position.z = 1.16;
  aero::Transform target_pose3_trans;
  tf::poseMsgToEigen(target_pose3, target_pose3_trans);

  for(int i = 0; i < 10; i++) {
    //mgi.move_group->setPoseTarget(target_pose2);
    mgi.move_group->clearPoseTargets();
    mgi.move_group->setPoseTarget(target_pose3_trans, "l_eef_grasp_link");
    {
      ROS_INFO("%d: plan2", i);
      aero::MoveGroupClass::Plan my_plan;
      bool success = (mgi.move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if (success) {
        ROS_INFO("%d: success plan2", i);
        mgi.move_group->execute(my_plan);
      } else {
        ROS_INFO("%d: failed plan2", i);
      }
    }

    std::map<std::string, double> jmap;
    robot->setPoseVariables(aero::pose::reset_manip);
    robot->getRobotStateVariables(jmap);
    mgi.move_group->setJointValueTarget(jmap);
    {
      ROS_INFO("%d: plan2 revert", i);
      aero::MoveGroupClass::Plan my_plan;
      bool success = (mgi.move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if (success) {
        ROS_INFO("%d: success plan2 revert", i);
        mgi.move_group->execute(my_plan);
      } else {
        ROS_INFO("%d: failed plan2 revert", i);
        robot->sendModelAngles(3000);
        robot->waitInterpolation();
      }
    }
  }
  //ros::Duration(10).sleep();
}
