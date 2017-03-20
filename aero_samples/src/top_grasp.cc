#include <aero_std/AeroMoveitInterface.hh>
#include <aero_std/TopGrasp-inl.hh>

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "top_grasp_sample_node");
  ros::NodeHandle nh;
  
  // init robot interface
  aero::interface::AeroMoveitInterfacePtr interface(new aero::interface::AeroMoveitInterface(nh));
  ROS_INFO("reseting robot pose");
  interface->sendResetManipPose();
  sleep(1);


  // grasp object from top
  Eigen::Vector3d obj;// target object's position
  obj << 0.5, 0.2, 0.7;

  aero::TopGrasp top;// grasping information
  top.arm = aero::arm::either;
  top.object_position = obj;
  top.height = 0.2;


  auto req = aero::Grasp<aero::TopGrasp>(top);
  req.mid_ik_range = aero::ikrange::torso;
  req.end_ik_range = aero::ikrange::torso;

  if (interface->sendGraspIK(req)) {
    ROS_INFO("success");
    sleep(1);
    interface->sendGrasp(req.arm);
    sleep(3);
    ROS_INFO("reseting robot pose");
    interface->sendResetManipPose();
  }
  else ROS_INFO("failed");

  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
