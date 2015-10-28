#include "/home/ssb/libsvm/svm.h"
#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"

static const double SIZE = 1000.0;
static const double OFFSET_Z = 500.0;

svm_model* model_x;
svm_model* model_y;
svm_model* model_z;

ros::Publisher pub;

struct points
{
  double x;
  double y;
  double z;
  double neck_p;
  double neck_y;
};

void Callback(const std_msgs::Float32MultiArray::ConstPtr& _msg)
{
  points data;

  data.x = _msg->data[0] / SIZE;
  data.y = _msg->data[1] / SIZE;
  data.z = (_msg->data[2] - OFFSET_Z) / SIZE;
  data.neck_p = _msg->data[3] / 90.0;
  data.neck_y = _msg->data[4] / 90.0;

  svm_node testdat[5];

  testdat[0].index = 1;
  testdat[0].value = data.x;
  testdat[1].index = 2;
  testdat[1].value = data.y;
  testdat[2].index = 3;
  testdat[2].value = data.z;
  testdat[3].index = 4;
  testdat[3].value = data.neck_y;
  testdat[4].index = -1;

  std::vector<float> res(3);
  res[0]  = svm_predict(model_x, testdat);
  res[1]  = svm_predict(model_y, testdat);
  res[2]  = svm_predict(model_z, testdat);

  std_msgs::Float32MultiArray msg;
  msg.data = res;

  pub.publish(msg);
};

int main(int argc, char **argv)
{
  model_x = svm_load_model("/home/ssb/larm_learned_x");
  model_y = svm_load_model("/home/ssb/larm_learned_y");
  model_z = svm_load_model("/home/ssb/larm_learned_z");

  ros::init(argc, argv, "ik_learned_offset");
  ros::NodeHandle nh;

  pub = nh.advertise<std_msgs::Float32MultiArray>("/aero_ik_res", 100);
  ros::Subscriber sub = nh.subscribe("/aero_ik", 1000, Callback);

  ros::spin();

  return 0;
}
