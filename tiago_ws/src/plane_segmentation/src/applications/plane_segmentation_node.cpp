#include <plane_segmentation/plane_segmentation.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plane_segmentation");
  ros::NodeHandle nh;

  // 设置 TIAGo 的点云话题（来自 xtion 的注册深度点云）
  std::string pointcloud_topic_name = "/xtion/depth_registered/points";

  // 设置机器人底座坐标系（地面高度为 0 的 frame）
  // !!! IMPORTANT !!!
  // 如果 TF 不通，把 "base_link" 换成 "base_footprint"
  std::string base_frame_name = "base_link";

  // 构建分割对象
  PlaneSegmentation segmentation(
      pointcloud_topic_name,
      base_frame_name);

  // 初始化
  if(!segmentation.initalize(nh))
  {
    ROS_ERROR_STREAM("Error init PlaneSegmentation");
    return -1;
  }

  ROS_INFO_STREAM("PlaneSegmentation started. Subscribing to: "
                  << pointcloud_topic_name
                  << " with base frame: " << base_frame_name);

  // 更新循环
  ros::Rate rate(30);
  while(ros::ok())
  {
    segmentation.update(ros::Time::now());
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}



// #include <plane_segmentation/plane_segmentation.h>

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "plane_segmentation");
//   ros::NodeHandle nh;
  
//   //#>>>>TODO: Set the correct topic name of the robot
//   std::string pointcloud_topic_name = "";

//   //#>>>>TODO: Set the name of a frame on the floor/ground of the robot (height=0)
//   std::string base_fame_name = "";

//   // construct the object
//   PlaneSegmentation segmentation(
//     pointcloud_topic_name, 
//     base_fame_name);
  
//   // initialize the object
//   if(!segmentation.initalize(nh))
//   {
//     ROS_ERROR_STREAM("Error init PlaneSegmentation");
//     return -1;
//   }

//   // update the processing
//   ros::Rate rate(30);
//   while(ros::ok())
//   {
//     segmentation.update(ros::Time::now());
//     ros::spinOnce();
//     rate.sleep();
//   }

//   return 0;
// }
