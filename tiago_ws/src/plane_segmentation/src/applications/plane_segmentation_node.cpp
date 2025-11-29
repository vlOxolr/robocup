#include <plane_segmentation/plane_segmentation.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plane_segmentation");
  ros::NodeHandle nh;

  // Set TIAGo point cloud topic (registered depth from xtion)
  std::string pointcloud_topic_name = "/xtion/depth_registered/points";

  // Set robot base frame (frame with ground height at 0)
  // !!! IMPORTANT !!!
  // If TF is unavailable, replace "base_link" with "base_footprint"
  std::string base_frame_name = "base_link";

  // Construct segmentation object
  PlaneSegmentation segmentation(
      pointcloud_topic_name,
      base_frame_name);

  // Initialize
  if(!segmentation.initalize(nh))
  {
    ROS_ERROR_STREAM("Error init PlaneSegmentation");
    return -1;
  }

  ROS_INFO_STREAM("PlaneSegmentation started. Subscribing to: "
                  << pointcloud_topic_name
                  << " with base frame: " << base_frame_name);

  // Update loop
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
