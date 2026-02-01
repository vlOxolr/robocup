#include <grasp/grasp.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grasp_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    // initialize the gripper publisher
    Grasper grasper(nh);

    grasper.table_cloud_sub_once_ = nh.subscribe("/plane_segmentation/table_cloud", 1, &Grasper::tableCloudCallback, &grasper);

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/object_labeling/labeled_cloud", 1, &Grasper::pointCloudCallback, &grasper);

    ROS_INFO("Grasp node is running...");

    ros::waitForShutdown();
    return 0;
}