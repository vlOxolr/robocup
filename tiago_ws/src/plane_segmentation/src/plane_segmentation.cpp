#include <plane_segmentation/plane_segmentation.h>

PlaneSegmentation::PlaneSegmentation(
    const std::string& pointcloud_topic,
    const std::string& base_frame)
  : pointcloud_topic_(pointcloud_topic)
  , base_frame_(base_frame)
  , is_cloud_updated_(false)
{
}

PlaneSegmentation::~PlaneSegmentation()
{
}

bool PlaneSegmentation::initalize(ros::NodeHandle& nh)
{
  // >>> TODO: subscribe to the pointcloud_topic_ and link it to the right callback
  point_cloud_sub_ = nh.subscribe(
      pointcloud_topic_,           // Point cloud topic passed in via constructor
      1,
      &PlaneSegmentation::cloudCallback,
      this);

  // >>> TODO: advertise the pointcloud for the table plane
  plane_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(
      "plane_segmentation/table_cloud",  // Point cloud of the table plane
      1);

  // >>> TODO: advertise the pointcloud for the remaining points (objects)
  objects_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(
      "plane_segmentation/objects_cloud", // Point cloud of objects on the table
      1);

  // Most PCL functions accept pointers as their arguments, as such we first set
  // initialize these pointers, otherwise we will run into segmentation faults...
  raw_cloud_.reset(new PointCloud);
  preprocessed_cloud_.reset(new PointCloud);
  plane_cloud_.reset(new PointCloud);
  objects_cloud_.reset(new PointCloud);

  return true;
}

void PlaneSegmentation::update(const ros::Time& time)
{
  // update as soon as new pointcloud is available
  if (is_cloud_updated_)
  {
    is_cloud_updated_ = false;

    // >>> Note: To check preProcessCloud() you can publish its output for testing
    // apply all preprocessing steps
    if (!preProcessCloud(raw_cloud_, preprocessed_cloud_))
      return;

    // segment cloud into table and objects
    if (!segmentCloud(preprocessed_cloud_, plane_cloud_, objects_cloud_))
      return;

    // >>> TODO: publish both pointclouds obtained by segmentCloud()
    sensor_msgs::PointCloud2 plane_msg;
    sensor_msgs::PointCloud2 objects_msg;

    pcl::toROSMsg(*plane_cloud_, plane_msg);
    pcl::toROSMsg(*objects_cloud_, objects_msg);

    plane_msg.header.frame_id   = base_frame_;
    plane_msg.header.stamp      = time;

    objects_msg.header.frame_id = base_frame_;
    objects_msg.header.stamp    = time;

    plane_cloud_pub_.publish(plane_msg);
    objects_cloud_pub_.publish(objects_msg);
  }
}

bool PlaneSegmentation::preProcessCloud(CloudPtr& input, CloudPtr& output)
{
  // >>> Goal: Subsample and Filter the pointcloud
  if (!input || input->empty())
    return false;

  // 1) VoxelGrid downsampling
  CloudPtr ds_cloud(new PointCloud);  // downsampled pointcloud

  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(input);
  vg.setLeafSize(0.01f, 0.01f, 0.01f);   // 1 cm voxels
  vg.filter(*ds_cloud);

  // 2) transform to base_frame
  CloudPtr transf_cloud(new PointCloud);  // expressed in base frame

  try
  {
    // Use Time(0) with an explicit source frame
    pcl_ros::transformPointCloud(
        base_frame_,                 // Target frame (base_link or base_footprint)
        ros::Time(0),                // Use the latest transform from the TF buffer
        *ds_cloud,
        ds_cloud->header.frame_id,   // Original point cloud frame, usually xtion_rgb_optical_frame
        *transf_cloud,
        tfListener_);
  }
  catch (tf::TransformException &ex)
  {
    ROS_WARN("TF transform failed in preProcessCloud: %s", ex.what());
    return false;
  }

  // 3) PassThrough to remove points below the ground
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(transf_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.01, 1.5);   // Keep points between 1 cm and 1.5 m
  pass.filter(*output);

  if (output->empty())
  {
    ROS_WARN("preProcessCloud: output cloud is empty after PassThrough");
    return false;
  }

  // 4) Filter points by distance from robot (1.0m range)
  CloudPtr distance_filtered(new PointCloud);
  const float max_distance = 1.0f;  // Maximum distance from robot in meters
  const float max_distance_sq = max_distance * max_distance;

  for (const auto& point : output->points)
  {
    // Calculate xy distance from robot origin
    float xy_distance_sq = point.x * point.x + point.y * point.y;
    
    if (xy_distance_sq <= max_distance_sq)
    {
      distance_filtered->points.push_back(point);
    }
  }

  distance_filtered->width = static_cast<uint32_t>(distance_filtered->points.size());
  distance_filtered->height = 1;
  distance_filtered->is_dense = false;
  distance_filtered->header = output->header;

  if (distance_filtered->empty())
  {
    ROS_WARN("preProcessCloud: output cloud is empty after distance filtering");
    return false;
  }

  *output = *distance_filtered;

  return true;
}


bool PlaneSegmentation::segmentCloud(CloudPtr& input,
                                     CloudPtr& plane_cloud,
                                     CloudPtr& objects_cloud)
{
  if (!input || input->empty())
    return false;

  // ---------- 1) Fit plane with RANSAC ----------
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr        inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr   coefficients(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);   // Plane model
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.01);          // Points within 1 cm are treated as on the plane
  seg.setProbability(0.99);

  seg.setInputCloud(input);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty())
  {
    ROS_WARN("PlaneSegmentation: No plane found by RANSAC.");
    return false;
  }

  // ---------- 2) Extract plane inliers (table surface) ----------
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(input);
  extract.setIndices(inliers);

  extract.setNegative(false);   // Keep only inliers
  extract.filter(*plane_cloud); // Write table point cloud

  // ---------- 3) Use plane equation to select objects by height ----------
  objects_cloud->clear();

  if (coefficients->values.size() < 4)
  {
    ROS_WARN("PlaneSegmentation: invalid plane coefficients, skip object extraction.");
    return true;  // Table is already filled in; leave objects empty
  }

  // Plane coefficients: a x + b y + c z + d = 0
  const float a = coefficients->values[0];
  const float b = coefficients->values[1];
  const float c = coefficients->values[2];
  const float d = coefficients->values[3];

  Eigen::Vector3f n(a, b, c);
  float n_norm = n.norm();
  if (n_norm < 1e-6f)
  {
    ROS_WARN("PlaneSegmentation: plane normal too small.");
    return true;
  }

  // Normalized normal vector used to compute signed distance to the plane
  const float inv_norm = 1.0f / n_norm;

  // Height thresholds; tweak as needed:
  const float table_thickness = 0.01f;  // Plane thickness 1 cm
  const float obj_min_height  = 0.02f;  // At least 2 cm above the table
  const float obj_max_height  = 0.25f;  // No more than 25 cm; taller points may be background

  // Iterate all points and pick objects based on height above plane
  for (std::size_t i = 0; i < input->points.size(); ++i)
  {
    const PointT& p = input->points[i];

    // Skip invalid points (NaN)
    if (!pcl::isFinite(p))
      continue;

    // Signed distance to plane along the normal
    float dist = (a * p.x + b * p.y + c * p.z + d) * inv_norm;

    // Very small absolute value -> table surface (already in plane_cloud, ignore)
    if (std::fabs(dist) <= table_thickness)
      continue;

    // Above the table within a reasonable range -> treat as object
    if (dist > obj_min_height && dist < obj_max_height)
    {
      objects_cloud->points.push_back(p);
    }
  }

  objects_cloud->width  = static_cast<uint32_t>(objects_cloud->points.size());
  objects_cloud->height = 1;
  objects_cloud->is_dense = false;

  ROS_INFO_STREAM("segmentCloud: plane points = " << plane_cloud->size()
                  << ", object points = " << objects_cloud->size());

  return true;
}

void PlaneSegmentation::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  // convert ros msg to pcl raw_cloud
  is_cloud_updated_ = true;

  // >>> TODO: Convert the msg to the internal variable raw_cloud_ that holds
  // >>> the raw input pointcloud
  // >>> Hint: pcl::fromROSMsg() can do the job
  pcl::fromROSMsg(*msg, *raw_cloud_);
}
