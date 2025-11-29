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
      pointcloud_topic_,           // 构造函数中传进来的点云话题
      1,
      &PlaneSegmentation::cloudCallback,
      this);

  // >>> TODO: advertise the pointcloud for the table plane
  plane_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(
      "plane_segmentation/table_cloud",  // 桌子平面的点云
      1);

  // >>> TODO: advertise the pointcloud for the remaining points (objects)
  objects_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(
      "plane_segmentation/objects_cloud", // 桌子上的物体点云
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

  // 1) VoxelGrid 下采样
  CloudPtr ds_cloud(new PointCloud);  // downsampled pointcloud

  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(input);
  vg.setLeafSize(0.01f, 0.01f, 0.01f);   // 1cm 体素
  vg.filter(*ds_cloud);

  // 2) transform 到 base_frame
  CloudPtr transf_cloud(new PointCloud);  // expressed in base frame

  try
  {
    // ⭐⭐ 关键修改：使用 Time(0) + 指定源坐标系 ⭐⭐
    pcl_ros::transformPointCloud(
        base_frame_,                 // 目标坐标系（base_link 或 base_footprint）
        ros::Time(0),                // 使用 TF buffer 中最新的变换
        *ds_cloud,
        ds_cloud->header.frame_id,   // 点云原来的 frame，一般是 xtion_rgb_optical_frame
        *transf_cloud,
        tfListener_);
  }
  catch (tf::TransformException &ex)
  {
    ROS_WARN("TF transform failed in preProcessCloud: %s", ex.what());
    return false;
  }

  // 3) PassThrough 去掉地面以下的点
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(transf_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.01, 1.5);   // 保留 1cm~1.5m 之间的点
  pass.filter(*output);

  if (output->empty())
  {
    ROS_WARN("preProcessCloud: output cloud is empty after PassThrough");
    return false;
  }

  return true;
}


bool PlaneSegmentation::segmentCloud(CloudPtr& input,
                                     CloudPtr& plane_cloud,
                                     CloudPtr& objects_cloud)
{
  if (!input || input->empty())
    return false;

  // ---------- 1) 用 RANSAC 拟合平面 ----------
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr        inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr   coefficients(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);   // 平面模型
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.01);          // 1 cm 内视为在平面上
  seg.setProbability(0.99);

  seg.setInputCloud(input);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty())
  {
    ROS_WARN("PlaneSegmentation: No plane found by RANSAC.");
    return false;
  }

  // ---------- 2) 提取平面 inliers（桌面） ----------
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(input);
  extract.setIndices(inliers);

  extract.setNegative(false);   // 只要 inliers
  extract.filter(*plane_cloud); // 写入桌面点云

  // ---------- 3) 利用平面方程，按“高度”挑选物体 ----------
  objects_cloud->clear();

  if (coefficients->values.size() < 4)
  {
    ROS_WARN("PlaneSegmentation: invalid plane coefficients, skip object extraction.");
    return true;  // 桌面已经有了，物体就空着
  }

  // 平面系数: a x + b y + c z + d = 0
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

  // 归一化后的法向量，用来计算“到平面的垂直距离”
  const float inv_norm = 1.0f / n_norm;

  // 高度阈值，可按需要微调:
  const float table_thickness = 0.01f;  // 平面厚度 1 cm
  const float obj_min_height  = 0.02f;  // 至少高出桌面 2 cm
  const float obj_max_height  = 0.25f;  // 不超过 25 cm，太高可能是背景

  // 遍历所有点，根据“离平面的高度”挑出物体
  for (std::size_t i = 0; i < input->points.size(); ++i)
  {
    const PointT& p = input->points[i];

    // 有些无效点（NaN）直接跳过
    if (!pcl::isFinite(p))
      continue;

    // 到平面的有符号距离（沿法线方向）
    float dist = (a * p.x + b * p.y + c * p.z + d) * inv_norm;

    // 绝对值很小的 -> 桌面本身（已经在 plane_cloud 里，不用管）
    if (std::fabs(dist) <= table_thickness)
      continue;

    // 高于桌面的，并且在一个合理高度范围内 -> 认为是物体
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
