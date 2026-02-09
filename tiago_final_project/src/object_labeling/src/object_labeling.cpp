// #include <object_labeling/object_labeling.h>

// ObjectLabeling::ObjectLabeling(
//     const std::string& objects_cloud_topic_, 
//     const std::string& camera_info_topic,
//     const std::string& camera_frame) :
//   is_cloud_updated_(false),
//   has_camera_info_(false),
//   objects_cloud_topic_(objects_cloud_topic_),
//   camera_info_topic_(camera_info_topic),
//   camera_frame_(camera_frame),
//   K_(Eigen::Matrix3d::Zero())
// {
// }

// ObjectLabeling::~ObjectLabeling()
// {
// }

// bool ObjectLabeling::initalize(ros::NodeHandle& nh)
// {
//   //#>>>>TODO: subscribe to objects pointcloud published by the plane_segmentation_node
//   //#>>>>TODO: subscribe to bounding boxes from yolo (object_labeling_node)
//   //#>>>>TODO: subscribe to camera info from robot to obtain the camera matrix K

//   //#>>>>TODO: publish the labled objects as PointCloudl type (see typedefs in header)

//   // publish the LABELED object names as visulaization marker (http://wiki.ros.org/rviz/DisplayTypes/Marker)
//   text_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/text_markers", 1);

//   // init internal pointclouds for processing (again pcl uses pointers)
//   object_point_cloud_.reset(new PointCloud);    // holds unlabled object point cloud
//   labeled_point_cloud_.reset(new PointCloudl);  // holds labled object point cloud

//   //#>>>>TODO: setup a mapping from class names the ones given by yolo (see yolo bounding_boxes message for classes)
//   //#>>>>TODO: to lables / ids (number from 1 to n) used by the pointcloud
//   //#>>>>Note: We will use 0 as 'unkown' type 
//   dict_["sports ball"] = 1;
//   // ... bananna, cup, apple, ...

//   return true;
// }

// void ObjectLabeling::update(const ros::Time& time)
// {
//   // camera info and point cloud available
//   if(is_cloud_updated_ && has_camera_info_)
//   {
//     is_cloud_updated_ = false;

//     // label the objects in pointcloud based on 2d bounding boxes 
//     if(!labelObjects(object_point_cloud_, labeled_point_cloud_))
//       return;

//     //#>>>>TODO: publish labeled_point_cloud_ to ros

//     //#>>>>TODO: publish text_markers_ to ros

//   }
// }

// bool ObjectLabeling::labelObjects(CloudPtr& input, CloudPtrl& output)
// {
//   //#>>>>GOAL: Split input pointcloud into seperate blobs, compute centroid,
//   //#>>>>GOAL: project centorid into the camera image and match with bounding box
//   //#>>>>GOAL: finally label the pointcloud with object type

//   // First we need to cluster the input cloud into seperated clusters,
//   // each of them represents an object on the table.

//   //#>>>>TODO: Use EuclideanClusterExtraction to seperate the pointcloud into clusters
//   //#>>>>Hint: https://pcl.readthedocs.io/projects/tutorials/en/master/cluster_extraction.html?highlight=EuclideanClusterExtraction
  
//   // holds the extracted cluster indices (just a integer for identifiction)
//   std::vector<pcl::PointIndices> cluster_indices;

//   //#>>>>TODO: Iterate over each cluster and compute its centroid point (= mean)
//   //#>>>>TODO: Push the centroid into the vector of centroids
//   std::vector<Eigen::Vector3d> centroids;

//   // Next we need to find the pixel coordinates of the centroids within the 2d
//   // camera image. This projection is handled by the camera matrix
//   // First, the centorids need to be transformed from the pointcloud frame into the
//   // camera frame.

//   //#>>>>TODO: Get the homogenous transformation matrix of the base frame with respect
//   //#>>>>TODO: to the camera frame.
//   //#>>>>Hint: look up the transformation through the tf tree: tfListener_.lookupTransform(...)
//   //#>>>>TODO: Convert the tf::StampedTransform into an Eigen::Affine3d
//   //#>>>>Hint: tf::transformTFToEigen(...) can do the job
//   Eigen::Affine3d T_base_camera; // = ?;

//   //#>>>>TODO: Transform the centorids into the camera frame by multiplying them 
//   //#>>>>TODO: with the transformation that takes a point in the pointcloud frame and turns it
//   //#>>>>TODO: into a point in the camera frame. Do this for all centroids.
//   std::vector<Eigen::Vector3d> centroids_camera;

//   //#>>>>TODO: Project the transformed centorids into the camera plane by using the camera matrix K
//   //#>>>>Hint: Multiplying a 3d vector with the 3x3 camera matrix gives a vector in R^3
//   //#>>>>Hint: To get pixel coordinates in R^2 you need to convert them to homogenous 2d coordinates
//   //#>>>>Hint: ( = divide by the last component and drop the one in third component.)
//   std::vector<Eigen::Vector2d> pixel_centroids; // = ?

//   // Now the centorids of each cluster are given as pixel coordinates in the 2d image
//   // plane of the camera. What remains is to find the bounding box that matches to each of 
//   // those controids.

//   //#>>>>TODO: Find the best match between pixel_centroids and detections_
//   //#>>>>Hint: Use the euclidian distance between the pixel_centroids and the boundingbox centers
//   //#>>>>Hint: For each bounding box find the closest cenroid 
//   //#>>>>TODO: If a cluster cant be matched (no bounding boxes left) assign 0 as label

//   std::vector<int> assigned_labels(cluster_indices.size(), 0);                  // lables of each centroid
//   std::vector<std::string> assigned_classes(cluster_indices.size(), "unknown"); // class names of each centroid

//   for(size_t i = 0; i < detections_.size(); ++i)
//   {
//     // get the bounding box we want to find the closest cenroid 
//     const darknet_ros_msgs::BoundingBox& bounding_box = detections_[i];

//     //#>>>>TODO: For all cenroids compute the distance to the boudning box
//     //#>>>>TODO: select the clostes as match and get its index in pixel_centroids
//     int match; // = ?

//     // remember the label of match
//     if(dict_.find(bounding_box.Class) != dict_.end())
//     {
//       assigned_labels[match] = dict_[bounding_box.Class]; // set match to defined class index
//       assigned_classes[match] = bounding_box.Class;       // set match to class name
//     }
//   }

//   // relabel the point cloud
//   output->points.clear();
//   output->header = input->header;

//   PointTl pt;
//   i = 0;
//   cit = cluster_indices.begin();
//   for(; cit != cluster_indices.end(); ++cit, ++i ) 
//   {
//     // relabel all the points inside cluster
//     std::vector<int>::const_iterator it = cit->indices.begin();
//     for(; it != cit->indices.end(); ++it ) 
//     {
//       PointT& cpt = input->points[*it];
//       pt.x = cpt.x;
//       pt.y = cpt.y;
//       pt.z = cpt.z;
//       pt.label = assigned_labels[i];    // Note: To test clustering without the matching stuff just use pt.lable = i
//       output->points.push_back( pt );
//     }
//   }

//   // create a text marker that displays the assigned class name (assigned_classes) 
//   // at the 3d position of the corresponding centroid
//   text_markers_.markers.resize(assigned_classes.size());
//   for(size_t i = 0; i < assigned_classes.size(); ++i)
//   {
//     visualization_msgs::Marker marker;
//     marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//     marker.text = assigned_classes[i];
//     marker.pose.position.x = centroids.col(i).x();
//     marker.pose.position.y = centroids.col(i).y();
//     marker.pose.position.z = centroids.col(i).z() + 0.1;
//     marker.color.a = 1.0;
//     marker.scale.z = 0.1;
//     marker.id = i;
//     marker.header.frame_id = input->header.frame_id;
//     marker.header.stamp = ros::Time::now();
//     text_markers_.markers[i] = marker;
//   }

//   return true;
// }


// void ObjectLabeling::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
// {
//   // convert to pcl
//   is_cloud_updated_ = true;
//   //#>>>>TODO: convert to pcl and store in object_point_cloud_
//   //#>>>>Hint: pcl::fromROSMsg()
// }

// void ObjectLabeling::detectionCallback(const darknet_ros_msgs::BoundingBoxesConstPtr &msg)
// {
//   //#>>>>TODO: copy the YOLO bounding boxes
//   detections_; // = ?;
// }

// void ObjectLabeling::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
// {
//   // copy camera info
//   has_camera_info_ = true;
//   Eigen::Matrix3d K = Eigen::Matrix3d::Zero();

//   //#>>>>TODO: copy the 3x3 camera matrix to K_
//   //#>>>>Hint: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
//   /*for(size_t i = 0; i < 9; ++i)
//   {
//     K(i) = msg->K[i];
//   }
//   K_ = K.transpose();*/
// }
#include <object_labeling/object_labeling.h>

ObjectLabeling::ObjectLabeling(
    const std::string& objects_cloud_topic,
    const std::string& camera_info_topic,
    const std::string& camera_frame)
  : is_cloud_updated_(false)
  , has_camera_info_(false)
  , camera_frame_(camera_frame)
  , objects_cloud_topic_(objects_cloud_topic)
  , camera_info_topic_(camera_info_topic)
  , K_(Eigen::Matrix3d::Zero())
{
}

ObjectLabeling::~ObjectLabeling()
{
}

bool ObjectLabeling::initalize(ros::NodeHandle& nh)
{
  
  object_point_cloud_sub_ = nh.subscribe(
      objects_cloud_topic_,             // 例如 "/plane_segmentation/objects_cloud"
      1,
      &ObjectLabeling::cloudCallback,
      this);

  //////////////////////////////////////////////////////////////////////////////
  
  //////////////////////////////////////////////////////////////////////////////
  object_detections_sub_ = nh.subscribe(
      "/darknet_ros/bounding_boxes",    // 也可以以后改成参数
      1,
      &ObjectLabeling::detectionCallback,
      this);

  //////////////////////////////////////////////////////////////////////////////
 
  //////////////////////////////////////////////////////////////////////////////
  camera_info_sub_ = nh.subscribe(
      camera_info_topic_,               // 例如 "/xtion/rgb/camera_info"
      1,
      &ObjectLabeling::cameraInfoCallback,
      this);

  //////////////////////////////////////////////////////////////////////////////
  
  //////////////////////////////////////////////////////////////////////////////
  labeled_object_cloud_pub_ = nh.advertise<PointCloudl>(
      "/object_labeling/labeled_cloud",
      1);

  //////////////////////////////////////////////////////////////////////////////
  
  //////////////////////////////////////////////////////////////////////////////
  text_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
      "/text_markers",
      1);

  //////////////////////////////////////////////////////////////////////////////
  item_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
    "/item_markers",
    1);


  
  object_point_cloud_.reset(new PointCloud);    // 未标注的 objects cloud
  labeled_point_cloud_.reset(new PointCloudl);  // 已标注的 objects cloud

 
  dict_.clear();
  dict_["sports ball"] = 1;
  dict_["bottle"]      = 2;
  dict_["cup"]         = 3;
  dict_["apple"]       = 4;
  dict_["banana"]      = 5;

  ROS_INFO_STREAM("ObjectLabeling initialized. Subscribing to objects cloud: "
                  << objects_cloud_topic_
                  << ", camera info: " << camera_info_topic_
                  << ", camera frame: " << camera_frame_);

  is_cloud_updated_ = false;
  has_camera_info_  = false;

  return true;
}

void ObjectLabeling::update(const ros::Time& time)
{
  
  if (is_cloud_updated_ && has_camera_info_)
  {
    is_cloud_updated_ = false;

    // label the objects in pointcloud based on 2d bounding boxes 
    if (!labelObjects(object_point_cloud_, labeled_point_cloud_))
      return;

    ////////////////////////////////////////////////////////////////////////////
    
    ////////////////////////////////////////////////////////////////////////////
    labeled_object_cloud_pub_.publish(labeled_point_cloud_);

    
    ////////////////////////////////////////////////////////////////////////////
    text_marker_pub_.publish(text_markers_);
    item_marker_pub_.publish(item_markers_);
  }
}

bool ObjectLabeling::labelObjects(CloudPtr& input, CloudPtrl& output)
{
  
  if (!input || input->empty())
  {
    ROS_WARN("ObjectLabeling: input cloud is empty.");
    return false;
  }

  std::vector<pcl::PointIndices> cluster_indices;
  std::vector<Eigen::Vector3d>   centroids;         // in base frame (点云 frame)
  std::vector<Eigen::Vector3d>   centroids_cam;     // in camera frame
  std::vector<Eigen::Vector2d>   pixel_centroids;   // 2D 像素坐标

  
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(input);

  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(0.03);    // 3cm
  ec.setMinClusterSize(50);        // 最少点数
  ec.setMaxClusterSize(25000);     // 最大点数
  ec.setSearchMethod(tree);
  ec.setInputCloud(input);
  ec.extract(cluster_indices);

  if (cluster_indices.empty())
  {
    ROS_WARN("ObjectLabeling: no clusters found in objects cloud.");
    return false;
  }

  
  centroids.reserve(cluster_indices.size());
  for (const auto& indices : cluster_indices)
  {
    Eigen::Vector3d c(0.0, 0.0, 0.0);
    if (indices.indices.empty())
      continue;

    for (int idx : indices.indices)
    {
      const PointT& p = input->points[idx];
      c.x() += p.x;
      c.y() += p.y;
      c.z() += p.z;
    }
    c /= static_cast<double>(indices.indices.size());
    centroids.push_back(c);
  }

  
  Eigen::Affine3d T_cloud_camera = Eigen::Affine3d::Identity();
  try
  {
    
    std::string cloud_frame = input->header.frame_id;

    tf::StampedTransform tf_transform;
    tfListener_.lookupTransform(
        camera_frame_,      // 目标：相机坐标系
        cloud_frame,        // 源：点云坐标系（例如 base_link）
        ros::Time(0),       // 最近的 TF
        tf_transform);

    tf::transformTFToEigen(tf_transform, T_cloud_camera);
  }
  catch (tf::TransformException &ex)
  {
    ROS_WARN("ObjectLabeling: TF lookup failed: %s", ex.what());
    return false;
  }

  
  centroids_cam.reserve(centroids.size());
  for (const auto& c : centroids)
  {
    Eigen::Vector3d c_cam = T_cloud_camera * c;
    centroids_cam.push_back(c_cam);
  }

  
  pixel_centroids.reserve(centroids_cam.size());
  for (const auto& c_cam : centroids_cam)
  {
    // 相机坐标系中点为 [X, Y, Z]^T
    if (c_cam.z() <= 0.0)
    {
      pixel_centroids.emplace_back(Eigen::Vector2d(-1, -1));  // 标个非法
      continue;
    }

    Eigen::Vector3d uvw = K_ * c_cam;   // [u * Z, v * Z, Z]
    double u = uvw.x() / uvw.z();
    double v = uvw.y() / uvw.z();

    pixel_centroids.emplace_back(Eigen::Vector2d(u, v));
  }

  
  std::vector<int>         assigned_labels(cluster_indices.size(), 0);         // label
  std::vector<std::string> assigned_classes(cluster_indices.size(), "unknown"); // class name

  // 将 centroids 转为矩阵形式，方便 findMatch 使用
  Eigen::MatrixXd centroid_mat(2, pixel_centroids.size());
  for (size_t i = 0; i < pixel_centroids.size(); ++i)
  {
    centroid_mat(0, i) = pixel_centroids[i].x();
    centroid_mat(1, i) = pixel_centroids[i].y();
  }

  // 对每一个 YOLO 检测框找匹配
  for (size_t i = 0; i < detections_.size(); ++i)
  {
    const darknet_ros_msgs::BoundingBox& bbox = detections_[i];

    int match_idx = findMatch(bbox, centroid_mat);
    if (match_idx < 0 || match_idx >= static_cast<int>(cluster_indices.size()))
      continue;

    // 查字典，如果找不到，label = 0 (unknown)
    int label = 0;
    if (dict_.find(bbox.Class) != dict_.end())
      label = dict_[bbox.Class];

    assigned_labels[match_idx]  = label;
    assigned_classes[match_idx] = bbox.Class;
  }

  //==========================
  // 7) 重新打 label，生成输出点云
  //==========================
  output->points.clear();
  output->header = input->header;

  PointTl pt;
  for (size_t i = 0; i < cluster_indices.size(); ++i)
  {
    const auto& indices = cluster_indices[i];
    int label = assigned_labels[i];

    for (int idx : indices.indices)
    {
      const PointT& src = input->points[idx];
      pt.x     = src.x;
      pt.y     = src.y;
      pt.z     = src.z;
      pt.label = label;   // 如果你只是想测试聚类，可以用 pt.label = i;

      output->points.push_back(pt);
    }
  }

 
  text_markers_.markers.clear();
  text_markers_.markers.resize(centroids.size());

  for (size_t i = 0; i < centroids.size(); ++i)
  {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = assigned_classes[i];

    marker.pose.position.x = centroids[i].x();
    marker.pose.position.y = centroids[i].y();
    marker.pose.position.z = centroids[i].z() + 0.1;

    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    marker.scale.z = 0.1;
    marker.id = static_cast<int>(i);
    marker.header.frame_id = input->header.frame_id;
    marker.header.stamp    = ros::Time::now();

    text_markers_.markers[i] = marker;
  }

    //==========================
  // 9) 为每个聚类生成 CUBE 包围盒 item_markers_
  //==========================
  item_markers_.markers.clear();
  item_markers_.markers.resize(cluster_indices.size());

  for (size_t i = 0; i < cluster_indices.size(); ++i)
  {
    const auto& indices = cluster_indices[i];
    if (indices.indices.empty())
      continue;

    // 计算该聚类的点云 AABB（轴对齐包围盒）
    double min_x =  std::numeric_limits<double>::infinity();
    double min_y =  std::numeric_limits<double>::infinity();
    double min_z =  std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();

    for (int idx : indices.indices)
    {
      const PointT& p = input->points[idx];
      if (p.x < min_x) min_x = p.x;
      if (p.y < min_y) min_y = p.y;
      if (p.z < min_z) min_z = p.z;
      if (p.x > max_x) max_x = p.x;
      if (p.y > max_y) max_y = p.y;
      if (p.z > max_z) max_z = p.z;
    }

    // 中心和尺寸
    double cx = 0.5 * (min_x + max_x);
    double cy = 0.5 * (min_y + max_y);
    double cz = 0.5 * (min_z + max_z);

    double sx = std::max(0.01, max_x - min_x);
    double sy = std::max(0.01, max_y - min_y);
    double sz = std::max(0.01, max_z - min_z);

    visualization_msgs::Marker box;
    box.header.frame_id = input->header.frame_id;
    box.header.stamp    = ros::Time::now();
    box.ns              = assigned_classes[i];
    box.id              = static_cast<int>(i);

    box.type   = visualization_msgs::Marker::CUBE;
    box.action = visualization_msgs::Marker::ADD;

    box.pose.position.x = cx;
    box.pose.position.y = cy;
    box.pose.position.z = cz;

    // 不考虑旋转，直接做轴对齐 CUBE
    box.pose.orientation.x = 0.0;
    box.pose.orientation.y = 0.0;
    box.pose.orientation.z = 0.0;
    box.pose.orientation.w = 1.0;

    box.scale.x = sx;
    box.scale.y = sy;
    box.scale.z = sz;

    // 颜色：统一蓝色半透明，你可以按 label 分颜色
    box.color.a = 0.4;
    box.color.r = 0.0;
    box.color.g = 0.0;
    box.color.b = 1.0;

    item_markers_.markers[i] = box;
  }

  return true;
}

int ObjectLabeling::findMatch(
    const darknet_ros_msgs::BoundingBox& rect,
    const Eigen::MatrixXd& centroids)
{
  // 计算 bounding box 中心点
  double cx = 0.5 * (rect.xmin + rect.xmax);
  double cy = 0.5 * (rect.ymin + rect.ymax);

  int best_idx = -1;
  double best_dist2 = std::numeric_limits<double>::infinity();

  for (int i = 0; i < centroids.cols(); ++i)
  {
    double dx = centroids(0, i) - cx;
    double dy = centroids(1, i) - cy;
    double d2 = dx*dx + dy*dy;

    if (d2 < best_dist2)
    {
      best_dist2 = d2;
      best_idx   = i;
    }
  }

  return best_idx;
}

void ObjectLabeling::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  // convert to pcl
  is_cloud_updated_ = true;
  object_point_cloud_->clear();

  pcl::fromROSMsg(*msg, *object_point_cloud_);
}

void ObjectLabeling::detectionCallback(
    const darknet_ros_msgs::BoundingBoxesConstPtr &msg)
{
  // copy YOLO bounding boxes
  detections_ = msg->bounding_boxes;
}

void ObjectLabeling::cameraInfoCallback(
    const sensor_msgs::CameraInfoConstPtr &msg)
{
  has_camera_info_ = true;

 
  Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < 9; ++i)
  {
    K(i / 3, i % 3) = msg->K[i];
  }
  K_ = K;
}
