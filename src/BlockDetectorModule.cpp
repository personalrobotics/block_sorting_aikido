// #include <aikido/perception/BlockDetectorModule.hpp>

#include <stdexcept>
#include <utility>
#include <aikido/perception/shape_conversions.hpp>
#include <boost/make_shared.hpp>
#include <dart/common/Console.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <Eigen/Geometry>
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/topic.h>
#include <tabletop_perception_tools/Block.h>
#include <tabletop_perception_tools/FindBlocks.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>

#include "BlockDetectorModule.hpp"

namespace aikido {
namespace perception {

//=============================================================================
BlockDetectorModule::BlockDetectorModule(
        ros::NodeHandle node,
        std::string serviceName,
        std::string cloudTopic,
        const dart::common::ResourceRetrieverPtr resourceRetriever,
        std::string blockUri,
        std::string referenceFrameId,
        dart::dynamics::Frame* referenceLink):
    mServiceName(std::move(serviceName)),
    mCloudTopic(std::move(cloudTopic)),
    mReferenceFrameId(std::move(referenceFrameId)),
    mReferenceLink(std::move(referenceLink)),
    mResourceRetriever(std::move(resourceRetriever)),
    mBlockUri(std::move(blockUri)),
    mNode(std::move(node)),
    mListener(mNode)
{
}

//=============================================================================
std::vector<tabletop_perception_tools::Block> BlockDetectorModule::findBlocksFromService(
    const std::string& serviceName,
    const std::string& cloud_topic,
    const bool segment_planes=true,
    const int num_planes=1,
    const float plane_distance=0.015,
    const bool segment_depth=true,
    const float min_depth=0.5,
    const float max_depth=1.5,
    const float cluster_tolerance=0.005,
    const int min_cluster_size=50,
    const int max_cluster_size=300,
    const bool segment_box=true,
    std::vector<float> box_min={-0.3, -0.05, 0.6},
    std::vector<float> box_max={0.3, 0.5, 1.5})
{

  geometry_msgs::Point box_min_pt;
  box_min_pt.x = box_min[0];
  box_min_pt.y = box_min[1];
  box_min_pt.z = box_min[2];

  geometry_msgs::Point box_max_pt;
  box_max_pt.x = box_max[0];
  box_max_pt.y = box_max[1];
  box_max_pt.z = box_max[2];

  ros::ServiceClient client =
      mNode.serviceClient<tabletop_perception_tools::FindBlocks>(serviceName);
  tabletop_perception_tools::FindBlocks srv;
  srv.request.point_cloud_topic = cloud_topic;
  srv.request.segment_planes = segment_planes;
  srv.request.num_planes = num_planes;
  srv.request.plane_distance = plane_distance;
  srv.request.segment_depth = segment_depth;
  srv.request.min_depth = min_depth;
  srv.request.max_depth = max_depth;
  srv.request.cluster_tolerance = cluster_tolerance;
  srv.request.min_cluster_size = min_cluster_size;
  srv.request.max_cluster_size = max_cluster_size;
  srv.request.segment_box = segment_box;
  srv.request.box_min = box_min_pt;
  srv.request.box_max = box_max_pt;

  std::vector<tabletop_perception_tools::Block> detected_blocks;

  if(client.call(srv))
  {
    detected_blocks = srv.response.blocks;
  }
  else
  {
    ROS_ERROR("Failed to get blocks from tabletop_perception_tools service");
    std::cout << "Failed to get blocks from tabletop_perception_tools service"
        << std::endl;
  }
  return detected_blocks;
}


const dart::dynamics::SkeletonPtr makeBodyFromURDF(const std::string& uri,
    const Eigen::Isometry3d& transform)
{
  // Resolves package:// URIs by emulating the behavior of 'catkin_find'.
  const auto resourceRetriever =
      std::make_shared<aikido::io::CatkinResourceRetriever>();

  dart::utils::DartLoader urdfLoader;
  const dart::dynamics::SkeletonPtr skeleton = urdfLoader.parseSkeleton(
    uri, resourceRetriever);

  if (!skeleton)
  {
    throw std::runtime_error("unable to load '" + uri + "'");
  }

  dynamic_cast<dart::dynamics::FreeJoint*>(skeleton->getJoint(0))->setTransform(transform);

  return skeleton;
}


bool BlockDetectorModule::detectObjects(const dart::simulation::WorldPtr& env, 
                                        ros::Duration timeout, 
                                        ros::Time timestamp)
{
  int nameCounter = 0;
  ros::Time t0 = ros::Time(0);
  bool any_valid = false;

  std::vector<tabletop_perception_tools::Block> detectedBlocks =
      findBlocksFromService(mServiceName, mCloudTopic);

  if (detectedBlocks.empty())
  {
    dtwarn << "[BlockDetectorModule::detectObjects] No blocks on topic "
           << mCloudTopic << std::endl;
    return false;
  }

  for (auto const& block : detectedBlocks)
  {
    // Get the transform of the detected block
    tf::StampedTransform transform;

    const auto& detection_frame = block.header.frame_id;

    try
    {
      mListener.waitForTransform(
          mReferenceFrameId, detection_frame, t0, timeout);

      mListener.lookupTransform(
          mReferenceFrameId, detection_frame, t0, transform);
    }
    catch (const tf::ExtrapolationException& ex)
    {
      dtwarn<< "[BlockDetectorModule::detectObjects] TF timestamp is "
               "out-of-date compared to marker timestamp "
               << ex.what() << std::endl;
      continue;
    }

    Eigen::Isometry3d blockPose =
        aikido::perception::convertROSPoseToEigen(block.pose);
    Eigen::Isometry3d frame_pose = 
        aikido::perception::convertStampedTransformToEigen(transform);
    Eigen::Isometry3d skel_pose = frame_pose * blockPose;
    Eigen::Isometry3d link_offset = mReferenceLink->getWorldTransform();
    skel_pose = link_offset * skel_pose;
    
    dart::dynamics::SkeletonPtr blockSkeletonPtr =
        makeBodyFromURDF(mBlockUri, skel_pose);
    
    // Adding the new skeleton from the detected block message
    std::string skel_name = "block";
    skel_name.append(std::to_string(nameCounter++));
    std::cout << skel_name << std::endl;

    blockSkeletonPtr->setName(skel_name);

    dart::dynamics::SkeletonPtr env_skeleton = 
        env->getSkeleton(skel_name);

    if (env_skeleton != nullptr)
    {
      env->removeSkeleton(env_skeleton);
    }

    env->addSkeleton(blockSkeletonPtr);
    any_valid = true;
  }

  if (!any_valid)
  {
    dtwarn << "[BlockDetectorModule::detectObjects] No marker up-to-date with "
              "timestamp parameter" << std::endl;
    return false;
  }

  return true;
}

} // namespace perception
} // namespace aikido
