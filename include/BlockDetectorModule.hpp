#ifndef AIKIDO_PERCEPTION_BLOCKDETECTORMODULE_H
#define AIKIDO_PERCEPTION_BLOCKDETECTORMODULE_H

#include <memory>
#include <string>
#include <aikido/io/CatkinResourceRetriever.hpp>
#include <aikido/io/yaml.hpp>
#include <aikido/perception/PerceptionModule.hpp>
#include <dart/dart.hpp>
#include <ros/forwards.h>
#include <ros/ros.h>
#include <ros/single_subscriber_publisher.h>
#include <tf/transform_listener.h>

namespace aikido{
namespace perception{

/// Instantiates the \c PerceptionModule specifically for BlockDetector.
///
/// TODO: class description
class BlockDetectorModule : public PerceptionModule
{
public:
  /// Construct an AprilTags receiver that subscribes to the specified topic
  /// where AprilTag information is being published as a
  /// \c visualization_msg::MarkerArray, uses a database loader for
  /// configuration information related to tags and obtains the desired
  /// transformation frame for the object pose.
  ///
  /// \param[in] node The node handle to be passed to the detector
  /// \param[in] markerTopic The name of the topic on which april tags
  /// information is being published
  /// \param[in] configData The pointer to some configuration data loader
  /// \param[in] resourceRetriever A DART retriever for resources related to
  /// config files and models and so on
  /// \param[in] destinationFrame The desired TF for the detections
  /// \param[in] referenceLink A link on the robot with respect to which the
  /// pose is transformed
  BlockDetectorModule(
      ros::NodeHandle node, 
      std::string serviceName, 
      std::string cloudTopic,
      dart::common::ResourceRetrieverPtr resourceRetriever,
      std::string blockUri,
      std::string referenceFrameId, 
      dart::dynamics::Frame* referenceLink);

  virtual ~BlockDetectorModule() = default;

  // Documentation inherited
  bool detectObjects(
      const dart::simulation::WorldPtr& env,
      ros::Duration timeout = ros::Duration(0.0),
      ros::Time timestamp = ros::Time(0.0)) override;

private:
  std::string mServiceName;

  ///The name of the ROS topic to read marker info from
  std::string mCloudTopic;  

  ///The desired reference frame for the object pose  
  std::string mReferenceFrameId;

  ///The reference frame of HERB to transform with respect to
  dart::dynamics::Frame* mReferenceLink;

  ///To retrieve resources from disk and from packages
  dart::common::ResourceRetrieverPtr mResourceRetriever;
  
  std::string mBlockUri;

  ///For the ROS node that will work with the April Tags module
  ros::NodeHandle mNode;

  ///Listens to the transform attached to the node
  tf::TransformListener mListener;

  std::vector<tabletop_perception_tools::Block> findBlocksFromService(
      const std::string& serviceName,
      const std::string& cloud_topic,
      const bool segment_planes,
      const int num_planes,
      const float plane_distance,
      const bool segment_depth,
      const float min_depth,
      const float max_depth,
      const float cluster_tolerance,
      const int min_cluster_size,
      const int max_cluster_size,
      const bool segment_box,
      std::vector<float> box_min,
      std::vector<float> box_max);

};

} //namespace perception
} //namespace aikido

#endif //AIKIDO_PERCEPTION_BLOCKDETECTORMODULE_H
