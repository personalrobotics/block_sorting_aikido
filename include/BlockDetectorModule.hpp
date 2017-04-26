#ifndef AIKIDO_PERCEPTION_APRILTAGSMODULE_H
#define AIKIDO_PERCEPTION_APRILTAGSMODULE_H

#include <string>
#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include <tf/transform_listener.h>
#include <dart/dart.hpp>
#include <aikido/util/CatkinResourceRetriever.hpp>
#include <aikido/util/yaml.hpp>
#include <aikido/perception/PerceptionModule.hpp>
#include <memory>



namespace aikido{
namespace perception{

class BlockDetectorModule : public PerceptionModule
{
public:
	
	BlockDetectorModule(ros::NodeHandle node, std::string serviceName, std::string cloudTopic,
					dart::common::ResourceRetrieverPtr resourceRetriever,
					std::string blockUri,
					std::string referenceFrameId, dart::dynamics::Frame* referenceLink);

	virtual ~BlockDetectorModule() = default;

	// Documentation inherited
  std::vector<dart::dynamics::SkeletonPtr> detectObjects(
  					   const dart::simulation::WorldPtr& env,
					   ros::Duration timeout = ros::Duration(0.0), 
					   ros::Time timestamp=ros::Time(0.0)) override; 

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

	std::vector<tabletop_perception_tools::Block> BlockDetectorModule::findBlocksFromService(
																		const string& serviceName,
																		const string& cloud_topic,
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
																		vector<float> box_min,
																		vector<float> box_max)

};

} //namespace perception
} //namespace aikido

#endif //AIKIDO_PERCEPTION_APRILTAGSMODULE_H
