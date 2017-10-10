#include <iostream>
#include <aikido/constraint/TSR.hpp>
#include <aikido/io/CatkinResourceRetriever.hpp>
#include <aikido/perception/AprilTagsModule.hpp>
// #include <aikido/perception/BlockDetectorModule.hpp>
#include <aikido/perception/YamlAprilTagsDatabase.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <boost/program_options.hpp>
#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <Eigen/Dense>
#include <libherb/herb.hpp>
#include <tabletop_perception_tools/Block.h>
#include <pr_tsr/block.hpp>

#include "BlockDetectorModule.hpp"

namespace po = boost::program_options;

static const std::string topicName("block_detector_sample");
static const std::string herbFrameName("herb_frame");
static const std::string baseFrameName("map");

static const double detectionTimeout{5.};


const dart::dynamics::SkeletonPtr makeBodyFromURDF(
    const std::string& uri,
    const Eigen::Isometry3d& transform)
{
  // Resolves package:// URIs by emulating the behavior of 'catkin_find'.
  const auto resourceRetriever =
      std::make_shared<aikido::io::CatkinResourceRetriever>();

  dart::utils::DartLoader urdfLoader;
  const dart::dynamics::SkeletonPtr skeleton = urdfLoader.parseSkeleton(
      uri, resourceRetriever);

  if (!skeleton) {
    throw std::runtime_error("unable to load '" + uri + "'");
  }

  dynamic_cast<dart::dynamics::FreeJoint*>(skeleton->getJoint(0))->setTransform(transform);

  return skeleton;
}


int main(int argc, char** argv)
{
  // Default options for flags
  bool herbSim = false;
  bool perceptionSim = false;

  po::options_description po_desc("Block Sorting Options");
  po_desc.add_options()
    ("help", "produce help message")
    ("herbsim,h", po::bool_switch(&herbSim), "Run HERB in simulation")
    ("perceptionsim,p", po::bool_switch(&perceptionSim), "Run perception in simulation")
  ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, po_desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << po_desc << std::endl;
    return 1;
  }

  std::cout << "Starting ROS node." << std::endl;
  ros::init(argc, argv, topicName);
  ros::NodeHandle nodeHandle("~");

  // Loading HERB
  herb::Herb robot(true);
  dart::dynamics::SkeletonPtr robotSkeleton = robot.getSkeleton();

  // Start the RViz viewer.
  aikido::rviz::InteractiveMarkerViewer viewer(topicName, baseFrameName);
  std::cout << "Starting viewer. Please subscribe to the '" << topicName
            << "' InteractiveMarker topic in RViz." << std::endl;

  // Add Herb to the viewer.
  // viewer.addSkeleton(robotSkeleton);

  dart::simulation::WorldPtr env(new dart::simulation::World);

  dart::dynamics::BodyNode* herbBaseNode =
      robotSkeleton->getBodyNode(herbFrameName);

  const auto resourceRetriever =
      std::make_shared<aikido::io::CatkinResourceRetriever>();

  aikido::perception::BlockDetectorModule blockDetector(
      nodeHandle,
      "/tools_server/find_blocks",
      "/multisense/image_points2_color", // "head/kinect2/qhd/points"
      resourceRetriever,
      "package://pr_ordata/data/objects/block.urdf",
      herbFrameName,
      herbBaseNode);

  blockDetector.detectObjects(env, ros::Duration(detectionTimeout));

  viewer.setAutoUpdate(true);

  // bool running = true;

  // bool previously_found_blocks = true;
  // std::vector<dart::dynamics::SkeletonPtr> blocks;

  // while(running) {
    
  //   // Remove all blocks in the current env

  //   // Redetect the blocks on the table
  //   // blocks = block_detetor.detect_objects(env, ros::Duration(detectionTimeout));
  //   if (blocks.size() < 1){
  //     // Waiting for a bit
  //     continue;
  //   }

  //   // Robot grab the block

  //   // Robot verify the block has been grabbed

  //   // Robot place the block into the bin.
    
  // }

  std::cout << "Press <Ctrl> + C to exit." << std::endl;
  ros::spin();
  return 0;
}
