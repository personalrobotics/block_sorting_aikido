#include <Eigen/Dense>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/constraint/TSR.hpp>
#include <dart/dart.hpp>
#include <iostream>
#include <libherb/herb.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/util/CatkinResourceRetriever.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <tclap/CmdLine.h>
#include <aikido/perception/AprilTagsModule.hpp>
#include <aikido/perception/YamlAprilTagsDatabase.hpp>

static const double detectionTimeout{5.};

int main(int argc, char** argv)
{

	// Loading HERB
	herb::Herb robot;
	dart::dynamics::SkeletonPtr robotSkeleton = robot.getSkeleton();

	// Loading Env
	dart::simulation::WorldPtr env(new dart::simulation::World);

	// Getting the active manipulator

	/* Initial Perception (Only real perception is allowed)*/
		// Getting the table from AprilTag

	using aikido::perception::YamlAprilTagsDatabase;

	ros::NodeHandle nh("~");
	dart::dynamics::BodyNode* herbBaseNode =
		robotSkeleton->getBodyNode("herb_frame");

	const auto resourceRetriever =
        std::make_shared<aikido::util::CatkinResourceRetriever>();
    const std::string configDataURI(
        "package://pr_ordata/data/objects/aikido_tag_data.json");
    std::shared_ptr<YamlAprilTagsDatabase> yamlLoader(
        new YamlAprilTagsDatabase(resourceRetriever, configDataURI));
    aikido::perception::AprilTagsModule atDetector(
        nh,
        "/apriltags/marker_array",
        yamlLoader,
        resourceRetriever,
        "herb_frame",
        herbBaseNode);

    atDetector.detectObjects(env, ros::Duration(detectionTimeout));
    dart::dynamics::SkeletonPtr table = env->getSkeleton("table127");

    if (table == nullptr){
    	dtwarn << "[BlockSorting] Cannot find table  in the scene." << std::endl;
 		return 1;
    }

    	// Snap the table onto the ground

    	// Find the blocks and the bins

    /* Main loop and planning pipeline */

    bool running = true;

    bool previously_found_blocks = true;
    std::vector<dart::dynamics::SkeletonPtr> blocks;

    while(running) {
    	
    	// Remove all blocks in the current env

    	// Redetect the blocks on the table
    	// blocks = block_detetor.detect_objects(env, ros::Duration(detectionTimeout));
    	if (blocks.size() < 1){
    		// Waiting for a bit
    		continue;
    	}

    	// Robot grab the block

    	// Robot verify the block has been grabbed

    	// Robot place the block into the bin.
    	
    }
	return 0;
}