#include "ros2_motion_planning/rrt_node.h"

namespace motion_planning {

  RRTNode::RRTNode(
    const double& xArg, 
    const double& yArg, 
    const double& thetaArg, 
    const double& fArg, 
    const std::shared_ptr<RRTNode>& bpArg
  ) : x(xArg), y(yArg), theta(thetaArg), f(fArg), bp(bpArg) {

  }

}
