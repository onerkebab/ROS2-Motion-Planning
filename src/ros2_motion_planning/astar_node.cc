#include "ros2_motion_planning/astar_node.h"

namespace motion_planning {

  AStarNode::AStarNode(const int& xArg, const int& yArg, const double& thetaArg,
      const double& fArg, const double& gArg, const double& hArg,
      const std::shared_ptr<AStarNode>& bpArg) : x(xArg), y(yArg), theta(thetaArg),
      f(fArg), g(gArg), h(hArg), bp(bpArg) {

      }

}
