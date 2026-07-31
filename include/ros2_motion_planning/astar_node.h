#ifndef ASTAR_NODE_H
#define ASTAR_NODE_H

#include <memory>

namespace motion_planning {
  
  class AStarNode {
    
    public:

      AStarNode(
        const int& xArg = 0, 
        const int& yArg = 0, 
        const double& thetaArg = 0.0, 
        const double& fArg = 0.0, 
        const double& gArg = 0.0, 
        const double& hArg = 0.0,
        const std::shared_ptr<AStarNode>& bpArg = nullptr
      );
      AStarNode(const AStarNode& other) = default;
      virtual ~AStarNode() = default;
      AStarNode& operator=(const AStarNode& other) = default;

      // Position
      int x;
      int y;
      double theta;

      // Heuristics
      double f;
      double g;
      double h;

      // Back-pointer
      std::shared_ptr<AStarNode> bp;
  
  };

}

#endif /* ASTAR_NODE_H */
