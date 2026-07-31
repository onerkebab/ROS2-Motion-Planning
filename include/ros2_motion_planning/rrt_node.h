#ifndef RRT_NODE_H
#define RRT_NODE_H

#include <memory>

namespace motion_planning {
  
  class RRTNode {
    
    public:

      RRTNode(
        const double& xArg = 0.0, 
        const double& yArg = 0.0, 
        const double& thetaArg = 0.0, 
        const double& fArg = 0.0, 
        const std::shared_ptr<RRTNode>& bpArg = nullptr
      );
      RRTNode(const RRTNode& other) = default;
      virtual ~RRTNode() = default;
      RRTNode& operator=(const RRTNode& other) = default;

      // Position
      double x;
      double y;
      double theta;

      // Cost
      double f;

      // Back-pointer
      std::shared_ptr<RRTNode> bp;
  
  };

}

#endif /* RRT_NODE_H */
