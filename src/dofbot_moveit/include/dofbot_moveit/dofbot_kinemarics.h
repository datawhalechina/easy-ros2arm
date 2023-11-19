#ifndef ARM_MOVEIT_ARM_KINEMARICS_H
#define ARM_MOVEIT_ARM_KINEMARICS_H

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <iostream>

class Dofbot{
public:
    
    bool dofbot_getFK(const char *urdf_file, std::vector<double>& joints, std::vector<double>& currentPos);

    bool dofbot_getIK(const char* urdf_file, std::vector<double>& targetXYZ, std::vector<double>& targetRPY, std::vector<double>& outjoints);

}

#endif