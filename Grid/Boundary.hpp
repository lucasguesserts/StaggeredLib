#ifndef BOUNDARY_HPP
#define BOUNDARY_HPP

#include <vector>
#include <GeometricEntity/StaggeredTriangle.hpp>

struct Boundary
{
	std::vector<StaggeredTriangle*> staggeredTriangle;
};

#endif