#ifndef BOUNDARY_2_HPP
#define BOUNDARY_2_HPP

#include <vector>
#include <GeometricEntity/StaggeredElement2D.hpp>

struct Boundary
{
	std::vector<StaggeredElement2D*> staggeredTriangle;
};

#endif