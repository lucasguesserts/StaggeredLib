#ifndef BOUNDARY_2_HPP
#define BOUNDARY_2_HPP

#include <vector>
#include <GeometricEntity/StaggeredElement2D.hpp>

struct Boundary_2
{
	std::vector<StaggeredElement2D*> staggeredTriangle;
};

#endif