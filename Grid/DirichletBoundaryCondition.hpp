#ifndef DIRICHLET_BOUNDARY_CONDITION_HPP
#define DIRICHLET_BOUNDARY_CONDITION_HPP

#include <vector>
#include <Grid/Boundary.hpp>

struct DirichletBoundaryCondition: public Boundary
{
	std::vector<double> prescribedValue;
};

#endif