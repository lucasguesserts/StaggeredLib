#ifndef DIRICHLET_BOUNDARY_CONDITION_HPP
#define DIRICHLET_BOUNDARY_CONDITION_HPP

#include <vector>
#include <Grid/Boundary_2.hpp>

struct DirichletBoundaryCondition: public Boundary_2
{
	std::vector<double> prescribedValue;
};

#endif