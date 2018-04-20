#ifndef DIRICHLET_BOUNDARY_CONDITION_HPP
#define DIRICHLET_BOUNDARY_CONDITION_HPP

#include <Grid/Boundary.hpp>

struct DirichletBoundaryCondition: public Boundary
{
	double prescribedValue;
};

#endif