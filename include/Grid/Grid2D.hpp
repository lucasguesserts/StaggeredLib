#ifndef GRID2D_HPP
#define GRID2D_HPP

#include <vector>
#include <GeometricEntity/Quadrangle.hpp>
#include <GeometricEntity/Triangle.hpp>
#include <Grid/Grid.hpp>

struct Grid2D: public Grid
{
	std::vector<Quadrangle> quadrangles;
	std::vector<Triangle> triangles;
};

#endif
