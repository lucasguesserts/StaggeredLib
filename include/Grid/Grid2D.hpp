#ifndef GRID2D_HPP
#define GRID2D_HPP

#include <vector>
#include <Grid/Quadrangle.hpp>
#include <Grid/Triangle.hpp>
#include <Grid/Grid.hpp>

struct Grid2D: public Grid
{
	std::vector<Quadrangle> quadrangles;
	std::vector<Triangle> triangles;
};

#endif
