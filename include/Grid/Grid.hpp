#ifndef GRID_HPP
#define GRID_HPP

#include <vector>
#include <GeometricEntity/Element.hpp>
#include <GeometricEntity/Vertex.hpp>

struct Grid
{
	unsigned dimension;
	std::vector<Element*> elements;
	std::vector<Vertex> vertices;
};

#endif
