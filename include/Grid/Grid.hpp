#ifndef GRID_HPP
#define GRID_HPP

#include <vector>
#include <Grid/Element.hpp>
#include <Grid/Vertex.hpp>

struct Grid
{
	unsigned dimension;
	std::vector<Element*> elements;
	std::vector<Vertex> vertices;
};

#endif
