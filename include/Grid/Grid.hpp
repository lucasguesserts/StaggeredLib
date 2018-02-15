#ifndef GRID_HPP
#define GRID_HPP

#include <vector>
#include <Grid/GridData.hpp>
#include <GeometricEntity/Element.hpp>
#include <GeometricEntity/Vertex.hpp>

struct Grid
{
	public:
		Grid(const GridData& gridData);

		unsigned dimension;
		std::vector<Vertex> vertices;
		std::vector<Element*> elements;
	private:
		void buildVertices(const GridData& gridData);
};

#endif
