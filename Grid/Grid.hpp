#ifndef GRID_HPP
#define GRID_HPP

#include <vector>
#include <Grid/GridData_2.hpp>
#include <GeometricEntity/Element.hpp>
#include <GeometricEntity/Vertex.hpp>

struct Grid
{
	public:
		Grid(const GridData_2& gridData);

		unsigned dimension;
		std::vector<Vertex> vertices;
		std::vector<Element*> elements;
	private:
		void buildVertices(const GridData_2& gridData);
};

#endif
