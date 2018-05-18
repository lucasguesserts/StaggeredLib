#ifndef GRID2D_VERTICES_WITH_NEIGHBOR_ELEMENTS
#define GRID2D_VERTICES_WITH_NEIGHBOR_ELEMENTS

#include <vector>
#include <Grid/Grid2D.hpp>
#include <Grid/GridData_2.hpp>
#include <GeometricEntity/Element.hpp>

class Grid2DVerticesWithNeighborElements: public Grid2D
{
	public:
		Grid2DVerticesWithNeighborElements(const GridData_2& gridData);
		Grid2DVerticesWithNeighborElements(const std::string& fileName);

		std::vector<std::vector<Element*>> verticesNeighborElements;
	private:
		void setVerticesNeighborElements(void);
};

#endif
