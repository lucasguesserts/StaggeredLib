#ifndef GRID_2D_WITH_STAGGERED_ELEMENTS_2
#define GRID_2D_WITH_STAGGERED_ELEMENTS_2

#include <vector>

#include <Grid/GridData.hpp>
#include <Grid/Grid2DVerticesWithNeighborElements.hpp>
#include <GeometricEntity/StaggeredElement.hpp>
#include <GeometricEntity/Face.hpp>

class Grid2DWithStaggeredElements_2: public Grid2DVerticesWithNeighborElements
{
	public:
		Grid2DWithStaggeredElements_2(const GridData& gridData);
		std::vector<StaggeredElement> staggeredElements;
		std::vector<StaggeredElement*> staggeredQuadrangles;
		std::vector<StaggeredElement*> staggeredTriangles;
		std::vector<Face> faces;
};

#endif