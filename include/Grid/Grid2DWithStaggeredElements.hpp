#ifndef GRID_2D_WITH_STAGGERED_ELEMENTS
#define GRID_2D_WITH_STAGGERED_ELEMENTS

#include <vector>

#include <Grid/GridData.hpp>
#include <Grid/Grid2DVerticesWithNeighborElements.hpp>
#include <GeometricEntity/StaggeredQuadrangle.hpp>
#include <GeometricEntity/StaggeredTriangle.hpp>

class Grid2DWithStaggeredElements: public Grid2DVerticesWithNeighborElements
{
	public:
		Grid2DWithStaggeredElements(const GridData& gridData);

		std::vector<StaggeredQuadrangle> staggeredQuadrangles;
		std::vector<StaggeredTriangle> staggeredTriangles;
};

#endif
