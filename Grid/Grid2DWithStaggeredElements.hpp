#ifndef GRID_2D_WITH_STAGGERED_ELEMENTS
#define GRID_2D_WITH_STAGGERED_ELEMENTS

#include <vector>

#include <Grid/StaggeredElementDefinition.hpp>
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
		std::vector<StaggeredElementDefinition> staggeredElementDefinition;

		void addStaggeredElementDefinition(StaggeredElementDefinition& staggeredElementDefinition);
		std::tuple<bool,unsigned> findStaggeredElementDefinition(const StaggeredElementDefinition& staggeredElementDefinition);

	private:
		
		void allocateStaggeredElementDefinition(const GridData& gridData);
		void shrinkStaggeredElementDefinition(void);
};

#endif
