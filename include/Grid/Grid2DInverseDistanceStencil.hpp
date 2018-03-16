#ifndef GRID_2D_INVERSE_DISTANCE_STENCIL
#define GRID_2D_INVERSE_DISTANCE_STENCIL

#include <vector>

#include <Grid/GridData.hpp>
#include <Grid/Grid2DWithStaggeredElements.hpp>
#include <Stencil/ScalarStencil.hpp>

class Grid2DInverseDistanceStencil: public Grid2DWithStaggeredElements
{
	public:
		Grid2DInverseDistanceStencil(GridData& gridData);
		ScalarStencil computeScalarStencil(Vertex& vertex);
		std::vector<ScalarStencil> computeScalarStencilOnVertices(void);
	private:
		static void normalizeScalarStencil(ScalarStencil& scalarStencil);
};

#endif
