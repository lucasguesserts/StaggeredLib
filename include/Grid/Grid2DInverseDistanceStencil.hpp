#ifndef GRID_2D_INVERSE_DISTANCE_STENCIL
#define GRID_2D_INVERSE_DISTANCE_STENCIL

#include <vector>

#include <Grid/GridData.hpp>
#include <Grid/Grid2DWithStaggeredElements.hpp>
#include <Stencil/ScalarStencil.hpp>
#include <Stencil/VectorStencil.hpp>

class Grid2DInverseDistanceStencil: public Grid2DWithStaggeredElements
{
	public:
		Grid2DInverseDistanceStencil(GridData& gridData);
		ScalarStencil computeScalarStencil(Vertex& vertex);
		std::vector<ScalarStencil> computeScalarStencilOnVertices(void);
		VectorStencil computeVectorStencil(StaggeredQuadrangle& staggeredQuadrangle, const ScalarStencil& scalarStencilOnVertices, const std::vector<ScalarStencil>& scalarStencilOnElements);
		static ScalarStencil computeScalarStencilOnElement(Element* element);
	private:
		static void normalizeScalarStencil(ScalarStencil& scalarStencil);
};

#endif
