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
		VectorStencil computeVectorStencilOnQuadrangle(StaggeredQuadrangle& staggeredQuadrangle, std::vector<ScalarStencil>& scalarStencilOnVertices);

		static ScalarStencil computeScalarStencilOnElement(Element* element);
		static void normalizeScalarStencil(ScalarStencil& scalarStencil);
		static VectorStencil computeVectorStencil(const Eigen::Vector3d& firstPoint, const Eigen::Vector3d& secondPoint, const ScalarStencil& firstScalarStencil, const ScalarStencil& secondScalarStencil);
		static Eigen::Vector3d computeAreaVector(const Eigen::Vector3d& firstPoint, const Eigen::Vector3d& secondPoint);
		static ScalarStencil computeAverageScalarStencil(const ScalarStencil& firstScalarStencil, const ScalarStencil& secondScalarStencil);
};

#endif
