#ifndef GRID_2D_INVERSE_DISTANCE_STENCIL_2
#define GRID_2D_INVERSE_DISTANCE_STENCIL_2

#include <vector>

#include <Grid/GridData.hpp>
#include <Grid/Grid2DWithStaggeredElements.hpp>
#include <GeometricEntity/StaggeredElement2D.hpp>
#include <Stencil/ScalarStencil.hpp>
#include <Stencil/VectorStencil.hpp>

class Grid2DInverseDistanceStencil: public Grid2DWithStaggeredElements
{
	public:
		Grid2DInverseDistanceStencil(const GridData& gridData);
		ScalarStencil computeScalarStencil(Vertex& vertex);
		std::vector<ScalarStencil> computeScalarStencilOnVertices(void);
		std::vector<ScalarStencil> computeScalarStencilOnVerticesUsingStaggeredElements(void);
		VectorStencil computeVectorStencilOnQuadrangle(StaggeredElement2D& staggeredQuadrangle, std::vector<ScalarStencil>& scalarStencilOnVertices);

		static ScalarStencil computeScalarStencilOnElement(Element* element);
		static void normalizeScalarStencil(ScalarStencil& scalarStencil);
		static VectorStencil computeVectorStencil(const Eigen::Vector3d& firstPoint, const Eigen::Vector3d& secondPoint, const ScalarStencil& firstScalarStencil, const ScalarStencil& secondScalarStencil);
		static Eigen::Vector3d computeAreaVector(const Eigen::Vector3d& firstPoint, const Eigen::Vector3d& secondPoint);
		static ScalarStencil computeAverageScalarStencil(const ScalarStencil& firstScalarStencil, const ScalarStencil& secondScalarStencil);
};

#endif
