#ifndef SCALAR_STENCIL_COMPUTER_HPP
#define SCALAR_STENCIL_COMPUTER_HPP

#include <vector>
#include <SquareCavityHeatTransfer/ScalarStencil.hpp>
#include <SquareCavityHeatTransfer/VectorStencil.hpp>
#include <GeometricEntity/Vertex.hpp>
#include <GeometricEntity/Element.hpp>
#include <GeometricEntity/StaggeredQuadrangle.hpp>
#include <Grid/Grid2DVerticesWithNeighborElements.hpp>

class ScalarStencilComputer
{
	public:
		static ScalarStencil inverseDistance(const Vertex& vertex,const std::vector<Element*>& vertexNeighborElements);
		static std::vector<ScalarStencil> inverseDistance(const Grid2DVerticesWithNeighborElements& grid);
		static std::vector<ScalarStencil> elements(const Grid2DVerticesWithNeighborElements& grid);
		static VectorStencil vectorStencil(StaggeredQuadrangle& staggeredQuadrangle, std::vector<ScalarStencil>& scalarStencilOnVertices, std::vector<ScalarStencil>& scalarStencilOnElements);

		static VectorStencil computeVectorStencil(Eigen::Vector3d firstPoint, Eigen::Vector3d secondPoint, ScalarStencil& firstScalarStencil, ScalarStencil& secondScalarStencil);
		static Eigen::Vector3d computeAreaVector(Eigen::Vector3d firstPoint, Eigen::Vector3d secondPoint);
		static ScalarStencil computeAverageScalarStencil(ScalarStencil& firstScalarStencil, ScalarStencil& secondScalarStencil);
	private:
		static void normalizeScalarStencil(ScalarStencil& scalarStencil);
};

#endif
