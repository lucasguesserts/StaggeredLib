#ifndef SCALAR_STENCIL_COMPUTER_HPP
#define SCALAR_STENCIL_COMPUTER_HPP

#include <vector>
#include <SquareCavityHeatTransfer/ScalarStencil.hpp>
#include <GeometricEntity/Vertex.hpp>
#include <GeometricEntity/Element.hpp>
#include <Grid/Grid2DVerticesWithNeighborElements.hpp>

class ScalarStencilComputer
{
	public:
		static ScalarStencil inverseDistance(const Vertex& vertex,const std::vector<Element*>& vertexNeighborElements);
		static std::vector<ScalarStencil> inverseDistance(const Grid2DVerticesWithNeighborElements& grid);
	private:
		static void normalizeScalarStencil(ScalarStencil& scalarStencil);
};

#endif
