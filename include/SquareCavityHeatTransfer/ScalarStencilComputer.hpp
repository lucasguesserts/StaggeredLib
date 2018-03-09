#ifndef SCALAR_STENCIL_COMPUTER_HPP
#define SCALAR_STENCIL_COMPUTER_HPP

#include <vector>
#include <SquareCavityHeatTransfer/ScalarStencil.hpp>
#include <GeometricEntity/Vertex.hpp>
#include <GeometricEntity/Element.hpp>

class ScalarStencilComputer
{
	public:
		static ScalarStencil inverseDistance(const Vertex& vertex,const std::vector<Element*>& vertexNeighborElements);
};

#endif
