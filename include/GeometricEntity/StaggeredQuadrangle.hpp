#ifndef STAGGERED_QUADRANGLE_HPP
#define STAGGERED_QUADRANGLE_HPP

#include <array>
#include <Eigen/Core>
#include <GeometricEntity/Entity.hpp>
#include <GeometricEntity/Vertex.hpp>
#include <SquareCavityHeatTransfer/ScalarStencil.hpp>
#include <SquareCavityHeatTransfer/VectorStencil.hpp>

class StaggeredQuadrangle: public Entity
{
	public:
		std::array<Vertex*,2> vertices;
		std::array<Element*,2> elements;
		StaggeredQuadrangle(const unsigned index, Vertex& vertex_0, Element* element_0, Vertex& vertex_1, Element* element_1)
			: Entity(index)
		{
			this->vertices[0] = &vertex_0;
			this->vertices[1] = &vertex_1;
			this->elements[0] = element_0;
			this->elements[1] = element_1;
			return;
		}
		VectorStencil computeGreenGaussGradient(
				ScalarStencil& temperatureVertex_0,
				ScalarStencil& temperatureVertex_1,
				ScalarStencil& temperatureElement_0,
				ScalarStencil& temperatureElement_1)
		{
			return VectorStencil{ { 0, {0,0,0} } };
		}
};

#endif
