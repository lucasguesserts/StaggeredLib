#ifndef STAGGERED_QUADRANGLE_HPP
#define STAGGERED_QUADRANGLE_HPP

#include <array>
#include <Eigen/Core>
#include <GeometricEntity/Entity.hpp>
#include <GeometricEntity/Vertex.hpp>
#include <SquareCavityHeatTransfer/ScalarStencil.hpp>
#include <SquareCavityHeatTransfer/VectorStencil.hpp>

class StaggeredQuadrangle: public Element
{
	public:
		std::array<Element*,2> elements;

		StaggeredQuadrangle(const unsigned index, Vertex& vertex_0, Element* element_0, Vertex& vertex_1, Element* element_1)
		{
			this->setIndex(index);
			this->addVertex(vertex_0);
			this->addVertex(vertex_1);
			this->elements[0] = element_0;
			this->elements[1] = element_1;
			return;
		}
		virtual Eigen::Vector3d getCentroid(void) override
		{
			return 0.5 * (*(this->vertices[0]) + *(this->vertices[1]));
		}
		virtual Eigen::Vector3d getAreaVector(void) override
		{
			return Element::computeTriangleAreaVector(*(this->vertices[0]), this->elements[0]->getCentroid(), this->elements[1]->getCentroid()) +
			       Element::computeTriangleAreaVector(this->elements[0]->getCentroid(), *(this->vertices[1]), this->elements[1]->getCentroid());
		}
		virtual double getVolume(void) override
		{
			return this->getAreaVector().norm();
		}
};

#endif
