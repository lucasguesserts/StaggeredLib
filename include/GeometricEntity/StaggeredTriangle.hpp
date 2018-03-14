#ifndef STAGGERED_TRIANGLE_HPP
#define STAGGERED_TRIANGLE_HPP

#include <array>
#include <Eigen/Core>
#include <GeometricEntity/Entity.hpp>
#include <GeometricEntity/Vertex.hpp>
#include <SquareCavityHeatTransfer/ScalarStencil.hpp>
#include <SquareCavityHeatTransfer/VectorStencil.hpp>



class StaggeredTriangle: public Element
{
	public:
		Element* element;

		StaggeredTriangle(const unsigned index, Vertex& vertex_0, Element* element, Vertex& vertex_1)
		{
			this->setIndex(index);
			this->addVertex(vertex_0);
			this->addVertex(vertex_1);
			this->element = element;
			return;
		}
		virtual Eigen::Vector3d getCentroid(void) override
		{
			return 0.5 * (*(this->vertices[0]) + *(this->vertices[1]));
		}
		virtual Eigen::Vector3d getAreaVector(void) override
		{
			return Element::computeTriangleAreaVector(*(this->vertices[0]), this->element->getCentroid(), *(this->vertices[1]));
		}
		virtual double getVolume(void) override
		{
			return this->getAreaVector().norm();
		}
};

#endif
