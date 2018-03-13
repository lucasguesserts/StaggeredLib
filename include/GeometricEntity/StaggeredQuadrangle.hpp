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
		double getVolume(void)
		{
			return this->getAreaVector().norm();
		}
	private:
		Eigen::Vector3d getAreaVector(void)
		{
			Eigen::Vector3d firstTriangle = this->computeTriangleAreaVector(*(this->vertices[0]), this->elements[0]->getCentroid(), this->elements[1]->getCentroid());
			Eigen::Vector3d secondTriangle = this->computeTriangleAreaVector(this->elements[0]->getCentroid(), *(this->vertices[1]), this->elements[1]->getCentroid());
			return firstTriangle + secondTriangle;
		}
		Eigen::Vector3d computeTriangleAreaVector(
				Eigen::Vector3d firstPoint,
				Eigen::Vector3d secondPoint,
				Eigen::Vector3d thirdPoint)
		{
			return (1.0/2.0) * (secondPoint - firstPoint).cross(thirdPoint - firstPoint);
		}
};

#endif
