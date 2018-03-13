#ifndef STAGGERED_TRIANGLE_HPP
#define STAGGERED_TRIANGLE_HPP

#include <array>
#include <Eigen/Core>
#include <GeometricEntity/Entity.hpp>
#include <GeometricEntity/Vertex.hpp>
#include <SquareCavityHeatTransfer/ScalarStencil.hpp>
#include <SquareCavityHeatTransfer/VectorStencil.hpp>

class StaggeredTriangle: public Entity
{
	public:
		std::array<Vertex*,2> vertices;
		Element* element;
		StaggeredTriangle(const unsigned index, Vertex& vertex_0, Element* element, Vertex& vertex_1)
			: Entity(index)
		{
			this->vertices[0] = &vertex_0;
			this->element = element;
			this->vertices[1] = &vertex_1;
			return;
		}
		double getVolume(void)
		{
			return this->getAreaVector().norm();
		}
	private:
		Eigen::Vector3d getAreaVector(void)
		{
			return this->computeTriangleAreaVector(*(this->vertices[0]), this->element->getCentroid(), *(this->vertices[1]));
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
