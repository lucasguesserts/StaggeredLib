#include <GeometricEntity/Element.hpp>
#include <Eigen/Geometry>

void Element::addVertex(Vertex& vertex)
{
	this->vertices.push_back(&vertex);
	return;
}

Eigen::Vector3d Element::computeTriangleAreaVector(const Eigen::Vector3d& first, const Eigen::Vector3d& second, const Eigen::Vector3d& third)
{
	return (1.0/2.0) * (second - first).cross(third - first);
}