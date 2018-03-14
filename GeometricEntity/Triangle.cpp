#include <GeometricEntity/Triangle.hpp>

Eigen::Vector3d Triangle::getCentroid(void)
{
	Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
	for(const Vertex* vertex: this->vertices)
		centroid += *vertex;
	centroid /= 3;
	return centroid;
}

Eigen::Vector3d Triangle::getAreaVector(void)
{
	return Element::computeTriangleAreaVector(*(this->vertices[0]), *(this->vertices[1]), *(this->vertices[2]));
}

double Triangle::getVolume(void)
{
	return this->getAreaVector().norm();
}
