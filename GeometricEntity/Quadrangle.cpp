#include <GeometricEntity/Quadrangle.hpp>

Eigen::Vector3d Quadrangle::getCentroid(void)
{
	Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
	for(const Vertex* vertex: this->vertices)
		centroid += *vertex;
	centroid /= 4;
	return centroid;
}

Eigen::Vector3d Quadrangle::getAreaVector(void)
{
	return Element::computeTriangleAreaVector(*(this->vertices[0]), *(this->vertices[1]), *(this->vertices[2])) +
	       Element::computeTriangleAreaVector(*(this->vertices[0]), *(this->vertices[2]), *(this->vertices[3]));
}

double Quadrangle::getVolume(void)
{
	return this->getAreaVector().norm();
}

std::ostream& operator<<(std::ostream& os, const Quadrangle& quadrangle)
{
	os << "Quadrangle" << static_cast<const Element&>(quadrangle);
	return os;
}