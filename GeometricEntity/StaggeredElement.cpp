#include <GeometricEntity/StaggeredElement.hpp>

Eigen::Vector3d StaggeredElement::getCentroid(void)
{
	return 0.5 * (*(this->vertices[0]) + *(this->vertices[1]));
}

Eigen::Vector3d StaggeredElement::getAreaVector(void)
{
	Eigen::Vector3d areaVector = *(this->vertices[1]) - *(this->vertices[0]);
	std::swap(areaVector[0],areaVector[1]);
	areaVector[1] = - areaVector[1];
	return areaVector;
}