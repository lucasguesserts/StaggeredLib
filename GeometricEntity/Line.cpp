#include <GeometricEntity/Line.hpp>

Eigen::Vector3d Line::getCentroid(void)
{
	Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
	for(const Vertex* vertex: this->vertices)
		centroid += *vertex;
	centroid /= 2;
	return centroid;
}

Eigen::Vector3d Line::getAreaVector(void)
{
	Eigen::Vector3d areaVector = *(this->vertices[1]) - *(this->vertices[0]);
	std::swap(areaVector[0], areaVector[1]);
	areaVector[1] = - areaVector[1];
	return areaVector;
}

double Line::getVolume(void)
{
	return (*(this->vertices[1]) - *(this->vertices[0])).norm();
}