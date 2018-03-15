#include <GeometricEntity/StaggeredTriangle.hpp>

StaggeredTriangle::StaggeredTriangle(const unsigned index, Vertex& vertex_0, Element* element, Vertex& vertex_1)
{
	this->setIndex(index);
	this->addVertex(vertex_0);
	this->addVertex(vertex_1);
	this->element = element;
	return;
}

Eigen::Vector3d StaggeredTriangle::getCentroid(void)
{
	return 0.5 * (*(this->vertices[0]) + *(this->vertices[1]));
}

Eigen::Vector3d StaggeredTriangle::getAreaVector(void)
{
	return Element::computeTriangleAreaVector(*(this->vertices[0]), this->element->getCentroid(), *(this->vertices[1]));
}

double StaggeredTriangle::getVolume(void)
{
	return this->getAreaVector().norm();
}
