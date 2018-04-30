#include <GeometricEntity/StaggeredTriangle.hpp>

StaggeredTriangle::StaggeredTriangle(const unsigned index, Vertex& vertex_0, Element* element, Vertex& vertex_1)
{
	this->setIndex(index);
	this->addVertex(vertex_0);
	this->addVertex(vertex_1);
	this->elements.push_back(element);
	return;
}

double StaggeredTriangle::getVolume(void)
{
	return this->getAreaVector3D().norm();
}

Eigen::Vector3d StaggeredTriangle::getAreaVector3D(void)
{
	return Element::computeTriangleAreaVector(*(this->vertices[0]), this->elements[0]->getCentroid(), *(this->vertices[1]));
}