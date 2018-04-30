#include <GeometricEntity/StaggeredQuadrangle.hpp>

StaggeredQuadrangle::StaggeredQuadrangle(const unsigned index, Vertex& vertex_0, Element* element_0, Vertex& vertex_1, Element* element_1)
{
	this->setIndex(index);
	this->addVertex(vertex_0);
	this->addVertex(vertex_1);
	this->elements.push_back(element_0);
	this->elements.push_back(element_1);
	return;
}

double StaggeredQuadrangle::getVolume(void)
{
	return this->getAreaVector3D().norm();
}

Eigen::Vector3d StaggeredQuadrangle::getAreaVector3D(void)
{
	return Element::computeTriangleAreaVector(*(this->vertices[0]), this->elements[0]->getCentroid(), this->elements[1]->getCentroid()) +
		   Element::computeTriangleAreaVector(this->elements[0]->getCentroid(), *(this->vertices[1]), this->elements[1]->getCentroid());
}