#include <GeometricEntity/StaggeredQuadrangle.hpp>

StaggeredQuadrangle::StaggeredQuadrangle(const unsigned index, Vertex& vertex_0, Element* element_0, Vertex& vertex_1, Element* element_1)
{
	this->setIndex(index);
	this->addVertex(vertex_0);
	this->addVertex(vertex_1);
	this->elements[0] = element_0;
	this->elements[1] = element_1;
	return;
}

Eigen::Vector3d StaggeredQuadrangle::getCentroid(void)
{
	return 0.5 * (*(this->vertices[0]) + *(this->vertices[1]));
}

Eigen::Vector3d StaggeredQuadrangle::getAreaVector(void)
{
	Eigen::Vector3d areaVector = *(this->vertices[1]) - *(this->vertices[0]);
	std::swap(areaVector[0],areaVector[1]);
	areaVector[1] = - areaVector[1];
	return areaVector;
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
