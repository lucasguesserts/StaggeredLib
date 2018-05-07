#include <GeometricEntity/StaggeredElement2D.hpp>

StaggeredElement2D::StaggeredElement2D(const unsigned index, Vertex& vertex_0, Element* element, Vertex& vertex_1)
{
	this->setIndex(index);
	this->vertices[0] = &vertex_0;
	this->vertices[1] = &vertex_1;
	this->elements[0] = element;
	this->elements[1] = nullptr;
	return;
}

StaggeredElement2D::StaggeredElement2D(const unsigned index, Vertex& vertex_0, Element* element_0, Vertex& vertex_1, Element* element_1)
{
	this->setIndex(index);
	this->vertices[0] = &vertex_0;
	this->vertices[1] = &vertex_1;
	this->elements[0] = element_0;
	this->elements[1] = element_1;
}

Eigen::Vector3d StaggeredElement2D::getCentroid(void)
{
	return 0.5 * (*(this->vertices[0]) + *(this->vertices[1]));
}

Eigen::Vector3d StaggeredElement2D::getAreaVector(void)
{
	Eigen::Vector3d areaVector = *(this->vertices[1]) - *(this->vertices[0]);
	std::swap(areaVector[0],areaVector[1]);
	areaVector[1] = - areaVector[1];
	return areaVector;
}

double StaggeredElement2D::getVolume(void)
{
	double volume = 0.0;
	volume += Element::computeTriangleAreaVector(*(this->vertices[0]), this->elements[0]->getCentroid(), *(this->vertices[1])).norm();
	if(this->elements[1]!=nullptr)
		volume += Element::computeTriangleAreaVector(*(this->vertices[0]), this->elements[1]->getCentroid(), *(this->vertices[1])).norm();
	return volume;
}