#include <GeometricEntity/StaggeredElement.hpp>

StaggeredElement::StaggeredElement(const unsigned index, Vertex& vertex_0, Element* element, Vertex& vertex_1)
{
	this->setIndex(index);
	this->addVertex(vertex_0);
	this->addVertex(vertex_1);
	this->elements.push_back(element);
	return;
}

StaggeredElement::StaggeredElement(const unsigned index, Vertex& vertex_0, Element* element_0, Vertex& vertex_1, Element* element_1)
{
	this->setIndex(index);
	this->addVertex(vertex_0);
	this->addVertex(vertex_1);
	this->elements.push_back(element_0);
	this->elements.push_back(element_1);
}

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

double StaggeredElement::getVolume(void)
{
	double volume = 0.0;
	for(Element* element: this->elements)
	{
		volume += Element::computeTriangleAreaVector(*(this->vertices[0]), element->getCentroid(), *(this->vertices[1])).norm();
	}
	return volume;
}