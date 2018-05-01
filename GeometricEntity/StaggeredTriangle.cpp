#include <GeometricEntity/StaggeredTriangle.hpp>

StaggeredTriangle::StaggeredTriangle(const unsigned index, Vertex& vertex_0, Element* element, Vertex& vertex_1)
	: StaggeredElement(index, vertex_0, element, vertex_1)
{}

Eigen::Vector3d StaggeredTriangle::getAreaVector3D(void)
{
	return Element::computeTriangleAreaVector(*(this->vertices[0]), this->elements[0]->getCentroid(), *(this->vertices[1]));
}