#include <GeometricEntity/StaggeredQuadrangle.hpp>

StaggeredQuadrangle::StaggeredQuadrangle(const unsigned index, Vertex& vertex_0, Element* element_0, Vertex& vertex_1, Element* element_1)
: StaggeredElement(index,vertex_0,element_0,vertex_1,element_1)
{}

Eigen::Vector3d StaggeredQuadrangle::getAreaVector3D(void)
{
	return Element::computeTriangleAreaVector(*(this->vertices[0]), this->elements[0]->getCentroid(), this->elements[1]->getCentroid()) +
		   Element::computeTriangleAreaVector(this->elements[0]->getCentroid(), *(this->vertices[1]), this->elements[1]->getCentroid());
}