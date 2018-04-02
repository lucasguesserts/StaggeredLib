#include <GeometricEntity/StaggeredTriangle.hpp>

StaggeredTriangle::StaggeredTriangle(const unsigned index, Vertex& vertex_0, Element* element, Vertex& vertex_1)
{
	this->setIndex(index);
	this->addVertex(vertex_0);
	this->addVertex(vertex_1);
	this->element = element;
	return;
}

Eigen::Vector3d StaggeredTriangle::getAreaVector(void)
{
	Eigen::Vector3d areaVector = *(this->vertices[1]) - *(this->vertices[0]);
	std::swap(areaVector[0],areaVector[1]);
	areaVector[1] = - areaVector[1];
	return areaVector;
}

Eigen::Vector3d StaggeredTriangle::getCentroid(void)
{
	return 0.5 * (*(this->vertices[0]) + *(this->vertices[1]));
}

double StaggeredTriangle::getVolume(void)
{
	return this->getAreaVector3D().norm();
}

Eigen::Vector3d StaggeredTriangle::getAreaVector3D(void)
{
	return Element::computeTriangleAreaVector(*(this->vertices[0]), this->element->getCentroid(), *(this->vertices[1]));
}

bool operator==(const StaggeredTriangle& lhs, const StaggeredTriangle& rhs)
{
	return lhs.getIndex()==rhs.getIndex() &&
	       lhs.vertices[0]==rhs.vertices[0] &&
	       lhs.vertices[1]==rhs.vertices[1] &&
	       lhs.element==rhs.element;
}

std::ostream& operator<<(std::ostream& os, const StaggeredTriangle& staggeredTriangle)
{
	os << "StaggeredTriangle{"
	   << "I:" << staggeredTriangle.getIndex()
	   << ","
	   << "v:" << staggeredTriangle.vertices[0]->getIndex()
	   << ","
	   << "e:" << staggeredTriangle.element->getIndex()
	   << ","
	   << "v:" << staggeredTriangle.vertices[1]->getIndex()
	   << "}";
	return os;
}