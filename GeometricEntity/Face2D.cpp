#include <GeometricEntity/Face2D.hpp>

Face2D::Face2D(
	const unsigned index,
	const unsigned localIndex,
	Element& parentElement,
	Vertex& adjacentVertex,
	StaggeredElement2D& backwardStaggeredElement,
	StaggeredElement2D& forwardStaggeredElement)
	: Entity(index),
	  localIndex(localIndex),
	  parentElement(&parentElement),
	  adjacentVertex(&adjacentVertex),
	  backwardStaggeredElement(&backwardStaggeredElement),
	  forwardStaggeredElement(&forwardStaggeredElement)
	{}

Eigen::Vector3d Face2D::getCentroid(void)
{
	return 0.5*(this->parentElement->getCentroid() + *(this->adjacentVertex));
}

Eigen::Vector3d Face2D::getAreaVector(void)
{
	Eigen::Vector3d areaVector = *(this->adjacentVertex) - this->parentElement->getCentroid();
	std::swap(areaVector[0],areaVector[1]);
	areaVector[1] = - areaVector[1];
	return areaVector;
}