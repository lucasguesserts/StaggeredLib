#include <GeometricEntity/Face.hpp>

Face::Face(
	const unsigned index,
	const unsigned localIndex,
	Element& parentElement,
	Vertex& adjacentVertex,
	StaggeredElement& backwardStaggeredElement,
	StaggeredElement& forwardStaggeredElement)
	: Entity(index),
	  localIndex(localIndex),
	  parentElement(&parentElement),
	  adjacentVertex(&adjacentVertex),
	  backwardStaggeredElement(&backwardStaggeredElement),
	  forwardStaggeredElement(&forwardStaggeredElement)
	{}

Eigen::Vector3d Face::getAreaVector(void)
{
	Eigen::Vector3d areaVector = *(this->adjacentVertex) - this->parentElement->getCentroid();
	std::swap(areaVector[0],areaVector[1]);
	areaVector[1] = - areaVector[1];
	return areaVector;
}