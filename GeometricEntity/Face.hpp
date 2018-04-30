#ifndef FACE_HPP
#define FACE_HPP

#include <Eigen/Core>
#include <GeometricEntity/Entity.hpp>
#include <GeometricEntity/Vertex.hpp>
#include <GeometricEntity/Element.hpp>
#include <GeometricEntity/StaggeredElement.hpp>

class Face
{
	public:
		Face(const unsigned localIndex,
		     Element& parentElement,
		     Vertex& adjacentVertex,
		     StaggeredElement& backwardStaggeredElement,
		     StaggeredElement& forwardStaggeredElement);

		unsigned localIndex;
		Element* parentElement;
		Vertex* adjacentVertex;
		StaggeredElement* backwardStaggeredElement;
		StaggeredElement* forwardStaggeredElement;

		Eigen::Vector3d getAreaVector(void);
};

#endif