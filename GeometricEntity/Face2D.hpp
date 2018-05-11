#ifndef FACE_2D_HPP
#define FACE_2D_HPP

#include <Eigen/Core>
#include <GeometricEntity/Entity.hpp>
#include <GeometricEntity/Vertex.hpp>
#include <GeometricEntity/Element.hpp>
#include <GeometricEntity/StaggeredElement2D.hpp>

class Face2D: public Entity
{
	public:
		Face2D(const unsigned index,
		     const unsigned localIndex,
		     Element& parentElement,
		     Vertex& adjacentVertex,
		     StaggeredElement2D& backwardStaggeredElement,
		     StaggeredElement2D& forwardStaggeredElement);

		unsigned localIndex;
		Element* parentElement;
		Vertex* adjacentVertex;
		StaggeredElement2D* backwardStaggeredElement;
		StaggeredElement2D* forwardStaggeredElement;

		Eigen::Vector3d getCentroid(void);
		Eigen::Vector3d getAreaVector(void);
};

#endif