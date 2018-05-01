#ifndef STAGGERED_ELEMENT_HPP
#define STAGGERED_ELEMENT_HPP

#include <vector>
#include <GeometricEntity/Vertex.hpp>
#include <GeometricEntity/Element.hpp>

class StaggeredElement: public Element
{
	public:
		std::vector<Element*> elements;
		virtual Eigen::Vector3d getCentroid(void) final;
		virtual Eigen::Vector3d getAreaVector(void) final;
		virtual double getVolume(void) final;
};

#endif