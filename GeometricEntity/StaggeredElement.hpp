#ifndef STAGGERED_ELEMENT_HPP
#define STAGGERED_ELEMENT_HPP

#include <vector>
#include <GeometricEntity/Vertex.hpp>
#include <GeometricEntity/Element.hpp>

class StaggeredElement: public Element
{
	public:
		StaggeredElement(const unsigned index, Vertex& vertex_0, Element* element, Vertex& vertex_1);
		StaggeredElement(const unsigned index, Vertex& vertex_0, Element* element_0, Vertex& vertex_1, Element* element_1);
		std::vector<Element*> elements;
		virtual Eigen::Vector3d getCentroid(void) final;
		virtual Eigen::Vector3d getAreaVector(void) final;
		virtual double getVolume(void) final;
};

#endif