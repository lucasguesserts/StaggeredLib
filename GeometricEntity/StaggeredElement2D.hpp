#ifndef STAGGERED_ELEMENT_2D_HPP
#define STAGGERED_ELEMENT_2D_HPP

#include <array>
#include <GeometricEntity/Vertex.hpp>
#include <GeometricEntity/Element.hpp>

class StaggeredElement2D: public Entity
{
	public:
		StaggeredElement2D(const unsigned index, Vertex& vertex_0, Element* element, Vertex& vertex_1);
		StaggeredElement2D(const unsigned index, Vertex& vertex_0, Element* element_0, Vertex& vertex_1, Element* element_1);
		Eigen::Vector3d getCentroid(void);
		Eigen::Vector3d getAreaVector(void);
		double getVolume(void);

		std::array<Vertex*,2> vertices;
		std::array<Element*,2> elements;
};

#endif