#ifndef STAGGERED_TRIANGLE_HPP
#define STAGGERED_TRIANGLE_HPP

#include <array>
#include <Eigen/Core>
#include <GeometricEntity/Element.hpp>
#include <GeometricEntity/Vertex.hpp>
#include <GeometricEntity/StaggeredElement.hpp>

class StaggeredTriangle: public StaggeredElement
{
	public:
		StaggeredTriangle(const unsigned index, Vertex& vertex_0, Element* element, Vertex& vertex_1);
	private:
		Eigen::Vector3d getAreaVector3D(void);
};

#endif
