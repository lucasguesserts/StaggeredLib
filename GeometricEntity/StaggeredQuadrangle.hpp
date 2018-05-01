#ifndef STAGGERED_QUADRANGLE_HPP
#define STAGGERED_QUADRANGLE_HPP

#include <array>
#include <Eigen/Core>
#include <GeometricEntity/Element.hpp>
#include <GeometricEntity/Vertex.hpp>
#include <GeometricEntity/StaggeredElement.hpp>

class StaggeredQuadrangle: public StaggeredElement
{
	public:
		StaggeredQuadrangle(const unsigned index, Vertex& vertex_0, Element* element_0, Vertex& vertex_1, Element* element_1);
	private:
		Eigen::Vector3d getAreaVector3D(void);
};

#endif
