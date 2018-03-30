#ifndef STAGGERED_QUADRANGLE_HPP
#define STAGGERED_QUADRANGLE_HPP

#include <array>
#include <iostream>
#include <Eigen/Core>
#include <GeometricEntity/Element.hpp>
#include <GeometricEntity/Vertex.hpp>

class StaggeredQuadrangle: public Element
{
	public:
		std::array<Element*,2> elements;

		StaggeredQuadrangle(const unsigned index, Vertex& vertex_0, Element* element_0, Vertex& vertex_1, Element* element_1);
		virtual Eigen::Vector3d getCentroid(void) override;
		virtual Eigen::Vector3d getAreaVector(void) override; // Front: element 0. Back: element 1.
		virtual double getVolume(void) override;
	private:
		Eigen::Vector3d getAreaVector3D(void);
};

bool operator==(const StaggeredQuadrangle& lhs, const StaggeredQuadrangle& rhs);
std::ostream& operator<< (std::ostream& os, const StaggeredQuadrangle& StaggeredQuadrangle);

#endif
