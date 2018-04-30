#ifndef STAGGERED_TRIANGLE_HPP
#define STAGGERED_TRIANGLE_HPP

#include <array>
#include <Eigen/Core>
#include <GeometricEntity/Element.hpp>
#include <GeometricEntity/Vertex.hpp>

class StaggeredTriangle: public Element
{
	public:
		std::vector<Element*> elements;

		StaggeredTriangle(const unsigned index, Vertex& vertex_0, Element* element, Vertex& vertex_1);
		virtual Eigen::Vector3d getCentroid(void) override;
		virtual Eigen::Vector3d getAreaVector(void) override;
		virtual double getVolume(void) override;
	private:
		Eigen::Vector3d getAreaVector3D(void);
};

#endif
