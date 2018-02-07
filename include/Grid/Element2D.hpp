#ifndef ELEMENT2D_HPP
#define ELEMENT2D_HPP

#include <Grid/Element.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

class Element2D: public Element
{
	public:
		static const unsigned dimension;
		virtual Eigen::Vector3d getAreaVector(void) = 0;
	protected:
		Eigen::Vector3d computeTriangleAreaVector(const Vertex* const firstPoint, const Vertex* const secondPoint, const Vertex* const thirdPoint) const;
};

#endif
