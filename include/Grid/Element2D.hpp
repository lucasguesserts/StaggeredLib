#ifndef ELEMENT2D_HPP
#define ELEMENT2D_HPP

#include <Grid/Element.hpp>
#include <Eigen/Core>

class Element2D: public Element
{
	public:
		static const unsigned dimension;
		virtual Eigen::Vector3d getAreaVector(void) const = 0;
};

#endif
