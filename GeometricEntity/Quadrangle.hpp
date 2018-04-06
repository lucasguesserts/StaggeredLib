#ifndef QUADRANGLE_HPP
#define QUADRANGLE_HPP

#include <Eigen/Core>
#include <GeometricEntity/Element.hpp>

class Quadrangle: public Element
{
	public:
		virtual Eigen::Vector3d getCentroid(void) override;
		virtual Eigen::Vector3d getAreaVector(void) override;
		virtual double getVolume(void) override;
};

#endif
