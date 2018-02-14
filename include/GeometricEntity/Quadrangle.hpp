#ifndef QUADRANGLE_HPP
#define QUADRANGLE_HPP

#include <Eigen/Core>
#include <GeometricEntity/Element2D.hpp>

class Quadrangle: public Element2D
{
	public:
		virtual Eigen::Vector3d getAreaVector(void);
		virtual double getVolume(void);
		virtual Eigen::VectorXd getShapeFunctionValues(const Eigen::Vector3d localCoordinates) const;
		virtual Eigen::MatrixXd getShapeFunctionDerivatives(const Eigen::Vector3d localCoordinates) const;
};

#endif
