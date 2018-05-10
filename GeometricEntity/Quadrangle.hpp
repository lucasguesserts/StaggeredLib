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
		virtual Eigen::VectorXd getShapeFunctionValues(const Eigen::Vector3d localCoordinates) final;
		virtual Eigen::MatrixXd getShapeFunctionDerivatives(const Eigen::Vector3d localCoordinates) final;

};

#endif
