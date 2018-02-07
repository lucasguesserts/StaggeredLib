#ifndef TRIANGLE_HPP
#define TRIANGLE_HPP

#include <Eigen/Core>
#include <Grid/Element2D.hpp>

class Triangle: public Element2D
{
	public:
		virtual Eigen::Vector3d getAreaVector(void);
		virtual double getVolume(void);
		virtual Eigen::VectorXd getShapeFunctionValues(const Eigen::Vector3d localCoordinates) const;
		virtual Eigen::MatrixXd getShapeFunctionDerivatives(const Eigen::Vector3d localCoordinates) const;
};

#endif
