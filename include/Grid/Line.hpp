#ifndef LINE_HPP
#define LINE_HPP

#include <Eigen/Core>
#include <Grid/Element1D.hpp>

class Line: public Element1D
{
	public:
		virtual double getVolume(void);
		virtual Eigen::VectorXd getShapeFunctionValues(const Eigen::Vector3d localCoordinates) const;
		virtual Eigen::MatrixXd getShapeFunctionDerivatives(const Eigen::Vector3d localCoordinates) const;
};

#endif
