#include <Grid/Triangle.hpp>

Eigen::Vector3d Triangle::getAreaVector(void) const
{
	return Eigen::Vector3d(0.0, 0.0, 0.0);
}

double Triangle::getVolume(void) const
{
	return 0;
}

Eigen::VectorXd Triangle::getShapeFunctionValues(const Eigen::Vector3d localCoordinates) const
{
	return Eigen::Vector3d(0.0, 0.0, 0.0);
}

Eigen::MatrixXd Triangle::getShapeFunctionDerivatives(const Eigen::Vector3d localCoordinates) const
{
	Eigen::Matrix3d matrix;
	matrix << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	return matrix;
}
