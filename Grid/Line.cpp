#include <Grid/Line.hpp>

double Line::getVolume(void)
{
	return (*(this->vertices[1]) - *(this->vertices[0])).norm();
}

Eigen::VectorXd Line::getShapeFunctionValues(const Eigen::Vector3d localCoordinates) const
{
	const double xi = localCoordinates[0];
	Eigen::Vector2d shapeFunctionValues;
	shapeFunctionValues[0] = 1 - xi;
	shapeFunctionValues[1] = xi;
	return shapeFunctionValues;
}

Eigen::MatrixXd Line::getShapeFunctionDerivatives(const Eigen::Vector3d localCoordinates) const
{
	Eigen::Matrix<double,2,3> shapeFunctionDerivatives;
	shapeFunctionDerivatives << -1.0, 0.0, 0.0,
	                             1.0, 0.0, 0.0;
	return shapeFunctionDerivatives;
}
