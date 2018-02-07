#include <Grid/Triangle.hpp>

Eigen::Vector3d Triangle::getAreaVector(void)
{
	std::vector<const Vertex *> vertices = this->getVertices();
	return this->computeTriangleAreaVector(vertices[0], vertices[1], vertices[2]);
}

double Triangle::getVolume(void)
{
	return this->getAreaVector().norm();
}

Eigen::VectorXd Triangle::getShapeFunctionValues(const Eigen::Vector3d localCoordinates) const
{
	const double xi = localCoordinates[0];
	const double eta = localCoordinates[1];
	Eigen::Vector3d shapeFunctionValues;
	shapeFunctionValues[0] = 1 - xi - eta;
	shapeFunctionValues[1] = xi;
	shapeFunctionValues[2] = eta;
	return shapeFunctionValues;
}

Eigen::MatrixXd Triangle::getShapeFunctionDerivatives(const Eigen::Vector3d localCoordinates) const
{
	Eigen::Matrix3d shapeFunctionDerivatives;
	shapeFunctionDerivatives << -1.0, -1.0,  0.0,
	                             1.0,  0.0,  0.0,
	                             0.0,  1.0,  0.0;
	return shapeFunctionDerivatives;
}
