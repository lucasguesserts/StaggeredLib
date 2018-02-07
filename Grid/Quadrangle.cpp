#include <Grid/Quadrangle.hpp>

Eigen::Vector3d Quadrangle::getAreaVector(void)
{
	std::vector<const Vertex *> vertices = this->getVertices();
	Eigen::Vector3d firstTriangle = this->computeTriangleAreaVector(vertices[0], vertices[1], vertices[2]);
	Eigen::Vector3d secondTriangle = this->computeTriangleAreaVector(vertices[0], vertices[2], vertices[3]);
	return firstTriangle + secondTriangle;
}

double Quadrangle::getVolume(void)
{
	return this->getAreaVector().norm();
}

Eigen::VectorXd Quadrangle::getShapeFunctionValues(const Eigen::Vector3d localCoordinates) const
{
	const double xi = localCoordinates[0];
	const double eta = localCoordinates[1];
	Eigen::Vector4d shapeFunctionValues;
	shapeFunctionValues[0] = (1-xi)*(1-eta);
	shapeFunctionValues[1] = xi*(1-eta);
	shapeFunctionValues[2] = xi*eta;
	shapeFunctionValues[3] = (1-xi)*eta;
	return shapeFunctionValues;
}

Eigen::MatrixXd Quadrangle::getShapeFunctionDerivatives(const Eigen::Vector3d localCoordinates) const
{
	const double xi = localCoordinates[0];
	const double eta = localCoordinates[1];
	Eigen::Matrix<double,4,3> shapeFunctionDerivatives;
	shapeFunctionDerivatives << (eta-1), (xi-1), 0,
	                            (1-eta), -xi,    0,
	                             eta,     xi,    0,
	                            -eta,    (1-xi), 0;
	return shapeFunctionDerivatives;
}
