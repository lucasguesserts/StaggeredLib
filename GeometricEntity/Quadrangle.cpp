#include <GeometricEntity/Quadrangle.hpp>

Eigen::Vector3d Quadrangle::getCentroid(void)
{
	Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
	for(const Vertex* vertex: this->vertices)
		centroid += *vertex;
	centroid /= 4;
	return centroid;
}

Eigen::Vector3d Quadrangle::getAreaVector(void)
{
	return Element::computeTriangleAreaVector(*(this->vertices[0]), *(this->vertices[1]), *(this->vertices[2])) +
	       Element::computeTriangleAreaVector(*(this->vertices[0]), *(this->vertices[2]), *(this->vertices[3]));
}

double Quadrangle::getVolume(void)
{
	return this->getAreaVector().norm();
}

Eigen::VectorXd Quadrangle::getShapeFunctionValues(const Eigen::Vector3d localCoordinates)
{
	const double xi = localCoordinates[0];
	const double eta = localCoordinates[1];
	Eigen::VectorXd shapeFunctionValues(4);
	shapeFunctionValues[0] = (1-xi)*(1-eta);
	shapeFunctionValues[1] = xi*(1-eta);
	shapeFunctionValues[2] = xi*eta;
	shapeFunctionValues[3] = (1-xi)*eta;
	return shapeFunctionValues;
}

Eigen::MatrixXd Quadrangle::getShapeFunctionDerivatives(const Eigen::Vector3d localCoordinates)
{
	const double xi = localCoordinates[0];
	const double eta = localCoordinates[1];
	Eigen::MatrixXd shapeFunctionDerivatives(4,3);
	shapeFunctionDerivatives << (eta-1), (xi-1), 0,
	                            (1-eta), -xi,    0,
	                             eta,     xi,    0,
	                            -eta,    (1-xi), 0;
	return shapeFunctionDerivatives;
}