#include <GeometricEntity/Triangle.hpp>

const std::array<Eigen::Vector3d,3> Triangle::staggeredElementFaceCentroidLocalIndex = {{
			{1.0/6.0, 1.0/6.0, 0.0},
			{2.0/3.0, 1.0/6.0, 0.0},
			{1.0/6.0, 2.0/3.0, 0.0}
}};

Eigen::Vector3d Triangle::getCentroid(void)
{
	Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
	for(const Vertex* vertex: this->vertices)
		centroid += *vertex;
	centroid /= 3;
	return centroid;
}

Eigen::Vector3d Triangle::getAreaVector(void)
{
	return Element::computeTriangleAreaVector(*(this->vertices[0]), *(this->vertices[1]), *(this->vertices[2]));
}

double Triangle::getVolume(void)
{
	return this->getAreaVector().norm();
}

Eigen::VectorXd Triangle::getShapeFunctionValues(const Eigen::Vector3d localCoordinates)
{
	const double xi = localCoordinates[0];
	const double eta = localCoordinates[1];
	Eigen::VectorXd shapeFunctionValues(3);
	shapeFunctionValues[0] = 1 - xi - eta;
	shapeFunctionValues[1] = xi;
	shapeFunctionValues[2] = eta;
	return shapeFunctionValues;
}

Eigen::MatrixXd Triangle::getShapeFunctionDerivatives(const Eigen::Vector3d localCoordinates)
{
	Eigen::MatrixXd shapeFunctionDerivatives(3,3);
	shapeFunctionDerivatives << -1.0, -1.0,  0.0,
	                             1.0,  0.0,  0.0,
	                             0.0,  1.0,  0.0;
	return shapeFunctionDerivatives;
}