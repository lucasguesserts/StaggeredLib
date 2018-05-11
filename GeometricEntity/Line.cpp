#include <GeometricEntity/Line.hpp>

const std::array<Eigen::Vector3d,2> Line::staggeredElementFaceCentroidLocalIndex = {{
			{1.0/4.0, 0.0, 0.0},
			{3.0/4.0, 0.0, 0.0}
}};

Eigen::Vector3d Line::getCentroid(void)
{
	Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
	for(const Vertex* vertex: this->vertices)
		centroid += *vertex;
	centroid /= 2;
	return centroid;
}

Eigen::Vector3d Line::getAreaVector(void)
{
	Eigen::Vector3d areaVector = *(this->vertices[1]) - *(this->vertices[0]);
	std::swap(areaVector[0], areaVector[1]);
	areaVector[1] = - areaVector[1];
	return areaVector;
}

double Line::getVolume(void)
{
	return (*(this->vertices[1]) - *(this->vertices[0])).norm();
}

Eigen::VectorXd Line::getShapeFunctionValues(const Eigen::Vector3d localCoordinates)
{
	const double xi = localCoordinates[0];
	Eigen::VectorXd shapeFunctionValues(2);
	shapeFunctionValues[0] = 1 - xi;
	shapeFunctionValues[1] = xi;
	return shapeFunctionValues;
}

Eigen::MatrixXd Line::getShapeFunctionDerivatives(const Eigen::Vector3d localCoordinates)
{
	Eigen::MatrixXd shapeFunctionDerivatives(2,3);
	shapeFunctionDerivatives << -1.0, 0.0, 0.0,
	                             1.0, 0.0, 0.0;
	return shapeFunctionDerivatives;
}

Eigen::Vector3d Line::getFaceLocalCoordinates(const unsigned faceLocalIndex)
{
	return staggeredElementFaceCentroidLocalIndex[faceLocalIndex];
}