#include <GeometricEntity/Element.hpp>
#include <Eigen/Geometry>
#include <Eigen/Dense>

void Element::addVertex(Vertex& vertex)
{
	this->vertices.push_back(&vertex);
	return;
}

Eigen::Vector3d Element::computeTriangleAreaVector(const Eigen::Vector3d& first, const Eigen::Vector3d& second, const Eigen::Vector3d& third)
{
	return (1.0/2.0) * (second - first).cross(third - first);
}

Eigen::MatrixXd Element::getGradientMatrix2D(const Eigen::Vector3d& localCoordinates)
{
	Eigen::MatrixXd shapeFunctionDerivatives = this->getShapeFunctionDerivatives(localCoordinates);
	Eigen::MatrixXd positionsMatrix = this->getPositionsMatrix();
	Eigen::MatrixXd jacobian = positionsMatrix * shapeFunctionDerivatives;
		const unsigned lastRow = jacobian.rows() - 1;
		const unsigned lastColumn = jacobian.cols() - 1;
		for(unsigned column=0 ; column<jacobian.cols() ; ++column)
			jacobian(lastRow,column) = 0.0;
		for(unsigned row=0 ; row<jacobian.rows() ; ++row)
			jacobian(row,lastColumn) = 0.0;
		jacobian(lastRow,lastColumn) = 1.0;
	return jacobian.inverse().transpose() * shapeFunctionDerivatives.transpose();
}

Eigen::MatrixXd Element::getPositionsMatrix(void)
{
	const unsigned numberOfVertices = this->vertices.size();
	Eigen::MatrixXd positionsMatrix(3,numberOfVertices);
	for(unsigned vertexLocalIndex=0 ; vertexLocalIndex<numberOfVertices ; ++vertexLocalIndex)
	{
		positionsMatrix(0,vertexLocalIndex) = this->vertices[vertexLocalIndex]->coeff(0);
		positionsMatrix(1,vertexLocalIndex) = this->vertices[vertexLocalIndex]->coeff(1);
		positionsMatrix(2,vertexLocalIndex) = this->vertices[vertexLocalIndex]->coeff(2);
	}
	return positionsMatrix;
}