#include <LinearSystem/EigenLinearSystem.hpp>

void EigenLinearSystem::setSize(const unsigned size)
{
	this->matrix = Eigen::MatrixXd::Zero(size,size);
	this->independent = Eigen::VectorXd::Zero(size);
	return;
}

void EigenLinearSystem::addScalarStencil(const unsigned line, const ScalarStencil& scalarStencil)
{
	for(auto& keyValuePair: scalarStencil)
	{
		const unsigned key = keyValuePair.first;
		const double value = keyValuePair.second;
		this->matrix(line,key) += value;
	}
	return;
}

Eigen::VectorXd EigenLinearSystem::solve(void)
{
	return this->matrix.fullPivLu().solve(this->independent);
}

Eigen::VectorXd EigenLinearSystem::solveDecomposed(void)
{
	return this->luDecomposition.solve(this->independent);
}

void EigenLinearSystem::computeLU(void)
{
	this->luDecomposition.compute(this->matrix);
	return;
}