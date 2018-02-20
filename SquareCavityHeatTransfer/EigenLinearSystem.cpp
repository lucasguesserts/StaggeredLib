#include <SquareCavityHeatTransfer/EigenLinearSystem.hpp>
#include <Eigen/LU>

void EigenLinearSystem::setSize(const unsigned size)
{
	this->matrix.resize(size,size);
	this->independent.resize(size);
	return;
}

Eigen::VectorXd EigenLinearSystem::solve(void)
{
	return this->matrix.fullPivLu().solve(this->independent);
}
