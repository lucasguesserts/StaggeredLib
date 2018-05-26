#include <LinearSystem/EigenSparseLinearSystem.hpp>
#include <Eigen/SparseLU>

EigenSparseLinearSystem::EigenSparseLinearSystem(const unsigned size)
	: independent(Eigen::VectorXd::Zero(size)), matrix(size,size)
{}

void EigenSparseLinearSystem::addScalarStencil(const unsigned line, const ScalarStencil& scalarStencil)
{
	this->coefficients.reserve(this->coefficients.size() + scalarStencil.size());
	for(auto& keyValuePair: scalarStencil)
		this->coefficients.emplace_back( Eigen::Triplet<double,unsigned>(line, keyValuePair.first, keyValuePair.second) );
	return;
}

void EigenSparseLinearSystem::computeLU(void)
{
	this->matrix.setFromTriplets(this->coefficients.begin(), this->coefficients.end());
	this->matrix.makeCompressed();
	this->luDecomposition.analyzePattern(this->matrix);
	this->luDecomposition.factorize(this->matrix);
	return;
}

Eigen::VectorXd EigenSparseLinearSystem::solve(void)
{
	return luDecomposition.solve(this->independent);
}