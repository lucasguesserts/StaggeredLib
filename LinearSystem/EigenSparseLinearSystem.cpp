#include <LinearSystem/EigenSparseLinearSystem.hpp>
#include <Eigen/SparseLU>

EigenSparseLinearSystem::EigenSparseLinearSystem(const unsigned size)
	: independent(Eigen::VectorXd::Zero(size)), matrix(size,size)
{}

Eigen::VectorXd EigenSparseLinearSystem::solve(void)
{
	this->matrix.setFromTriplets(this->coefficients.begin(), this->coefficients.end());
	this->matrix.makeCompressed();
	Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
	solver.analyzePattern(this->matrix);
	solver.factorize(this->matrix);
	return solver.solve(this->independent);
}

void EigenSparseLinearSystem::addScalarStencil(const unsigned line, const ScalarStencil& scalarStencil)
{
	this->coefficients.reserve(this->coefficients.size() + scalarStencil.size());
	for(auto& keyValuePair: scalarStencil)
		this->coefficients.emplace_back( Eigen::Triplet<double>(line, keyValuePair.first, keyValuePair.second) );
	return;
}