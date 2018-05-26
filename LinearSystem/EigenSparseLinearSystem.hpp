#ifndef EIGEN_SPARSE_LINEAR_SYSTEM_HPP
#define EIGEN_SPARSE_LINEAR_SYSTEM_HPP

#include <vector>
#include <Eigen/Core>
#include <Eigen/Sparse>

#include <Stencil/ScalarStencil.hpp>

class EigenSparseLinearSystem
{
	public:
		EigenSparseLinearSystem(void) = default;
		EigenSparseLinearSystem(const unsigned size);
		std::vector<Eigen::Triplet<double,unsigned>> coefficients; // column major
		Eigen::VectorXd independent;
		Eigen::SparseMatrix<double> matrix;
		Eigen::SparseLU<Eigen::SparseMatrix<double>> luDecomposition;

		void addScalarStencil(const unsigned line, const ScalarStencil& scalarStencil);
		void computeLU(void);
		Eigen::VectorXd solve(void);
};

#endif