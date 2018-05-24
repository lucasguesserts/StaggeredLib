#ifndef EIGEN_SPARSE_LINEAR_SYSTEM_HPP
#define EIGEN_SPARSE_LINEAR_SYSTEM_HPP

#include <vector>
#include <Eigen/Core>
#include <Eigen/Sparse>

class EigenSparseLinearSystem
{
	public:
		EigenSparseLinearSystem(const unsigned size);
		std::vector<Eigen::Triplet<double>> coefficients; // column major
		Eigen::VectorXd independent;

		Eigen::VectorXd solve(void);
	private:
		Eigen::SparseMatrix<double> matrix;
};

#endif