#ifndef EIGEN_LINEAR_SYSTEM_HPP
#define EIGEN_LINEAR_SYSTEM_HPP

#include <Eigen/Core>
#include <Stencil/ScalarStencil.hpp>
#include <Eigen/LU>

class EigenLinearSystem
{
	public:
		void setSize(const unsigned size);
		void computeLU(void);
		Eigen::VectorXd solve(void);
		Eigen::VectorXd solveDecomposed(void);
		void addScalarStencil(const unsigned line, const ScalarStencil& scalarStencil);

		Eigen::MatrixXd matrix;
		Eigen::VectorXd independent;
		Eigen::FullPivLU<Eigen::MatrixXd> luDecomposition;
};

#endif
