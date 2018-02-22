#ifndef EIGEN_LINEAR_SYSTEM_HPP
#define EIGEN_LINEAR_SYSTEM_HPP

#include <Eigen/Core>
#include <SquareCavityHeatTransfer/ScalarStencil.hpp>

class EigenLinearSystem
{
	public:
		void setSize(const unsigned size);
		Eigen::VectorXd solve(void);
		void addScalarStencil(const unsigned line, const ScalarStencil& scalarStencil);

		Eigen::MatrixXd matrix;
		Eigen::VectorXd independent;
};

#endif
