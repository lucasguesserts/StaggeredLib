#ifndef EIGEN_LINEAR_SYSTEM_HPP
#define EIGEN_LINEAR_SYSTEM_HPP

#include <Eigen/Core>

class EigenLinearSystem
{
	public:
		void setSize(const unsigned size);
		Eigen::VectorXd solve(void);

		Eigen::MatrixXd matrix;
		Eigen::VectorXd independent;
};

#endif
