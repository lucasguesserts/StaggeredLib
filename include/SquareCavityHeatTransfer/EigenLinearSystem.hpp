#include <Eigen/Core>

class EigenLinearSystem
{
	public:
		void setSize(const unsigned size);
		Eigen::VectorXd solve(void);

		Eigen::MatrixXd matrix;
		Eigen::VectorXd independent;
};
