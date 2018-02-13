#include <Eigen/Core>

class SquareCavityHeatTransfer
{
	public:
		static Eigen::VectorXd computeAnalyticalSolution(const Eigen::Matrix<double,Eigen::Dynamic,3> coordinates);
};
