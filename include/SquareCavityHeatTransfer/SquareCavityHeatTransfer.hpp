#include <Eigen/Core>
#include <vector>
#include <Grid/Grid2D.hpp>
#include <Grid/GridBuilder.hpp>

class SquareCavityHeatTransfer
{
	public:
		static Eigen::VectorXd computeAnalyticalSolution(const Eigen::Matrix<double,Eigen::Dynamic,3> coordinates);
};
