#include <vector>
#include <string>
#include <Eigen/Core>
#include <Grid/GridData.hpp>
#include <Grid/Grid2D.hpp>
#include <SquareCavityHeatTransfer/EigenLinearSystem.hpp>

class SquareCavityHeatTransfer
{
	public:
		SquareCavityHeatTransfer(const GridData& gridData);
		void addAccumulationTerm(void);

		static Eigen::VectorXd computeAnalyticalSolution(const Eigen::Matrix<double,Eigen::Dynamic,3> coordinates);

		double rho, cp;
		Grid2D grid2D;
		EigenLinearSystem linearSystem;
		Eigen::VectorXd oldTemperature;
		Eigen::VectorXd temperature;

};
