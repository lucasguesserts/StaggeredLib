#include <Utils/Test.hpp>
#include <SquareCavityHeatTransfer/SquareCavityHeatTransfer.hpp>

TestCase("Analytical solution", "[SquareCavityHeatTransfer]")
{
	const unsigned numberOfPoints = 4;
	Eigen::Matrix<double,4,3> coordinates;
	coordinates << 0.1, 0.1, 0.0,
	               0.5, 0.4, 0.0,
	               0.9, 1.0, 0.0,
	               0.7, 3.0, 0.0;
	Eigen::Vector4d solution, correctSolution;
	correctSolution << 0.008545119856,
	                   0.1397977728,
	                   0.3090169944,
	                   434.0323775;
	solution = SquareCavityHeatTransfer::computeAnalyticalSolution(coordinates);
	for(unsigned pointIndex=0 ; pointIndex<numberOfPoints ; ++pointIndex)
	{
		check(solution[pointIndex]==Approx(correctSolution[pointIndex]));
	}
}
