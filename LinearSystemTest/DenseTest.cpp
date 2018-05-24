#include <Utils/Test.hpp>
#include <Utils/EigenTest.hpp>
#include <LinearSystem/EigenLinearSystem.hpp>

TestCase("Linear system build", "[Eigen][EigenSolver]")
{
	const unsigned linearSystemSize = 3;
	EigenLinearSystem linearSystem;
	linearSystem.setSize(linearSystemSize);
	linearSystem.matrix << 0.462026696884809, 0.289318074606471, 0.418654423757951,
	                       0.111618554500326, 0.846885511973542, 0.805839771614926,
	                       0.664857365801401, 0.240344399557209, 0.348561451918039;
	linearSystem.independent << 0.263941635488370, 0.246987790661933, 0.706521526629329;
	Eigen::VectorXd solution(linearSystemSize);
	solution << 1.74452683050882, 3.77822561620858, -3.90581157526298;
	Eigen::VectorXd linearSystemSolution = linearSystem.solve();
	check(solution==linearSystemSolution);
}