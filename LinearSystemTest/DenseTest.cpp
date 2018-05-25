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

TestCase("Add scalar stencil to eigen linear system", "[ScalarStencil][EigenLinearSystem]")
{
	const unsigned size = 3;
	std::vector<ScalarStencil> scalarStencil = {
		{{0, 0.6}, {1, 1.9}, {2, 7.6}},
		{{2, 8.9}, {0, 5.2}, {1, 8.4}},
		{{2, 3.7}, {1, 2.0}}
	};
	EigenLinearSystem linearSystem;
	linearSystem.setSize(size);
	linearSystem.matrix = Eigen::MatrixXd::Ones(size,size);
	for(unsigned line=0 ; line<size ; ++line)
		linearSystem.addScalarStencil(line,scalarStencil[line]);
	Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(size,size);
	matrix << 1+scalarStencil[0][0], 1+scalarStencil[0][1], 1+scalarStencil[0][2],
	          1+scalarStencil[1][0], 1+scalarStencil[1][1], 1+scalarStencil[1][2],
	          1+scalarStencil[2][0], 1+scalarStencil[2][1], 1+scalarStencil[2][2];
	check(linearSystem.matrix==matrix);
}