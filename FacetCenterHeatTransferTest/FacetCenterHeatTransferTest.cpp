#include <Utils/TestMain.hpp>
#include <Utils/EigenTest.hpp>

#include <FacetCenterHeatTransfer/FacetCenterHeatTransfer.hpp>

TestCase("Gradient matrices")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "two_triangles.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	// FacetCenterHeatTransfer problem(gridData);
	// section("triangle 0")
	// {
	// 	Eigen::MatrixXd gradientMatrix(3,3);
	// 	gradientMatrix << -0.50000,  0.50000, 0.00000,
	// 	                   0.00000, -0.50000, 0.50000,
	// 	                   0.00000,  0.00000, 0.00000;
	// 	for(unsigned faceIndex=0 ; faceIndex<3 ; ++faceIndex)
	// 	{
	// 		Face2D& face = problem.grid2D.faces[faceIndex];
	// 		check(problem.computeGradientMatrix(face)==gradientMatrix);
	// 	}
	// }
	// section("triangle 0")
	// {
	// 	Eigen::MatrixXd gradientMatrix(3,3);
	// 	gradientMatrix <<  0.00000, 0.50000, -0.50000,
	// 	                  -0.50000, 0.00000,  0.50000,
	// 	                   0.00000, 0.00000,  0.00000;
	// 	for(unsigned faceIndex=3 ; faceIndex<6 ; ++faceIndex)
	// 	{
	// 		Face2D& face = problem.grid2D.faces[faceIndex];
	// 		check(problem.computeGradientMatrix(face)==gradientMatrix);
	// 	}
	// }
}