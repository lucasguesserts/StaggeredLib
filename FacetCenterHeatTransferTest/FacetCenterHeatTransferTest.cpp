#include <Utils/TestMain.hpp>
#include <Utils/EigenTest.hpp>
#include <Stencil/Test.hpp>

#include <FacetCenterHeatTransfer/FacetCenterHeatTransfer.hpp>

TestCase("Facet center method - gradient matrices", "[FacetCenterHeatTransfer]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "two_triangles.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	FacetCenterHeatTransfer problem(gridData);
	section("triangle 0")
	{
		Eigen::MatrixXd gradientMatrix(3,3);
		gradientMatrix << -0.5,  0.5, 0.0,
		                   0.0, -0.5, 0.5,
		                   0.0,  0.0, 0.0;
		for(unsigned faceIndex=0 ; faceIndex<3 ; ++faceIndex)
		{
			Face2D& face = problem.grid2D.faces[faceIndex];
			check(problem.computeGradientMatrix(face)==gradientMatrix);
		}
	}
	section("triangle 0")
	{
		Eigen::MatrixXd gradientMatrix(3,3);
		gradientMatrix <<  0.0, 0.5, -0.5,
		                  -0.5, 0.0,  0.5,
		                   0.0, 0.0,  0.0;
		for(unsigned faceIndex=3 ; faceIndex<6 ; ++faceIndex)
		{
			Face2D& face = problem.grid2D.faces[faceIndex];
			check(problem.computeGradientMatrix(face)==gradientMatrix);
		}
	}
}

TestCase("Facet center method - scalar stencil on element vertices", "[FacetCenterHeatTransfer]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "two_triangles.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	FacetCenterHeatTransfer problem(gridData);
	section("triangle 0")
	{
		std::vector<ScalarStencil> correct = {
			{{0,0.5},{3,0.5}},
			{{1,1.0}},
			{{2,0.5},{4,0.5}}
		};
		for(unsigned faceIndex=0 ; faceIndex<3 ; ++faceIndex)
		{
			Face2D& face = problem.grid2D.faces[faceIndex];
			std::vector<ScalarStencil> scalarStencilOnElementVertices = problem.getScalarStencilOnElementVertices(face);
			check(scalarStencilOnElementVertices==correct);
		}
	}
	section("triangle 1")
	{
		std::vector<ScalarStencil> correct = {
			{{0,0.5},{3,0.5}},
			{{2,0.5},{4,0.5}},
			{{5,1.0}}
		};
		for(unsigned faceIndex=3 ; faceIndex<6 ; ++faceIndex)
		{
			Face2D& face = problem.grid2D.faces[faceIndex];
			std::vector<ScalarStencil> scalarStencilOnElementVertices = problem.getScalarStencilOnElementVertices(face);
			check(scalarStencilOnElementVertices==correct);
		}
	}
}