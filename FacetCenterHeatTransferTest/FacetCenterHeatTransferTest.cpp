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
			{ {0,0.36939806252}, {2,0.26120387496}, {4,0.36939806252} },
			{ {0,0.5},           {1,0.5} },
			{ {1,0.36939806252}, {2,0.26120387496}, {3,0.36939806252} }
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
			{ {0,0.36939806252}, {2,0.26120387496}, {4,0.36939806252} },
			{ {1,0.36939806252}, {2,0.26120387496}, {3,0.36939806252} },
			{ {3,0.5},           {4,0.5} },
		};
		for(unsigned faceIndex=3 ; faceIndex<6 ; ++faceIndex)
		{
			Face2D& face = problem.grid2D.faces[faceIndex];
			std::vector<ScalarStencil> scalarStencilOnElementVertices = problem.getScalarStencilOnElementVertices(face);
			check(scalarStencilOnElementVertices==correct);
		}
	}
}

TestCase("Facet center method - apply boundary conditions", "[FacetCenterHeatTransfer]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "two_triangles.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	FacetCenterHeatTransfer problem(gridData);
	const unsigned numberOfStaggeredElements = problem.grid2D.staggeredElements.size();
	problem.linearSystem.matrix = Eigen::MatrixXd::Random(numberOfStaggeredElements,numberOfStaggeredElements);
	DirichletBoundaryCondition dirichlet;
	auto addBoudaryWithRandomPrescribedValues = [&](const std::string& boundaryName) {
		dirichlet.staggeredTriangle = problem.grid2D.boundary[boundaryName].staggeredTriangle;
		dirichlet.prescribedValue.resize(dirichlet.staggeredTriangle.size());
		for(auto& entry: dirichlet.prescribedValue)
			entry = static_cast<double>(std::rand());
		problem.dirichletBoundaries.push_back(dirichlet);
	};
	addBoudaryWithRandomPrescribedValues("bottom boundary");
	addBoudaryWithRandomPrescribedValues("top boundary");
	addBoudaryWithRandomPrescribedValues("east boundary");
	addBoudaryWithRandomPrescribedValues("west boundary");
	problem.applyBoundaryConditions();
	for(unsigned i=0 ; i<dirichlet.staggeredTriangle.size() ; ++i)
	{
		unsigned row = dirichlet.staggeredTriangle[i]->getIndex();
		for(unsigned col=0 ; col<problem.linearSystem.matrix.cols() ; ++col)
			if(row==col)
				check(problem.linearSystem.matrix(row,col)==1.0);
			else
				check(problem.linearSystem.matrix(row,col)==0.0);
		check(problem.linearSystem.independent(row)==dirichlet.prescribedValue[i]);
	}
}