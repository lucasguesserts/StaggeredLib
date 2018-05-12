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

TestCase("Facet center method - gradient on faces", "[FacetCenterHeatTransfer]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "two_triangles.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	FacetCenterHeatTransfer problem(gridData);
	std::array<double,3> auxuliarValue = {0.36939806252/2.0, 0.26120387496/2.0, 0.5/2.0};
	section("triangle 1")
	{
		VectorStencil gradientOnTriangle = {
			{ 0 , { auxuliarValue[2]-auxuliarValue[0], -auxuliarValue[2],                  0.0} },
			{ 1 , { auxuliarValue[2],                   auxuliarValue[0]-auxuliarValue[2], 0.0} },
			{ 2 , {-auxuliarValue[1],                   auxuliarValue[1],                  0.0} },
			{ 3 , { 0.0,                                auxuliarValue[0],                  0.0} },
			{ 4 , {-auxuliarValue[0],                   0.0,                               0.0} },
		};
		for(unsigned faceIndex=0 ; faceIndex<3 ; ++faceIndex)
			check(problem.gradientOnFaces[faceIndex]==gradientOnTriangle);
	}
	section("triangle 1")
	{
		VectorStencil gradientOnTriangle = {
			{ 0 , { 0.0,                               -auxuliarValue[0],                  0.0} },
			{ 1 , { auxuliarValue[0],                   0.0,                               0.0} },
			{ 2 , { auxuliarValue[1],                  -auxuliarValue[1],                  0.0} },
			{ 3 , { auxuliarValue[0]-auxuliarValue[2],  auxuliarValue[2],                  0.0} },
			{ 4 , {-auxuliarValue[2],                   auxuliarValue[2]-auxuliarValue[0], 0.0} },
		};
		for(unsigned faceIndex=3 ; faceIndex<6 ; ++faceIndex)
			check(problem.gradientOnFaces[faceIndex]==gradientOnTriangle);
	}
}

TestCase("Facet center method - linear system assembly for diffusive term", "[FacetCenterHeatTransfer]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "two_triangles.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	FacetCenterHeatTransfer problem(gridData);
	const unsigned numberOfStaggeredElements = problem.grid2D.staggeredElements.size();
	section("diffusion scalar stencil on faces")
	{
		std::vector<ScalarStencil> diffusionOnFaces = {
			{ {0, -0.376867312493333}, {1, -0.253734624986667}, {2,  0.261203874960000}, {3,  0.246265375013333}, {4,  0.123132687506667} },
			{ {0,  0.123132687506667}, {1, -0.123132687506667}, {2,  0.000000000000000}, {3, -0.123132687506667}, {4,  0.123132687506667} },
			{ {0,  0.253734624986667}, {1,  0.376867312493333}, {2, -0.261203874960000}, {3, -0.123132687506667}, {4, -0.246265375013333} },
			{ {0, -0.123132687506667}, {1, -0.246265375013333}, {2, -0.261203874960000}, {3,  0.253734624986667}, {4,  0.376867312493333} },
			{ {0,  0.246265375013333}, {1,  0.123132687506667}, {2,  0.261203874960000}, {3, -0.376867312493333}, {4, -0.253734624986667} },
			{ {0, -0.123132687506667}, {1,  0.123132687506667}, {2,  0.000000000000000}, {3,  0.123132687506667}, {4, -0.123132687506667} }
		};
		require(problem.grid2D.faces.size()==diffusionOnFaces.size());
		for(unsigned i=0 ; i<problem.grid2D.faces.size() ; ++i)
		{
			Face2D& face = problem.grid2D.faces[i];
			ScalarStencil heatDiffusion = face.getAreaVector() * problem.gradientOnFaces[face.getIndex()];
			check(heatDiffusion==diffusionOnFaces[i]);
		}
	}
	section("matrix")
	{
		Eigen::MatrixXd matrix(numberOfStaggeredElements,numberOfStaggeredElements);
		matrix <<
			-0.500000000000000, -0.130601937480000,  0.261203874960000,  0.369398062520000,  0.000000000000000,
			-0.130601937480000, -0.500000000000000,  0.261203874960000,  0.000000000000000,  0.369398062520000,
			 0.261203874960000,  0.261203874960000, -1.044815499840000,  0.261203874960000,  0.261203874960000,
			 0.369398062520000,  0.000000000000000,  0.261203874960000, -0.500000000000000, -0.130601937480000,
			 0.000000000000000,  0.369398062520000,  0.261203874960000, -0.130601937480000, -0.500000000000000;
		problem.addDiffusiveTerm();
		check(problem.linearSystem.matrix==matrix);
	}
	section("independent")
	{
		problem.addDiffusiveTerm();
		Eigen::VectorXd independent = Eigen::VectorXd::Zero(numberOfStaggeredElements);
		check(problem.linearSystem.independent==independent);
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