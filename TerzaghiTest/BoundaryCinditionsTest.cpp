#include <Utils/Test.hpp>
#include <Utils/EigenTest.hpp>
#include <Stencil/Test.hpp>

#include <Terzaghi/Terzaghi.hpp>
#include <Terzaghi/TerzaghiBoundary.hpp>

TestCase("Terzaghi boundary use", "[TerzaghiBoundary]")
{
	TerzaghiBoundary boundary;
	boundary.prescribedValue[2] = 5.7;
	boundary.component[1] = Component::P;
	boundary.boundaryConditionType[0] = BoundaryConditionType::Dirichlet;
	check(boundary.prescribedValue[2]==5.7);
	check(boundary.component[1]==Component::P);
	check(boundary.boundaryConditionType[0]==BoundaryConditionType::Dirichlet);
}

TestCase("Terzaghi boundary conditions build", "[Terzaghi][TerzaghiBoundary]")
// It tests just some boundaries, not all of them
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	section("bottom")
	{
		check(terzaghi.boundary[0].name==std::string("bottom boundary"));
		check(terzaghi.boundary[0].component[1]==Component::U);
		check(terzaghi.boundary[0].boundaryConditionType[2]==BoundaryConditionType::Dirichlet);
		check(terzaghi.boundary[0].prescribedValue[0]==0.0);
	}
	section("west")
	{
		check(terzaghi.boundary[3].name==std::string("west boundary"));
		check(terzaghi.boundary[3].component[2]==Component::V);
		check(terzaghi.boundary[3].boundaryConditionType[0]==BoundaryConditionType::Neumann);
		check(terzaghi.boundary[3].prescribedValue[1]==0.0);
	}
}