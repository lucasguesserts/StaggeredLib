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

TestCase("Displacement gradient on staggered triangle", "[Terzaghi]")
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	section("staggered triangle 0")
	{
		auto staggeredTriangle = terzaghi.grid.staggeredTriangles[0];
		VectorStencil gradient = {
			{0, {-0.5, -1.5, 0.0}},
			{1, {0.5, 0.5, 0.0}},
			{2, {0.0, 1.0, 0.0}}
		};
		auto staggeredTriangleGradient = terzaghi.getDisplacementGradientOnStaggeredTriangle(staggeredTriangle);
		check(staggeredTriangleGradient==gradient);
	}
	section("staggered triangle 1")
	{
		auto staggeredTriangle = terzaghi.grid.staggeredTriangles[1];
		VectorStencil gradient = {
			{0, {-0.5, -0.5, 0.0}},
			{1, {1.5, 0.5, 0.0}},
			{2, {-1.0, 0.0, 0.0}}
		};
		auto staggeredTriangleGradient = terzaghi.getDisplacementGradientOnStaggeredTriangle(staggeredTriangle);
		check(staggeredTriangleGradient==gradient);
	}
	section("staggered triangle 3")
	{
		auto staggeredTriangle = terzaghi.grid.staggeredTriangles[2];
		VectorStencil gradient = {
			{2, {0.0, -1.0, 0.0}},
			{3, {0.5, 1.5, 0.0}},
			{4, {-0.5, -0.5, 0.0}}
		};
		auto staggeredTriangleGradient = terzaghi.getDisplacementGradientOnStaggeredTriangle(staggeredTriangle);
		check(staggeredTriangleGradient==gradient);
	}
	section("staggered triangle 4")
	{
		auto staggeredTriangle = terzaghi.grid.staggeredTriangles[3];
		VectorStencil gradient = {
			{2, {1.0, 0.0, 0.0}},
			{3, {0.5, 0.5, 0.0}},
			{4, {-1.5, -0.5, 0.0}}
		};
		auto staggeredTriangleGradient = terzaghi.getDisplacementGradientOnStaggeredTriangle(staggeredTriangle);
		check(staggeredTriangleGradient==gradient);
	}
}

TestCase("Terzaghi apply displacement dirichlet boundary condition to staggered triangle", "[Terzaghi]")
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	terzaghi.poissonCoefficient = 0.25;
	terzaghi.shearModulus = 1;
	terzaghi.insertDisplacementTensionTermInMatrix();
	section("component U")
	{
		for(auto staggeredTriangle: terzaghi.boundary[2].staggeredTriangle) // east
			terzaghi.applyDisplacementDirichletBoundaryCondition(Component::U, staggeredTriangle);
		// check coefficients
		for(auto staggeredTriangle: terzaghi.boundary[2].staggeredTriangle) // east
		{
			const unsigned row = terzaghi.transformIndex(Component::U, staggeredTriangle);
			for(auto& triplet: terzaghi.linearSystem.coefficients)
				if((triplet.row()==row) && (triplet.col()!=row))
					check(triplet.value()==0.0);
		}
	}
	section("component V")
	{
		for(auto staggeredTriangle: terzaghi.boundary[0].staggeredTriangle) // bottom
			terzaghi.applyDisplacementDirichletBoundaryCondition(Component::V, staggeredTriangle);
		// check coefficients
		for(auto staggeredTriangle: terzaghi.boundary[0].staggeredTriangle) // bottom
		{
			const unsigned row = terzaghi.transformIndex(Component::V, staggeredTriangle);
			for(auto& triplet: terzaghi.linearSystem.coefficients)
				if((triplet.row()==row) && (triplet.col()!=row))
					check(triplet.value()==0.0);
		}
	}
}

TestCase("Terzaghi insert displacement dirichlet boundary condition to matrix", "[Terzaghi]")
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	terzaghi.poissonCoefficient = 0.25;
	terzaghi.shearModulus = 1;
	terzaghi.insertDisplacementTensionTermInMatrix();
	terzaghi.insertDisplacementDirichletBoundaryConditionToMatrix();
	auto checkIfBoundaryConditionWasApplied = [&](Component component, StaggeredElement2D* staggeredElement) -> void
	{
		const unsigned row = terzaghi.transformIndex(component, staggeredElement);
		for(auto& triplet: terzaghi.linearSystem.coefficients)
			if((triplet.row()==row) && (triplet.col()!=row))
				check(triplet.value()==0.0);
	};
	checkIfBoundaryConditionWasApplied(Component::U, &(terzaghi.grid.staggeredElements[1]));
	checkIfBoundaryConditionWasApplied(Component::U, &(terzaghi.grid.staggeredElements[4]));
	checkIfBoundaryConditionWasApplied(Component::V, &(terzaghi.grid.staggeredElements[0]));
}