#include <Utils/Test.hpp>
#include <Utils/contains.hpp>
#include <vector>
#include <string>

#include <CGNSFile/ElementDefinition.hpp>
#include <CGNSFile/CGNSFile.hpp>

#include <GeometricEntity/Test.hpp>
#include <GeometricEntity/Vertex.hpp>
#include <GeometricEntity/Element.hpp>
#include <GeometricEntity/StaggeredQuadrangle.hpp>
#include <GeometricEntity/StaggeredTriangle.hpp>

#include <Grid/StaggeredElementDefinition.hpp>
#include <Grid/Grid2DWithStaggeredElements.hpp>

TestCase("Grid2D with staggered elements - find staggered element definition", "[Grid2DWithStaggeredElements]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "two_triangles.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DWithStaggeredElements grid(gridData);
	section("Find StaggeredElementDefinition")
	{
		StaggeredElementDefinition staggExists(0, 1, 0);
		StaggeredElementDefinition staggDoNotExists(3, 7, 9);
		section("staggered element that exists")
		{
			auto staggeredElementLocation = grid.findStaggeredElementDefinition(staggExists);
			check(std::get<bool>(staggeredElementLocation)==true);
			check(std::get<unsigned>(staggeredElementLocation)==0);
		}
		section("staggered element that does not exists")
		{
			auto staggeredElementLocation = grid.findStaggeredElementDefinition(staggDoNotExists);
			check(std::get<bool>(staggeredElementLocation)==false);
		}
	}
}

TestCase("Grid2D with staggered elements - add staggered element definition", "[Grid2DWithStaggeredElements]")
{
	GridData gridData;
	Grid2DWithStaggeredElements grid(gridData);
	std::vector<StaggeredElementDefinition> staggeredElementDefinitionVector = {
		{1, 5, 4},
		{5, 1 ,9}, // Same as above
		{3, 7, 9}
	};
	grid.staggeredElementDefinition.push_back(staggeredElementDefinitionVector[0]);
	grid.addStaggeredElementDefinition(staggeredElementDefinitionVector[1]);
	grid.addStaggeredElementDefinition(staggeredElementDefinitionVector[2]);
	check(grid.staggeredElementDefinition.size()==2); // One quadrangle and one triangle
	check(grid.staggeredElementDefinition[0]==StaggeredElementDefinition(1, 5, 0));
	check(grid.staggeredElementDefinition[0].elements[0]==4);
	check(grid.staggeredElementDefinition[0].elements[1]==9);
	check(grid.staggeredElementDefinition[1]==StaggeredElementDefinition(3, 7, 9));
}

TestCase("Grid2D with staggered elements - add staggered element definition from element definition", "[Grid2DWithStaggeredElements]")
{
	GridData gridData;
	Grid2DWithStaggeredElements grid(gridData);
	section("quadrangle")
	{
		ElementDefinition<4> quadrangleDefinition;
			quadrangleDefinition.index = 6;
			quadrangleDefinition.connectivity << 8, 2, 4, 0;
		grid.addStaggeredElementDefinitionFromElementDefinition(quadrangleDefinition);
		check(grid.staggeredElementDefinition.size()==4);
		check( contains(grid.staggeredElementDefinition,StaggeredElementDefinition(8, 2, 6)) );
		check( contains(grid.staggeredElementDefinition,StaggeredElementDefinition(2, 4, 6)) );
		check( contains(grid.staggeredElementDefinition,StaggeredElementDefinition(4, 0, 6)) );
		check( contains(grid.staggeredElementDefinition,StaggeredElementDefinition(0, 8, 6)) );
	}
	section("triangle")
	{
		ElementDefinition<3> triangleDefinition;
			triangleDefinition.index = 6;
			triangleDefinition.connectivity << 8, 2, 4;
		grid.addStaggeredElementDefinitionFromElementDefinition(triangleDefinition);
		check(grid.staggeredElementDefinition.size()==3);
		check( contains(grid.staggeredElementDefinition,StaggeredElementDefinition(8, 2, 6)) );
		check( contains(grid.staggeredElementDefinition,StaggeredElementDefinition(2, 4, 6)) );
		check( contains(grid.staggeredElementDefinition,StaggeredElementDefinition(4, 8, 6)) );
	}
}

TestCase("Grid2D with staggered elements - staggered element definition constructor", "[Grid2DWithStaggeredElements]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "two_triangles.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DWithStaggeredElements grid(gridData);
	section("staggered triangles")
	{
		check( contains(grid.staggeredElementDefinition,StaggeredElementDefinition(0,1,0)) );
		check( contains(grid.staggeredElementDefinition,StaggeredElementDefinition(3,1,0)) );
		check( contains(grid.staggeredElementDefinition,StaggeredElementDefinition(0,2,1)) );
		check( contains(grid.staggeredElementDefinition,StaggeredElementDefinition(2,3,1)) );
	}
	section("staggered quadrangles")
	{
		StaggeredElementDefinition quadrangle(0,3,1);
		quadrangle.addElement(0);
		check( contains(grid.staggeredElementDefinition,quadrangle) );
	}
}

TestCase("Grid2D with staggered elements - correct staggered element entities order", "[Grid2DWithStaggeredElements]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "two_triangles.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2D grid(gridData);
	section("staggered quadrangle")
	{
		StaggeredQuadrangle staggeredQuadrangleOrganized(0, grid.vertices[0],grid.elements[0],grid.vertices[3],grid.elements[1]);
		StaggeredQuadrangle staggeredQuadrangle(0, grid.vertices[0],grid.elements[1],grid.vertices[3],grid.elements[0]);
		Grid2DWithStaggeredElements::organizeQuadrangle(staggeredQuadrangle);
		check(staggeredQuadrangle==staggeredQuadrangleOrganized);
	}
	section("staggered triangle")
	{
		StaggeredTriangle staggeredTriangleOrganized(0, grid.vertices[1],grid.elements[0],grid.vertices[0]);
		StaggeredTriangle staggeredTriangle(0, grid.vertices[0],grid.elements[0],grid.vertices[1]);
		Grid2DWithStaggeredElements::organizeTriangle(staggeredTriangle);
		check(staggeredTriangle==staggeredTriangleOrganized);
	}
}

TestCase("Grid2D with staggered elements - staggered element construction", "[Grid2DWithStaggeredElements]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "two_triangles.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DWithStaggeredElements grid(gridData);
	section("grid vertices")
	{
		constexpr unsigned numberOfVertices = 4;
		std::vector<Vertex> vertices;
		vertices.push_back(Vertex(0.0, 0.0, 0.0, 0));
		vertices.push_back(Vertex(2.0, 0.0, 0.0, 1));
		vertices.push_back(Vertex(0.0, 2.0, 0.0, 2));
		vertices.push_back(Vertex(2.0, 2.0, 0.0, 3));
		for(Vertex& vertex: grid.vertices)
			require(vertex==vertices[vertex.getIndex()]);
	}
	section("grid triangles")
	{
		constexpr unsigned numberOfTriangles = 2;
		// 0
		require(grid.elements[0]->vertices[0]==&(grid.vertices[0]));
		require(grid.elements[0]->vertices[1]==&(grid.vertices[1]));
		require(grid.elements[0]->vertices[2]==&(grid.vertices[3]));
		// 1
		require(grid.elements[1]->vertices[0]==&(grid.vertices[0]));
		require(grid.elements[1]->vertices[1]==&(grid.vertices[3]));
		require(grid.elements[1]->vertices[2]==&(grid.vertices[2]));
	}
	section("staggered quadrangles")
	{
		constexpr unsigned index = 2;
		StaggeredQuadrangle staggeredQuadrangle(index,grid.vertices[3],grid.elements[1],grid.vertices[0],grid.elements[0]);
		check(grid.staggeredQuadrangles[0]==staggeredQuadrangle);
	}
	section("staggered triangles")
	{
		std::vector<StaggeredTriangle> staggeredTriangles = {
			{0, grid.vertices[1], grid.elements[0], grid.vertices[0]},
			{1, grid.vertices[3], grid.elements[0], grid.vertices[1]},
			{3, grid.vertices[2], grid.elements[1], grid.vertices[3]},
			{4, grid.vertices[0], grid.elements[1], grid.vertices[2]}
		};
		check(grid.staggeredTriangles==staggeredTriangles);
	}
}

TestCase("Find staggered triangles in a boundary definition", "[Grid2DWithStaggeredElements]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "CGNSFile_boundary_read_test.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DWithStaggeredElements grid(gridData);
	section("bottom")
	{
		std::string boundaryName = "bottom boundary";
		std::vector<StaggeredTriangle*> staggeredTraingles = { &grid.staggeredTriangles[0], &grid.staggeredTriangles[2]};
		check(staggeredTraingles==grid.findStaggeredTrianglesInBoundaryDefinition(gridData.getBoundaryDefinition(boundaryName)));
	}
	section("top")
	{
		std::string boundaryName = "top boundary";
		std::vector<StaggeredTriangle*> staggeredTraingles = { &grid.staggeredTriangles[4], &grid.staggeredTriangles[7]};
		check(staggeredTraingles==grid.findStaggeredTrianglesInBoundaryDefinition(gridData.getBoundaryDefinition(boundaryName)));
	}
	section("west")
	{
		std::string boundaryName = "west boundary";
		std::vector<StaggeredTriangle*> staggeredTraingles = { &grid.staggeredTriangles[1], &grid.staggeredTriangles[5]};
		check(staggeredTraingles==grid.findStaggeredTrianglesInBoundaryDefinition(gridData.getBoundaryDefinition(boundaryName)));
	}
	section("east")
	{
		std::string boundaryName = "east boundary";
		std::vector<StaggeredTriangle*> staggeredTraingles = { &grid.staggeredTriangles[3], &grid.staggeredTriangles[6]};
		check(staggeredTraingles==grid.findStaggeredTrianglesInBoundaryDefinition(gridData.getBoundaryDefinition(boundaryName)));
	}
}

TestCase("Find line", "[Grid2DWithStaggeredElements]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "CGNSFile_boundary_read_test.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DWithStaggeredElements grid(gridData);
	check(&grid.lines[0]==&grid.findLine(6));
	check(&grid.lines[1]==&grid.findLine(7));
	check(&grid.lines[2]==&grid.findLine(8));
	check(&grid.lines[3]==&grid.findLine(9));
	check(&grid.lines[4]==&grid.findLine(10));
	check(&grid.lines[5]==&grid.findLine(11));
	check(&grid.lines[6]==&grid.findLine(12));
	check(&grid.lines[7]==&grid.findLine(13));
}

TestCase("Find staggered triangle using line", "[Grid2DWithStaggeredElements]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "CGNSFile_boundary_read_test.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DWithStaggeredElements grid(gridData);
	section("bottom")
	{
		check(grid.findStaggeredTriangle(grid.lines[0])==&grid.staggeredTriangles[0]);
		check(grid.findStaggeredTriangle(grid.lines[1])==&grid.staggeredTriangles[2]);
	}
	section("top")
	{
		check(grid.findStaggeredTriangle(grid.lines[2])==&grid.staggeredTriangles[4]);
		check(grid.findStaggeredTriangle(grid.lines[3])==&grid.staggeredTriangles[7]);
	}
	section("west")
	{
		check(grid.findStaggeredTriangle(grid.lines[4])==&grid.staggeredTriangles[1]);
		check(grid.findStaggeredTriangle(grid.lines[5])==&grid.staggeredTriangles[5]);
	}
	section("east")
	{
		check(grid.findStaggeredTriangle(grid.lines[6])==&grid.staggeredTriangles[3]);
		check(grid.findStaggeredTriangle(grid.lines[7])==&grid.staggeredTriangles[6]);
	}
}

TestCase("Staggered triangle boundary", "[Grid2DWithStaggeredElements]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "CGNSFile_boundary_read_test.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DWithStaggeredElements grid(gridData);
	section("bottom")
	{
		std::string boundaryName = "bottom boundary";
		std::vector<StaggeredTriangle*> staggeredTraingles = { &grid.staggeredTriangles[0], &grid.staggeredTriangles[2]};
		std::vector<StaggeredTriangle*>& boundaryStaggeredTriangles = grid.boundary[boundaryName];
		check(boundaryStaggeredTriangles==staggeredTraingles);
	}
	section("top")
	{
		std::string boundaryName = "top boundary";
		std::vector<StaggeredTriangle*> staggeredTraingles = { &grid.staggeredTriangles[4], &grid.staggeredTriangles[7]};
		std::vector<StaggeredTriangle*>& boundaryStaggeredTriangles = grid.boundary[boundaryName];
		check(boundaryStaggeredTriangles==staggeredTraingles);
	}
	section("west")
	{
		std::string boundaryName = "west boundary";
		std::vector<StaggeredTriangle*> staggeredTraingles = { &grid.staggeredTriangles[1], &grid.staggeredTriangles[5]};
		std::vector<StaggeredTriangle*>& boundaryStaggeredTriangles = grid.boundary[boundaryName];
		check(boundaryStaggeredTriangles==staggeredTraingles);
	}
	section("east")
	{
		std::string boundaryName = "east boundary";
		std::vector<StaggeredTriangle*> staggeredTraingles = { &grid.staggeredTriangles[3], &grid.staggeredTriangles[6]};
		std::vector<StaggeredTriangle*>& boundaryStaggeredTriangles = grid.boundary[boundaryName];
		check(boundaryStaggeredTriangles==staggeredTraingles);
	}
}