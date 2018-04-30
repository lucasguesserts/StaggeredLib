#include <Utils/EigenTest.hpp>
#include <GeometricEntity/Test.hpp>

#include <string>
#include <vector>

#include <CGNSFile/CGNSFile.hpp>
#include <GeometricEntity/Face.hpp>
#include <Grid/GridData.hpp>
#include <Grid/Grid2DWithStaggeredElements.hpp>

TestCase("Face constructor", "[Face]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "GridReaderTest_CGNS.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DWithStaggeredElements grid(gridData);
	constexpr unsigned index = 12;
	constexpr unsigned localIndex = 53;
	Face face(index, localIndex, *grid.elements[0], grid.vertices[0], grid.staggeredTriangles[0], grid.staggeredTriangles[1]);
	check(face.getIndex()==index);
	check(face.parentElement==grid.elements[0]);
	check(face.localIndex==localIndex);
	check(face.adjacentVertex==&grid.vertices[0]);
	check(face.backwardStaggeredElement==&grid.staggeredTriangles[0]);
	check(face.forwardStaggeredElement==&grid.staggeredTriangles[1]);
	check(face.getAreaVector()==Eigen::Vector3d(-0.75,0.75,0.0));
}

TestCase("Face operator==", "[Face]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "GridReaderTest_CGNS.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DWithStaggeredElements grid(gridData);
	std::vector<Face> faces = {
		{0, 1, *(grid.elements[0]), grid.vertices[1], static_cast<StaggeredElement&>(grid.staggeredTriangles[1]),   static_cast<StaggeredElement&>(grid.staggeredTriangles[0])  },
		{1, 2, *(grid.elements[0]), grid.vertices[3], static_cast<StaggeredElement&>(grid.staggeredQuadrangles[0]), static_cast<StaggeredElement&>(grid.staggeredTriangles[1])  },
		{2, 0, *(grid.elements[0]), grid.vertices[0], static_cast<StaggeredElement&>(grid.staggeredTriangles[0]),   static_cast<StaggeredElement&>(grid.staggeredQuadrangles[0])},
		{3, 1, *(grid.elements[1]), grid.vertices[3], static_cast<StaggeredElement&>(grid.staggeredTriangles[3]),   static_cast<StaggeredElement&>(grid.staggeredQuadrangles[0])},
		{4, 2, *(grid.elements[1]), grid.vertices[3], static_cast<StaggeredElement&>(grid.staggeredTriangles[4]),   static_cast<StaggeredElement&>(grid.staggeredTriangles[3])  },
		{5, 0, *(grid.elements[1]), grid.vertices[3], static_cast<StaggeredElement&>(grid.staggeredQuadrangles[0]), static_cast<StaggeredElement&>(grid.staggeredTriangles[4])  }
	};
}

TestCase("Grid2DWithStaggeredElements Face constructor", "[Face][Grid2DWithStaggeredElements]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "two_triangles.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DWithStaggeredElements grid(gridData);
	std::vector<Face> faces = {
		{0, 1, *(grid.elements[0]), grid.vertices[1], static_cast<StaggeredElement&>(grid.staggeredTriangles[1]),   static_cast<StaggeredElement&>(grid.staggeredTriangles[0])  },
		{1, 2, *(grid.elements[0]), grid.vertices[3], static_cast<StaggeredElement&>(grid.staggeredQuadrangles[0]), static_cast<StaggeredElement&>(grid.staggeredTriangles[1])  },
		{2, 0, *(grid.elements[0]), grid.vertices[0], static_cast<StaggeredElement&>(grid.staggeredTriangles[0]),   static_cast<StaggeredElement&>(grid.staggeredQuadrangles[0])},
		{3, 1, *(grid.elements[1]), grid.vertices[3], static_cast<StaggeredElement&>(grid.staggeredTriangles[3]),   static_cast<StaggeredElement&>(grid.staggeredQuadrangles[0])},
		{4, 2, *(grid.elements[1]), grid.vertices[3], static_cast<StaggeredElement&>(grid.staggeredTriangles[4]),   static_cast<StaggeredElement&>(grid.staggeredTriangles[3])  },
		{5, 0, *(grid.elements[1]), grid.vertices[3], static_cast<StaggeredElement&>(grid.staggeredQuadrangles[0]), static_cast<StaggeredElement&>(grid.staggeredTriangles[3])  }
	};
	check(grid.faces==faces);
}