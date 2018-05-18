#include <Utils/EigenTest.hpp>
#include <GeometricEntity/Test.hpp>

#include <string>
#include <vector>

#include <CGNSFile/CGNSFile.hpp>
#include <GeometricEntity/Face2D.hpp>
#include <Grid/GridData_2.hpp>
#include <Grid/Grid2D.hpp>

TestCase("Face2D constructor", "[Face2D]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "GridReaderTest_CGNS.cgns";
	Grid2D grid(cgnsGridFileName);
	constexpr unsigned index = 12;
	constexpr unsigned localIndex = 53;
	StaggeredElement2D staggeredTriangle(7,grid.vertices[1],grid.elements[0], grid.vertices[0]);
	StaggeredElement2D staggeredQuadrangle(14,grid.vertices[0],grid.elements[0], grid.vertices[1],grid.elements[1]);
	Face2D face(index, localIndex, *grid.elements[0], grid.vertices[0], staggeredTriangle, staggeredQuadrangle);
	check(face.getIndex()==index);
	check(face.parentElement==grid.elements[0]);
	check(face.localIndex==localIndex);
	check(face.adjacentVertex==&grid.vertices[0]);
	check(face.backwardStaggeredElement==&staggeredTriangle);
	check(face.forwardStaggeredElement==&staggeredQuadrangle);
	check(face.getCentroid()==Eigen::Vector3d(0.375,0.375,0.0));
	check(face.getAreaVector()==Eigen::Vector3d(-0.75,0.75,0.0));
}