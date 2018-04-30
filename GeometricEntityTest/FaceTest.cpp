#include <Utils/EigenTest.hpp>
#include <GeometricEntity/Test.hpp>

#include <string>

#include <CGNSFile/CGNSFile.hpp>
#include <GeometricEntity/Face.hpp>
#include <Grid/GridData.hpp>
#include <Grid/Grid2DWithStaggeredElements.hpp>

TestCase("Face constructor", "Face")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "GridReaderTest_CGNS.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DWithStaggeredElements grid(gridData);
	constexpr unsigned localIndex = 53;
	Face face(localIndex, *grid.elements[0], grid.vertices[0], grid.staggeredTriangles[0], grid.staggeredTriangles[1]);
	check(face.parentElement==grid.elements[0]);
	check(face.localIndex==localIndex);
	check(face.adjacentVertex==&grid.vertices[0]);
	check(face.backwardStaggeredElement==&grid.staggeredTriangles[0]);
	check(face.forwardStaggeredElement==&grid.staggeredTriangles[1]);
	check(face.getAreaVector()==Eigen::Vector3d(-0.75,0.75,0.0));
}