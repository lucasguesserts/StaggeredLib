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
#include <Grid/Grid2DWithStaggeredElements_2.hpp>

TestCase("Grid2DWithStaggeredElements build", "[Grid][Grid2D]")
{
	const std::string cgnsGridFileName = CGNSFile::gridDirectory + "GridReaderTest_CGNS.cgns";
	CGNSFile cgnsFile(cgnsGridFileName);
	GridData gridData(cgnsFile);
	Grid2DWithStaggeredElements_2 grid2D(gridData);
}