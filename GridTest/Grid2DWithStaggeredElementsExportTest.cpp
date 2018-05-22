#include <Utils/Test.hpp>

#include <Grid/Grid2DWithStaggeredElementsExport.hpp>

TestCase("Export staggered grid", "[Grid2DWithStaggeredElementsExport]")
{
	const std::string cgnsGridFileName = gridDirectory + "two_triangles.cgns";
	const std::string testFileName = gridDirectory + ".grid2DWithStaggeredElementExport_Test.cgns";
	Grid2DWithStaggeredElements grid(cgnsGridFileName);
	Grid2DWithStaggeredElementsExport::cgns(testFileName, grid);
	boost::filesystem::remove_all(testFileName);
}