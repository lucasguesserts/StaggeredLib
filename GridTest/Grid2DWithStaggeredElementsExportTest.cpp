#include <Utils/Test.hpp>

#include <Grid/Grid2DWithStaggeredElementsExport.hpp>

TestCase("opa")
{
	const std::string cgnsGridFileName = gridDirectory + "GridReaderTest_CGNS.cgns";
	const std::string testFileName = gridDirectory + "grid2DWithStaggeredElementExport_Test.cgns";
	Grid2DWithStaggeredElements grid(cgnsGridFileName);
	Grid2DWithStaggeredElementsExport::cgns(testFileName, grid);
	boost::filesystem::remove_all(testFileName);
}