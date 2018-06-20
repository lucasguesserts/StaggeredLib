#include <Utils/Test.hpp>

#include <Grid/Grid2DExport.hpp>

TestCase("Export 2D grid", "[Grid2DExport]")
{
	const std::string cgnsGridFileName = gridDirectory + "two_triangles.cgns";
	const std::string testFileName = gridDirectory + ".grid2DExport_Test.cgns";
	Grid2D grid(cgnsGridFileName);
	Grid2DExport::cgns(testFileName, grid);
	Grid2D testGrid(testFileName);
	section("vertices")
	{
		std::vector<Vertex> vertices;
		vertices.push_back(Vertex(0.0, 0.0, 0.0, 0));
		vertices.push_back(Vertex(2.0, 0.0, 0.0, 1));
		vertices.push_back(Vertex(0.0, 2.0, 0.0, 2));
		vertices.push_back(Vertex(2.0, 2.0, 0.0, 3));
		check(testGrid.vertices==vertices);
	}
	section("triangles")
	{
		require(testGrid.triangles.size()==2);
		// 0
		check(testGrid.triangles[0].vertices[0]==&(testGrid.vertices[0]));
		check(testGrid.triangles[0].vertices[1]==&(testGrid.vertices[1]));
		check(testGrid.triangles[0].vertices[2]==&(testGrid.vertices[3]));
		// 1
		check(testGrid.triangles[1].vertices[0]==&(testGrid.vertices[0]));
		check(testGrid.triangles[1].vertices[1]==&(testGrid.vertices[3]));
		check(testGrid.triangles[1].vertices[2]==&(testGrid.vertices[2]));
	}
	boost::filesystem::remove_all(testFileName);
}

TestCase("Export grid2D to csv", "[Grid2DExport]")
{
	const std::string cgnsGridFileName = gridDirectory + "two_triangles.cgns";
	const std::string testFileName = gridDirectory + "grid2DExport_Test.csv";
	Grid2D grid(cgnsGridFileName);
	Grid2DExport::csv(testFileName, grid);
	for(unsigned i=0 ; i<10 ; ++i)
		Grid2DExport::csvAppendTimeSolution(testFileName, 2.0, std::vector<double>(3, 1.0));
	// Manual checking
}