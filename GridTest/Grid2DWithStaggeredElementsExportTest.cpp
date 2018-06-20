#include <Utils/Test.hpp>

#include <Grid/Grid2DWithStaggeredElementsExport.hpp>

TestCase("Export staggered grid", "[Grid2DWithStaggeredElementsExport]")
{
	const std::string cgnsGridFileName = gridDirectory + "two_triangles.cgns";
	const std::string testFileName = gridDirectory + ".grid2DWithStaggeredElementExport_Test.cgns";
	Grid2DWithStaggeredElements grid(cgnsGridFileName);
	Grid2DWithStaggeredElementsExport::cgns(testFileName, grid);
	Grid2D testGrid(testFileName);
	section("vertices")
	{
		std::vector<Vertex> vertices;
		vertices.push_back(Vertex(0.0, 0.0, 0.0, 0));
		vertices.push_back(Vertex(2.0, 0.0, 0.0, 1));
		vertices.push_back(Vertex(0.0, 2.0, 0.0, 2));
		vertices.push_back(Vertex(2.0, 2.0, 0.0, 3));
		vertices.push_back(Vertex(4.0/3.0, 2.0/3.0, 0.0, 4));
		vertices.push_back(Vertex(2.0/3.0, 4.0/3.0, 0.0, 5));
		check(testGrid.vertices==vertices);
	}
	section("quadrangle")
	{
		check(testGrid.quadrangles[0].vertices[0]==&(testGrid.vertices[0]));
		check(testGrid.quadrangles[0].vertices[1]==&(testGrid.vertices[4]));
		check(testGrid.quadrangles[0].vertices[2]==&(testGrid.vertices[3]));
		check(testGrid.quadrangles[0].vertices[3]==&(testGrid.vertices[5]));
	}
	section("triangles")
	{
		require(testGrid.triangles.size()==4);
		// 0
		check(testGrid.triangles[0].vertices[0]==&(testGrid.vertices[1]));
		check(testGrid.triangles[0].vertices[1]==&(testGrid.vertices[4]));
		check(testGrid.triangles[0].vertices[2]==&(testGrid.vertices[0]));
		// 1
		check(testGrid.triangles[1].vertices[0]==&(testGrid.vertices[3]));
		check(testGrid.triangles[1].vertices[1]==&(testGrid.vertices[4]));
		check(testGrid.triangles[1].vertices[2]==&(testGrid.vertices[1]));
		// 2
		check(testGrid.triangles[2].vertices[0]==&(testGrid.vertices[2]));
		check(testGrid.triangles[2].vertices[1]==&(testGrid.vertices[5]));
		check(testGrid.triangles[2].vertices[2]==&(testGrid.vertices[3]));
		// 3
		check(testGrid.triangles[3].vertices[0]==&(testGrid.vertices[0]));
		check(testGrid.triangles[3].vertices[1]==&(testGrid.vertices[5]));
		check(testGrid.triangles[3].vertices[2]==&(testGrid.vertices[2]));
	}
	boost::filesystem::remove_all(testFileName);
}

TestCase("Export grid2D with staggered elements to csv", "[Grid2DWithStaggeredElementsExport]")
{
	const std::string cgnsGridFileName = gridDirectory + "two_triangles.cgns";
	const std::string testFileName = gridDirectory + "grid2DWithStaggeredElementExport_Test.csv";
	Grid2DWithStaggeredElements grid(cgnsGridFileName);
	Grid2DWithStaggeredElementsExport::csv(testFileName, grid);
	for(unsigned i=0 ; i<10 ; ++i)
		Grid2DWithStaggeredElementsExport::csvAppendTimeSolution(testFileName, 2.0, std::vector<double>(3, 1.0));
	// Manual checking
}