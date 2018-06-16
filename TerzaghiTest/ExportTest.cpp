#include <Utils/Test.hpp>
#include <Utils/EigenTest.hpp>
#include <Stencil/Test.hpp>

#include <Terzaghi/Terzaghi.hpp>

#include <Grid/Grid2DExport.hpp>
#include <Grid/Grid2DWithStaggeredElementsExport.hpp>
#include <CgnsInterface/CgnsWriter.hpp>
#include <CgnsInterface/CgnsReader/CgnsReader2D.hpp>

TestCase("Terzaghi solution export test", "[Terzaghi]")
{
	const std::string gridFile = gridDirectory + "terzaghi.cgns";
	const std::string outputDirectory = gridDirectory + std::string("output/");
	Terzaghi terzaghi(gridFile);


	// Facet center export
	double count = 0.0;
	for(auto staggeredElement: terzaghi.grid.staggeredElements)
	{
		for(auto component : terzaghi.displacementComponents)
		{
			auto index = terzaghi.transformIndex(component, staggeredElement.getIndex());
			terzaghi.oldSolution[index] = count;
		}
		++count;
	}
	std::string facetCenterResult = outputDirectory + "terzaghi_export_test_facet_center.cgns";
	Grid2DWithStaggeredElementsExport::cgns(facetCenterResult, terzaghi.grid);
	CgnsWriter cgnsFacetCenterWriter(facetCenterResult, "CellCenter");
	cgnsFacetCenterWriter.writePermanentSolution("steadySolution");
	cgnsFacetCenterWriter.writePermanentField("u_displacement", terzaghi.getComponentFromOldSolution(Component::U));
	cgnsFacetCenterWriter.writePermanentField("v_displacement", terzaghi.getComponentFromOldSolution(Component::V));
	cgnsFacetCenterWriter.writePermanentField("w_displacement", terzaghi.getComponentFromOldSolution(Component::W));

	// Element center export
	count = 0.0;
	for(auto element: terzaghi.grid.elements)
	{
		auto index = terzaghi.transformIndex(Component::P, element);
		terzaghi.oldSolution[index] = count;
		++count;
	}
	std::string elementCenterResult = outputDirectory + "terzaghi_export_test_element_center.cgns";
	Grid2DExport::cgns(elementCenterResult, terzaghi.grid);
	CgnsWriter cgnsElementCenterWriter(elementCenterResult, "CellCenter");
	cgnsElementCenterWriter.writePermanentSolution("steadySolution");
	cgnsElementCenterWriter.writePermanentField("pressure", terzaghi.getComponentFromOldSolution(Component::P));
}