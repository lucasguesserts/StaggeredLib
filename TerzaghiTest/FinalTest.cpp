#include <Utils/Test.hpp>
#include <Utils/EigenTest.hpp>
#include <Stencil/Test.hpp>

#include <Terzaghi/Terzaghi.hpp>

#include <Grid/Grid2DExport.hpp>
#include <Grid/Grid2DWithStaggeredElementsExport.hpp>
#include <CgnsInterface/CgnsWriter.hpp>
#include <CgnsInterface/CgnsReader/CgnsReader2D.hpp>

TestCase("Assembly linear system", "[Terzaghi]")
{
	const std::string gridFile = gridDirectory + "terzaghi.cgns";
	Terzaghi terzaghi(gridFile);
	// Values from Gustavo Thesis
	terzaghi.fluidViscosity = 0.001;
	terzaghi.porosity = 0.19;
	terzaghi.alpha = 0.77777777777777;
	terzaghi.fluidCompressibility = 3.03030303030E-10;
	terzaghi.solidCompressibility = 2.77777777777E-11;
	terzaghi.permeability = 1.9E-15;
	terzaghi.timeInterval = 1.0e-1;
	terzaghi.timeImplicitCoefficient = 1;
	terzaghi.shearModulus = 6.0E+9;
	terzaghi.poissonCoefficient = 0.2;

	const std::vector<double> initialPressureValues(terzaghi.numberOfElements, 4E+5);
	terzaghi.setOldPressure(initialPressureValues);
	const std::vector<Eigen::Vector3d> oldDisplacements(terzaghi.numberOfStaggeredElements, Eigen::Vector3d::Zero());
	terzaghi.setOldDisplacement(oldDisplacements);

	terzaghi.assemblyLinearSystemMatrix();
	terzaghi.assemblyLinearSystemIndependent();

	// Facet center export
	const std::string outputDirectory = gridDirectory + std::string("output/");
	std::string resultFileName = outputDirectory + "terzaghi_facet_center.cgns";
	std::string uResultCSV = outputDirectory + "terzaghi_facet_center_u.csv";
	std::string vResultCSV = outputDirectory + "terzaghi_facet_center_v.csv";
	Grid2DWithStaggeredElementsExport::cgns(resultFileName, terzaghi.grid);
	Grid2DWithStaggeredElementsExport::csv(uResultCSV, terzaghi.grid);
	Grid2DWithStaggeredElementsExport::csv(vResultCSV, terzaghi.grid);
	CgnsWriter cgnsWriterFacetCenter(resultFileName, "CellCenter");

	// Element center export
	std::string elementCenterResult = outputDirectory + "terzaghi_element_center.cgns";
	std::string pResultCSV = outputDirectory + "terzaghi_facet_center_p.csv";
	Grid2DExport::cgns(elementCenterResult, terzaghi.grid);
	Grid2DExport::csv(pResultCSV, terzaghi.grid);
	CgnsWriter cgnsElementCenterWriter(elementCenterResult, "CellCenter");

	constexpr unsigned numberOfTimeSteps = 100;
	for(unsigned timeStep=0 ; timeStep<numberOfTimeSteps ; ++timeStep)
	{
		terzaghi.solve();
		cgnsElementCenterWriter.writeTransientSolution(terzaghi.timeInterval * timeStep);
		cgnsElementCenterWriter.writeTransientField(terzaghi.getComponentFromOldSolution(Component::P), "pressure");
		cgnsWriterFacetCenter.writeTransientSolution(terzaghi.timeInterval * timeStep);
		cgnsWriterFacetCenter.writeTransientField(terzaghi.getComponentFromOldSolution(Component::U), "u_displacement");
		cgnsWriterFacetCenter.writeTransientField(terzaghi.getComponentFromOldSolution(Component::V), "v_displacement");
		Grid2DWithStaggeredElementsExport::csvAppendTimeSolution(uResultCSV, terzaghi.timeInterval*timeStep, terzaghi.getComponentFromOldSolution(Component::U));
		Grid2DWithStaggeredElementsExport::csvAppendTimeSolution(vResultCSV, terzaghi.timeInterval*timeStep, terzaghi.getComponentFromOldSolution(Component::V));
		Grid2DExport::csvAppendTimeSolution(pResultCSV, terzaghi.timeInterval*timeStep, terzaghi.getComponentFromOldSolution(Component::P));
		std::cout << timeStep << std::endl;
	}
}