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
	terzaghi.timeInterval = 100;
	terzaghi.timeImplicitCoefficient = 1;
	terzaghi.shearModulus = 6.0E+9;
	terzaghi.poissonCoefficient = 0.2;

	const std::vector<double> initialPressureValues(terzaghi.numberOfElements, 0.0);
	terzaghi.setOldPressure(initialPressureValues);
	const std::vector<Eigen::Vector3d> oldDisplacements(terzaghi.numberOfStaggeredElements, Eigen::Vector3d::Zero());
	terzaghi.setOldDisplacement(oldDisplacements);

	terzaghi.assemblyLinearSystemMatrix();
	terzaghi.assemblyLinearSystemIndependent();
	// // Eigen::MatrixXd dense(terzaghi.linearSystem.matrix);
	// // std::cout << "Matrix:" << std::endl << dense << std::endl << std::endl;
	// // std::cout << "Independent:" << std::endl << eigenVectorToString(terzaghi.linearSystem.independent) << std::endl << std::endl;

	constexpr unsigned numberOfTimeSteps = 100;
	for(unsigned timeStep=0 ; timeStep<numberOfTimeSteps ; ++timeStep)
	{
		terzaghi.solve();
		// // std::cout << timeStep << std::endl;
	}
	// // std::cout << "final solution:" << std::endl << eigenVectorToString(terzaghi.oldSolution) << std::endl << std::endl;

	// Facet center export
	const std::string outputDirectory = gridDirectory + std::string("output/");
	std::string resultFileName = outputDirectory + "terzaghi_facet_center.cgns";
	Grid2DWithStaggeredElementsExport::cgns(resultFileName, terzaghi.grid);
	CgnsWriter cgnsWriter(resultFileName, "CellCenter");
	cgnsWriter.writePermanentSolution("steadySolution");
	cgnsWriter.writePermanentField("u_displacement", terzaghi.getComponentFromOldSolution(Component::U));
	cgnsWriter.writePermanentField("v_displacement", terzaghi.getComponentFromOldSolution(Component::V));

	// Element center export
	std::string elementCenterResult = outputDirectory + "terzaghi_element_center.cgns";
	Grid2DExport::cgns(elementCenterResult, terzaghi.grid);
	CgnsWriter cgnsElementCenterWriter(elementCenterResult, "CellCenter");
	cgnsElementCenterWriter.writePermanentSolution("steadySolution");
	cgnsElementCenterWriter.writePermanentField("pressure", terzaghi.getComponentFromOldSolution(Component::P));
}