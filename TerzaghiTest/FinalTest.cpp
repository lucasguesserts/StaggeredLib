#include <Utils/Test.hpp>
#include <Utils/EigenTest.hpp>
#include <Stencil/Test.hpp>

#include <Terzaghi/Terzaghi.hpp>

TestCase("Assembly linear system", "[Terzaghi]")
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	// Values from Gustavo Thesis
	terzaghi.fluidViscosity = 0.001;
	terzaghi.porosity = 0.19;
	terzaghi.alpha = 0.77777777777777;
	terzaghi.fluidCompressibility = 3.03030303030E-10;
	terzaghi.solidCompressibility = 2.77777777777E-11;
	terzaghi.permeability = 1.9E-15;
	terzaghi.timeInterval = 0.1;
	terzaghi.timeImplicitCoefficient = 1;
	terzaghi.shearModulus = 6.0E+9;
	terzaghi.poissonCoefficient = 0.2;
	terzaghi.oldSolution = Eigen::VectorXd::Zero(terzaghi.linearSystemSize);
	terzaghi.assemblyLinearSystemMatrix();
	terzaghi.assemblyLinearSystemIndependent();
	Eigen::MatrixXd dense(terzaghi.linearSystem.matrix);
	std::cout << "Matrix:" << std::endl << dense << std::endl << std::endl;
	std::cout << "Independent:" << std::endl << eigenVectorToString(terzaghi.linearSystem.independent) << std::endl << std::endl;
}