#ifndef TERZAGHI_HPP
#define TERZAGHI_HPP

#include <Eigen/Core>

#include <Grid/Grid2DInverseDistanceStencil.hpp>
#include <Stencil/ScalarStencil.hpp>
#include <Stencil/VectorStencil.hpp>
#include <LinearSystem/EigenSparseLinearSystem.hpp>

class Terzaghi
{
	public:
		Terzaghi(const std::string& gridFile);

		double viscosity, porosity, alpha, fluidCompressibility, solidCompressibility, permeability;
		double timeInterval, timeImplicitCoefficient;

		Grid2DInverseDistanceStencil grid;

		Eigen::VectorXd pressure;
		Eigen::VectorXd uDisplacement, vDisplacement, wDisplacement;
		Eigen::VectorXd completeSolution;

		unsigned numberOfElements, numberOfStaggeredElements;
		unsigned getPindex(Element* element);
		// unsigned getUindex(StaggeredElement2D* staggeredElement);
		// unsigned getVindex(StaggeredElement2D* staggeredElement);
		// unsigned getWindex(StaggeredElement2D* staggeredElement);

		std::vector<ScalarStencil> pressureGradient;
		std::vector<VectorStencil> displacementGradient;

		void insertPressureAccumulationTermToMatrix(void);

		// void initializePressureGradient(void);
		// void initializeDisplacementGradient(void);

		EigenSparseLinearSystem linearSystem;
};

#endif