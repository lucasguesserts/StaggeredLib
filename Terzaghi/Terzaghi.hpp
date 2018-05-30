#ifndef TERZAGHI_HPP
#define TERZAGHI_HPP

#include <Eigen/Core>
#include <functional>
#include <vector>

#include <Grid/Grid2DInverseDistanceStencil.hpp>
#include <Stencil/ScalarStencil.hpp>
#include <Stencil/VectorStencil.hpp>
#include <LinearSystem/EigenSparseLinearSystem.hpp>

class Terzaghi
{
	public:
		Terzaghi(const std::string& gridFile);

		double fluidViscosity, porosity, alpha, fluidCompressibility, solidCompressibility, permeability;
		double timeInterval, timeImplicitCoefficient;

		Grid2DInverseDistanceStencil grid;

		Eigen::VectorXd oldSolution;

		unsigned numberOfElements, numberOfStaggeredElements, linearSystemSize;
		unsigned getPindex(Element* element);
		unsigned getPindex(const unsigned elementIndex);
		unsigned getUindex(StaggeredElement2D* staggeredElement);
		unsigned getVindex(StaggeredElement2D* staggeredElement);
		unsigned getWindex(StaggeredElement2D* staggeredElement);
		Eigen::Vector3d getDisplacementVector(StaggeredElement2D* staggeredElement);

		std::vector<ScalarStencil> scalarStencilOnVertices; // auxiliar
		std::vector<VectorStencil> pressureGradient; // stored on staggered quadrangles
		std::vector<VectorStencil> displacementGradient; // stored on staggered elements faces

		void insertPressureAccumulationTermInMatrix(void);
		void insertPressureDiffusiveTermInMatrix(void);
		void insertPressureVolumeDilatationTermInMatrix(void);
		void insertPressureAccumulationTermInIndependent(void);
		void insertPressureDiffusiveTermInIndependent(void);
		void insertPressureVolumeDilatationTermInIndependent(void);

		void insertPressureScalarStencilInLinearSystem(Element* element, const ScalarStencil& scalarStencilOnElements);
		double recoverPressureValueFromScalarStencil(const ScalarStencil& scalarStencilOnElements);

		// void initializeDisplacementGradient(void);

		EigenSparseLinearSystem linearSystem;

		// just to help in tests
		void setOldPressure(const std::function<double(Eigen::Vector3d)> oldPressureFunction);
		void setOldPressure(const std::vector<double> oldPressureValues);
		void setOldDisplacement(const std::vector<Eigen::Vector3d>& displacements);

	private:
		void initializeScalarStencilOnVertices(void);
		void initializePressureGradient(void);
};

#endif