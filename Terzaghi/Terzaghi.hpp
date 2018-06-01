#ifndef TERZAGHI_HPP
#define TERZAGHI_HPP

#include <Eigen/Core>
#include <functional>
#include <vector>

#include <Grid/Grid2DInverseDistanceStencil.hpp>
#include <Stencil/ScalarStencil.hpp>
#include <Stencil/VectorStencil.hpp>
#include <LinearSystem/EigenSparseLinearSystem.hpp>

struct DisplacementIndex
{
	static constexpr unsigned U = 0u;
	static constexpr unsigned V = 1u;
	static constexpr unsigned W = 2u;
	static constexpr unsigned numberOfComponents = 3u;
};

class Terzaghi
{
	public:
		Terzaghi(const std::string& gridFile);

		double fluidViscosity, porosity, alpha, fluidCompressibility, solidCompressibility, permeability;
		double timeInterval, timeImplicitCoefficient;
		double shearModulus, poissonCoefficient;

		Grid2DInverseDistanceStencil grid;

		Eigen::VectorXd oldSolution;

		unsigned numberOfElements, numberOfStaggeredElements, linearSystemSize;
		unsigned getPindex(Element* element);
		unsigned getPindex(const unsigned elementIndex);
		unsigned getUindex(StaggeredElement2D* staggeredElement);
		unsigned getUindex(const unsigned staggeredElementIndex);
		unsigned getVindex(StaggeredElement2D* staggeredElement);
		unsigned getVindex(const unsigned staggeredElementIndex);
		unsigned getWindex(StaggeredElement2D* staggeredElement);
		unsigned getWindex(const unsigned staggeredElementIndex);
		Eigen::Vector3d getDisplacementVector(StaggeredElement2D* staggeredElement);

		// pressure
		std::vector<ScalarStencil> scalarStencilOnVertices; // auxiliar
		std::vector<VectorStencil> pressureGradient; // stored on staggered quadrangles
		// displacement
		std::vector<ScalarStencil> displacementScalarStencilOnElements;
		std::vector<ScalarStencil> displacementScalarStencilOnVertices;
		std::vector<VectorStencil> displacementGradient;

		void insertPressureAccumulationTermInMatrix(void);
		void insertPressureDiffusiveTermInMatrix(void);
		void insertPressureVolumeDilatationTermInMatrix(void);
		void insertDisplacementTensionTermInMatrix(void);
		void insertPressureAccumulationTermInIndependent(void);
		void insertPressureDiffusiveTermInIndependent(void);
		void insertPressureVolumeDilatationTermInIndependent(void);

		void insertPressureScalarStencilInLinearSystem(Element* element, const ScalarStencil& scalarStencilOnElements);
		double recoverPressureValueFromScalarStencil(const ScalarStencil& scalarStencilOnElements);
		void insertScalarStencilDisplacementComponentInMatrix(const unsigned forceComponent, const unsigned displacementComponent, StaggeredElement2D* staggeredElement, const ScalarStencil& scalarStencilOnStaggeredElements);

		// Displacement auxiliar
		Eigen::MatrixXd getPermutationMatrix(unsigned i, unsigned j);
		Eigen::MatrixXd getMechanicalPropertiesMatrix(const unsigned i, const unsigned j);

		EigenSparseLinearSystem linearSystem;

		// just to help in tests
		void setOldPressure(const std::function<double(Eigen::Vector3d)> oldPressureFunction);
		void setOldPressure(const std::vector<double> oldPressureValues);
		void setOldDisplacement(const std::vector<Eigen::Vector3d>& displacements);

	private:
		// Pressure
		void initializeScalarStencilOnVertices(void);
		void initializePressureGradient(void);
		// Displacement
		void initializeDisplacementScalarStencilOnElements(void);
		void initializeDisplacementScalarStencilOnVertices(void);
		void initializeDisplacementGradient(void);
};

#endif