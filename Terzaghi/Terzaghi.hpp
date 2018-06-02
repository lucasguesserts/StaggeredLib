#ifndef TERZAGHI_HPP
#define TERZAGHI_HPP

#include <Eigen/Core>
#include <functional>
#include <vector>

#include <Grid/Grid2DInverseDistanceStencil.hpp>
#include <Stencil/ScalarStencil.hpp>
#include <Stencil/VectorStencil.hpp>
#include <LinearSystem/EigenSparseLinearSystem.hpp>

#include <Terzaghi/Enums.hpp>
#include <Terzaghi/TerzaghiBoundary.hpp>

class Terzaghi
{
	public:
		Terzaghi(const std::string& gridFile);

		double fluidViscosity, porosity, alpha, fluidCompressibility, solidCompressibility, permeability;
		double timeInterval, timeImplicitCoefficient;
		double shearModulus, poissonCoefficient;

		Grid2DInverseDistanceStencil grid;

		Eigen::VectorXd oldSolution;

		std::function<unsigned(const unsigned)> transformToP, transformToU, transformToV, transformToW;
		std::vector<std::function<unsigned(const unsigned)>> transform;

		static const std::vector<Component> displacementComponents;
		unsigned numberOfElements, numberOfStaggeredElements, linearSystemSize;
		unsigned transformIndex(const Component component, const unsigned index);
		unsigned transformIndex(const Component component, Entity* entity);
		Eigen::Vector3d getDisplacementVector(StaggeredElement2D* staggeredElement);

		// pressure
		std::vector<ScalarStencil> scalarStencilOnVertices; // auxiliar
		std::vector<VectorStencil> pressureGradient; // stored on staggered elements
		// displacement
		std::vector<ScalarStencil> displacementScalarStencilOnElements;
		std::vector<ScalarStencil> displacementScalarStencilOnVertices;
		std::vector<VectorStencil> displacementGradient;

		void insertPressureAccumulationTermInMatrix(void);
		void insertPressureDiffusiveTermInMatrix(void);
		void insertPressureVolumeDilatationTermInMatrix(void);
		void insertDisplacementTensionTermInMatrix(void);
		void insertDisplacementPressureTermInMatrix(void);
		void insertPressureAccumulationTermInIndependent(void);
		void insertPressureDiffusiveTermInIndependent(void);
		void insertPressureVolumeDilatationTermInIndependent(void);

		void insertPressureScalarStencilInLinearSystem(Element* element, const ScalarStencil& scalarStencilOnElements);
		double recoverPressureValueFromScalarStencil(const ScalarStencil& scalarStencilOnElements);
		void insertScalarStencilDisplacementComponentInMatrix(const Component forceComponent, const Component displacementComponent, StaggeredElement2D* staggeredElement, const ScalarStencil& scalarStencilOnStaggeredElements);
		void insertPressureGradientInMatrix(const Component forceComponent, StaggeredElement2D* staggeredQuadrangle);

		// Displacement auxiliar
		Eigen::MatrixXd getPermutationMatrix(const Component c0, const Component c1);
		Eigen::MatrixXd getMechanicalPropertiesMatrix(const Component c0, const Component c1);

		EigenSparseLinearSystem linearSystem;

		// Boundary
		std::array<TerzaghiBoundary, 4> boundary;

		// just to help in tests
		void setOldPressure(const std::function<double(Eigen::Vector3d)> oldPressureFunction);
		void setOldPressure(const std::vector<double> oldPressureValues);
		void setOldDisplacement(const std::vector<Eigen::Vector3d>& displacements);

	private:
		// Pressure
		void initializeScalarStencilOnVertices(void);
		void initializePressureGradient(void);
		void initializePressureGradientOnStaggeredTriangles(void);
		// Displacement
		void initializeDisplacementScalarStencilOnElements(void);
		void initializeDisplacementScalarStencilOnVertices(void);
		void initializeDisplacementGradient(void);
		// Boundary
		void initializeBoundaryConditions(void);
};

#endif