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

		// Displacement auxiliar matrices
		static const std::vector<Eigen::Matrix<double,1,3>> leftDisplacementMatrix;
		static const std::vector<Eigen::Matrix<double,6,3>> rightDisplacementMatrix;

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
		std::vector<ScalarStencil> scalarStencilOnVertices;
		std::vector<VectorStencil> pressureGradient;
		std::vector<Eigen::Vector3d> pressureGradientIndependent;
		// displacement
		std::vector<ScalarStencil> displacementScalarStencilOnElements;
		std::vector<ScalarStencil> displacementScalarStencilOnVertices;
		std::vector<VectorStencil> displacementGradient;

		// pressure
		void insertPressureAccumulationTermInMatrix(void);
		void insertPressureDiffusiveTermInMatrix(void);
		void insertPressureVolumeDilatationTermInMatrix(void);
		void insertPressureAccumulationTermInIndependent(void);
		void insertPressureDiffusiveTermInIndependent(void);
		void insertPressureVolumeDilatationTermInIndependent(void);

		// displacement
		void insertDisplacementTensionTermInMatrix(void);
		void insertDisplacementPressureTermInMatrix(void);

		void insertPressureScalarStencilInLinearSystem(Element* element, const ScalarStencil& scalarStencilOnElements);
		double recoverPressureValueFromScalarStencil(const ScalarStencil& scalarStencilOnElements);
		void insertScalarStencilDisplacementComponentInMatrix(const Component forceComponent, const Component displacementComponent, StaggeredElement2D* staggeredElement, const ScalarStencil& scalarStencilOnStaggeredElements);
		void insertPressureGradientInMatrix(const Component forceComponent, StaggeredElement2D* staggeredElement);
		void insertForceComponentInIndependent(Eigen::Vector3d force, StaggeredElement2D* staggeredTriangle);

		// Displacement auxiliar
		VectorStencil getDisplacementGradientOnStaggeredTriangle(StaggeredElement2D* staggeredTriangle);
			StaggeredElement2D* findStaggeredTriangleNeighbor(StaggeredElement2D* staggeredTriangle, Vertex* adjacentVertex, Element* parentElement);
		std::vector<std::vector<ScalarStencil>> computeDisplacementScalarStencilMatrix(Face2D& face);
		Eigen::MatrixXd voigtTransformation(const Eigen::Vector3d& vector);
		Eigen::MatrixXd getPhysicalPropertiesMatrix(void);

		EigenSparseLinearSystem linearSystem;

		// Boundary
		std::vector<TerzaghiBoundary> boundaries;
		void initializeBoundaryConditions(void);
		void insertPrescribedStressInIndependent(void);
		// void insertDisplacementBoundaryTensionTermInMatrix(void);
		// 	Eigen::MatrixXd getNeumannAppliedPhysicalPropertiesMatrix(std::array<bool,6>& isStressPrescribed);
		// 	std::vector<std::vector<ScalarStencil>> computeDisplacementScalarStencilMatrix(StaggeredElement2D* staggeredTriangle, Eigen::MatrixXd& physicalPropertiesMatrix);
		void insertDisplacementDirichletBoundaryConditionToMatrix(void);
			void applyDisplacementDirichletBoundaryCondition(const Component component, StaggeredElement2D* staggeredTriangle);
		void insertDisplacementDirichletBoundaryConditionToIndependent(void);
		void insertDisplacementPressureDirichletBoundaryConditionToMatrix(void);
		void insertDisplacementPressureDirichletBoundaryConditionToIndependent(void);
			void insertPressureGradientInIndependent(const Component forceComponent, StaggeredElement2D* staggeredElement, double prescribedValue);
		void insertDisplacementPressureNeumannBoundaryConditionToIndependent(void);
			void insertPressureGradientNeumannInIndependent(const Component forceComponent, StaggeredElement2D* staggeredElement, Eigen::Vector3d pressureGradientPrescribed);
		void insertPressureDirichletBoundaryConditionToMatrix(void);
		void insertPressureDirichletBoundaryConditionToIndependent(void);
		void insertPressureNeumannBoundaryConditionToIndependent(void);
		void insertPrescribedStressPressureTermInMatrix(void);
			void insertPressureGradientInMatrix(StaggeredElement2D* staggeredElement, const VectorStencil& pressureTerm);

		// just to help in tests
		void setOldPressure(const std::function<double(Eigen::Vector3d)> oldPressureFunction);
		void setOldPressure(const std::vector<double> oldPressureValues);
		void setOldDisplacement(const std::vector<Eigen::Vector3d>& displacements);

		// Solve
		void assemblyLinearSystemMatrix(void);
		void assemblyLinearSystemIndependent(void);
		void solve(void);

		// Output
		std::vector<double> getComponentFromOldSolution(const Component component);

	private:
		// Pressure
		void initializeScalarStencilOnVertices(void);
		void initializePressureGradient(void);
		void initializePressureGradientOnStaggeredTriangles(void);
		// Displacement
		void initializeDisplacementScalarStencilOnElements(void);
		void initializeDisplacementScalarStencilOnVertices(void);
		void initializeDisplacementGradient(void);
};

#endif