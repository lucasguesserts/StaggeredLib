#include <vector>
#include <string>
#include <functional>
#include <Eigen/Core>
#include <Grid/Grid2DInverseDistanceStencil.hpp>
#include <Grid/DirichletBoundaryCondition.hpp>
#include <LinearSystem/EigenLinearSystem.hpp>

class SquareCavityHeatTransfer
{
	public:
		SquareCavityHeatTransfer(const std::string& fileName);
		void addAccumulationTerm(void);
		void addDiffusiveTerm(void);
		void addDiffusiveTerm(StaggeredElement2D& staggeredQuadrangle);

		static Eigen::VectorXd computeAnalyticalSolution(const Eigen::Matrix<double,Eigen::Dynamic,3>& coordinates);
		Eigen::VectorXd computeAnalyticalSolution(void);

		double rho, cp, k;
		double timeInterval, timeImplicitCoefficient;
		Grid2DInverseDistanceStencil grid2D;
		EigenLinearSystem linearSystem;
		Eigen::VectorXd oldTemperature;
		Eigen::VectorXd temperature;
		std::vector<ScalarStencil> scalarStencilOnVertices;
		std::vector<DirichletBoundaryCondition> dirichletBoundaries;

		void insertDirichletBoundaryCondition(const std::string& boundaryName, const std::function<double(Eigen::Vector3d)> prescribedValueFunction);
		void insertDirichletBoundaryCondition(const std::string& boundaryName, const double prescribedValue);
		void applyBoundaryConditions(void);
		void applyBoundaryCondition(DirichletBoundaryCondition& dirichlet);
		void applyDirichletBoundaryConditionInStaggeredTriangle(StaggeredElement2D& staggeredTriangle, const double prescribedValue);

		Eigen::VectorXd nextTimeStep(void);

		void addAccumulationTermToMatrix(void);
		void addAccumulationTermToIndependent(void);
		void addDiffusiveTermToMatrix(void);
		void addDiffusiveTermToIndependent(void);
		void applyBoundaryConditionsToMatrix(void);
		void applyBoundaryConditionToMatrix(DirichletBoundaryCondition& dirichlet);
		void applyDirichletBoundaryConditionInStaggeredTriangleToMatrix(StaggeredElement2D& staggeredTriangle, const double prescribedValue);
		void applyBoundaryConditionsToIndependent(void);
		void applyBoundaryConditionToIndependent(DirichletBoundaryCondition& dirichlet);
		void applyDirichletBoundaryConditionInStaggeredTriangleToIndependent(StaggeredElement2D& staggeredTriangle, const double prescribedValue);
	private:
		void initializeLinearSystem(void);
		void initializeTemperatureVectors(void);
		void initializeScalarStencilOnVertices(void);
		ScalarStencil computeDiffusiveTerm(StaggeredElement2D& staggeredQuadrangle); //areaVector * k * timeInterval * gradientScalarStencil
};