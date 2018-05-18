#include <vector>
#include <string>
#include <Eigen/Core>
#include <Grid/GridData_2.hpp>
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

		static Eigen::VectorXd computeAnalyticalSolution(const Eigen::Matrix<double,Eigen::Dynamic,3> coordinates);

		double rho, cp, k;
		double timeInterval, timeImplicitCoefficient;
		Grid2DInverseDistanceStencil grid2D;
		EigenLinearSystem linearSystem;
		Eigen::VectorXd oldTemperature;
		Eigen::VectorXd temperature;
		std::vector<ScalarStencil> scalarStencilOnVertices;
		std::vector<DirichletBoundaryCondition> dirichletBoundaries;

		void applyBoundaryConditions(void);
		void applyBoundaryCondition(DirichletBoundaryCondition& dirichlet);
		void applyDirichletBoundaryConditionInStaggeredTriangle(StaggeredElement2D& staggeredTriangle, const double prescribedValue);

		Eigen::VectorXd nextTimeStep(void);

	private:
		void initializeLinearSystem(void);
		void initializeTemperatureVectors(void);
		void initializeScalarStencilOnVertices(void);
		ScalarStencil computeDiffusiveTerm(StaggeredElement2D& staggeredQuadrangle); //areaVector * k * timeInterval * gradientScalarStencil
};