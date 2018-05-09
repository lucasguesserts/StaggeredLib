#include <vector>
#include <string>
#include <Eigen/Core>
#include <Grid/GridData.hpp>
#include <Grid/Grid2DInverseDistanceStencil.hpp>
#include <Grid/DirichletBoundaryCondition.hpp>
#include <SquareCavityHeatTransfer/EigenLinearSystem.hpp>

class SquareCavityHeatTransfer
{
	public:
		SquareCavityHeatTransfer(const GridData& gridData);
		void addAccumulationTerm(void);
		void addDiffusiveTerm(void);
		void addDiffusiveTerm(StaggeredQuadrangle& staggeredQuadrangle);

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
		void applyDirichletBoundaryConditionInStaggeredTriangle(StaggeredTriangle& staggeredTriangle, const double prescribedValue);

		Eigen::VectorXd nextTimeStep(void);

	private:
		void initializeLinearSystem(void);
		void initializeTemperatureVectors(void);
		void initializeScalarStencilOnVertices(void);
		ScalarStencil computeDiffusiveTerm(StaggeredQuadrangle& staggeredQuadrangle); //areaVector * k * timeInterval * gradientScalarStencil
};
