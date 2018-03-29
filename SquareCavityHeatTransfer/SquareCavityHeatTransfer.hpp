#include <vector>
#include <string>
#include <Eigen/Core>
#include <Grid/GridData.hpp>
#include <Grid/Grid2DInverseDistanceStencil.hpp>
#include <SquareCavityHeatTransfer/EigenLinearSystem.hpp>

class SquareCavityHeatTransfer
{
	public:
		SquareCavityHeatTransfer(const GridData& gridData);
		void addAccumulationTerm(void);
		void addDiffusiveTerm(void);
		void addDiffusiveTerm(StaggeredQuadrangle& staggeredQuadrangle);
		void addDiffusiveTerm(StaggeredTriangle& staggeredTriangle);

		static Eigen::VectorXd computeAnalyticalSolution(const Eigen::Matrix<double,Eigen::Dynamic,3> coordinates);

		double rho, cp, k;
		double timeInterval, timeImplicitCoefficient;
		Grid2DInverseDistanceStencil grid2D;
		EigenLinearSystem linearSystem;
		Eigen::VectorXd oldTemperature;
		Eigen::VectorXd temperature;
		std::vector<ScalarStencil> scalarStencilOnVertices;

	private:
		void initializeLinearSystem(void);
		void initializeTemperatureVectors(void);
		void initializeScalarStencilOnVertices(void);
		ScalarStencil computeDiffusiveTerm(StaggeredQuadrangle& staggeredQuadrangle); //areaVector * k * timeInterval * gradientScalarStencil
		ScalarStencil computeDiffusiveTerm(StaggeredTriangle& staggeredTriangle); //areaVector * k * timeInterval * gradientScalarStencil
};
