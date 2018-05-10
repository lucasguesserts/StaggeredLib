#include <vector>
#include <Eigen/Core>
#include <Grid/GridData.hpp>
#include <Grid/Grid2DInverseDistanceStencil.hpp>
#include <Grid/DirichletBoundaryCondition.hpp>
#include <LinearSystem/EigenLinearSystem.hpp>

class FacetCenterHeatTransfer
{
	public:
		FacetCenterHeatTransfer(const GridData& gridData);

		Grid2DInverseDistanceStencil grid2D;
		EigenLinearSystem linearSystem;
		Eigen::VectorXd temperature;
		std::vector<ScalarStencil> scalarStencilOnVertices;

	private:
		void initializeLinearSystem(void);
		void initializeTemperatureVectors(void);
};