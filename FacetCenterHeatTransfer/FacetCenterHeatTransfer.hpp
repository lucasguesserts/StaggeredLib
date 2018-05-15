#include <vector>
#include <Eigen/Core>
#include <Grid/GridData_2.hpp>
#include <Grid/Grid2DInverseDistanceStencil.hpp>
#include <Grid/DirichletBoundaryCondition.hpp>
#include <LinearSystem/EigenLinearSystem.hpp>
#include <Stencil/VectorStencil.hpp>

class FacetCenterHeatTransfer
{
	public:
		FacetCenterHeatTransfer(const GridData_2& gridData);

		void addDiffusiveTerm(void);
		void applyBoundaryConditions(void);

		Grid2DInverseDistanceStencil grid2D;
		EigenLinearSystem linearSystem;
		Eigen::VectorXd temperature;
		std::vector<ScalarStencil> scalarStencilOnVertices;
		std::vector<VectorStencil> gradientOnFaces;
		std::vector<DirichletBoundaryCondition> dirichletBoundaries;

		Eigen::MatrixXd computeGradientMatrix(Face2D& face);
		std::vector<ScalarStencil> getScalarStencilOnElementVertices(Face2D& face);

	private:
		void initializeLinearSystem(void);
		void initializeScalarStencilOnVertices(void);
		void initializeGradientOnFaces(void);
};

VectorStencil operator*(Eigen::MatrixXd gradientMatrix, std::vector<ScalarStencil> scalarStencilOnElementVertices);