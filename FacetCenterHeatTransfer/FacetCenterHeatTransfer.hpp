#include <vector>
#include <Eigen/Core>
#include <Grid/Grid2DInverseDistanceStencil.hpp>
#include <Grid/DirichletBoundaryCondition.hpp>
#include <LinearSystem/EigenSparseLinearSystem.hpp>
#include <Stencil/VectorStencil.hpp>

class FacetCenterHeatTransfer
{
	public:
		FacetCenterHeatTransfer(const std::string& fileName);

		void addDiffusiveTerm(void);
		void applyBoundaryConditions(void);

		void insertDirichletBoundaryCondition(const std::string& boundaryName, const std::function<double(Eigen::Vector3d)> prescribedValueFunction);
		void insertDirichletBoundaryCondition(const std::string& boundaryName, const double prescribedValue);

		Grid2DInverseDistanceStencil grid2D;
		EigenSparseLinearSystem linearSystem;
		Eigen::VectorXd temperature;
		std::vector<ScalarStencil> scalarStencilOnVertices;
		std::vector<ScalarStencil> scalarStencilOnElements;
		std::vector<VectorStencil> gradientOnFaces;
		std::vector<DirichletBoundaryCondition> dirichletBoundaries;

		std::vector<ScalarStencil> getScalarStencilOnElementVertices(Face2D& face);

	private:
		void initializeLinearSystem(void);
		void initializeScalarStencilOnVertices(void);
		void initializeScalarStencilOnElements(void);
		void initializeGradientOnFaces(void);
		void applyBoundaryConditionsToMatrix(void);
		void applyBoundaryConditionsToIndependent(void);
};

VectorStencil operator*(Eigen::MatrixXd gradientMatrix, std::vector<ScalarStencil> scalarStencilOnElementVertices);