#include <Utils/Test.hpp>
#include <Utils/EigenTest.hpp>
#include <Stencil/Test.hpp>

#include <Terzaghi/Terzaghi.hpp>

TestCase("Displacement stress term")
{
	const std::string gridFile = gridDirectory + "two_triangles.cgns";
	Terzaghi terzaghi(gridFile);
	terzaghi.shearModulus = 3.6;
	terzaghi.poissonCoefficient = 0.2;
	section("physical properties matrix")
	{
		Eigen::MatrixXd physicalPropertiesMatrix(6,6);
		physicalPropertiesMatrix <<
			9.6, 2.4, 0.0, 0.0, 0.0, 0.0,
			2.4, 9.6, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 3.6, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		check(terzaghi.getPhysicalPropertiesMatrix()==physicalPropertiesMatrix);
	}
	section("matrix")
	{
		terzaghi.displacementGradient = {
			{ {0, {5.3, -1.4, 0.0}}, {2, {-8.8, -2.9, 0.0}} },
			{ {0, Eigen::Vector3d::Zero()} },
			{ {0, Eigen::Vector3d::Zero()} },
			{ {0, Eigen::Vector3d::Zero()} },
			{ {1, {-2.9, 5.6, 0.0}}, {2, {4.2, -5.5, 0.0}} },
			{ {0, Eigen::Vector3d::Zero()} }
		};
		std::vector<Eigen::Triplet<double,unsigned>> triplets = {
			// face 0
				// staggered element 0
				{2, 2, -40.64},
				{2, 4, +42.40},
				{2, 7, +27.68},
				{2, 9, -37.60},
				{7, 2, +20.32},
				{7, 4, -21.20},
				{7, 7, -30.64},
				{7, 9, -16.00},
				// staggered element 2
				{4, 2, +40.64},
				{4, 4, -42.40},
				{4, 7, -27.68},
				{4, 9, +37.60},
				{9, 2, -20.32},
				{9, 4, +21.20},
				{9, 7, +30.64},
				{9, 9, +16.00},
			// face 4
				// staggered element 3
				{ 5, 3, -45.44},
				{ 5, 4, +53.28},
				{ 5, 8, +22.88},
				{ 5, 9, -28.96},
				{10, 3, +22.72},
				{10, 4, -26.64},
				{10, 8, -78.64},
				{10, 9, +80.48},
				// staggered element 2
				{ 4, 3, +45.44},
				{ 4, 4, -53.28},
				{ 4, 8, -22.88},
				{ 4, 9, +28.96},
				{ 9, 3, -22.72},
				{ 9, 4, +26.64},
				{ 9, 8, +78.64},
				{ 9, 9, -80.48},
		};
		Eigen::SparseMatrix<double> matrix(terzaghi.linearSystemSize,terzaghi.linearSystemSize);
		matrix.setFromTriplets(triplets.cbegin(), triplets.cend());
		terzaghi.insertDisplacementTensionTermInMatrix();
		terzaghi.linearSystem.assemblyMatrix();
		check(terzaghi.linearSystem.matrix==matrix);
	}
}