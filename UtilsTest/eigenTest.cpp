#include <Utils/Test.hpp>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Utils/EigenTest.hpp>

TestCase("eigen vector", "[eigen]")
{
	const unsigned size = 2;
	unsigned value[size] = {0,1};
	Eigen::VectorXd vector;
	vector.resize(size);
	for(unsigned i=0 ; i<size ; ++i)
		vector[i] = value[i];
	for(unsigned i=0 ; i<size ; ++i)
		check(vector[i]==value[i]);
}

TestCase("eigen vector 3D", "[eigen]")
{
	Eigen::Vector3d vector(1,2,3);
	for(unsigned i=0 ; i<3 ; ++i)
		check(vector[i]==(i+1));
}

TestCase("Matrix solver", "[eigen]")
{
	const unsigned size = 2;
	Eigen::MatrixXd matrix;
		matrix.resize(size, size);
		matrix << 2, -1, -1, 2;
	Eigen::VectorXd independent;
		independent.resize(size);
		independent << 0, 12;
	Eigen::VectorXd solution;
		solution.resize(size);
		solution << 4, 8;
	Eigen::VectorXd calculatedSolution = matrix.fullPivLu().solve(independent);
	check(solution==calculatedSolution);
}

TestCase("Eigen equality operator", "[eigen]")
{
	Eigen::Vector2d vector(0.0, 1.0);
	Eigen::Vector2d copyVector(vector);
	check(vector==copyVector);
}

TestCase("Vector3d operator==", "[eigen]")
{
    std::array<Eigen::Vector3d,3> vectors = {{
        {0.0, 1.9999999, 0.0},
        {0.0, 2.0000000, 0.0},
        {0.0, 0.0000000, 0.0}
    }};
    Eigen::Vector3d vec0(0,0,1);
    Eigen::Vector3d vec1(0,0,2);
    check(vectors[0]==vectors[1]);
    checkFalse(vectors[0]==vectors[2]);
}

TestCase("VectorXd operator==", "[eigen]")
{
	std::array<Eigen::VectorXd,6> vectors;
	vectors[0].resize(3); vectors[0] << 2.4, 1.0, 8.6;
	vectors[1].resize(3); vectors[1] << 2.4, 1.0, 8.6;
	vectors[2].resize(3); vectors[2] << 2.4, 1.0, 8.6000001;
	vectors[3].resize(3); vectors[3] << 2.4, 1.0, -3;
	vectors[4].resize(4); vectors[4] << 2.4, 1.0, 8.6, 9.4;
	vectors[5].resize(2); vectors[5] << 2.4, 1.0;
	section("true tests")
	{
		check(vectors[0]==vectors[1]);
		check(vectors[0]==vectors[2]);
	}
	section("false test")
	{
		checkFalse(vectors[0]==vectors[3]);
		checkFalse(vectors[0]==vectors[4]);
		checkFalse(vectors[0]==vectors[5]);
	}
}