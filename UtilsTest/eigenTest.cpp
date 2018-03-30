#include <Utils/Test.hpp>
#include <Eigen/Core>
#include <Eigen/LU>

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
	check(solution==matrix.fullPivLu().solve(independent));
}

TestCase("Eigen equality operator", "[eigen]")
{
	Eigen::Vector2d vector(0.0, 1.0);
	Eigen::Vector2d copyVector(vector);
	check(vector==copyVector);
}