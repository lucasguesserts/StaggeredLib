#include <vector>
#include <chrono>
#include <Utils/Test.hpp>
#include <iostream>

#include <Utils/EigenTest.hpp>
#include <Utils/RateOfConvergence.hpp>
#include <LinearSystem/EigenSparseLinearSystem.hpp>
#include <LinearSystem/EigenLinearSystem.hpp>

TestCase("Compare sparse and dense solvers", "[EigenSparseLinearSystem]")
{
	// initialization
	constexpr unsigned numberOfPoints = 100u;
	constexpr double initialValue = 0.0;
	constexpr double finalValue = 1.0;
	constexpr double initialPosition = 0.0;
	constexpr double finalPosition = 1.0;
	// Functions
	auto buildSparseMatrix = [](EigenSparseLinearSystem& linearSystem, const unsigned numberOfPoints) -> void
	{
		const double neg = -1.0;
		const double pos = 2.0;
		const unsigned numberOfCoefficients = 1 + 3*(numberOfPoints-2) + 1;
		linearSystem.coefficients.reserve(numberOfCoefficients);
		linearSystem.coefficients.push_back(Eigen::Triplet<double>(0, 0, 1.0));
		for(unsigned i=1 ; i<(numberOfPoints-1) ; ++i)
		{
			linearSystem.coefficients.push_back(Eigen::Triplet<double>(i, i-1, neg));
			linearSystem.coefficients.push_back(Eigen::Triplet<double>(i, i, pos));
			linearSystem.coefficients.push_back(Eigen::Triplet<double>(i, i+1, neg));
		}
		linearSystem.coefficients.push_back(Eigen::Triplet<double>(numberOfPoints-1, numberOfPoints-1, 1.0));
		return;
	};
	auto buildDenseMatrix = [](EigenLinearSystem& linearSystem, const unsigned numberOfPoints) -> void
	{
		linearSystem.setSize(numberOfPoints);
		const double neg = -1.0;
		const double pos = 2.0;
		linearSystem.matrix(0, 0) = 1.0;
		for(unsigned i=1 ; i<(numberOfPoints-1) ; ++i)
		{
			linearSystem.matrix(i, i-1) = neg;
			linearSystem.matrix(i, i) = pos;
			linearSystem.matrix(i, i+1) = neg;
		}
		linearSystem.matrix(numberOfPoints-1, numberOfPoints-1) = 1.0;
		return;
	};
	auto buildIndependent = [](Eigen::VectorXd& independent, const unsigned numberOfPoints, double initialValue, double finalValue) -> void
	{
		independent[0] = initialValue;
		independent[numberOfPoints-1] = finalValue;
		return;
	};
	auto computeAnalyticalSolution = [](unsigned numberOfPoints, double initialValue, double finalValue, double initialPosition, double finalPosition) -> Eigen::VectorXd
	{
		const double angularCoefficient = (finalValue - initialValue) / (finalPosition - initialPosition);
		const double dx = (finalValue - initialValue) / (numberOfPoints - 1);
		Eigen::VectorXd analytical = Eigen::VectorXd::Zero(numberOfPoints);
		for(unsigned i=0 ; i<numberOfPoints ; ++i)
		{
			double position = i * dx;
			analytical[i] = angularCoefficient * (position - initialPosition) + initialValue;
		}
		return analytical;
	};
	// Sparse
	EigenSparseLinearSystem sparseLinearSystem(numberOfPoints);
	buildSparseMatrix(sparseLinearSystem, numberOfPoints);
	buildIndependent(sparseLinearSystem.independent, numberOfPoints, initialValue, finalValue);
	auto sparseStart = std::chrono::high_resolution_clock::now();
	Eigen::VectorXd sparseSolution = sparseLinearSystem.solve();
	auto sparseEnd = std::chrono::high_resolution_clock::now();
	double sparseDuration = std::chrono::duration<double>(sparseEnd-sparseStart).count();
	// Dense
	EigenLinearSystem denseLinearSystem;
	buildDenseMatrix(denseLinearSystem, numberOfPoints);
	buildIndependent(denseLinearSystem.independent, numberOfPoints, initialValue, finalValue);
	auto denseStart = std::chrono::high_resolution_clock::now();
	Eigen::VectorXd denseSolution = denseLinearSystem.solve();
	auto denseEnd = std::chrono::high_resolution_clock::now();
	double denseDuration = std::chrono::duration<double>(denseEnd-denseStart).count();
	// Validation
	Eigen::VectorXd analyticalSolution = computeAnalyticalSolution(numberOfPoints, initialValue, finalValue, initialPosition, finalPosition);
	require(denseSolution==analyticalSolution);
	require(sparseSolution==analyticalSolution);
	check(sparseDuration<denseDuration);
	std::cout << "sparse duration = " << sparseDuration << std::endl;
	std::cout << "dense  duration = " << denseDuration << std::endl;
}