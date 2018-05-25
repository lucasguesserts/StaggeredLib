#include <vector>
#include <chrono>
#include <iostream>
#include <Utils/Test.hpp>

#include <Utils/EigenTest.hpp>
#include <Utils/RateOfConvergence.hpp>
#include <LinearSystem/EigenSparseLinearSystem.hpp>
#include <LinearSystem/EigenLinearSystem.hpp>

#include <LinearSystemTest/FiniteDifferenceEnvironment.hpp>

TestCase("Compare sparse and dense solvers", "[EigenSparseLinearSystem]")
{
	// initialization
	constexpr unsigned numberOfPoints = 100u;
	constexpr double initialValue = 0.0;
	constexpr double finalValue = 1.0;
	constexpr double initialPosition = 0.0;
	constexpr double finalPosition = 1.0;

	// Sparse
	EigenSparseLinearSystem sparseLinearSystem(numberOfPoints);
	buildSparseMatrix(sparseLinearSystem, numberOfPoints);
	buildIndependent(sparseLinearSystem.independent, numberOfPoints, initialValue, finalValue);
	auto sparseStart = std::chrono::high_resolution_clock::now();
	sparseLinearSystem.computeLU();
	auto sparseSolution = sparseLinearSystem.solve();
	auto sparseEnd = std::chrono::high_resolution_clock::now();
	double sparseDuration = std::chrono::duration<double>(sparseEnd-sparseStart).count();

	// Dense
	EigenLinearSystem denseLinearSystem;
	buildDenseMatrix(denseLinearSystem, numberOfPoints);
	buildIndependent(denseLinearSystem.independent, numberOfPoints, initialValue, finalValue);
	auto denseStart = std::chrono::high_resolution_clock::now();
	auto denseSolution = denseLinearSystem.solve();
	auto denseEnd = std::chrono::high_resolution_clock::now();
	double denseDuration = std::chrono::duration<double>(denseEnd-denseStart).count();

	// Validation
	auto analyticalSolution = computeAnalyticalSolution(numberOfPoints, initialValue, finalValue, initialPosition, finalPosition);
	require(denseSolution==analyticalSolution);
	require(sparseSolution==analyticalSolution);
	check(sparseDuration<denseDuration);
	std::cout << "sparse duration = " << sparseDuration << std::endl;
	std::cout << "dense  duration = " << denseDuration << std::endl;
}