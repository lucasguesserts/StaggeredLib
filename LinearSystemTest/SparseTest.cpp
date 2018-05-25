#include <vector>
#include <Utils/Test.hpp>

#include <Utils/EigenTest.hpp>
#include <Utils/RateOfConvergence.hpp>
#include <LinearSystem/EigenSparseLinearSystem.hpp>

#include <LinearSystemTest/FiniteDifferenceEnvironment.hpp>


TestCase("Solving finite differences problem", "[EigenSparseLinearSystem]")
{
	std::vector<double> error, characteristicLength;
	for(const auto numberOfPoints: std::vector<unsigned>{3u, 10u, 20u, 25u})
	{
		constexpr double initialValue = 0.0;
		constexpr double finalValue = 1.0;
		constexpr double initialPosition = 0.0;
		constexpr double finalPosition = 1.0;
		EigenSparseLinearSystem linearSystem(numberOfPoints);
		buildSparseMatrix(linearSystem, numberOfPoints);
		buildIndependent(linearSystem.independent, numberOfPoints, initialValue, finalValue);
		auto numericalSolution = linearSystem.solve();
		auto analyticalSolution = computeAnalyticalSolution(numberOfPoints, initialValue, finalValue, initialPosition, finalPosition);
		error.push_back((numericalSolution - analyticalSolution).lpNorm<Eigen::Infinity>());
		characteristicLength.push_back((finalValue - initialValue) / (numberOfPoints - 1));
	}
	RateOfConvergence rateOfConvergence(error, characteristicLength);
	check(rateOfConvergence.converge());
	check(rateOfConvergence.isAnalytical);
}

TestCase("Build sparse linear system using ScalarStencil", "[EigenSparseLinearSystem]")
{
	std::vector<double> error, characteristicLength;
	for(const auto numberOfPoints: std::vector<unsigned>{3u, 10u, 20u, 25u})
	{
		constexpr double initialValue = 0.0;
		constexpr double finalValue = 1.0;
		constexpr double initialPosition = 0.0;
		constexpr double finalPosition = 1.0;
		EigenSparseLinearSystem linearSystem(numberOfPoints);
		buildSparseMatrixWithScalarStencil(linearSystem, numberOfPoints);
		buildIndependent(linearSystem.independent, numberOfPoints, initialValue, finalValue);
		auto numericalSolution = linearSystem.solve();
		auto analyticalSolution = computeAnalyticalSolution(numberOfPoints, initialValue, finalValue, initialPosition, finalPosition);
		error.push_back((numericalSolution - analyticalSolution).lpNorm<Eigen::Infinity>());
		characteristicLength.push_back((finalValue - initialValue) / (numberOfPoints - 1));
	}
	RateOfConvergence rateOfConvergence(error, characteristicLength);
	check(rateOfConvergence.converge());
	check(rateOfConvergence.isAnalytical);
}