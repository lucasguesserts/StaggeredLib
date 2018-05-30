#include <vector>
#include <chrono>
#include <Utils/Test.hpp>

#include <Utils/EigenTest.hpp>
#include <Utils/RateOfConvergence.hpp>
#include <LinearSystem/EigenSparseLinearSystem.hpp>

#include <LinearSystemTest/FiniteDifferenceEnvironment.hpp>

TestCase("sparse set size test", "[SparseLinearSystem]")
{
	EigenSparseLinearSystem linearSystem;
	constexpr unsigned size = 7;
	linearSystem.setSize(size);
	check(linearSystem.matrix.rows()==size);
	check(linearSystem.matrix.cols()==size);
	check(linearSystem.matrix.nonZeros()==0);
	check(linearSystem.independent.size()==size);
}

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
		linearSystem.computeLU();
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
		linearSystem.computeLU();
		auto numericalSolution = linearSystem.solve();
		auto analyticalSolution = computeAnalyticalSolution(numberOfPoints, initialValue, finalValue, initialPosition, finalPosition);
		error.push_back((numericalSolution - analyticalSolution).lpNorm<Eigen::Infinity>());
		characteristicLength.push_back((finalValue - initialValue) / (numberOfPoints - 1));
	}
	RateOfConvergence rateOfConvergence(error, characteristicLength);
	check(rateOfConvergence.converge());
	check(rateOfConvergence.isAnalytical);
}

TestCase("Compare sparse linear system with and without lu decomposition", "[EigenSparseLinearSystem]")
{
	constexpr unsigned numberOfTimesToRepeat = 10;
	constexpr unsigned numberOfPoints = 1e+3;
	constexpr double initialValue = 0.0;
	constexpr double finalValue = 1.0;
	constexpr double initialPosition = 0.0;
	constexpr double finalPosition = 1.0;
	std::chrono::high_resolution_clock::time_point noDecomposeStart, noDecomposeEnd, decomposeStart, decomposeEnd;
	double noDecomposeDuration, decomposeDuration;
	// no decomposed linear system
	{
		EigenSparseLinearSystem linearSystem(numberOfPoints);
		buildSparseMatrix(linearSystem, numberOfPoints);
		buildIndependent(linearSystem.independent, numberOfPoints, initialValue, finalValue);
		linearSystem.matrix.setFromTriplets(linearSystem.coefficients.begin(), linearSystem.coefficients.end());
		linearSystem.matrix.makeCompressed();
		noDecomposeStart = std::chrono::high_resolution_clock::now();
		for(unsigned count=0 ; count<numberOfTimesToRepeat ; ++count)
		{
			Eigen::SparseLU<Eigen::SparseMatrix<double>> luDecomposition;
			luDecomposition.analyzePattern(linearSystem.matrix);
			luDecomposition.factorize(linearSystem.matrix);
			luDecomposition.solve(linearSystem.independent);
		}
		noDecomposeEnd = std::chrono::high_resolution_clock::now();
		noDecomposeDuration = std::chrono::duration<double>(noDecomposeEnd-noDecomposeStart).count();
		// without lu decomposition
		linearSystem.matrix.setFromTriplets(linearSystem.coefficients.begin(), linearSystem.coefficients.end());
		linearSystem.matrix.makeCompressed();
		auto noDecomposeStart = std::chrono::high_resolution_clock::now();
		for(unsigned count=0 ; count<numberOfTimesToRepeat ; ++count)
		{
			Eigen::SparseLU<Eigen::SparseMatrix<double>> luDecomposition;
			luDecomposition.analyzePattern(linearSystem.matrix);
			luDecomposition.factorize(linearSystem.matrix);
			luDecomposition.solve(linearSystem.independent);
		}
		auto noDecomposeEnd = std::chrono::high_resolution_clock::now();
		double noDecomposeDuration = std::chrono::duration<double>(noDecomposeEnd-noDecomposeStart).count();
	}
	// decomposed linear system
	{
		EigenSparseLinearSystem linearSystem(numberOfPoints);
		buildSparseMatrix(linearSystem, numberOfPoints);
		buildIndependent(linearSystem.independent, numberOfPoints, initialValue, finalValue);
		// with lu decomposition
		decomposeStart = std::chrono::high_resolution_clock::now();
		linearSystem.computeLU();
		for(unsigned count=0 ; count<numberOfTimesToRepeat ; ++count)
			linearSystem.solve();
		decomposeEnd = std::chrono::high_resolution_clock::now();
		decomposeDuration = std::chrono::duration<double>(decomposeEnd-decomposeStart).count();
	}
	// check
	check(decomposeDuration<noDecomposeDuration);
	std::cout << "sparse no decompose duration: " << noDecomposeDuration << std::endl;
	std::cout << "sparse decomposes duration  : " << decomposeDuration << std::endl;
}