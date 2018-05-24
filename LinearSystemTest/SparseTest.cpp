#include <vector>
#include <Utils/Test.hpp>

#include <Utils/RateOfConvergence.hpp>
#include <LinearSystem/EigenSparseLinearSystem.hpp>

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
		// Matrix coefficients
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
		// Independent
		linearSystem.independent[0] = initialValue;
		linearSystem.independent[numberOfPoints-1] = finalValue;
		Eigen::VectorXd numericalSolution = linearSystem.solve();
		// Analytical solution
		auto computeAnalyticalSolution = [numberOfPoints, initialValue, finalValue, initialPosition, finalPosition]() -> Eigen::VectorXd
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
		Eigen::VectorXd analyticalSolution = computeAnalyticalSolution();
		// Compare
		error.push_back((numericalSolution - analyticalSolution).lpNorm<Eigen::Infinity>());
		characteristicLength.push_back((finalValue - initialValue) / (numberOfPoints - 1));
	}
	RateOfConvergence rateOfConvergence(error, characteristicLength);
	check(rateOfConvergence.converge());
	check(rateOfConvergence.isAnalytical);
}