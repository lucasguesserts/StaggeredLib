#include <Utils/Test.hpp>
#include <Utils/RateOfConvergence.hpp>

TestCase("Rate of convergence with random error", "[RateOfConvergence]")
{
	const std::vector<double> error = {0.386092, 0.676570, 0.095323, 0.036136, 0.156207};
	const std::vector<double> characteristicLength = {1.0, 1.0/2.0, 1.0/4.0, 1.0/8.0, 1.0/16.0};
	RateOfConvergence rateOfConvergence(error, characteristicLength);
	checkFalse(rateOfConvergence.isAnalytical);
	checkFalse(rateOfConvergence.isLinear());
	// checkFalse(rateOfConvergence.converge()); // Warning, it converges in my analysis!
}

TestCase("Linear rate of convergence", "[RateOfConvergence]")
{
	const std::vector<double> error = {1.0, 1.0/2.0, 1.0/4.0, 1.0/8.0, 1.0/16.0};
	const std::vector<double> characteristicLength = {1.0, 1.0/2.0, 1.0/4.0, 1.0/8.0, 1.0/16.0};
	RateOfConvergence rateOfConvergence(error, characteristicLength);
	checkFalse(rateOfConvergence.isAnalytical);
	check(rateOfConvergence.isLinear());
	checkFalse(rateOfConvergence.isQuadratic());
	check(rateOfConvergence.converge());
}

TestCase("Quadratic rate of convergence", "[RateOfConvergence]")
{
	const std::vector<double> error = {1.0, 1.0/4.0, 1.0/16.0, 1.0/64.0, 1.0/256.0};
	const std::vector<double> characteristicLength = {1.0, 1.0/2.0, 1.0/4.0, 1.0/8.0, 1.0/16.0};
	RateOfConvergence rateOfConvergence(error, characteristicLength);
	checkFalse(rateOfConvergence.isAnalytical);
	check(rateOfConvergence.isQuadratic());
	check(rateOfConvergence.converge());
}

TestCase("Analytical rate of convergence", "[RateOfConvergence]")
{
	const std::vector<double> error = {1.0, 1.0/4.0, 1.0/16.0, 1.0/64.0, 1E-15};
	const std::vector<double> characteristicLength = {1.0, 1.0/2.0, 1.0/4.0, 1.0/8.0, 1.0/16.0};
	RateOfConvergence rateOfConvergence(error, characteristicLength);
	check(rateOfConvergence.isAnalytical);
	check(rateOfConvergence.isQuadratic());
	check(rateOfConvergence.converge());
}

TestCase("Divergent rate of convergence", "[RateOfConvergence]")
{
	const std::vector<double> error = {1.0, 1.1, 1.2, 1.3, 1.4};
	const std::vector<double> characteristicLength = {1.0, 1.0/2.0, 1.0/4.0, 1.0/8.0, 1.0/16.0};
	RateOfConvergence rateOfConvergence(error, characteristicLength);
	checkFalse(rateOfConvergence.isAnalytical);
	checkFalse(rateOfConvergence.isLinear());
	checkFalse(rateOfConvergence.converge());
}