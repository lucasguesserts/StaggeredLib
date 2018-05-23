#ifndef RATE_OF_CONVERGENCE_HPP
#define RATE_OF_CONVERGENCE_HPP

#include <vector>

class RateOfConvergence
{
	public:
		RateOfConvergence(const std::vector<double>& error, const std::vector<double>& characteristicLength);
		bool converge(void);
		bool isLinear(void);
		bool isQuadratic(void);
		bool diverge(void);

		bool isAnalytical; // if some error is less than 1E-13
		double order;
	private:
		bool isEqual(const double& lhs, const double& rhs);
		void verifyInput(const std::vector<double>& error, const std::vector<double>& characteristicLength);
		void verifyCharacteristicLength(const std::vector<double>& characteristicLength);
		void verifyIfConvergenceIsAnalytical(const std::vector<double>& error);
		void buildOrderOfConvergence(const std::vector<double>& error, const std::vector<double>& characteristicLength);
};

#endif