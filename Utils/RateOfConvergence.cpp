#include <Utils/RateOfConvergence.hpp>
#include <cmath>
#include <stdexcept>
#include <numeric>
#include <limits>

RateOfConvergence::RateOfConvergence(const std::vector<double>& error, const std::vector<double>& characteristicLength)
{
	this->verifyInput(error, characteristicLength);
	this->verifyCharacteristicLength(characteristicLength);
	this->verifyIfConvergenceIsAnalytical(error);
	if(! this->isAnalytical)
		this->buildOrderOfConvergence(error, characteristicLength);
	else
		this->order = std::numeric_limits<double>::infinity();
	return;
}

void RateOfConvergence::verifyInput(const std::vector<double>& error, const std::vector<double>& characteristicLength)
{
	if(error.size()<=1)
		throw std::runtime_error("Rate of convergence error: error vector has only one entry.");
	if(error.size()!=characteristicLength.size())
		throw std::runtime_error("Rate of convergence error: error and characteristic length vectors size mismatch.");
	return;
}

void RateOfConvergence::verifyCharacteristicLength(const std::vector<double>& characteristicLength)
{
	for(unsigned count=0 ; count<characteristicLength.size()-1 ; ++count)
		if(this->isEqual(characteristicLength[count], characteristicLength[count+1]))
			throw std::runtime_error("Rate of convergence error: characteristic length have repeated values.");
	return;
}

bool RateOfConvergence::isEqual(const double& lhs, const double& rhs)
{
	return ( lhs==0.0 && rhs==0.0 ) ||
	       ( std::fabs( (lhs - rhs) / (std::fabs(lhs) + std::fabs(rhs)) ) < 1E-10 );
}

void RateOfConvergence::verifyIfConvergenceIsAnalytical(const std::vector<double>& error)
{
	this->isAnalytical = false;
	for(auto& value: error)
	{
		if(value < 1E-13)
		{
			this->isAnalytical = true;
			break;
		}
	}
	return;
}

void RateOfConvergence::buildOrderOfConvergence(const std::vector<double>& error, const std::vector<double>& characteristicLength)
{
	const unsigned orderVectorSize = error.size()-1;
	std::vector<double> order(orderVectorSize, 0.0);
	for(unsigned count=0 ; count<orderVectorSize ; ++count)
	{
		order[count] = std::log(error[count]/error[count+1]) / std::log(characteristicLength[count]/characteristicLength[count+1]);
	}
	this->order = std::accumulate(order.cbegin(), order.cend(), 0.0) / orderVectorSize;
	return;
}

bool RateOfConvergence::converge(void)
{
	return this->order > 0.0;
}

bool RateOfConvergence::isLinear(void)
{
	return this->order >= 1.0;
}

bool RateOfConvergence::isQuadratic(void)
{
	return this->order >= 2.0;
}

bool RateOfConvergence::diverge(void)
{
	return this->order <= 0.0;
}