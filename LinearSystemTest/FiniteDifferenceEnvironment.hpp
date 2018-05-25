#ifndef FINITE_DIFFERENCE_ENVIRONMENT_HPP
#define FINITE_DIFFERENCE_ENVIRONMENT_HPP

#include <Stencil/ScalarStencil.hpp>
#include <LinearSystem/EigenSparseLinearSystem.hpp>
#include <LinearSystem/EigenLinearSystem.hpp>

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

auto buildSparseMatrixWithScalarStencil = [](EigenSparseLinearSystem& linearSystem, const unsigned numberOfPoints) -> void
{
	ScalarStencil scalarStencil;
	const double neg = -1.0;
	const double pos = 2.0;
	linearSystem.addScalarStencil(0, ScalarStencil{{0,1.0}});
	for(unsigned i=1 ; i<(numberOfPoints-1) ; ++i)
	{
		linearSystem.addScalarStencil(i, ScalarStencil{{i-1,neg}});
		linearSystem.addScalarStencil(i, ScalarStencil{{i  ,pos/2.0}});
		linearSystem.addScalarStencil(i, ScalarStencil{{i  ,pos/2.0}});
		linearSystem.addScalarStencil(i, ScalarStencil{{i+1,neg}});
	}
	linearSystem.addScalarStencil(numberOfPoints-1, ScalarStencil{{numberOfPoints-1,1.0}});
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

#endif