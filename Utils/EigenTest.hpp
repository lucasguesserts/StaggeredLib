#ifndef EIGEN_TEST_HPP
#define EIGEN_TEST_HPP

#include <Eigen/Core>
#include <string>
#include <cstdio>
#include <utility>
#include <iostream>
#include <boost/format.hpp>

#include <Utils/Test.hpp>

namespace Catch {
	template<>
		struct StringMaker<Eigen::Vector3d> {
		static std::string convert( Eigen::Vector3d const& vector ) {
			char buf[100];
			std::sprintf(buf,
				"[%10.10le,%10.10le,%10.10le]",
				vector.coeff(0),
				vector.coeff(1),
				vector.coeff(2) );
			return std::string(std::move(buf));
			}
	   };
}

bool operator==(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs);

std::string eigenVectorToString(const Eigen::VectorXd& vector);
namespace Catch
{
	template<>
	struct StringMaker<Eigen::VectorXd> {
		static std::string convert( Eigen::VectorXd const& vector )
		{
			return eigenVectorToString(vector);
		}
	};
}

bool operator==(const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs);


bool operator==(const Eigen::MatrixXd& lhs, const Eigen::MatrixXd& rhs);

std::string eigenMatrixRowToString(const std::string& initialChars, const Eigen::MatrixXd& matrix, const unsigned row, const std::string& finalChars);
namespace Catch {
	template<>
	struct StringMaker<Eigen::MatrixXd>
	{
		static std::string convert( Eigen::MatrixXd const& matrix )
		{
			// Format:
			// [2.4000000000e+00, 1.0000000000e+00,
			//  8.6000000000e+00, 5.7000000000e+00,
			//  4.1000000000e+00, 9.0000000000e-01]
			constexpr int numberStringSize = 18 + 2;
			int stringSize = numberStringSize*matrix.size() + matrix.rows();
			std::string stringToPrint;
			unsigned row;
			row = 0;
				stringToPrint += eigenMatrixRowToString("[", matrix, row, ",\n");
			for(row=1 ; row<(matrix.rows()-1) ; ++row)
				stringToPrint += eigenMatrixRowToString(" ", matrix, row, ",\n");
			row = matrix.rows()-1;
				stringToPrint += eigenMatrixRowToString(" ", matrix, row, "]");
			return stringToPrint;
		}
	};
}

#endif