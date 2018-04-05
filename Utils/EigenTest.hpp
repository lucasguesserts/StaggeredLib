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

namespace Catch {
	template<>
		struct StringMaker<Eigen::VectorXd> {
		static std::string convert( Eigen::VectorXd const& vector ) {
			constexpr int tempBufSize = 18 + 1;
			char tempBuf[tempBufSize];
			int bufSize = 1 + tempBufSize*vector.size();
			char* buf = new char[bufSize];
			unsigned entry;
			std::strcpy(buf,"[");
			for(entry=0 ; entry<(vector.size()-1) ; entry++)
			{
				std::sprintf(tempBuf,"%10.10le,", vector.coeff(entry));
				std::strcat(buf,tempBuf);
			}
			entry = (vector.size()-1);
				std::sprintf(tempBuf,"%10.10le]", vector.coeff(entry));
				std::strcat(buf,tempBuf);
			std::string message(buf);
			delete[] buf;
			return message;
			}
		};
}

bool operator==(const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs);


bool operator==(const Eigen::MatrixXd& lhs, const Eigen::MatrixXd& rhs);

std::string rowToString(const std::string& initialChars, const Eigen::MatrixXd& matrix, const unsigned row, const std::string& finalChars);
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
				stringToPrint += rowToString("[", matrix, row, ",\n");
			for(row=1 ; row<(matrix.rows()-1) ; ++row)
				stringToPrint += rowToString(" ", matrix, row, ",\n");
			row = matrix.rows()-1;
				stringToPrint += rowToString(" ", matrix, row, "]");
			return stringToPrint;
		}
	};
}

#endif