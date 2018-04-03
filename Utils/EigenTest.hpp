#ifndef EIGEN_TEST_HPP
#define EIGEN_TEST_HPP

#include <Eigen/Core>
#include <string>
#include <cstdio>
#include <utility>

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

#endif