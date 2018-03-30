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

bool operator==(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs)
{
	return lhs.coeff(0)==Approx(rhs.coeff(0)) &&
	       lhs.coeff(1)==Approx(rhs.coeff(1)) &&
	       lhs.coeff(2)==Approx(rhs.coeff(2));
}

#endif