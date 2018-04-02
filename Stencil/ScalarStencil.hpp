#ifndef SCALAR_MAP_HPP
#define SCALAR_MAP_HPP

#include <Utils/catch.hpp>
#include <cstring>
#include <map>
#include <iostream>
#include <Eigen/Core>

using ScalarStencil = std::map<unsigned,double>;

ScalarStencil operator+(const ScalarStencil& lhs, const ScalarStencil& rhs);
ScalarStencil operator*(const double scalar, const ScalarStencil& scalarMap);
double operator*(const ScalarStencil& scalarStencil, const Eigen::VectorXd& scalarField);

bool operator==(const ScalarStencil& lhs, const ScalarStencil& rhs);

namespace Catch{
	template<>
	struct StringMaker<ScalarStencil>{
		static std::string convert( ScalarStencil const& scalarStencil ){
            constexpr int pairBufSize = (3+7+18);
			int bufSize = 15 + pairBufSize*scalarStencil.size();
			char* buf = new char[bufSize]; std::strcpy(buf,"ScalarStencil{");
            char pairBuf[pairBufSize];
			for(auto& keyValuePair: scalarStencil)
            {
				std::sprintf(pairBuf, "{%u,%+10.10le},", keyValuePair.first, keyValuePair.second);
                std::strcat(buf,pairBuf);
            }
            std::strcat(buf,"}");
            std::string message(buf);
            delete[] buf;
			return message;
		}
	};
}

#endif
