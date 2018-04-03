#ifndef VECTOR_STENCIL_HPP
#define VECTOR_STENCIL_HPP

#include <map>
#include <Eigen/Core>
#include <Stencil/ScalarStencil.hpp>
#include <string>
#include <cstring>

using VectorStencil = std::map<unsigned,Eigen::Vector3d>;

VectorStencil operator*(const ScalarStencil& scalarStencil, const Eigen::Vector3d& vector);
VectorStencil operator+(const VectorStencil& lhs, const VectorStencil& rhs);
ScalarStencil operator*(const Eigen::Vector3d& vector, const VectorStencil& vectorStencil);
VectorStencil operator*(const double scalar, const VectorStencil& vectorStencil);
Eigen::Vector3d operator*(const VectorStencil& vectorStencil, const Eigen::VectorXd& scalarField);

bool operator==(const VectorStencil& lhs, const VectorStencil& rhs);

namespace Catch{
	template<>
	struct StringMaker<VectorStencil>{
		static std::string convert( VectorStencil const& vectorStencil ){
            constexpr int pairBufSize = 3*(7+7+18);
			int bufSize = 15 + pairBufSize*vectorStencil.size();
			char* buf = new char[bufSize]; std::strcpy(buf,"VectorStencil{");
            char pairBuf[pairBufSize];
			for(auto& keyValuePair: vectorStencil)
            {
				std::sprintf(pairBuf, "{%u,[%+10.10le,%+10.10le,%+10.10le]},",
					keyValuePair.first,
					keyValuePair.second.coeff(0),
					keyValuePair.second.coeff(1),
					keyValuePair.second.coeff(2));
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