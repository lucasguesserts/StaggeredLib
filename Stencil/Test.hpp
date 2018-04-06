#ifndef STENCIL_TEST_HPP
#define STENCIL_TEST_HPP

#include <Utils/Test.hpp>
#include <Utils/String.hpp>
#include <Utils/EigenTest.hpp>
#include <iostream>
#include <iomanip>

#include <Stencil/ScalarStencil.hpp>
#include <Stencil/VectorStencil.hpp>

bool operator==(const ScalarStencil& lhs, const ScalarStencil& rhs);
bool operator==(const VectorStencil& lhs, const VectorStencil& rhs);

std::string scalarStencilToString(const ScalarStencil& scalarStencil);
namespace Catch
{
	template<>
	struct StringMaker<ScalarStencil>
	{
		static std::string convert( ScalarStencil const& scalarStencil )
		{
			return scalarStencilToString(scalarStencil);
		}
	};
}

std::string vectorStencilPairToString(const std::string& initialChars, const std::pair<unsigned,Eigen::Vector3d>& pair, const std::string& finalChars);
std::string vectorStencilToString(const VectorStencil& vectorStencil);
namespace Catch
{
	template<>
	struct StringMaker<VectorStencil>
	{
		static std::string convert( VectorStencil const& vectorStencil )
		{
			return vectorStencilToString(vectorStencil);
		}
	};
}

#endif