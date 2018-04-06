#ifndef EIGEN_TEST_HPP
#define EIGEN_TEST_HPP

#include <Eigen/Core>
#include <utility>

#include <Utils/String.hpp>
#include <Utils/Test.hpp>

bool operator==(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs);
bool operator==(const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs);
bool operator==(const Eigen::MatrixXd& lhs, const Eigen::MatrixXd& rhs);

std::string eigenVector3dToString(const Eigen::Vector3d& vector);
namespace Catch
{
	template<>
	struct StringMaker<Eigen::Vector3d>
	{
		static std::string convert( Eigen::Vector3d const& vector )
		{
			return eigenVector3dToString(vector);
		}
	};
}

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

std::string eigenMatrixRowToString(const std::string& initialChars, const Eigen::MatrixXd& matrix, const unsigned row, const std::string& finalChars);
std::string eigenMatrixToString(const Eigen::MatrixXd& matrix);
namespace Catch {
	template<>
	struct StringMaker<Eigen::MatrixXd>
	{
		static std::string convert( Eigen::MatrixXd const& matrix )
		{
			return eigenMatrixToString(matrix);
		}
	};
}

#endif