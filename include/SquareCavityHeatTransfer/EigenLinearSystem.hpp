#include <Eigen/Core>
#include <Eigen/LU>

class EigenLinearSystem
{
	public:
		void setSize(const unsigned size)
		{
			this->matrix.resize(size,size);
			this->independent.resize(size);
			return;
		}
		Eigen::MatrixXd matrix;
		Eigen::VectorXd independent;
		Eigen::VectorXd solve(void)
		{
			return this->matrix.fullPivLu().solve(this->independent);
		}
};
