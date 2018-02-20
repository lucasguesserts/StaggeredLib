#include <Eigen/Core>

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
};
