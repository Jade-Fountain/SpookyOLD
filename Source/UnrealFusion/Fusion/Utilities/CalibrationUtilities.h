#include<iostream>
#include<string>
#include<Eigen/Core>
#include<Eigen/SVD>
#include<Eigen/Geometry>
//#include "Logging.h"
#pragma once
namespace fusion{
	namespace utility{

		//Source: https://fuyunfei1.gitbooks.io/c-tips/content/pinv_with_eigen.html
		template<typename _Matrix_Type_>
		_Matrix_Type_ pInv(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
		{
			Eigen::JacobiSVD< _Matrix_Type_ > svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
			double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
			return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
		}

		//Source:http://eigen.tuxfamily.org/index.php?title=FAQ#Is_there_a_method_to_compute_the_.28Moore-Penrose.29_pseudo_inverse_.3F
		//void pInv(MatrixType& pinvmat) const
		//{
		//	eigen_assert(m_isInitialized && "SVD is not initialized.");
		//	double  pinvtoler = 1.e-6; // choose your tolerance wisely!
		//	SingularValuesType singularValues_inv = m_singularValues;
		//	for (long i = 0; i<m_workMatrix.cols(); ++i) {
		//		if (m_singularValues(i) > pinvtoler)
		//			singularValues_inv(i) = 1.0 / m_singularValues(i);
		//		else singularValues_inv(i) = 0;
		//	}
		//	pinvmat = (m_matrixV*singularValues_inv.asDiagonal()*m_matrixU.transpose());
		//}

		class PositionalCalibration{
		public:
			//For calibrating a pair of systems with two sensors measuring the same point from two different reference frames
			// Xa = b
			// or XA = B
			static inline Eigen::Transform<float, 3, Eigen::Affine> calibrateIdenticalPair(const std::vector<Eigen::Vector3f>& samplesA, const std::vector<Eigen::Vector3f> samplesB){
				if (samplesA.size() != samplesB.size()) {
					throw std::runtime_error(std::string(__FILE__) + std::to_string(__LINE__) + std::string(" : samplesA and samplesB of different size"));
				}

				Eigen::MatrixXf A(4, samplesA.size());
				Eigen::MatrixXf B(4, samplesB.size());

				for (int i = 0; i < samplesA.size(); i++) {
					A.col(i) << samplesA[i] , 1;
					B.col(i) << samplesB[i] , 1;
				}
				
				//Make sure normalised
				Eigen::Matrix4f X = B * pInv(A);
				Eigen::Quaternionf q(X.block<3,3>(0,0));
				q.normalize();
				Eigen::Translation3f t(X.block<3,1>(0,3));
				
				Eigen::Transform<float, 3, Eigen::Affine> TX(t);
				TX.rotate(q);

				return TX;
			}	


		};



	}
}
