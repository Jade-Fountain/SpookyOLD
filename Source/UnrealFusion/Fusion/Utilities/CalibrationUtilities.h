#include<iostream>
#include<string>
#include<Eigen/Core>
#include<Eigen/SVD>
#include<Eigen/Geometry>
//#include "Logging.h"
#pragma once
namespace fusion{
	namespace utility{
		namespace calibration {

			//Source: https://fuyunfei1.gitbooks.io/c-tips/content/pinv_with_eigen.html
			//Pseudo inverse
			template<typename _Matrix_Type_>
			_Matrix_Type_ pInv(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
			{
				Eigen::JacobiSVD< _Matrix_Type_ > svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
				double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
				return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
			}

			//For calibrating data with position only
			namespace Position {
				//For calibrating a pair of systems with two sensors measuring the same point from two different reference frames
				// Xa = b
				// or XA = B
				static inline Eigen::Transform<float, 3, Eigen::Affine> calibrateIdenticalPair(
					const std::vector<Eigen::Vector3f>& samplesA,
					const std::vector<Eigen::Vector3f> samplesB,
					float* error = NULL) {
					if (samplesA.size() != samplesB.size()) {
						throw std::runtime_error(std::string(__FILE__) + std::to_string(__LINE__) + std::string(" : samplesA and samplesB of different size"));
					}

					Eigen::MatrixXf A(4, samplesA.size());
					Eigen::MatrixXf B(4, samplesB.size());

					for (int i = 0; i < samplesA.size(); i++) {
						A.col(i) << samplesA[i], 1;
						B.col(i) << samplesB[i], 1;
					}

					//Make sure normalised
					Eigen::Matrix4f X = B * pInv(A);
					Eigen::Quaternionf q(X.block<3, 3>(0, 0));
					q.normalize();
					Eigen::Translation3f t(X.block<3, 1>(0, 3));

					Eigen::Transform<float, 3, Eigen::Affine> TX(t);
					TX.rotate(q);

					if (error != NULL) {
						*error = (TX.matrix() * A - B).norm() / samplesA.size();
					}

					return TX;
				}
			}

			//For calibrating rotation only
			namespace Rotation {

			}

			//For calibrating position and rotation data simultaneously
			namespace Transform {

				//For calibrating a pair of systems with two sensors connected by a rigid body
				// XA = bY
				static inline Eigen::Transform<float, 3, Eigen::Affine> twoSystems(
					std::vector<std::vector<Eigen::Matrix4f>> samplesA, 
					std::vector<std::vector<Eigen::Matrix4f>> samplesB,
					float* error = NULL)
				{

					if (error != NULL) {
						*error = 0;// (TX.matrix() * A - B);
					}
					//TODO
					return Eigen::Transform<float, 3, Eigen::Affine>();
				}
			}

		}
	}
}
