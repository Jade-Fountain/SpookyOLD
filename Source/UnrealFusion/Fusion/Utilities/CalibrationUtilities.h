/*  This file is part of UnrealFusion, a sensor fusion plugin for VR in the Unreal Engine
    Copyright (C) 2017 Jake Fountain
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
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
			static inline _Matrix_Type_ pInv(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
			{
				Eigen::JacobiSVD< _Matrix_Type_ > svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
				double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
				return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
			}

			static inline Eigen::Transform<float, 3, Eigen::Affine> matrixToTransform3D(const Eigen::Matrix4f& X) {
				//Make sure normalised
				Eigen::Quaternionf q(X.block<3, 3>(0, 0));
				q.normalize();
				Eigen::Translation3f t(X.block<3, 1>(0, 3));

				Eigen::Transform<float, 3, Eigen::Affine> TX(t);
				TX.rotate(q);
				return TX;
			}

			static inline Eigen::Transform<float, 3, Eigen::Affine> slerpTransform3D(
				const Eigen::Transform<float, 3, Eigen::Affine>& T1,
				const Eigen::Transform<float, 3, Eigen::Affine>& T2, 
				float t
			) {
				Eigen::Quaternionf q1(T1.rotation());
				Eigen::Quaternionf q2(T2.rotation());

				Eigen::Quaternionf q = q1.slerp(t, q2);

				Eigen::Vector3f v1(T1.translation());
				Eigen::Vector3f v2(T2.translation());
				Eigen::Translation3f v(v1 * (1 - t) + v2 * t);

				Eigen::Transform<float, 3, Eigen::Affine> T(v);
				T.rotate(q);
				return T;
			}

			//For calibrating data with position only
			namespace Position {
				//For calibrating a pair of systems with two sensors measuring the same point from two different reference frames
				// Xa = b
				// or XA = B
				static inline Eigen::Transform<float, 3, Eigen::Affine> calibrateIdenticalPair(
					const std::vector<Eigen::Vector3f>& samplesA,
					const std::vector<Eigen::Vector3f>& samplesB,
					float* error = NULL
				) {
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
					Eigen::Transform<float, 3, Eigen::Affine> TX = matrixToTransform3D(X);

					if (error != NULL) {
						auto E = TX.matrix() * A - B;
						float normE = E.norm();
						*error = normE / samplesA.size();
					}

					return TX;
				}
				
				//TODO:FIX THIS METHOD:
				static inline Eigen::Transform<float, 3, Eigen::Affine> refineIdenticalPair(
					const std::vector<Eigen::Vector3f>& samplesA,
					const std::vector<Eigen::Vector3f>& samplesB,
					const Eigen::Transform<float, 3, Eigen::Affine>& X,
					float* error = NULL
				) {
					Eigen::MatrixXf A(4, samplesA.size());
					Eigen::MatrixXf B(4, samplesB.size());

					for (int i = 0; i < samplesA.size(); i++) {
						A.col(i) << samplesA[i], 1;
						B.col(i) << samplesB[i], 1;
					}

					Eigen::MatrixXf E = B - X.matrix() * A;

					Eigen::Matrix4f dX;

					for (int i = 0; i < 4; i++) {
						for (int j = 0; j < 4; j++) {
							dX(i, j) = 0;
							float max_n = A.cols() / 4;
							for (int n = 0; n < max_n; n++) {
								dX(i, j) = A(i, j + 4 * n) / (max_n - 1);
							}
						}
					}

					dX = 0.1 * dX / dX.norm();

					Eigen::Transform<float, 3, Eigen::Affine> TX = matrixToTransform3D(X.matrix() + dX);

					if (error != NULL) {
						*error = (TX.matrix() * A - B).norm() / samplesA.size();
					}

					return TX;
				}

				//Simple refinement
				static inline Eigen::Transform<float, 3, Eigen::Affine> refineIdenticalPairSimple(
					const std::vector<Eigen::Vector3f>& samplesA,
					const std::vector<Eigen::Vector3f>& samplesB,
					const Eigen::Transform<float, 3, Eigen::Affine>& X,
					float* error = NULL
				) {
					float learning_rate = 0.1;
					Eigen::Transform<float, 3, Eigen::Affine> X_new = calibrateIdenticalPair(samplesA, samplesB);

					Eigen::Transform<float, 3, Eigen::Affine> TX = slerpTransform3D(X, X_new, learning_rate);

					if (error != NULL) {
						Eigen::MatrixXf A(4, samplesA.size());
						Eigen::MatrixXf B(4, samplesB.size());

						for (int i = 0; i < samplesA.size(); i++) {
							A.col(i) << samplesA[i], 1;
							B.col(i) << samplesB[i], 1;
						}

						*error = (TX.matrix() * A - B).norm() / samplesA.size() * (learning_rate) + *error;
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
					const std::vector<std::vector<Eigen::Matrix4f>>& samplesA, 
					const std::vector<std::vector<Eigen::Matrix4f>>& samplesB,
					float* error = NULL)
				{

					if (error != NULL) {
						*error = 0;// (TX.matrix() * A - B);
					}
					//TODO
					return Eigen::Transform<float, 3, Eigen::Affine>();
				}
			}

			static inline float qualityFromError(float error, float scale) {
				float e = error / scale;
				return 1 / (1 + e*e);
			}

		}
	}
}
