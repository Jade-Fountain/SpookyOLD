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
#include<Eigen/unsupported/KroneckerProduct>
#include "CommonMath.h"

#include "Logging.h"
#pragma once
namespace fusion{
	namespace utility{
		namespace calibration {

			namespace Time {
				//Returns the time that that x leads y
				static inline float estimateLatency(
					const Eigen::VectorXf& x,
					const Eigen::VectorXf& tx,
					const Eigen::VectorXf& y,
					const Eigen::VectorXf& ty
				) {
					if (x.cols() != tx.cols() || y.cols() != ty.cols()) {
						throw std::runtime_error(__LINE__ + "data and timestamps differ in count");
					}

					std::vector<float> latencies;
					for (int i = 0; i < y.size(); i++) {
						for (int j = 0; j < x.size() - 1; j++) {
							float det = (x(j) - y(i))*(x(j + 1) - y(i));
							if (det < 0) {
								//Interpolate fraction alpha between x(j+1) and x(j)
								float alpha = (y(i) - x(j)) / (x(j + 1) - x(j));
								//Get t based on interpolation between two successive measurements
								float t = tx(j + 1) * alpha + tx(j) * (1 - alpha);
								//Latency corresponding to this timestamp
								float latency = t - ty(j);
								//TODO: throwout some latencies bigger or smaller than maximum possible
								latencies.push_back(latency);
							}
						}
					}
					//Debug:
					std::stringstream ss;
					ss << "Data: x = " << std::endl << x << std::endl << "tx" << std::endl << tx << std::endl;
					ss << "Data: y = " << std::endl << y << std::endl << "ty" << std::endl << ty;
					FUSION_LOG(ss.str());

					return getHistogramPeak(latencies);
				}
			}

			//For calibrating data with position only
			namespace Position {

				//Common functions for position data
				static inline float errorFunc(const Eigen::MatrixXf E) {
					//return E.rowwise().mean().norm();
					return E.norm() / E.cols();
				}
				static inline float getError(
					const std::vector<Eigen::Vector3f>& samplesA,
					const std::vector<Eigen::Vector3f>& samplesB,
					const Eigen::Transform<float, 3, Eigen::Affine>& X
				) {
					Eigen::MatrixXf A(4, samplesA.size());
					Eigen::MatrixXf B(4, samplesB.size());

					for (int i = 0; i < samplesA.size(); i++) {
						A.col(i) << samplesA[i], 1;
						B.col(i) << samplesB[i], 1;
					}

					return errorFunc(X.matrix() * A - B);
				}
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

					Eigen::Matrix4f X = B * pInv(A);
					//Make sure normalised
					Eigen::Transform<float, 3, Eigen::Affine> TX = matrixToTransform3D(X);

					if (error != NULL) {
						auto E = TX.matrix() * A - B;
						*error = errorFunc(E);
					}

					return TX;
				}
				
				static inline Eigen::Transform<float, 3, Eigen::Affine> calibrateWeightedIdenticalPair(
					const std::vector<Eigen::Vector3f>& samplesA,
					const std::vector<Eigen::Vector3f>& samplesB,
					const std::vector<Eigen::Matrix3f>& invVariances,
					float* error = NULL
				) {
					if (samplesA.size() != samplesB.size()) {
						throw std::runtime_error(std::string(__FILE__) + std::to_string(__LINE__) + std::string(" : samplesA and samplesB of different size"));
					}

					Eigen::MatrixXf A(4, samplesA.size());
					Eigen::MatrixXf B(4, samplesB.size());
					Eigen::MatrixXf sigInv = Eigen::MatrixXf::Identity(4*samplesA.size(), 4 * samplesA.size());

					for (int i = 0; i < samplesA.size(); i++) {
						A.col(i) << samplesA[i], 1;
						B.col(i) << samplesB[i], 1;
						//Variance for each vector
						sigInv.block<3, 3>(4 * i, 4 * i) = invVariances[i];
						//Low variance for fourth component
						sigInv(4 * i + 3, 4 * i + 3) = 1.0/0.001;
					}

					//Get vecB, stacked columns
					Eigen::Map<Eigen::MatrixXf> vecB(B.data(), B.rows()*B.cols(), 1);

					//kronA = kron(A.t,I_m) and m=p=4
					Eigen::MatrixXf kronA = Eigen::kroneckerProduct(A.transpose(),Eigen::MatrixXf::Identity(4,4));

					//kronA.T * sig
					Eigen::MatrixXf AsigAInv = (kronA.transpose() * sigInv * kronA).inverse();					

					Eigen::MatrixXf vecX = AsigAInv * kronA.transpose() * sigInv * vecB;

					Eigen::Map<Eigen::MatrixXf> X(vecX.data(), 4, 4);
					//Make sure normalised
					Eigen::Transform<float, 3, Eigen::Affine> TX = matrixToTransform3D(X);
					
					//DEBUG
					//std::stringstream ss;
					//ss << "B = \n" << B << std::endl;
					//ss << "vecB = \n" << vecB << std::endl;
					//ss << "sigInv =\n " << sigInv << std::endl;
					//ss << "vecX = \n" << vecX << std::endl;
					//ss << "X = \n" << X << std::endl;
					//FUSION_LOG(ss.str());
					
					if (error != NULL) {
						auto E = TX.matrix() * A - B;
						*error = errorFunc(E);
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
						*error = errorFunc(TX.matrix() * A - B);
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
						*error = getError(samplesA,samplesB,X_new) * (learning_rate) + *error;
					}

					return TX;
				}


				//Simple refinement
				static inline Eigen::Transform<float, 3, Eigen::Affine> refineIdenticalPairPosition(
					const std::vector<Eigen::Vector3f>& samplesA,
					const std::vector<Eigen::Vector3f>& samplesB,
					const Eigen::Transform<float, 3, Eigen::Affine>& X,
					float* error = NULL
				) {
					float learning_rate = 1;
					
					Eigen::MatrixXf A(4, samplesA.size());
					Eigen::MatrixXf B(4, samplesB.size());

					for (int i = 0; i < samplesA.size(); i++) {
						A.col(i) << samplesA[i], 1;
						B.col(i) << samplesB[i], 1;
					}

					Eigen::MatrixXf E = B - X.matrix() * A;

					Eigen::Vector3f meanError = E.rowwise().mean();
					//std::stringstream ss;
					//ss << "errorMat = " << E << std::endl;
					//ss << "average error = " << meanError << std::endl;
					//ss << "average error norm = " << meanError.norm() << std::endl;
					//FUSION_LOG(ss.str());

					Eigen::Transform<float, 3, Eigen::Affine> X_new = X;

					X_new.pretranslate(meanError);

					if (error != NULL) {
						*error = getError(samplesA, samplesB, X_new) * (learning_rate) + *error * (1-learning_rate);
					}

					return X_new;
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

		}
	}
}
