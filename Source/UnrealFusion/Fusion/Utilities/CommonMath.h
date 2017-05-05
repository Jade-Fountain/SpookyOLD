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
#include "Logging.h"
#pragma once

#define M_PI 3.141592654

namespace fusion{
	namespace utility{

		//Source: https://fuyunfei1.gitbooks.io/c-tips/content/pinv_with_eigen.html
		//Pseudo inverse
		template<typename _Matrix_Type_>
		static inline _Matrix_Type_ pInv(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
		{
			Eigen::JacobiSVD< _Matrix_Type_ > svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
			double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
			return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
		}

		static inline Eigen::Matrix3f orthogonaliseBasic(const Eigen::Matrix3f& M) {
			Eigen::Vector3f x = M.col(0);
			x.normalize();
			Eigen::Vector3f y = M.col(1);
			y.normalize();
			Eigen::Vector3f z = M.col(2);
			z.normalize();
			
			float error = x.dot(y);
			float theta = std::acos(error);
			float x_angle = theta / 2 - M_PI / 4;

			Eigen::Vector3f n = x.cross(y);
			n.normalize();

			Eigen::Matrix3f Rx_angle = Eigen::AngleAxisf(x_angle, n).matrix();
			Eigen::Matrix3f Ry_angle = Rx_angle.transpose();

			Eigen::Vector3f x_temp = Rx_angle * x;
			Eigen::Vector3f y_temp = Ry_angle * y;

			Eigen::Vector3f xy = (x + y);
			xy.normalize();
			float phi = std::atan2(z.dot(xy), z.dot(n));

			Eigen::Vector3f r = z.cross(xy);
			
			Eigen::Matrix3f Rxy = Eigen::AngleAxisf(phi / 2, r).matrix();
			Eigen::Matrix3f Rz = Rxy.transpose();

			Eigen::Vector3f x_new = Rxy * x_temp;
			Eigen::Vector3f y_new = Rxy * y_temp;
			Eigen::Vector3f z_new = Rz * z;

			Eigen::Matrix3f result;
			result.col(0) = x_new;
			result.col(1) = y_new;
			result.col(2) = z_new;

			return result;

		}

		static inline Eigen::Transform<float, 3, Eigen::Affine> matrixToTransform3D(const Eigen::Matrix4f& X) {
			//Make sure normalised
			//Eigen::Quaternionf q(X.block<3, 3>(0, 0));
			//q.normalize();
			Eigen::Translation3f t(X.block<3, 1>(0, 3));

			Eigen::Transform<float, 3, Eigen::Affine> TX(t);
			//TX.rotate(q);
			TX.rotate(orthogonaliseBasic(X.topLeftCorner(3, 3)));
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


		static inline float qualityFromError(float error, float scale) {
			float e = error / scale;
			return 1 / (1 + e*e);
		}

		//Takes on the order of 3N, where N = X.size()
		static inline float getHistogramPeak(std::vector<float> X){
			float min = std::numeric_limits<float>::max();
			float max = std::numeric_limits<float>::min();
			//Compute range of data
			for(auto& x : X){
				if(x < min) {
					min = x;
				}
				if(x > max) {
					max = x;
				}
			}
			float range = max - min;
			float delta = range / X.size();

			//Create histogram
			std::vector<int> histogram(X.size(),0);
			for(auto& x : X){
				//Numeric limits excludes the case where x == max, which would give histogram[histogram.size()] which is invalid
				histogram[std::floor((x - min) * (1-std::numeric_limits<float>::min()) / delta)]++; 
			}

			//Find peak of histogram
			int max_index = 0;
			int max_count = 0;
			for(int i = 0; i < histogram.size(); i++){
				if(histogram[i] > max_count){
					max_count = histogram[i];
					max_index = i;
				}
				//FUSION_LOG("Histogram: " + std::to_string(i * delta + min) + " count = " + std::to_string(histogram[i]));
			}

			//Return centred max bin
			return (max_index + 0.5) * delta + min;

		}

	
	}
}
