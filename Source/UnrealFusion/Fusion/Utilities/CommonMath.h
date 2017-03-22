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


		static inline float qualityFromError(float error, float scale) {
			float e = error / scale;
			return 1 / (1 + e*e);
		}

	
	}
}
