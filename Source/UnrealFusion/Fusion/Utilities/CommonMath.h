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
#include<numeric>
#include<algorithm>
#include<Eigen/Core>
#include<Eigen/SVD>
#include<Eigen/Geometry>
#include<Eigen/Eigenvalues>
#include<Eigen/unsupported/KroneckerProduct>
//#include "Logging.h"
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


		//TODO: put this somewhere nicer
		class Sphere {
		public:
			float r = 0;
			Eigen::VectorXf center;

			float getDistanceToPoint(const Eigen::Vector3f& p){
				return std::fabs((p - center).norm() - r);
			}

			float getMedianError(const Eigen::MatrixXf& points, const std::vector<int>& inliers){
				int num_points = inliers.size();
				std::vector<float> errors;
				for(int i = 0; i < num_points; i++){
					float error = getDistanceToPoint(points.col(inliers[i]));
					//Insert in order
					std::vector<float>::iterator below = errors.begin();
					std::vector<float>::iterator above = errors.end();
					//Binary search for location to insert
					while(std::distance(above,below) > 1){
						std::vector<float>::iterator middle = below;
						std::advance(middle, std::distance(above, below) / 2);
						if (*middle > error) {
							above = middle;
						}
						else {
							below = middle;
						}
					}
					errors.insert(above, error);
				}
				//Return median
				return errors[errors.size() / 2];
			}

			std::vector<int> getInliers(const Eigen::MatrixXf& points, float inlier_threshold) {
				int num_points = points.cols();
				std::vector<int> inliers;
				for (int i = 0; i < num_points; i++) {
					float error = getDistanceToPoint(points.col(i));;
					if (error < inlier_threshold) {
						inliers.push_back(i);
					}
				}
				//Return medium
				return inliers;
			}

		};

		class Line {
		public:

			Eigen::VectorXf origin;
			Eigen::VectorXf direction;

			Eigen::VectorXf getPoint(const float& t) const {
				return origin + t * direction;
			}

			Eigen::VectorXf intersect(const Line& other, bool* success) {
				//X * [t1,t2]^T = y
				Eigen::MatrixXf X(origin.rows(), 2);
				Eigen::MatrixXf y = other.origin - origin;
				
				X.col(0) = direction;
				X.col(1) = -other.direction;

				Eigen::MatrixXf Xinv = pInv(X);
				Eigen::Vector2f tVals = Xinv * y;

				Eigen::VectorXf thisPoint = getPoint(tVals(0));
				Eigen::VectorXf thatPoint = other.getPoint(tVals(1));

				if((thisPoint - thatPoint).norm() < 0.01){
					*success = true;
				} else {
					*success = false;
				}
				return (thisPoint + thatPoint) / 2;
			}
		};


		static inline Line getCircleNormal(const Eigen::Vector3f& A,const Eigen::Vector3f& B, const Eigen::Vector3f& C){
			Eigen::Vector3f AB = A-B;
			Eigen::Vector3f BC = B-C;
			Eigen::Vector3f CA = C-A;
			
			Line result;
			Eigen::Vector3f normal = AB.cross(BC);
			normal.normalize();
			result.direction = normal; 

			Line lineAB;
			lineAB.origin = B + AB / 2;
			lineAB.direction = AB.cross(normal);

			Line lineBC;
			lineBC.origin = C + BC / 2;
			lineBC.direction = BC.cross(normal);

			bool success = false;
			result.origin = lineBC.intersect(lineAB, &success);
			
			return result;
		}

		static inline Sphere getSphereFrom4Points(const Eigen::Vector3f& A, const Eigen::Vector3f& B, const Eigen::Vector3f& C, const Eigen::Vector3f& D) {
			Sphere result;
			Line ray1 = getCircleNormal(A, B, C);
			Line ray2 = getCircleNormal(B, C, D);
			bool success = false;
			result.center = ray1.intersect(ray2, &success);
			result.r = (result.center - A).norm();
			return result;
		}

		static inline Sphere sphereRANSAC(const Eigen::MatrixXf& points){
			//Number of points to fit sphere to
			int num_points = points.cols();

			if (num_points < 4) {
				return Sphere();
			}
			//State of optimisation
			std::vector<Sphere> models;
			float best_error = 100000000;
			int best_model_index = 0;
			int max_models = 100;
			float inlier_threshold = 0.01;
			int inliers_needed = num_points / 5;

			//Random number selection
			std::vector<int> indices(num_points);
			//Fill blank vector from zero to num_points-1
			std::iota(std::begin(indices), std::end(indices), 0);

			//Point stats
			Eigen::Vector3f mean = points.rowwise().mean();
			float variance = (points.colwise() + mean).colwise().norm().rowwise().mean()[0];

			//Compute models and record the best one
			while(models.size() < max_models){
				//Shuffle points to sample
				std::random_shuffle(std::begin(indices), std::end(indices));
				//Get a model
				Sphere model = getSphereFrom4Points(points.col(indices[0]), points.col(indices[1]), points.col(indices[2]), points.col(indices[3]));
				//Count inliers
				std::vector<int> inliers = model.getInliers(points,inlier_threshold);

				//If there are enough inliers, check the actual error
				if(inliers.size() > inliers_needed){
					float error = model.getMedianError(points,inliers);
					models.push_back(model);
					if(error < best_error){
						best_error = error;
						best_model_index = models.size()-1;
					}
				}
			}
			//return best
			return models[best_model_index];
		}

		static inline Sphere fitSphere(const Eigen::MatrixXf& points){
			Sphere result;
			Eigen::VectorXf mean = points.rowwise().mean();
			result.center = mean;

			//Check dimension
			if (mean.rows() == 3) {
				result = sphereRANSAC(points);
			}

			return result;
		}

		//source: http://stackoverflow.com/questions/12374087/average-of-multiple-quaternions
		//sources source: http://www.acsu.buffalo.edu/~johnc/ave_quat07.pdf
		//wQ (4xn mat) = unit quaternions in columns multiplied by their weights
		static inline Eigen::Quaternionf averageQuaternions(const Eigen::MatrixXf& wQ) {
			Eigen::MatrixXf Q2 = wQ * wQ.transpose();
			Eigen::EigenSolver<Eigen::MatrixXf> es(Q2);
			//Eigenvalues will be real and positive because matrix is pos semidefinite
			Eigen::VectorXf eval = es.eigenvalues().real();
			int best = 0;
			eval.maxCoeff(&best);
			Eigen::MatrixXf evec = es.eigenvectors().real();
			Eigen::Vector4f best_evec = evec.col(best);
			best_evec.normalize();
			return Eigen::Quaternionf(best_evec);
		}


	
	}
}
