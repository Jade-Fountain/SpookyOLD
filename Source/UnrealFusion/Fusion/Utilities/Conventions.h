#pragma once
#include "Eigen/Core"
#include "Eigen/Geometry"

namespace fusion {
	namespace utility {
		//Defines transforms between data of different types
		namespace convention{

			static Eigen::Matrix4f unserialiseTo4x4f(Eigen::VectorXf data) {
				Eigen::Translation3f v(data.block<3, 1>(0, 0));
				Eigen::Quaternionf q(data[3], data[4], data[5], data[6]);
				Eigen::Transform<float, 3, Eigen::Affine> T(v);
				T.rotate(q);
				return T.matrix();
			}

		}
	}
}