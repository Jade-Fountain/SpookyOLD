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
#include "UnrealFusion.h"
#include "Articulation.h"
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"

namespace fusion{
    
    Articulation::Articulation(){

    }


    Transform3D Articulation::getTransform(Eigen::VectorXf theta){

		//TODO: make these cases into methods
		Transform3D T = Transform3D::Identity();
		Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
		Sophus::Vector6f vec;
		Eigen::Matrix3f W = Eigen::Matrix3f::Identity();

		switch(type){
		case(CARTESIAN):
			w.normalize();
			R = Sophus::SO3f::exp(theta(0) * w).matrix(); // = e^(theta * ^w)
			T.translate(v);
			T.rotate(R);
			return T;
		case(TWIST):
			vec.block<3, 1>(0, 0) = v;
			vec.block<3, 1>(3, 0) = w;
			T.matrix() = Sophus::SE3f::exp(theta(0) * vec).matrix();
			return T;
		case(BONE):
			//v should be along the x-axis for this one
			R = Sophus::SO3f::exp(theta).matrix(); // = e^(theta * ^theta)
			T.translate(v);
			T.rotate(R);
			return T;

        }
		return T;
    }
    
    Articulation Articulation::createArticulationFrom(Eigen::VectorXf T, Type type){
        Articulation result;
        //TODO: make the articulation
        return result;
    }

}
