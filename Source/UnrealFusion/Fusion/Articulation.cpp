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
namespace fusion{
    
    Articulation::Articulation(){

    }


    Transform3D Articulation::getTransform(Eigen::VectorXf theta){
        return Transform3D();
        //switch(type){
        //  //TODO: actually implement these things / use library
        //  case(CARTESIAN):
        //      return translation(v) * exp(theta(0) * skew(w.normalise()));
        //      break;
        //  case(TWIST):
        //      Eigen::Matrix4f zeta = Eigen::Matrix4f::Identity();
        //      zeta.topLeftCorner(3,3) = skew(w);
        //      zeta.column(3) = v;
        //      zeta(3,3) = 0;
        //      return exp(theta(0) * zeta);
        //      break;
        //  case(BONE):
        //      //v should be along the x-axis for this one
        //      return translation(v) * exp(skew(theta));
        //      break;

        //}     
    }
    
    static Articulation Articulation::createArticulationFrom(Eigen::VectorXf T, Type type){
        Articulation result;
        //TODO: make the articulation
        return result;
    }

}
