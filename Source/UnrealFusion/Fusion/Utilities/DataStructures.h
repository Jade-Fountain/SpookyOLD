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
#pragma once

#include <vector>
#include <memory>
#include <map>
#include <algorithm>
#include <iterator>


namespace fusion {
	namespace utility {
		template <class X, class Y>
		class SafeMap {
		private:
			std::map<X, Y> data;
		public:
			Y& operator[] (const X& x) {
				//TODO: make more efficient!
				if (data.count(x) == 0) data[x] = Y();
				return data[x];
			}
		};

		template<class X, class Y>
		Y& safeAccess(std::map<X, Y>& m, const X& x) {
			if (m.count(x) == 0) m[x] = Y();
			return m[x];
		}

		template<class X, class Y>
		std::shared_ptr<Y>& safeAccess(std::map<X, std::shared_ptr<Y>>& m, const X& x) {
			if (m.count(x) == 0) m[x] = std::make_shared<Y>();
			return m[x];
		}

		template<class X>
		std::set<X> setDiff(const std::set<X>& A, const std::set<X>& B){
		    std::set<X> diff;
		 
		    std::set_difference(A.begin(), A.end(), B.begin(), B.end(), 
		                        std::inserter(diff, diff.begin()));

		    return diff;
		}

	}
}
