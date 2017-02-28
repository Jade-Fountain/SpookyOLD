#pragma once

#include <vector>

//WARNING: this file is not complete


namespace fusion {

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

}
