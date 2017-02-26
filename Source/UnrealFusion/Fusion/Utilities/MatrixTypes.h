#pragma once

#include <vector>

//WARNING: this file is not complete


namespace fusion {
	//TODO: fix the memory issues with this class
	//template <class T>
	//class slice {
	//private:
	//	std::iterator<T> start_iter;
	//	int start;
	//	int step;
	//	int count;
	//public:
	//	slice(const vector<T>& v, const int& start_, const int& step_, const int& count_) {
	//		start = start_;
	//		step = step_;
	//		count = count_;
	//		start_iter = v.begin() + start_;
	//	}

	//	T& operator[] (int index) {
	//		if (index > count) {
	//			throw std::runtime_error("Slice index out of bounds:" + index + "/" + count);
	//		}
	//		std::iterator<T> pos = start_iter + start + step * index;
	//		//TODO: check memory location valid
	//		return *pos;
	//	}
	//};

	//TODO: finish
	template <class T>
	class ObMatrix {
	private:
		std::vector<T> data;
		int width = 0;
		int height = 0;
	public:
		ObMatrix() {}
		ObMatrix(int w, int h) : data(w*h), width(w), height(h) {}

		T& operator()(const int& row, const int& col) {
			if (row = > width || col = > height) {
				throw std::runtime_error("Matrix index out of bounds:" + row + ", " + col + " > " + width + ", " + height);
			}
			return data[col + row * width];
		}

		std::vector<int> size() {
			return { width, height };
		}
	};
}
