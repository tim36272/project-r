#ifndef TIM_SRC_TYPES_H
#define TIM_SRC_TYPES_H

#include <iostream>

namespace utility {
	template <class T> class Pair_ {
	public:
		T values[2];
		Pair_() {
			values[0] = -1;
			values[1] = -1;
		}
		Pair_(const T& first,const T& second) {
			values[0] = first;
			values[1] = second;
		}
		bool Empty() {
			return values[0]==-1;
		}
		T& operator[](int index) {
			return values[index];
		}
		const T& operator[](int index) const{
					return values[index];
				}
		Pair_<T>& operator = (const Pair_& rhs);
		friend std::ostream& operator <<(std::ostream& out,const Pair_<T>& rhs) {
			out<<rhs[0]<<" "<<rhs[1];
			return out;
		}
		friend bool operator==(const Pair_<T>& lhs, const Pair_<T>& rhs) {
			return lhs[0]==rhs[0] && lhs[1]==rhs[1];
		}
	};
	template <typename T>
	Pair_<T>& Pair_<T>::operator = (const Pair_& rhs) {
		values[0] = rhs.values[0];
		values[1] = rhs.values[1];
		return *this;
	}
	typedef Pair_<int> Pair;



	struct ColorHolder {
		cv::Scalar_<int> color;
		int dirty;
		ColorHolder() {
			dirty = false;
		}
	};

	typedef std::vector<ColorHolder> ColorListType;
	typedef Pair_<cv::Scalar> ColorPair;
	typedef std::vector<ColorPair> ColorPairListType;
	typedef std::vector<cv::Scalar> ColorSingleListType;
}
#endif
