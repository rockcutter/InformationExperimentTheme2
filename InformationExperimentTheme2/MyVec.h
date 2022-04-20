#pragma once
#include <stdexcept>

template <class T>
class MyVec: public std::pair<T, T>{
public:
	MyVec(const std::pair<T, T>& p) : std::pair<T, T>(p)
	{}
	MyVec(T x, T y) : std::pair<T, T>(x, y)
	{}
	
	friend MyVec<T> operator+(const MyVec<T>& obj1, const MyVec<T>& obj2) {
		return MyVec<T>(obj1.first + obj2.first, obj1.second + obj2.second);
	}
 	friend MyVec<T> operator-(const MyVec<T>& obj1, const MyVec<T>& obj2) {
		return MyVec<T>(obj1.first - obj2.first, obj1.second - obj2.second);
	}
 	friend MyVec<T> operator*(const MyVec<T>& obj1, const MyVec<T>& obj2) {
		return MyVec<T>(obj1.first * obj2.first, obj1.second * obj2.second);
	}
 	friend MyVec<T> operator/(const MyVec<T>& obj1, const MyVec<T>& obj2) {
		if (!obj2.first || !obj2.second) throw std::domain_error("0除算");
		return MyVec<T>(obj1.first / obj2.first, obj1.second / obj2.second);
	}
	MyVec<T> operator+=(const MyVec<T>& obj) {
		return *this + obj;
	}
	MyVec<T> operator-=(const MyVec<T>& obj) {
		return *this - obj;
	}
	MyVec<T> operator*=(const MyVec<T>& obj) {
		return *this * obj;
	}
	MyVec<T> operator/=(const MyVec<T>& obj) {
		return *this / obj;
	}

};

