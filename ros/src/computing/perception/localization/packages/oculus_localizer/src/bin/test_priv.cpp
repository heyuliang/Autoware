/*
 * test_priv.cpp
 *
 *  Created on: Aug 12, 2018
 *      Author: sujiwo
 */


#include <iostream>


template<typename Tag, typename Tag::type M>
struct Rob {
  friend typename Tag::type get(Tag) {
    return M;
  }
};

struct A {
  A(int a):a(a) { }
private:
  int a;
};

struct A_f {
  typedef int A::*type;
  friend type get(A_f);
};

template struct Rob<A_f, &A::a>;


int main(int argc, char *argv[])
{
	A a(42);
	int mx = a.*get(A_f());
	std::cout << "proof: " << mx << std::endl;
}
