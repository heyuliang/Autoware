/*
 * test_priv.cpp
 *
 *  Created on: Aug 12, 2018
 *      Author: sujiwo
 */


#include <iostream>
#include <string>
#include <utility>
#include "access_private.hpp"


using namespace std;


class Widget
{
public:
	Widget(int a_, int b_):
		a(a_), b(b_)
	{}

private:
	int a, b;

	int maximum()
	{ return max(a,b); }
};


ACCESS_PRIVATE_FUN (Widget, int(), maximum);


int main()
{
	Widget W(100, 200);
	Widget &X = W;
	cout << call_private::maximum(X) << endl;
}
