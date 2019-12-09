all:
	g++ -std=c++14 -Wall -Wextra -pedantic -Werror -Ideps/mpl/src -isystem/usr/include/eigen3 test.cpp
