cmake_minimum_required(VERSION 3.14)
project(flow_contest)

set(CMAKE_CXX_STANDARD 17)

add_executable(flow_contest main.cpp network.h dinica_algorithm.h MKM_algorithm.h goldberg_algorithm.h)