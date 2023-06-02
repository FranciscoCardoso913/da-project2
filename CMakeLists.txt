cmake_minimum_required(VERSION 3.25)
project(da_project2)

set(CMAKE_CXX_STANDARD 17)

add_executable(da_project2 src/main.cpp src/services/Program.cpp src/view/Menu.cpp src/services/actions/Action.cpp src/model/Graph.cpp src/services/actions/ChangeMenu.cpp src/view/DrawUtils.cpp src/services/Scrapper.cpp src/model/Node.cpp src/model/Edge.cpp src/services/actions/ChooseGraph.cpp src/services/actions/ChooseGraph.h src/services/actions/Christofide.cpp src/services/actions/Backtracking.cpp src/model/MutablePriorityQueue.h src/view/DrawPaths.cpp src/services/actions/NearestNeighbor.cpp src/services/actions/NearestNeighbor.h)
