#include "pch.h"
#include <utility>
#include <iostream>
#include <vector>
#include <chrono>
#include "CppUnitTest.h"
//Astar dependencies
#include "Astar/Astar.h"
using namespace Microsoft::VisualStudio::CppUnitTestFramework;

float defaultHeuristic(const std::pair<int, int> &currentPos, const std::pair<int, int> &targetPos) {
	return sqrt(pow(currentPos.first - targetPos.first, 2) + pow(currentPos.second - targetPos.second, 2));
}

namespace ChaseTagAiTest
{
	TEST_CLASS(FindPath)
	{
	public:
		TEST_METHOD(LinePath)
		{
			Logger logger;

			std::pair<int, int> startIndex(0,0);
			std::pair<int, int> targetIndex(4,0);
			std::function<float(std::pair<int, int>, std::pair<int, int>)> h = defaultHeuristic;
			std::pair<int, int> boardSize(5, 2);
			// 0 0 0 0 0 
			// 0 0 0 0 0
			CELL_TYPE* board = new CELL_TYPE[]{
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY };
			std::vector<std::pair<int, int>>* outPath = new std::vector<std::pair<int, int>>();
			std::vector<std::pair<int, int>>* expectedPath = new std::vector<std::pair<int, int>>();
			expectedPath->push_back(std::pair<int, int>(4, 0));
			expectedPath->push_back(std::pair<int, int>(3, 0));
			expectedPath->push_back(std::pair<int, int>(2, 0));
			expectedPath->push_back(std::pair<int, int>(1, 0));
			expectedPath->push_back(std::pair<int, int>(0, 0));

			std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

			bool pathFound = Astar::findPath(startIndex, targetIndex, h, board, boardSize, outPath);

			std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
			double elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

			Assert::IsTrue(elapsedTime < 100); //assert execution time is <0.1 ms 
			std::string timeStr = "Elapsed time " + std::to_string(elapsedTime);
			logger.WriteMessage(timeStr.c_str());

			Assert::IsTrue(*expectedPath == *outPath);
		}

	};
}
