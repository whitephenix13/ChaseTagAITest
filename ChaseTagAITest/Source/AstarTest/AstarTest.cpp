#include "../pch.h"
#include <utility>
#include <iostream>
#include <vector>
#include <chrono>
#include "CppUnitTest.h"
//Astar dependencies
#include "Astar/Astar.h"
#include <ChaseTagAI\Source\AstarDataOriented\AstarDataOriented.h>
using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace {
	float defaultHeuristic(const std::pair<int, int>& currentPos, const std::pair<int, int>& targetPos) {
		return sqrt(pow(currentPos.first - targetPos.first, 2) + pow(currentPos.second - targetPos.second, 2));
	}
	void drawBoard(Logger logger, CELL_TYPE* board,std::pair<int,int> boardSize) {
		std::string boardStr = "";
		for (int i = 0; i < boardSize.first; ++i) {
			for (int j = 0; j < boardSize.second; ++j) {
				if (board[i * boardSize.second + j] == CELL_TYPE::WALL)
					boardStr += "x";
				else
					boardStr += ".";
			}
			boardStr += "\n";
		}
		logger.WriteMessage(boardStr.c_str());
	}
	std::map<CELL_TYPE, float> getTravelCostMap() {
		std::map<CELL_TYPE, float> travelCostMap;
		travelCostMap.insert(std::pair<CELL_TYPE, int>(CELL_TYPE::EMPTY, 1));
		travelCostMap.insert(std::pair<CELL_TYPE, int>(CELL_TYPE::BAR, 3));
		travelCostMap.insert(std::pair<CELL_TYPE, int>(CELL_TYPE::STEP, 2));
		travelCostMap.insert(std::pair<CELL_TYPE, int>(CELL_TYPE::STEP_BAR, 2));
		travelCostMap.insert(std::pair<CELL_TYPE, int>(CELL_TYPE::WALL, 100000));
		return travelCostMap;
	}
}

namespace AstarTest
{
	TEST_CLASS(FindPath)
	{
	public:
		TEST_METHOD(LinePath)
		{
			std::pair<int, int> startIndex(0,0);
			std::pair<int, int> targetIndex(0,4);
			std::function<float(std::pair<int, int>, std::pair<int, int>)> h = defaultHeuristic;
			std::pair<int, int> boardSize(2, 5);
			CELL_TYPE* board = new CELL_TYPE[]{
				//0                     1                  2             3                   4  
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY, //0
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY };//1
			std::vector<std::pair<int, int>>* outPath = new std::vector<std::pair<int, int>>();
			std::vector<std::pair<int, int>>* expectedPath = new std::vector<std::pair<int, int>>();
			expectedPath->push_back(std::pair<int, int>(0,4));
			expectedPath->push_back(std::pair<int, int>(0,3));
			expectedPath->push_back(std::pair<int, int>(0,2));
			expectedPath->push_back(std::pair<int, int>(0,1));
			expectedPath->push_back(std::pair<int, int>(0, 0));

			bool pathFound = Astar::findPath(startIndex, targetIndex, h, board, boardSize, outPath);			
			Assert::IsTrue(*expectedPath == *outPath);
		}

		TEST_METHOD(WallPath)
		{
			std::pair<int, int> startIndex(2,0);
			std::pair<int, int> targetIndex(2,4);
			std::function<float(std::pair<int, int>, std::pair<int, int>)> h = defaultHeuristic;
			std::pair<int, int> boardSize(6,5);
			CELL_TYPE* board = new CELL_TYPE[]{
				//0                     1                  2             3                   4  
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,// 0
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,// 1
				CELL_TYPE::EMPTY,CELL_TYPE::WALL, CELL_TYPE::WALL,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,//2
				CELL_TYPE::EMPTY,CELL_TYPE::WALL, CELL_TYPE::WALL,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY ,//3
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY, //4
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY };//5
			std::vector<std::pair<int, int>>* outPath = new std::vector<std::pair<int, int>>();
			std::vector<std::pair<int, int>>* expectedPath = new std::vector<std::pair<int, int>>();
			expectedPath->push_back(std::pair<int, int>(2,4));
			expectedPath->push_back(std::pair<int, int>(1,3));
			expectedPath->push_back(std::pair<int, int>(1,2));
			expectedPath->push_back(std::pair<int, int>(1,1));
			expectedPath->push_back(std::pair<int, int>(1,0));
			expectedPath->push_back(std::pair<int, int>(2,0));

			bool pathFound = Astar::findPath(startIndex, targetIndex, h, board, boardSize, outPath);
			Assert::IsTrue(*expectedPath == *outPath);
		}
		TEST_METHOD(StuckInHolePath)
		{
			std::pair<int, int> startIndex(2, 3);
			std::pair<int, int> targetIndex(2, 0);
			std::function<float(std::pair<int, int>, std::pair<int, int>)> h = defaultHeuristic;
			std::pair<int, int> boardSize(6, 5);
			CELL_TYPE* board = new CELL_TYPE[]{
				//0                     1                  2             3                   4  
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,// 0
				CELL_TYPE::EMPTY,CELL_TYPE::WALL ,CELL_TYPE::WALL ,CELL_TYPE::WALL,CELL_TYPE::EMPTY,// 1
				CELL_TYPE::EMPTY,CELL_TYPE::WALL ,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,//2
				CELL_TYPE::EMPTY,CELL_TYPE::WALL ,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY ,//3
				CELL_TYPE::EMPTY,CELL_TYPE::WALL ,CELL_TYPE::WALL ,CELL_TYPE::WALL,CELL_TYPE::EMPTY, //4
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY };//5
			std::vector<std::pair<int, int>>* outPath = new std::vector<std::pair<int, int>>();
			std::vector<std::pair<int, int>>* expectedPath = new std::vector<std::pair<int, int>>();
			expectedPath->push_back(std::pair<int, int>(2, 0));
			expectedPath->push_back(std::pair<int, int>(1, 0));
			expectedPath->push_back(std::pair<int, int>(0, 0));
			expectedPath->push_back(std::pair<int, int>(0, 1));
			expectedPath->push_back(std::pair<int, int>(0, 2));
			expectedPath->push_back(std::pair<int, int>(0, 3));
			expectedPath->push_back(std::pair<int, int>(0, 4));
			expectedPath->push_back(std::pair<int, int>(1, 4));
			expectedPath->push_back(std::pair<int, int>(2, 4));
			expectedPath->push_back(std::pair<int, int>(2, 3));

			bool pathFound = Astar::findPath(startIndex, targetIndex, h, board, boardSize, outPath);
			Assert::IsTrue(*expectedPath == *outPath);
		}
	};
	TEST_CLASS(SpeedCheck)
	{
	public:
		/*TEST_METHOD(BigPath)
		{
			Logger logger;

			std::pair<int, int> boardSize(500, 500);

			std::pair<int, int> startIndex(boardSize.first/2 - 1, 0);
			std::pair<int, int> targetIndex(boardSize.first/2 - 1, boardSize.second-1 );
			std::function<float(std::pair<int, int>, std::pair<int, int>)> h = defaultHeuristic;

			// board is as followed
			//
			// . . . . . . . .
			// . X X X X X X .
			// . . . . . . X .
			// A . . . . . X B
			// . . . . . . X .
			// . . . . . . X .
			// . X X X X X X .
			// . . . . . . . . .

			CELL_TYPE* board = new CELL_TYPE[boardSize.first * boardSize.second];
			for(int i=0; i<boardSize.first; ++i)
				for (int j = 0; j < boardSize.second; ++j) {
					if (i == 1 || i == boardSize.first - 2)
						if (j > 0 && j < boardSize.second - 1) {
							board[i * boardSize.second + j] = CELL_TYPE::WALL;
							continue;
						}
					if(j== boardSize.second-2)
						if (i > 0 && i < boardSize.first - 1) {
							board[i * boardSize.second + j] = CELL_TYPE::WALL;
							continue;
						}
					board[i * boardSize.second + j] = CELL_TYPE::EMPTY;
				}


			std::vector<std::pair<int, int>>* outPath = new std::vector<std::pair<int, int>>();
			std::vector<std::pair<int, int>>* expectedPath = new std::vector<std::pair<int, int>>();
			for(int i = targetIndex.first; i>0; --i)
				expectedPath->push_back(std::pair<int, int>(i, targetIndex.second));
			for (int i = targetIndex.second; i > startIndex.second; --i)
				expectedPath->push_back(std::pair<int, int>(0,i));
			for (int i = 0; i <= startIndex.first; ++i)
				expectedPath->push_back(std::pair<int, int>(i, 0));

			std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

			bool pathFound = Astar::findPath(startIndex, targetIndex, h, board, boardSize, outPath);

			std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
			double elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

			Assert::IsTrue(elapsedTime > 2000); //assert takes more than 2 sec 
			std::string timeStr = "Elapsed time " + std::to_string(elapsedTime);
			logger.WriteMessage(timeStr.c_str());

			Assert::IsTrue(*expectedPath == *outPath);
		}*/
	};
}

namespace AstarDataOrientedTest
{
	TEST_CLASS(FindPath)
	{
	public:
		TEST_METHOD(LinePath)
		{
			const int NB_TARGET = 1;
			std::pair<int, int> startIndex(0, 0);
			std::pair<int, int>* targetIndex= new std::pair<int, int>(0, 4);
			std::function<float(std::pair<int, int>, std::pair<int, int>)> h = defaultHeuristic;
			std::pair<int, int> boardSize(2, 5);
			CELL_TYPE* board = new CELL_TYPE[]{
				//0                     1                  2             3                   4  
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY, //0
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY };//1
			std::vector<std::pair<int, int>>* outPaths = new std::vector<std::pair<int, int>>[NB_TARGET];
			for (int i = 0; i < NB_TARGET; ++i)
				outPaths[i] = std::vector<std::pair<int, int>>();
			bool* outSuccess = new bool[NB_TARGET];
			for (int i = 0; i < NB_TARGET; ++i)
				outSuccess[i] = false;
			std::vector<std::pair<int, int>>* expectedPath = new std::vector<std::pair<int, int>>();
			expectedPath->push_back(std::pair<int, int>(0, 4));
			expectedPath->push_back(std::pair<int, int>(0, 3));
			expectedPath->push_back(std::pair<int, int>(0, 2));
			expectedPath->push_back(std::pair<int, int>(0, 1));
			expectedPath->push_back(std::pair<int, int>(0, 0));

			AstarDataOriented::findPaths(startIndex, targetIndex,NB_TARGET, h, board, boardSize, getTravelCostMap(), outPaths,outSuccess);
			Assert::IsTrue(*expectedPath == outPaths[0]);
		}

		TEST_METHOD(WallPath)
		{
			const int NB_TARGET = 1;

			std::pair<int, int> startIndex(2, 0);
			std::pair<int, int>* targetIndex = new std::pair<int, int>(2, 4);
			std::function<float(std::pair<int, int>, std::pair<int, int>)> h = defaultHeuristic;
			std::pair<int, int> boardSize(6, 5);
			CELL_TYPE* board = new CELL_TYPE[]{
				//0                     1                  2             3                   4  
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,// 0
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,// 1
				CELL_TYPE::EMPTY,CELL_TYPE::WALL, CELL_TYPE::WALL,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,//2
				CELL_TYPE::EMPTY,CELL_TYPE::WALL, CELL_TYPE::WALL,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY ,//3
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY, //4
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY };//5
			
			std::vector<std::pair<int, int>>* outPaths = new std::vector<std::pair<int, int>>[NB_TARGET];
			for (int i = 0; i < NB_TARGET; ++i)
				outPaths[i] = std::vector<std::pair<int, int>>();
			bool* outSuccess = new bool[NB_TARGET];
			for (int i = 0; i < NB_TARGET; ++i)
				outSuccess[i] = false;

			std::vector<std::pair<int, int>>* expectedPath = new std::vector<std::pair<int, int>>();
			expectedPath->push_back(std::pair<int, int>(2, 4));
			expectedPath->push_back(std::pair<int, int>(1, 3));
			expectedPath->push_back(std::pair<int, int>(1, 2));
			expectedPath->push_back(std::pair<int, int>(1, 1));
			expectedPath->push_back(std::pair<int, int>(1, 0));
			expectedPath->push_back(std::pair<int, int>(2, 0));

			AstarDataOriented::findPaths(startIndex, targetIndex, NB_TARGET, h, board, boardSize, getTravelCostMap(), outPaths, outSuccess);
			Assert::IsTrue(*expectedPath == outPaths[0]);
		}
		TEST_METHOD(StuckInHolePath)
		{
			const int NB_TARGET = 1;

			std::pair<int, int> startIndex(2, 3);
			std::pair<int, int>* targetIndex = new std::pair<int, int>(2, 0);
			std::function<float(std::pair<int, int>, std::pair<int, int>)> h = defaultHeuristic;
			std::pair<int, int> boardSize(6, 5);
			CELL_TYPE* board = new CELL_TYPE[]{
				//0                     1                  2             3                   4  
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,// 0
				CELL_TYPE::EMPTY,CELL_TYPE::WALL ,CELL_TYPE::WALL ,CELL_TYPE::WALL,CELL_TYPE::EMPTY,// 1
				CELL_TYPE::EMPTY,CELL_TYPE::WALL ,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,//2
				CELL_TYPE::EMPTY,CELL_TYPE::WALL ,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY ,//3
				CELL_TYPE::EMPTY,CELL_TYPE::WALL ,CELL_TYPE::WALL ,CELL_TYPE::WALL,CELL_TYPE::EMPTY, //4
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY };//5
			std::vector<std::pair<int, int>>* outPaths = new std::vector<std::pair<int, int>>[NB_TARGET];
			for (int i = 0; i < NB_TARGET; ++i)
				outPaths[i] = std::vector<std::pair<int, int>>();
			bool* outSuccess = new bool[NB_TARGET];
			for (int i = 0; i < NB_TARGET; ++i)
				outSuccess[i] = false;
			std::vector<std::pair<int, int>>* expectedPath = new std::vector<std::pair<int, int>>();
			expectedPath->push_back(std::pair<int, int>(2, 0));
			expectedPath->push_back(std::pair<int, int>(1, 0));
			expectedPath->push_back(std::pair<int, int>(0, 0));
			expectedPath->push_back(std::pair<int, int>(0, 1));
			expectedPath->push_back(std::pair<int, int>(0, 2));
			expectedPath->push_back(std::pair<int, int>(0, 3));
			expectedPath->push_back(std::pair<int, int>(0, 4));
			expectedPath->push_back(std::pair<int, int>(1, 4));
			expectedPath->push_back(std::pair<int, int>(2, 4));
			expectedPath->push_back(std::pair<int, int>(2, 3));

			AstarDataOriented::findPaths(startIndex, targetIndex, NB_TARGET, h, board, boardSize, getTravelCostMap(), outPaths, outSuccess);
			Assert::IsTrue(*expectedPath == outPaths[0]);
		}
	};
	TEST_CLASS(SpeedCheck)
	{
	public:
		TEST_METHOD(BigPath)
		{
			const int NB_TARGET = 1;

			Logger logger;

			std::pair<int, int> boardSize(500, 500);

			std::pair<int, int> startIndex(boardSize.first / 2 - 2, 0);
			std::pair<int, int>* targetIndex = new std::pair<int, int>(startIndex.first, boardSize.second - 1);
			std::function<float(std::pair<int, int>, std::pair<int, int>)> h = defaultHeuristic;

			// board is as followed
			//
			// . . . . . . . .
			// . X X X X X X .
			// . . . . . . X .
			// A . . . . . X B
			// . . . . . . X .
			// . . . . . . X .
			// . X X X X X X .
			// . . . . . . . . .

			CELL_TYPE* board = new CELL_TYPE[boardSize.first * boardSize.second];
			for (int i = 0; i < boardSize.first; ++i)
				for (int j = 0; j < boardSize.second; ++j) {
					if (i == 1 || i == boardSize.first - 2)
						if (j > 0 && j < boardSize.second - 1) {
							board[i * boardSize.second + j] = CELL_TYPE::WALL;
							continue;
						}
					if (j == boardSize.second - 2)
						if (i > 0 && i < boardSize.first - 1) {
							board[i * boardSize.second + j] = CELL_TYPE::WALL;
							continue;
						}
					board[i * boardSize.second + j] = CELL_TYPE::EMPTY;
				}


			std::vector<std::pair<int, int>>* outPaths = new std::vector<std::pair<int, int>>[NB_TARGET];
			for (int i = 0; i < NB_TARGET; ++i)
				outPaths[i] = std::vector<std::pair<int, int>>();
			bool* outSuccess = new bool[NB_TARGET];
			for (int i = 0; i < NB_TARGET; ++i)
				outSuccess[i] = false;

			std::vector<std::pair<int, int>>* expectedPath = new std::vector<std::pair<int, int>>();
			for (int i = targetIndex[0].first; i > 0; --i)
				expectedPath->push_back(std::pair<int, int>(i, targetIndex[0].second));
			for (int i = targetIndex[0].second; i > startIndex.second; --i)
				expectedPath->push_back(std::pair<int, int>(0, i));
			for (int i = 0; i <= startIndex.first; ++i)
				expectedPath->push_back(std::pair<int, int>(i, 0));

			std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

			AstarDataOriented::findPaths(startIndex, targetIndex, NB_TARGET, h, board, boardSize, getTravelCostMap(), outPaths, outSuccess);

			std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
			double elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

			std::string timeStr = "Elapsed time " + std::to_string(elapsedTime);
			logger.WriteMessage(timeStr.c_str());

			Assert::IsTrue(*expectedPath == outPaths[0]);
		}
	}; 
	TEST_CLASS(ParallelFindPath)
	{
	public:
		TEST_METHOD(LinePathSameDirection)
		{
			const int NB_TARGET = 4;

			std::pair<int, int> startIndex(0, 0);
			std::pair<int, int>* targetIndices = new std::pair<int, int>[NB_TARGET];
			for(int i =0; i< NB_TARGET; ++i)
				targetIndices[i] = std::pair<int,int>(0, 4);

			std::function<float(std::pair<int, int>, std::pair<int, int>)> h = defaultHeuristic;
			std::pair<int, int> boardSize(2, 5);
			CELL_TYPE* board = new CELL_TYPE[]{
				//0                     1                  2             3                   4  
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY, //0
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY };//1
			std::vector<std::pair<int, int>>* outPaths = new std::vector<std::pair<int, int>>[NB_TARGET];
			for (int i = 0; i < NB_TARGET; ++i)
				outPaths[i] = std::vector<std::pair<int,int>>();
			bool* outSuccess = new bool[NB_TARGET];
			for (int i = 0; i < NB_TARGET; ++i)
				outSuccess[i] = false;

			std::vector<std::pair<int, int>>* expectedPath = new std::vector<std::pair<int, int>>();
			expectedPath->push_back(std::pair<int, int>(0, 4));
			expectedPath->push_back(std::pair<int, int>(0, 3));
			expectedPath->push_back(std::pair<int, int>(0, 2));
			expectedPath->push_back(std::pair<int, int>(0, 1));
			expectedPath->push_back(std::pair<int, int>(0, 0));

			AstarDataOriented::findPaths(startIndex, targetIndices,NB_TARGET, h, board, boardSize, getTravelCostMap(), outPaths,outSuccess);

			for (int i = 0; i < NB_TARGET; ++i) {
				Assert::IsTrue(outSuccess[i]);
				Assert::IsTrue(*expectedPath == outPaths[i]);
			}
		}
		TEST_METHOD(LinePathDifferentDirection)
		{
			const int NB_TARGET = 4;

			std::pair<int, int> startIndex(2, 2);
			std::pair<int, int>* targetIndices = new std::pair<int, int>[NB_TARGET];
			targetIndices[0] = std::pair<int, int>(2, 0);
			targetIndices[1] = std::pair<int, int>(0, 2);
			targetIndices[2] = std::pair<int, int>(2, 4);
			targetIndices[3] = std::pair<int, int>(4, 2);

			std::function<float(std::pair<int, int>, std::pair<int, int>)> h = defaultHeuristic;
			std::pair<int, int> boardSize(5, 5);
			CELL_TYPE* board = new CELL_TYPE[]{
				//0                     1                  2             3                   4  
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY, //0
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY, //1
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY, //2
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY , //3
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY };//4
			std::vector<std::pair<int, int>>* outPaths = new std::vector<std::pair<int, int>> [NB_TARGET];
			for (int i = 0; i < NB_TARGET; ++i)
				outPaths[i] = std::vector<std::pair<int, int>>();
			bool* outSuccess = new bool[NB_TARGET];
			for (int i = 0; i < NB_TARGET; ++i)
				outSuccess[i] = false;

			std::vector<std::pair<int, int>>* expectedPath = new std::vector<std::pair<int, int>>[NB_TARGET];
			std::pair<int, int> currentCell;
			for (int i = 0; i < NB_TARGET; ++i) {
				expectedPath[i] = std::vector<std::pair<int, int>>();
				currentCell = targetIndices[i];

				int columnDir = i==0?1:(i==2?-1 : 0);
				int lineDir = i==1?1:(i==3?-1:0);
				while (currentCell.first != startIndex.first || currentCell.second != startIndex.second) {
					expectedPath[i].push_back(currentCell);
					currentCell = std::pair<int, int>(currentCell.first + lineDir, currentCell.second + columnDir);
				}
				expectedPath[i].push_back(currentCell); //push start cell
			}
	
			AstarDataOriented::findPaths(startIndex, targetIndices, NB_TARGET, h, board, boardSize, getTravelCostMap(), outPaths, outSuccess);

			for (int i = 0; i < NB_TARGET; ++i) {
				Assert::IsTrue(outSuccess[i]);
				Assert::IsTrue(expectedPath[i] == outPaths[i]);
			}
		}
		TEST_METHOD(StraightLineWithFear)
		{
			double fearStrength = 100;
			double fearArea = 2; // The mouse will start to avoid the cell when it is 2 cells aways from it 
			const int NB_TARGET = 1;
			std::pair<int, int> startIndex(0, 0);
			std::pair<int, int> cellToFleeIndex(0, 2);
			bool sameCellTypePenalty = 0;
			/* The fear cost is as followed
			  1    10   100  10    1
			0.58  3.8   10   3.8  0.58
			*/
			std::pair<int, int>* targetIndex = new std::pair<int, int>(0, 4);
			std::function<float(std::pair<int, int>, std::pair<int, int>)> h = defaultHeuristic;
			std::pair<int, int> boardSize(2, 5);
			CELL_TYPE* board = new CELL_TYPE[]{
				//0                     1                  2             3                   4  
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY, //0
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY };//1
			std::vector<std::pair<int, int>>* outPaths = new std::vector<std::pair<int, int>>[NB_TARGET];
			for (int i = 0; i < NB_TARGET; ++i)
				outPaths[i] = std::vector<std::pair<int, int>>();
			bool* outSuccess = new bool[NB_TARGET];
			for (int i = 0; i < NB_TARGET; ++i)
				outSuccess[i] = false;

			//The solution is expected to avoid cells  0,1 0,2 and 0,3 as they have a very high fear cost 
			//The direct path has a fear cost of 122 while the solution path has a fear cost of 19.6 
			std::vector<std::pair<int, int>>* expectedPath = new std::vector<std::pair<int, int>>();
			expectedPath->push_back(std::pair<int, int>(0, 4));
			expectedPath->push_back(std::pair<int, int>(1, 3));
			expectedPath->push_back(std::pair<int, int>(1, 2));
			expectedPath->push_back(std::pair<int, int>(1, 1));
			expectedPath->push_back(std::pair<int, int>(0, 0));

			AstarDataOriented::findPaths(startIndex, targetIndex, NB_TARGET, h, fearStrength, fearArea, cellToFleeIndex, sameCellTypePenalty, board, boardSize, getTravelCostMap(),outPaths, outSuccess);
			Assert::IsTrue(*expectedPath == outPaths[0]);
		}
		TEST_METHOD(StraightLineWithBar)
		{
			const int NB_TARGET = 1;
			std::pair<int, int> startIndex(0, 0);
			std::pair<int, int> cellToFleeIndex(0, 2);
			std::map<CELL_TYPE, float> travelCostMap;
			travelCostMap.insert(std::pair<CELL_TYPE, int>(CELL_TYPE::EMPTY, 1));
			travelCostMap.insert(std::pair<CELL_TYPE, int>(CELL_TYPE::BAR, 1.1));
			travelCostMap.insert(std::pair<CELL_TYPE, int>(CELL_TYPE::STEP, 2));
			travelCostMap.insert(std::pair<CELL_TYPE, int>(CELL_TYPE::STEP_BAR, 2));
			travelCostMap.insert(std::pair<CELL_TYPE, int>(CELL_TYPE::WALL, 100000));
			/* The fear cost is as followed
			  1    10   100  10    1
			0.58  3.8   10   3.8  0.58
			*/
			std::pair<int, int>* targetIndex = new std::pair<int, int>(0, 4);
			std::function<float(std::pair<int, int>, std::pair<int, int>)> h = defaultHeuristic;
			std::pair<int, int> boardSize(2, 5);
			CELL_TYPE* board = new CELL_TYPE[]{
				//0                     1                  2             3                   4  
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::BAR,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY, //0
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY };//1
			std::vector<std::pair<int, int>>* outPaths = new std::vector<std::pair<int, int>>[NB_TARGET];
			for (int i = 0; i < NB_TARGET; ++i)
				outPaths[i] = std::vector<std::pair<int, int>>();
			bool* outSuccess = new bool[NB_TARGET];
			for (int i = 0; i < NB_TARGET; ++i)
				outSuccess[i] = false;

			//The solution is expected to avoid cells  0,1 0,2 and 0,3 as they have a very high fear cost 
			//The direct path has a fear cost of 122 while the solution path has a fear cost of 19.6 
			std::vector<std::pair<int, int>>* expectedPath = new std::vector<std::pair<int, int>>();
			expectedPath->push_back(std::pair<int, int>(0, 4));
			expectedPath->push_back(std::pair<int, int>(0, 3));
			expectedPath->push_back(std::pair<int, int>(0, 2));
			expectedPath->push_back(std::pair<int, int>(0, 1));
			expectedPath->push_back(std::pair<int, int>(0, 0));

			AstarDataOriented::findPaths(startIndex, targetIndex, NB_TARGET, h, board, boardSize, travelCostMap, outPaths, outSuccess);
			Assert::IsTrue(*expectedPath == outPaths[0]);
		}
	};
}