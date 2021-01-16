#include "../pch.h"
#include <utility>
#include <iostream>
#include <vector>
#include <chrono>
#include <functional>
#include "CppUnitTest.h"
//Astar dependencies
#include <ChaseTagAI\Source\AstarDataOriented\AstarDataOriented.h>
#include <ChaseTagAI\Source\Strategy\PathFinding\FleePathFinding.h>
using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace {
	CELL_TYPE* convertToCellType(int arr[], int length) {
		CELL_TYPE* board = new CELL_TYPE[length];
		for (int i = 0; i < length; ++i) {
			if (arr[i] == 0)
				board[i] = CELL_TYPE::EMPTY;
			else if (arr[i] == 1) {
				board[i] = CELL_TYPE::WALL;
			}
			else {
				throw std::invalid_argument("Got integer of value " + std::to_string(arr[i]) + " but expected 0 (EMPTY) or 1 (WALL)");
			}
		}
		return board;
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

namespace FleePathFindingTest
{
	TEST_CLASS(Execute)
	{
	public:
		TEST_METHOD(FleeNoFear)
		{
			std::pair<int, int> boardSize(10, 10);
			float fearStrength = 0;
			float fearArea = 1;
			float sameCellTypePenalty = 0;
			CELL_TYPE* board = convertToCellType(new int[] {
			  //0  1  2  3  4  5  6  7  8  9
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //0
				0, 1, 1, 1, 0, 0, 0, 0, 0, 0, //1
				0, 1, 0, 0, 0, 0, 0, 0, 0, 0, //2
				0, 1, 0, 0, 0, 0, 0, 0, 0, 0, //3
				0, 1, 0, 0, 0, 0, 0, 0, 0, 0, //4
				0, 1, 0, 0, 0, 0, 0, 0, 0, 0, //5
				0, 1, 0, 0, 0, 0, 0, 0, 0, 0, //6
				0, 1, 0, 0, 0, 1, 1, 0, 0, 0, //7
				0, 1, 1, 1, 0, 0, 0, 0, 0, 0, //8
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0  //9
			}, 100);
			
			std::pair<int, int> mouseCell(5, 5);
			std::pair<int, int> catCell(4, 4);
			//The cat is 11+ sqrt(2) distant from 5,0 which is the furthest cell
			std::vector<std::pair<int, int>>* outPath = new std::vector<std::pair<int, int>>();
			std::vector<std::pair<int, int>>* expectedPath = new std::vector<std::pair<int, int>>();
			//Reminder: std::pair<line number, column number>
			expectedPath->push_back(std::pair<int, int>(5, 0));
			expectedPath->push_back(std::pair<int, int>(6, 0));
			expectedPath->push_back(std::pair<int, int>(7, 0));
			expectedPath->push_back(std::pair<int, int>(8, 0));
			expectedPath->push_back(std::pair<int, int>(9, 0));
			expectedPath->push_back(std::pair<int, int>(9, 1));
			expectedPath->push_back(std::pair<int, int>(9, 2));
			expectedPath->push_back(std::pair<int, int>(9, 3));
			expectedPath->push_back(std::pair<int, int>(9, 4));
			expectedPath->push_back(std::pair<int, int>(8, 4));
			expectedPath->push_back(std::pair<int, int>(7, 4));
			expectedPath->push_back(std::pair<int, int>(6, 4));
			expectedPath->push_back(std::pair<int, int>(5, 5));


			FleePathFinding::execute(board, boardSize, getTravelCostMap(), mouseCell, catCell, fearStrength,fearArea, sameCellTypePenalty, outPath);
			if (outPath != nullptr && outPath->size()>0) {
				Assert::AreEqual(expectedPath->size(), outPath->size());
				for (int i = 0; i < expectedPath->size(); ++i) {
					Assert::IsTrue(*expectedPath == *outPath);
				}
			}
			else {
				//This assert will fail because of if condition
				Assert::IsFalse(outPath == nullptr);
			}

		}

		TEST_METHOD(FleeFear)
		{
			std::pair<int, int> boardSize(10, 10);
			float fearStrength = 100;
			float fearArea = 50;
			float sameCellTypePenalty = 0;
			CELL_TYPE* board = convertToCellType(new int[] {
				  //0  1  2  3  4  5  6  7  8  9
					0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //0
					0, 1, 1, 1, 0, 0, 0, 0, 0, 0, //1
					0, 1, 0, 0, 0, 0, 0, 0, 0, 0, //2
					0, 1, 0, 0, 0, 0, 0, 0, 0, 0, //3
					0, 1, 0, 0, 0, 0, 0, 0, 0, 0, //4
					0, 1, 0, 0, 0, 0, 0, 0, 0, 0, //5
					0, 1, 0, 0, 0, 0, 0, 0, 0, 0, //6
					0, 1, 0, 0, 0, 1, 1, 0, 0, 0, //7
					0, 1, 1, 1, 0, 0, 0, 0, 0, 0, //8
					0, 0, 0, 0, 0, 0, 0, 0, 0, 0  //9
				}, 100);

			std::pair<int, int> mouseCell(5, 5);
			std::pair<int, int> catCell(4, 4);
			//The cat is 11+ sqrt(2) distant from 5,0 which is the furthest cell
			std::vector<std::pair<int, int>>* outPath = new std::vector<std::pair<int, int>>();
			std::vector<std::pair<int, int>>* expectedPath = new std::vector<std::pair<int, int>>();
			//Reminder: std::pair<line number, column number>
			expectedPath->push_back(std::pair<int, int>(5, 0));
			expectedPath->push_back(std::pair<int, int>(6, 0));
			expectedPath->push_back(std::pair<int, int>(7, 0));
			expectedPath->push_back(std::pair<int, int>(8, 0));
			expectedPath->push_back(std::pair<int, int>(9, 0));
			expectedPath->push_back(std::pair<int, int>(9, 1));
			expectedPath->push_back(std::pair<int, int>(9, 2));
			expectedPath->push_back(std::pair<int, int>(9, 3));
			expectedPath->push_back(std::pair<int, int>(9, 4));
			expectedPath->push_back(std::pair<int, int>(8, 5));
			expectedPath->push_back(std::pair<int, int>(8, 6));
			expectedPath->push_back(std::pair<int, int>(8, 7));
			expectedPath->push_back(std::pair<int, int>(7, 7));
			expectedPath->push_back(std::pair<int, int>(6, 7));
			expectedPath->push_back(std::pair<int, int>(6, 6));
			expectedPath->push_back(std::pair<int, int>(5, 5));


			FleePathFinding::execute(board, boardSize, getTravelCostMap(), mouseCell, catCell, fearStrength, fearArea, sameCellTypePenalty, outPath);
			if (outPath != nullptr && outPath->size() > 0) {
				Assert::AreEqual(expectedPath->size(), outPath->size());
				for (int i = 0; i < expectedPath->size(); ++i) {
					Assert::IsTrue(*expectedPath == *outPath);
				}
			}
			else {
				//This assert will fail because of if condition
				Assert::IsFalse(outPath == nullptr);
			}

		}
	};
}
