#include "pch.h"
#include <utility>
#include "CppUnitTest.h"
#include "../../ChaseTagAI/ChaseTagAI/Astar.h"
using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace ChaseTagAiTest
{
	TEST_CLASS(UnitTest1)
	{
	public:
		
		TEST_METHOD(SimplePath)
		{
			std::pair<int, int> startIndex(0,0);
			std::pair<int, int> targetIndex(4,0);
			std::function<float(std::pair<int, int>&)> h;
			std::pair<int, int> boardSize(5, 2);
			// 0 0 0 0 0 
			// 0 0 0 0 0
			CELL_TYPE* board = new CELL_TYPE[]{
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,
				CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY,CELL_TYPE::EMPTY };
			std::vector<std::pair<int, int>*> outPath;
			bool pathFound = Astar::findPath(startIndex, targetIndex, h, board, boardSize, outPath);
			//bool pathFound =true;
			//float dist = distance(std::make_pair(0, 0), std::make_pair(0, 1));
			Assert::IsTrue(pathFound);
		}
	};
}
