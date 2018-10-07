#ifndef PROBLEM_FINDPATH_H
#define PROBLEM_FINDPATH_H

#include "AStarEngine.h"

using namespace AStarEng;

class PathNode : public AANode
{
public:
    PathNode() 
        : m_x(0), m_y(0)
    {
        genKey();
    }
    PathNode(const int x, const int y)
        : m_x(x), m_y(y)
    {
        genKey();
    }
    ~PathNode() {}

    int X() const { return m_x; }
    int Y() const { return m_y; }
    void SetXY(const int x, const int y) {
        m_x = x; 
        m_y = y;
        genKey();
    }
    void Print() {
        printf("Node position : (%d,%d)\n", m_x, m_y);
    }

private:
    void genKey()
    {
        int key = 0;
        key = m_x * 100 + m_y;
        m_key = key;
    }
private:
    int m_x;
    int m_y;
};
inline int distance(const PathNode &nodeA, const PathNode &nodeB)
{
    int dist = abs(nodeA.X() - nodeB.X()) + abs(nodeA.Y() - nodeB.Y());
    return dist;
}

namespace {
    const int MAP_WIDTH = 20;
    const int MAP_HEIGHT = 20;
    const int world_map[MAP_WIDTH * MAP_HEIGHT] =
    {

    // 0001020304050607080910111213141516171819
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,   // 00
        1,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,1,   // 01
        1,9,9,1,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 02
        1,9,9,1,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 03
        1,9,1,1,1,1,9,9,1,9,1,9,1,1,1,1,9,9,1,1,   // 04
        1,9,1,1,9,1,1,1,1,9,1,1,1,1,9,1,1,1,1,1,   // 05
        1,9,9,9,9,1,1,1,1,1,1,9,9,9,9,1,1,1,1,1,   // 06
        1,9,9,9,9,9,9,9,9,1,1,1,9,9,9,9,9,9,9,1,   // 07
        1,9,1,1,1,1,1,1,1,1,1,9,1,1,1,1,1,1,1,1,   // 08
        1,9,1,9,9,9,9,9,9,9,1,1,9,9,9,9,9,9,9,1,   // 09
        1,9,1,1,1,1,9,1,1,9,1,1,1,1,1,1,1,1,1,1,   // 10
        1,9,9,9,9,9,1,9,1,9,1,9,9,9,9,9,1,1,1,1,   // 11
        1,9,1,9,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 12
        1,9,1,9,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 13
        1,9,1,1,1,1,9,9,1,9,1,9,1,1,1,1,9,9,1,1,   // 14
        1,9,1,1,9,1,1,1,1,9,1,1,1,1,9,1,1,1,1,1,   // 15
        1,9,9,9,9,1,1,1,1,1,1,9,9,9,9,1,1,1,1,1,   // 16
        1,1,9,9,9,9,9,9,9,1,1,1,9,9,9,1,9,9,9,9,   // 17
        1,9,1,1,1,1,1,1,1,1,1,9,1,1,1,1,1,1,1,1,   // 18
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,   // 19
    };
}

class PathGrid : public AAGrid<PathNode>
{
public:
    // Calculate the heuristic distance from _srcNode_ to _goalNode_
    double CalH(const PathNode &srcNode, const PathNode &goalNode)
    {
        return distance(srcNode, goalNode);
    }
    // Calculate the distance(cost) of travelling from _srcNode_ to _dstNode_
    double CalG(const PathNode &srcNode, const PathNode &dstNode)
    {
        return GetMap(dstNode);
    }
    // Judge that if _nodeA_ is the _goalNode_
    bool IsGoalNode(const PathNode& node, const PathNode& goalNode)
    {
        return IsSameNode(node, goalNode);
    }
    // Get the successor nodes of _srcNode_
    unsigned int GetSuccessors(const PathNode &node, const PathNode * const pParent, vector<PathNode> &successorNodes)
    {
        vector<PathNode> successors;

        int px = -1;
        int py = -1;
        if (pParent)
        {
            px = pParent->X();
            py = pParent->Y();
        }
        int x = node.X();
        int y = node.Y();
        // push each possible move except allowing the search to go backwards

        if (GetMap(x - 1, y) < 9 &&
            (px != x - 1 || py != y) ) {
            successors.push_back(PathNode(x - 1, y));
        }

        if (GetMap(x, y - 1) < 9 &&
            (px != x || py != y - 1) ) {
            successors.push_back(PathNode(x, y - 1));
        }

        if (GetMap(x + 1, y) < 9 &&
            (px != x + 1 || py != y) ) {
            successors.push_back(PathNode(x + 1, y));
        }      

        if (GetMap(x, y + 1) < 9 &&
            (px != x || py != y + 1) ) {
            successors.push_back(PathNode(x, y + 1));
        }

        std::swap(successors, successorNodes);
        return successorNodes.size();
    }
    

    int GetMap(const PathNode& node) const
    { 
        return GetMap(node.X(), node.Y());
    }

    int GetMap(const int x, const int y) const
    {
        int n = 9;
        if (x >= 0 && x < MAP_WIDTH &&
            y >= 0 && y < MAP_HEIGHT )
        {
            n = world_map[(y*MAP_WIDTH) + x];
        }
        return n;
    }
};

inline void doFindPathTest()
{
    printf("  Test FindPath problem \n");
    printf("==============================\n");
    srand(201810);
    PathNode nodeStart(rand() % MAP_WIDTH, rand() % MAP_HEIGHT);
    PathNode nodeGoal(rand() % MAP_WIDTH, rand() % MAP_HEIGHT);
    printf("(%d,%d) to (%d,%d) \n\n", nodeStart.X(), nodeStart.Y(), nodeGoal.X(), nodeGoal.Y());

    AAEngine<PathNode, PathGrid> astarsearch(512);
    astarsearch.SetStartAndGoalNodes(nodeStart, nodeGoal);
    astarsearch.Search();

    if (astarsearch.GetState() == ASE_STATE_SUCCEEDED)
    {
        printf("[:)] Search found goal state.\n\n");

        int steps = 0;

        printf("Displaying solution >> \n\n");
        steps = 0;
        PathNode *node = astarsearch.GetSolutionStart();
        while (node) {
            node->Print();
            node = astarsearch.GetSolutionNext();
            printf("\n");
            steps++;
        }
        printf("Solution steps %d\n", steps);

        printf("\n-----\n\n");

        printf("Displaying reverse solution >> \n\n");
        steps = 0;
        node = astarsearch.GetSolutionEnd();
        while (node) {
            node->Print();
            node = astarsearch.GetSolutionPrev();
            printf("\n");
            steps++;
        }
        printf("Solution steps %d\n", steps);

        node = astarsearch.GetSolutionEnd();
    }
    else if (astarsearch.GetState() == ASE_STATE_FAILED)
    {
        printf("[:(] Search terminated. Did not find goal state.\n\n");
    }
    else if (astarsearch.GetState() == ASE_STATE_OUT_OF_MEMORY)
    {
        printf("[:(] Search terminated. Out of memory. \n\n");
    }
    printf("\n-----\n\n");
    printf("Total search steps : %d \n", astarsearch.GetStepCount());
}

#endif //!PROBLEM_FINDPATH_H
