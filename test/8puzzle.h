#ifndef PROBLEM_EIGHTPUZZLE_H
#define PROBLEM_EIGHTPUZZLE_H

#include "AStarEngine.h"

using namespace AStarEng;

#define BOARD_WIDTH   (3)
#define BOARD_HEIGHT  (3)

#define GM_TILE     (-1)
#define GM_SPACE	 (0)
#define GM_OFF_BOARD (1)

typedef enum {
    TL_SPACE,
    TL_1,
    TL_2,
    TL_3,
    TL_4,
    TL_5,
    TL_6,
    TL_7,
    TL_8
} TILE;

class PuzzleNode : public AANode
{
public:
    PuzzleNode(const TILE *tile = 0)
    {
        SetTile(tile);
    }
    ~PuzzleNode() {}

    void Print() {
        printf("%c %c %c\n%c %c %c\n%c %c %c\n",
            m_tile[0] + '0', m_tile[1] + '0', m_tile[2] + '0',
            m_tile[3] + '0', m_tile[4] + '0', m_tile[5] + '0',
            m_tile[6] + '0', m_tile[7] + '0', m_tile[8] + '0'
        );
    }
    PuzzleNode& operator= (const PuzzleNode& node)
    {
        memcpy(m_tile, node.Tile(), sizeof(m_tile));
        m_key = node.Key();
        return *this;
    }
    const TILE* Tile() const {
        return m_tile;
    }
    void SetTile(const TILE *tile)
    {
        if (tile) {
            memcpy(m_tile, tile, sizeof(m_tile));
            genKey();
        }
    }
    void GetSpacePosition (int &rx, int &ry) const
    {
        for (int i=0; i<BOARD_WIDTH*BOARD_HEIGHT; ++i)
        {
            if (m_tile[i] == TL_SPACE)
            {
                rx = i%BOARD_WIDTH;
                ry = i/BOARD_WIDTH;
                return;
            }
        }
        assert(false && "Something went wrong. There's no space on the board");
    }
    int GetMap(int x, int y) const
    {
        if (x < 0 || x >= BOARD_WIDTH ||
            y < 0 || y >= BOARD_HEIGHT )
            return GM_OFF_BOARD;

        if (m_tile[(y*BOARD_WIDTH) + x] == TL_SPACE) {
            return GM_SPACE;
        }
        return GM_TILE;
    }
    bool LegalMove(int spx, int spy, int tx, int ty)
    {
        if (GetMap(spx, spy) == GM_SPACE &&
            GetMap(tx, ty) == GM_TILE) {
            TILE temp = m_tile[(ty*BOARD_WIDTH) + tx];
            m_tile[(ty*BOARD_WIDTH) + tx] = m_tile[(spy*BOARD_WIDTH) + spx];
            m_tile[(spy*BOARD_WIDTH) + spx] = temp;
            genKey();
            return true;
        }
        return false;
    }
private:
    IDKey genKey()
    {
        int key = 0;
        for (int i = 0; i < BOARD_WIDTH*BOARD_HEIGHT; ++i)
        {
            key = key*10;
            key += (m_tile[i] & 0xF);
        }
        m_key = key;
        return m_key;
    }
private:
    TILE m_tile[BOARD_WIDTH*BOARD_HEIGHT];
};

class PuzzleGrid : public AAGrid<PuzzleNode>
{
public:
    // Calculate the heuristic distance from _srcNode_ to _goalNode_
    double CalH(const PuzzleNode &srcNode, const PuzzleNode &goalNode)
    {
        const TILE *p1 = srcNode.Tile();
        const TILE *p2 = goalNode.Tile();

        // Nilsson's sequence score

        int i, cx, cy, ax, ay, h = 0, s, t;

        // given a tile this returns the tile that should be clockwise
        TILE correct_follower_to[BOARD_WIDTH * BOARD_HEIGHT] =
        {
            TL_SPACE, // always wrong
            TL_2,
            TL_3,
            TL_4,
            TL_5,
            TL_6,
            TL_7,
            TL_8,
            TL_1,
        };

        // given a table index returns the index of the tile that is clockwise to it 3*3 only
        int clockwise_tile_of[BOARD_WIDTH * BOARD_HEIGHT] =
        {
            1,
            2,  	  // 012
            5,  	  // 345
            0,  	  // 678
            -1,  // never called with center square
            8,
            3,
            6,
            7
        };

        int tile_x[BOARD_WIDTH * BOARD_HEIGHT] =
        {
            /* TL_SPACE */ 1,
            /* TL_1 */ 0,
            /* TL_2 */ 1,
            /* TL_3 */ 2,
            /* TL_4 */ 2,
            /* TL_5 */ 2,
            /* TL_6 */ 1,
            /* TL_7 */ 0,
            /* TL_8 */ 0,
        };

        int tile_y[BOARD_WIDTH * BOARD_HEIGHT] =
        {
            /* TL_SPACE */ 1,
            /* TL_1 */ 0,
            /* TL_2 */ 0,
            /* TL_3 */ 0,
            /* TL_4 */ 1,
            /* TL_5 */ 2,
            /* TL_6 */ 2,
            /* TL_7 */ 2,
            /* TL_8 */ 1,
        };

        s = 0;

        // score 1 point if centre is not correct
        if (p1[(BOARD_HEIGHT*BOARD_WIDTH) / 2] != p2[(BOARD_HEIGHT*BOARD_WIDTH) / 2])
        {
            s = 1;
        }

        for (i = 0; i<(BOARD_HEIGHT*BOARD_WIDTH); i++)
        {
            // this loop adds up the totaldist element in h and
            // the sequence score in s

            // the space does not count
            if (p1[i] == TL_SPACE)
            {
                continue;
            }

            // get correct x and y of this tile
            cx = tile_x[p1[i]];
            cy = tile_y[p1[i]];

            // get actual
            ax = i % BOARD_WIDTH;
            ay = i / BOARD_WIDTH;

            // add manhatten distance to h
            h += abs(cx - ax);
            h += abs(cy - ay);

            // no s score for center tile
            if ((ax == (BOARD_WIDTH / 2)) && (ay == (BOARD_HEIGHT / 2)))
            {
                continue;
            }

            // score 2 points if not followed by successor
            if (correct_follower_to[p1[i]] != p1[clockwise_tile_of[i]])
            {
                s += 2;
            }
        }

        // mult by 3 and add to h
        t = h + (3 * s);

        return (float)t;
    }
    // Calculate the distance(cost) of travelling from _srcNode_ to _dstNode_
    double CalG(const PuzzleNode &srcNode, const PuzzleNode &dstNode)
    {
        return 1.0;
    }
    // Judge that if _node_ is the _goalNode_
    bool IsGoalNode(const PuzzleNode &node, const PuzzleNode &goalNode)
    {
        return IsSameNode(node, goalNode);
    }
    // Get the successor nodes of _node_
    unsigned int GetSuccessors(const PuzzleNode &node, const PuzzleNode * const pParent, vector<PuzzleNode> &successorNodes)
    {
        vector<PuzzleNode> successors;
        PuzzleNode tempNode;

        int sp_x, sp_y;
        node.GetSpacePosition(sp_x, sp_y);

        tempNode = node;
        if (tempNode.LegalMove(sp_x, sp_y, sp_x, sp_y - 1) == true) {
            successors.push_back(tempNode);
        }
        tempNode = node;
        if (tempNode.LegalMove(sp_x, sp_y, sp_x, sp_y + 1) == true) {
            successors.push_back(tempNode);
        }
        tempNode = node;
        if (tempNode.LegalMove(sp_x, sp_y, sp_x - 1, sp_y) == true) {
            successors.push_back(tempNode);
        }
        tempNode = node;
        if (tempNode.LegalMove(sp_x, sp_y, sp_x + 1, sp_y) == true) {
            successors.push_back(tempNode);
        }
        std::swap(successors, successorNodes);
        return successorNodes.size();
    }
};

static TILE g_startTile[BOARD_WIDTH*BOARD_HEIGHT] = {
#if 0 // 4+1 steps
    TL_1, TL_3,     TL_4,
    TL_8, TL_SPACE, TL_2,
    TL_7, TL_6,     TL_5,
#elif 0 // 5+1 steps
    TL_2, TL_8,     TL_3,
    TL_1, TL_6,     TL_4,
    TL_7, TL_SPACE, TL_5,
#elif 1 // 18+1 steps
    TL_2, TL_1,     TL_6,
    TL_4, TL_SPACE, TL_8,
    TL_7, TL_5,     TL_3,
#elif 0
    TL_6, TL_3,     TL_SPACE,
    TL_8, TL_8,     TL_5,
    TL_7, TL_2,     TL_1,
#elif 0
    TL_1, TL_2,     TL_3,
    TL_4, TL_5,     TL_6,
    TL_8, TL_7,     TL_SPACE,
#elif 0 // easy 5+1
    TL_1, TL_3,     TL_4,
    TL_8, TL_6,     TL_2,
    TL_7, TL_SPACE, TL_5,
#elif 0 // hard 12+1
    TL_2, TL_8,     TL_1,
    TL_4, TL_6,     TL_3,
    TL_SPACE, TL_7,     TL_5,
#elif 0 // medium 9+1
    TL_2, TL_8,     TL_1,
    TL_SPACE, TL_4, TL_3,
    TL_7, TL_6,     TL_5,
#elif 0 // worst 30+1
    TL_5, TL_6,     TL_7,
    TL_4, TL_SPACE, TL_8,
    TL_3, TL_2,     TL_1,
#endif
};
static TILE g_goalTile[BOARD_WIDTH*BOARD_HEIGHT] = {
    TL_1, TL_2,     TL_3,
    TL_8, TL_SPACE, TL_4,
    TL_7, TL_6,     TL_5,
};


inline void doPuzzleTest()
{
    printf("  Test 8-puzzle problem \n");
    printf("==============================\n");

    PuzzleNode nodeStart(g_startTile);
    PuzzleNode nodeGoal(g_goalTile);
    AAEngine<PuzzleNode, PuzzleGrid> astarsearch(512);
    astarsearch.SetStartAndGoalNodes(nodeStart, nodeGoal);
    astarsearch.Search();

    if (astarsearch.GetState() == ASE_STATE_SUCCEEDED)
    {
        printf("[:)] Search found goal state.\n\n");

        int steps = 0;

        printf("Displaying solution >> \n\n");
        steps = 0;
        PuzzleNode *node = astarsearch.GetSolutionStart();
        while (node) {
            node->Print();           
            printf("\n");
            steps++;
            node = astarsearch.GetSolutionNext();
        }
        printf("Solution steps %d\n", steps);

        printf("\n-----\n\n");

        printf("Displaying reverse solution >> \n\n");
        steps = 0;
        node = astarsearch.GetSolutionEnd();
        while (node) {
            node->Print();           
            printf("\n");
            steps++;
            node = astarsearch.GetSolutionPrev();
        }
        printf("Solution steps %d\n", steps);
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

#endif //!PROBLEM_EIGHTPUZZLE_H