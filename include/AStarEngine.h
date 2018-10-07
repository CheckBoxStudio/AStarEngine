/*******************************************************************************
 * AStartEngine.h
 * An implementation of A-Star algorithem in C++.
 * > Imitate 'stlastar.h' from 'https://github.com/weiquanmao/astar-algorithm-cpp'.
 *
 * History
 * ----------------
 * Created. 04 Oct. 2018. WeiQM<weiquanmao@hotmail.com>
 *
 * <Encodeing in UTF-8>
 *******************************************************************************/

#ifndef ASTARTENGINE_H
#define ASTARTENGINE_H

#include <stdio.h>
#include <assert.h>
#include <algorithm>
#include <vector>
#include <unordered_map>

#define _DEBUG_TEST_

#define _USE_MEM_POOL_

#ifdef _USE_MEM_POOL_
#include "MemPool.h"
#endif

namespace AStarEng
{
    using std::vector;
    using std::unordered_map;

    // 'AA' means A-Star. The former letter 'A' implies for the best,
    // as the same as the '*' in A-Star(A*).

    /***************************************************************************
     * AANode: Node class
     * Traits of node should be included.
     * Users should inherit this class and add node traits for their own needs.
     * Remember to generate the key and keep it consistent with the node data,
     * also pay attention to the assignment (operator = and copy constructor,
     * default or not).
     ***************************************************************************/
    class AANode
    {
    public:
        typedef int IDKey;
    public:
        AANode() : m_key(-1) {}
        ~AANode() {}
        virtual IDKey Key() const { return m_key; }
        virtual IDKey ID() const {return m_key; }

        virtual void Print() {
            printf("%d",m_key);
        }

    protected:
        IDKey m_key; // Unique identifier of the node, invalid node has m_key<0.
    };

    /***************************************************************************
     * AAGrid : Grid class
     * Methods to handle nodes are included.
     * Users have to inherit this class and re-implement all these pure virtual
     * methods for their own needs.
     ***************************************************************************/
    template <typename NodeType> class AAGrid
    {
    public:
        // Calculate the heuristic distance from _srcNode_ to _goalNode_
        virtual double CalH(const NodeType &srcNode, const NodeType &goalNode) = 0;
        // Calculate the distance(cost) of travelling from _srcNode_ to _dstNode_
        virtual double CalG(const NodeType &srcNode, const NodeType &dstNode) = 0;
        // Get the neighbor nodes of _srcNode_
        virtual unsigned int GetNeighbors(const NodeType &node, vector<NodeType> &neighborNodes) = 0;
        // Judge that if _nodeA_ and _nodeB_ is the same node
        virtual bool IsSameNode(const NodeType& nodeA, const NodeType& nodeB) = 0;
        // Judge that if _nodeA_ is the _goalNode_
        virtual bool IsGoalNode(const NodeType& node, const NodeType& goalNode) = 0;
        // Calculate the entire cost of one route(solution) _nodes_
        virtual double CalRouteCost(const vector<NodeType> &nodes)
        {
            double costVal = 0.0;
            if (nodes.size() > 1) {
                for (int i = 1; i < nodes.size(); ++i) {
                    costVal += CalG(nodes.at(i - 1), nodes.at(i));
                }
            }
            return costVal;
        };
    };


    /***************************************************************************
     * AAEngine : A-Start Engine
     * The core routine to perform the A-Star algorithm.
     * Users have to specify the node class and grid class that work for their
     * problem through template class.
     * Note that the users should be cautious when processing the solution data,
     * and that the users should not delete any pointer got from solution result.
     * =========================================================================
     * ==== A-Start algorithm detail ====
     * OPEN_LIST = priority queue containing START
     * CLOSED_LIST = empty set
     * while lowest rank in OPEN_LIST is not the GOAL:
     *   CURRENT = remove lowest rank item from OPEN_LIST
     *   add CURRENT to CLOSED_LIST
     *   for each NEIGHBOR of CURRENT:
     *     cost = g(CURRENT) + movementcost(CURRENT, NEIGHBOR)
     *     if NEIGHBOR in OPEN_LIST and cost less than g(NEIGHBOR):
     *       set g(NEIGHBOR) to cost
     *       set NEIGHBOR's parent to CURRENT
     *       rank OPEN_LIST by f=g+h
     *     if NEIGHBOR in CLOSED_LIST and cost less than g(NEIGHBOR):[^1]
     *       set g(NEIGHBOR) to cost
     *       set NEIGHBOR's parent to CURRENT
     *       add NEIGHBOR to OPEN_LIST
     *       rank OPEN_LIST by f=g+h
     *       remove NEIGHBOR from CLOSED_LIST
     *     if NEIGHBOR not in OPEN_LIST and NEIGHBOR not in CLOSED_LIST:
     *       set g(NEIGHBOR) to cost
     *       set NEIGHBOR's parent to CURRENT
     *       add NEIGHBOR to OPEN_LIST
     *       rank OPEN_LIST by f=g+h
     * reconstruct reverse path from GOAL to START by following parent pointers
     * ---
     * [^1] This should never happen for an consistent admissible heuristic
     * Ref: http://theory.stanford.edu/~amitp/GameProgramming/ImplementationNotes.html
     * =========================================================================
     ***************************************************************************/
    enum ASE_STATE {
        ASE_STATE_INVALID,
        ASE_STATE_INITIALISED,
        ASE_STATE_SEARCHING,
        ASE_STATE_SUCCEEDED,
        ASE_STATE_CANCELED,
        ASE_STATE_FAILED,
        ASE_STATE_OUT_OF_MEMORY
    };
    template <typename NodeType, typename GridType> class AAEngine
    {
    protected:
        class ASENode : public NodeType
        {
            // convenient conversion between NodeType
        public:
            ASENode()
                : h(0.0), g(0.0), f(0.0)
                , pParent(0), pChild(0) {}
            ASENode(const NodeType &node)
                : NodeType(node)
                , h(0.0), g(0.0), f(0.0)
                , pParent(0), pChild(0) {}
            ~ASENode() {}
        public:
            double h; // heuristic estimate of distance(cost) from this node to the goal
            double g; // distance(cost) from the start node to this node
            double f; // m_f = m_g + m_h
            ASENode *pParent; // used during the search
            ASENode *pChild;  // used after the search
        };

        class HeapCmp {
        public:
            bool operator() (const ASENode *nodeA, const ASENode *nodeB) const {
                return nodeA->f > nodeB->f;
            }
        };

    public:
#ifdef _USE_MEM_POOL_
        AAEngine(const unsigned int MemPoolSize)
#else
        AAEngine()
#endif
            : m_State(ASE_STATE_INVALID)
            , m_CancelRequest(false)
#ifdef _USE_MEM_POOL_
            , m_MemPool(MemPoolSize)
#endif
            , m_StartNode(NULL)
            , m_GoalNode(NULL)
            , m_SolutionNode(NULL)
            , m_AllocateNodeCount(0)
            , m_Step(0)
        {
        }

#ifdef _USE_MEM_POOL_
        AAEngine(const unsigned int MemPoolSize, const NodeType &startNode, const NodeType &goalNode)
#else
        AAEngine(const NodeType &startNode, const NodeType &goalNode)
#endif
            : m_State(ASE_STATE_INVALID)
            , m_CancelRequest(false)
#ifdef _USE_MEM_POOL_
            , m_MemPool(MemPoolSize)
#endif
            , m_StartNode(NULL)
            , m_GoalNode(NULL)
            , m_SolutionNode(NULL)
            , m_AllocateNodeCount(0)
            , m_Step(0)
        {
            SetStartAndGoalNodes(startNode, goalNode);
        }

        ~AAEngine()
        {
            releaseData(true);
        }

        void Clear()
        {
            releaseData(true);
        }

        ASE_STATE GetState() const {
            return m_State;
        }

        void SetStartAndGoalNodes(const NodeType &startNode, const NodeType &goalNode)
        {
            assert(m_State != ASE_STATE_SEARCHING);
            releaseData(true);

            m_StartNode = allocateASENode(&startNode);
            m_GoalNode = allocateASENode(&goalNode);
            assert(m_StartNode != NULL && m_GoalNode != NULL);

            m_StartNode->g = 0.0;
            m_StartNode->h = m_Grid.CalH(*m_StartNode, *m_GoalNode);
            m_StartNode->f = m_StartNode->g + m_StartNode->h;

            pushOpenList(m_StartNode);

            m_State = ASE_STATE_INITIALISED;
        }

        // return the step it has performed
        int Search(const NodeType &startNode, const NodeType &goalNode)
        {
            SetStartAndGoalNodes(startNode, goalNode);
            return Search();
        }

        // return the step it has performed
        int Search()
        {
            m_Step = 0;
            do {
                searchOneStep();
                m_Step++;
            } while(m_State == ASE_STATE_SEARCHING);
            return (m_State == ASE_STATE_SUCCEEDED) ? m_Step : -(int)m_Step;
        }

        void CancelSearch()
        {
            if(m_State == ASE_STATE_SEARCHING)
            {
                m_CancelRequest = true;
            }
        }

        // Get result
        unsigned int GetStepCount() const
        {
            return m_Step;
        }
        double GetSolutionCost()
        {
            assert(m_State == ASE_STATE_SUCCEEDED);
            return m_GoalNode->f;
        }

        vector<NodeType*> GetSolution()
        {
            assert(m_State == ASE_STATE_SUCCEEDED);
            vector<NodeType*> solutionNodes;
            ASENode *node = m_StartNode;
            while(node) {
                solutionNodes.push_back(node);
                node = node->pChild;
            }
            return solutionNodes;
        }
        int GetSolution(vector<NodeType> &solutionNodes)
        {
            assert(m_State == ASE_STATE_SUCCEEDED);
            vector<NodeType> tmpNodes;
            ASENode *node = m_StartNode;
            while(node) {
                tmpNodes.push_back(*node);
                node = node->pChild;
            }
            std::swap(tmpNodes, solutionNodes);
            return solutionNodes.size();
        }

        NodeType* GetSolutionStart()
        {
            assert(m_State == ASE_STATE_SUCCEEDED);
            m_SolutionNode = m_StartNode;
            return m_SolutionNode;
        }
        NodeType* GetSolutionNext()
        {
            assert(m_State == ASE_STATE_SUCCEEDED);
            if (m_SolutionNode) {
                m_SolutionNode = m_SolutionNode->pChild;
            }
            return m_SolutionNode;
        }
        NodeType* GetSolutionPrev()
        {
            assert(m_State == ASE_STATE_SUCCEEDED);
            if (m_SolutionNode) {
                m_SolutionNode = m_SolutionNode->pParent;
            }
            return m_SolutionNode;
        }
        NodeType* GetSolutionEnd()
        {
            assert(m_State == ASE_STATE_SUCCEEDED);
            m_SolutionNode = m_GoalNode;
            return m_SolutionNode;
        }

    private:
        ASE_STATE searchOneStep()
        {
            /// A. check state
            assert( m_State != ASE_STATE_INVALID);
            if (m_State == ASE_STATE_SUCCEEDED ||
                m_State == ASE_STATE_FAILED )
            { // have performed the whole routine before
                return m_State;
            }
            if (m_State == ASE_STATE_INITIALISED ||
                m_State == ASE_STATE_CANCELED ||
                m_State == ASE_STATE_OUT_OF_MEMORY)
            { // begin, continue or try again
                m_State = ASE_STATE_SEARCHING;
            }

            /// B. respoce to cancel request
            if (m_CancelRequest)
            {
                m_State = ASE_STATE_CANCELED;
                return m_State;
            }

            /// C. main stuff
            ASENode *node = popOpenList();
            if (node == NULL) {
                releaseData(false);
                m_State = ASE_STATE_FAILED;
                return m_State;
            }

            m_ClosedSet[node->Key()] = node;

            if (m_Grid.IsGoalNode(*node, *m_GoalNode))
            {
                *m_GoalNode = *node;
                // link child
                ASENode *nodeChild = m_GoalNode;
                ASENode *nodeParent = m_GoalNode->pParent;
                while(nodeParent)
                {
                    nodeParent->pChild = nodeChild;
                    nodeChild = nodeParent;
                    nodeParent = nodeParent->pParent;
                }
                releaseData(false);
                m_State = ASE_STATE_SUCCEEDED;
                return m_State;
            }
            else
            {
                vector<NodeType> neighborNodes;
                m_Grid.GetNeighbors(*node, neighborNodes);
                for(vector<NodeType>::iterator iter = neighborNodes.begin();
                    iter != neighborNodes.end();
                    ++iter)
                {
                    double gVal = node->g + m_Grid.CalG(*node, *iter);
                    if (m_OpenSet.find(iter->Key()) != m_OpenSet.end())
                    {
                        ASENode *nbNode = m_OpenSet[iter->Key()];
                        if (nbNode->g > gVal)
                        {
                            nbNode->g = gVal;
                            nbNode->f = nbNode->g + nbNode->h;
                            nbNode->pParent = node;
                            make_heap( m_OpenList.begin(), m_OpenList.end(), HeapCmp() );
                        }
                    }
                    else if (m_ClosedSet.find(iter->Key()) != m_ClosedSet.end())
                    {
                        ASENode *nbNode = m_ClosedSet[iter->Key()];
                        if (nbNode->g > gVal)
                        {
                            nbNode->g = gVal;
                            nbNode->f = nbNode->g + nbNode->h;
                            nbNode->pParent = node;
                            pushOpenList(nbNode);
                            m_ClosedSet.erase(nbNode->Key());
                        }
                    }
                    else
                    {
                        ASENode *nbNode = allocateASENode(&(*iter));
                        if (nbNode == NULL) {
                            releaseData(false);
                            m_State = ASE_STATE_OUT_OF_MEMORY;
                            return m_State;
                        }
                        nbNode->h = m_Grid.CalH(*nbNode, *m_GoalNode);
                        nbNode->g = gVal;
                        nbNode->f = nbNode->g + nbNode->h;
                        nbNode->pParent = node;
                        pushOpenList(nbNode);
                    }
                }
                return m_State;
            }
        }

        ASENode* allocateASENode(const NodeType *node)
        {
            ASENode *pNewNode = NULL;
#ifdef _USE_MEM_POOL_
            ASENode *address = m_MemPool.Alloc();
            if (address) {
                pNewNode = node ? new (address) ASENode(*node) : new (address) ASENode;
            }
#else
            pNewNode = node ? new (std::nothrow) ASENode(*node) : new (std::nothrow) ASENode;
#endif
            if (pNewNode) {
                m_AllocateNodeCount++;
            }
            return pNewNode;
        }
        void freeASENode(ASENode *node)
        {
            if (node)
            {
#ifdef _USE_MEM_POOL_
                node->~ASENode();
                m_MemPool.Free(node);
#else
                delete node;
#endif
                m_AllocateNodeCount--;
            }
        }

        void pushOpenList(ASENode *node)
        {
            if (node)
            {
                assert(m_OpenSet.find(node->Key()) == m_OpenSet.end());
                m_OpenList.push_back(node);
                push_heap(m_OpenList.begin(), m_OpenList.end(), HeapCmp() );
                m_OpenSet[node->Key()] = node;
            }
        }

        ASENode* popOpenList()
        {
            ASENode *node = NULL;
            if (!m_OpenList.empty()) {
                node = m_OpenList.front();
                pop_heap(m_OpenList.begin(), m_OpenList.end(), HeapCmp());
                m_OpenList.pop_back();
                m_OpenSet.erase(node->Key());
            }
            return node;
        }

        unsigned int releaseData(bool bToReleaseSolutionData)
        {
            unsigned int cnt = 0;
            unordered_map<AANode::IDKey, ASENode*>::iterator iter;

            iter = m_OpenSet.begin();
            while(iter != m_OpenSet.end())
            {
                if (!iter->second->pChild) {
                    freeASENode(iter->second);
                    cnt++;
                }
                iter++;
            }
            m_OpenSet.clear();
            m_OpenList.clear();

            iter = m_ClosedSet.begin();
            while(iter != m_ClosedSet.end())
            {
                if (!iter->second->pChild) {
                    freeASENode(iter->second);
                    cnt++;
                }
                iter++;
            }
            m_ClosedSet.clear();

            if (bToReleaseSolutionData)
            {
                ASENode *pNode = m_StartNode;
                while (pNode)
                {
                    ASENode *pNodeToFree = pNode;
                    pNode = pNode->pChild;
                    freeASENode(pNodeToFree);
                    cnt++;
                }
                m_StartNode = NULL;
                m_GoalNode = NULL;
                m_SolutionNode = NULL;
                m_State = ASE_STATE_INVALID;
                m_Step = 0;
            }
            return cnt;
        }

    private:
        ASENode *m_StartNode;
        ASENode *m_GoalNode;

        int m_AllocateNodeCount;
        unordered_map<AANode::IDKey, ASENode*> m_OpenSet;
        unordered_map<AANode::IDKey, ASENode*> m_ClosedSet;
        vector<ASENode*> m_OpenList;

        ASE_STATE m_State;
        bool m_CancelRequest;
        unsigned int m_Step;
        ASENode *m_SolutionNode;

#ifdef _USE_MEM_POOL_
        MemPool<ASENode> m_MemPool;
#endif
        GridType m_Grid;
    };

}

#endif // !ASTARTENGINE_H