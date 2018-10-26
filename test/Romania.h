#ifndef PROBLEM_ROMANIA_H
#define PROBLEM_ROMANIA_H

#include "AStarEngine.h"

#include <vector>
#include <string>


using namespace AStarEng;


const int MAX_CITIES = 20;

enum ENUM_CITIES { 
    Arad = 0, 
    Bucharest, 
    Craiova, 
    Drobeta, 
    Eforie, 
    Fagaras, 
    Giurgiu, 
    Hirsova, 
    Iasi, 
    Lugoj, 
    Mehadia, 
    Neamt, 
    Oradea, 
    Pitesti, 
    RimnicuVilcea, 
    Sibiu, 
    Timisoara, 
    Urziceni, 
    Vaslui, 
    Zerind 
};

static std::vector<std::string> CityNames(MAX_CITIES);
static double RomaniaMap[MAX_CITIES][MAX_CITIES];

inline void initCities()
{
    for (int i = 0; i<MAX_CITIES; i++)
        for (int j = 0; j<MAX_CITIES; j++)
            RomaniaMap[i][j] = -1.0;

    RomaniaMap[Arad][Sibiu] = 140;
    RomaniaMap[Arad][Zerind] = 75;
    RomaniaMap[Arad][Timisoara] = 118;
    RomaniaMap[Bucharest][Giurgiu] = 90;
    RomaniaMap[Bucharest][Urziceni] = 85;
    RomaniaMap[Bucharest][Fagaras] = 211;
    RomaniaMap[Bucharest][Pitesti] = 101;
    RomaniaMap[Craiova][Drobeta] = 120;
    RomaniaMap[Craiova][RimnicuVilcea] = 146;
    RomaniaMap[Craiova][Pitesti] = 138;
    RomaniaMap[Drobeta][Craiova] = 120;
    RomaniaMap[Drobeta][Mehadia] = 75;
    RomaniaMap[Eforie][Hirsova] = 75;
    RomaniaMap[Fagaras][Bucharest] = 211;
    RomaniaMap[Fagaras][Sibiu] = 99;
    RomaniaMap[Giurgiu][Bucharest] = 90;
    RomaniaMap[Hirsova][Eforie] = 86;
    RomaniaMap[Hirsova][Urziceni] = 98;
    RomaniaMap[Iasi][Vaslui] = 92;
    RomaniaMap[Iasi][Neamt] = 87;
    RomaniaMap[Lugoj][Timisoara] = 111;
    RomaniaMap[Lugoj][Mehadia] = 70;
    RomaniaMap[Mehadia][Lugoj] = 70;
    RomaniaMap[Mehadia][Drobeta] = 75;
    RomaniaMap[Neamt][Iasi] = 87;
    RomaniaMap[Oradea][Zerind] = 71;
    RomaniaMap[Oradea][Sibiu] = 151;
    RomaniaMap[Pitesti][Bucharest] = 101;
    RomaniaMap[Pitesti][RimnicuVilcea] = 97;
    RomaniaMap[Pitesti][Craiova] = 138;
    RomaniaMap[RimnicuVilcea][Pitesti] = 97;
    RomaniaMap[RimnicuVilcea][Craiova] = 146;
    RomaniaMap[RimnicuVilcea][Sibiu] = 80;
    RomaniaMap[Sibiu][RimnicuVilcea] = 80;
    RomaniaMap[Sibiu][Fagaras] = 99;
    RomaniaMap[Sibiu][Oradea] = 151;
    RomaniaMap[Sibiu][Arad] = 140;
    RomaniaMap[Timisoara][Arad] = 118;
    RomaniaMap[Timisoara][Lugoj] = 111;
    RomaniaMap[Urziceni][Bucharest] = 85;
    RomaniaMap[Urziceni][Hirsova] = 98;
    RomaniaMap[Urziceni][Vaslui] = 142;
    RomaniaMap[Vaslui][Urziceni] = 142;
    RomaniaMap[Vaslui][Iasi] = 92;
    RomaniaMap[Zerind][Arad] = 75;
    RomaniaMap[Zerind][Oradea] = 71;

    // City names
    CityNames[Arad].assign("Arad");
    CityNames[Bucharest].assign("Bucharest");
    CityNames[Craiova].assign("Craiova");
    CityNames[Drobeta].assign("Drobeta");
    CityNames[Eforie].assign("Eforie");
    CityNames[Fagaras].assign("Fagaras");
    CityNames[Giurgiu].assign("Giurgiu");
    CityNames[Hirsova].assign("Hirsova");
    CityNames[Iasi].assign("Iasi");
    CityNames[Lugoj].assign("Lugoj");
    CityNames[Mehadia].assign("Mehadia");
    CityNames[Neamt].assign("Neamt");
    CityNames[Oradea].assign("Oradea");
    CityNames[Pitesti].assign("Pitesti");
    CityNames[RimnicuVilcea].assign("RimnicuVilcea");
    CityNames[Sibiu].assign("Sibiu");
    CityNames[Timisoara].assign("Timisoara");
    CityNames[Urziceni].assign("Urziceni");
    CityNames[Vaslui].assign("Vaslui");
    CityNames[Zerind].assign("Zerind");
}

class CityNode : public AANode
{
public:
    CityNode() : m_city(Arad)
    {
        genKey();
    }
    CityNode(const ENUM_CITIES city) : m_city(city)
    {
        genKey();
    }
    ~CityNode() {}

    ENUM_CITIES City() const { return m_city; }
    void SetCity(const ENUM_CITIES city) {
        m_city = city;
        genKey();
    }
    void Print() {
        printf(" %s\n", CityNames[m_city].c_str());
    }

private:
    IDKey genKey()
    {
        m_key = m_city;
        return m_key;
    }
private:
    ENUM_CITIES m_city;
};

class CityGrid : public AAGrid<CityNode>
{
public:
    // Calculate the heuristic distance from _srcNode_ to _goalNode_
    double CalH(const CityNode &srcNode, const CityNode &goalNode)
    {
        ENUM_CITIES city = srcNode.City();
        // goal is always Bucharest!
        switch (city)
        {
        case Arad: return 366; break;
        case Bucharest: return 0; break;
        case Craiova: return 160; break;
        case Drobeta: return 242; break;
        case Eforie: return 161; break;
        case Fagaras: return 176; break;
        case Giurgiu: return 77; break;
        case Hirsova: return 151; break;
        case Iasi: return 226; break;
        case Lugoj: return 244; break;
        case Mehadia: return 241; break;
        case Neamt: return 234; break;
        case Oradea: return 380; break;
        case Pitesti: return 100; break;
        case RimnicuVilcea: return 193; break;
        case Sibiu: return 253; break;
        case Timisoara: return 329; break;
        case Urziceni: return 80; break;
        case Vaslui: return 199; break;
        case Zerind: return 374; break;
        }
        assert(false && "Invalid city");
        return 0;
    }
    // Calculate the distance(cost) of travelling from _srcNode_ to _dstNode_
    double CalG(const CityNode &srcNode, const CityNode &dstNode)
    {
        return RomaniaMap[srcNode.City()][dstNode.City()];
    }
    // Judge that if _node_ is the _goalNode_
    bool IsGoalNode(const CityNode &node, const CityNode &goalNode)
    {
        return node.City() == Bucharest;
    }
    // Get the successor nodes of _node_
    unsigned int GetSuccessors(const CityNode &node, const CityNode * const pParent, vector<CityNode> &successorNodes)
    {
        vector<CityNode> successors;
        
        ENUM_CITIES city = node.City();
        for (int c = 0; c<MAX_CITIES; c++)
        {
            if (RomaniaMap[city][c] < 0) continue;
            successors.push_back(CityNode((ENUM_CITIES)c));
        }

        std::swap(successors, successorNodes);
        return successorNodes.size();
    }
};

inline void doRomaniaTest()
{
    initCities();


    printf("  Test Romania problem \n");
    printf("==============================\n");
    CityNode nodeStart(Arad);
    CityNode nodeGoal(Bucharest);
    printf("From %s to %s \n\n", 
        CityNames[nodeStart.City()].c_str(),
        CityNames[nodeGoal.City()].c_str());

    AAEngine<CityNode, CityGrid> astarsearch(32);
    astarsearch.SetStartAndGoalNodes(nodeStart, nodeGoal);
    astarsearch.Search();

    if (astarsearch.GetState() == ASE_STATE_SUCCEEDED)
    {
        printf("[:)] Search found goal state.\n\n");

        int steps = 0;

        printf("Displaying solution >> \n\n");
        steps = 0;
        CityNode *node = astarsearch.GetSolutionStart();
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

#endif // !PROBLEM_ROMANIA_H
