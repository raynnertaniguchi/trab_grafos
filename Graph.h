/*
* Trabalho Final - Algoritmo em Grafos (GCC218)
* Sistemas de Informação - UFLA
* Implementação Grupo R: Turma 14A
* Patrícia Souza Couto, Matrícula: 202210524
* Raynner Gabriel Taniguchi Silva Matrícula:202010167
*/
#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <limits>
#include <algorithm>

using namespace std;

class Graph {
public:
    int V; // Número de vértices
    vector<vector<int>> adj; // Lista de adjacência
    vector<vector<int>> weight; // Matriz de pesos
    vector<vector<int>> edgeId; // Id das arestas
    bool directed;

    Graph(int V, bool directed);
    void addEdge(int id, int u, int v, int w = 1);

    bool isConnected();
    bool isBipartite();
    bool isEulerian();
    bool hasCycle();
    int connectedComponents();
    int stronglyConnectedComponents();
    vector<int> articulationPoints();
    int bridges();
    void printDFSTree();
    void printBFSTree();
    int minimumSpanningTree();
    vector<int> topologicalSort();
    int shortestPath(int src, int dest);
    int maxFlow();
    vector<vector<bool>> transitiveClosure();

private:
    void dfs(int v, vector<bool> &visited);
    void dfs(int v, vector<bool>& visited, vector<int> &Ids);
    void dfs(int v, vector<vector<bool>>& reachability, vector<bool>& visited, int start);
    void bfs(int start, vector<int> &Ids);
    bool bfs(const vector<vector<int>>& capacity, vector<vector<int>>& flow, int source, int sink, vector<int>& parent);
    Graph getUndirected(); // Função para obter o grafo não direcionado de um grafo direcionado
    bool hasCycleUtilUndirected(int v, vector<bool>& visited, int parent);
    bool hasCycleUtilDirected(int v, vector<int>& state);
    void stronglyConnectedComponentsUtil(int u, vector<int>& disc, vector<int>& low, stack<int>& st, vector<bool>& stackMember, int& sccCount);
    void articulationPointsUtil(int u, vector<bool>& visited, vector<int>& disc, vector<int>& low, int& time, int parent, vector<bool>& isAP);
    void bridgeUtil(int u, vector<bool>& visited, vector<int>& disc, vector<int>& low, int& time, int parent, int& bridgeCount);
    int find(int u, vector<int> &parent);
    void unionSets(int u, int v, vector<int> &parent, vector<int> &rank);
    void topologicalSortUtil(int v, vector<bool> &visited, stack<int> &Stack);
};

#endif
