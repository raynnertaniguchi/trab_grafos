/*
* Trabalho Final - Algoritmo em Grafos (GCC218)
* Sistemas de Informação - UFLA
* Implementação Grupo R: Turma 14A
* Patrícia Souza Couto, Matrícula: 202210524
* Raynner Gabriel Taniguchi Silva Matrícula:202010167
*/
#include <sstream>
#include "Graph.h"

// Função principal
int main() {
    string line;
    
    // Ler a linha das funções
    getline(cin, line);
    stringstream ss(line);
    vector<int> functions;
    int func;
    while (ss >> func) {
        functions.push_back(func);
    }

    // Ler o número de vértices e arestas
    int V, E;
    cin >> V >> E;

    // Ler se o grafo é direcionado ou não
    string type;
    cin >> type;
    bool directed = (type == "direcionado");

    // Criar o grafo
    Graph g(V, directed);

    // Ler as arestas
    for (int i = 0; i < E; ++i) {
        int id, u, v, w;
        cin >> id >> u >> v >> w;
        g.addEdge(id, u, v, w);
    }

    // Executar as funções na ordem especificada
    for (int choice : functions) {
        switch (choice) {
            case 0:
                cout << (g.isConnected() ? 1 : 0) << endl;
                break;
            case 1:
                cout << (g.isBipartite() ? 1 : 0) << endl;
                break;
            case 2:
                cout << (g.isEulerian() ? 1 : 0) << endl;
                break;
            case 3:
                cout << (g.hasCycle() ? 1 : 0) << endl;
                break;
            case 4:
                cout << g.connectedComponents() << endl;
                break;
            case 5:
                cout << g.stronglyConnectedComponents() << endl;
                break;
            case 6: {
                if (g.directed){
                    cout << -1 << endl;
                }
                else{
                    vector<int> points = g.articulationPoints();
                    if (points.empty()){
                        cout << -1 << endl;
                    }
                    else{
                        for (int v : points) cout << v << " ";
                        cout << endl;
                    }
                }
                break;
            }
            case 7:
                if (g.directed){
                    cout << -1 << endl;
                }
                else {
                    cout << g.bridges() << endl;
                }
                break;
            case 8:
                g.printDFSTree();
                break;
            case 9:
                g.printBFSTree();
                break;
            case 10:
                if (g.directed){
                    cout << -1 << endl;
                }
                else{
                    cout << g.minimumSpanningTree() << endl;
                }
                break;
            case 11: {
                if (!g.directed){
                    cout << -1 << endl;
                }
                else{
                    vector<int> topo = g.topologicalSort();
                    if (topo.empty()){
                        cout << -1 << endl;
                    }
                    else{
                        for (int v : topo) cout << v << " ";
                        cout << endl;
                    }
                }
                break;
            }
            case 12: {
                if (g.directed){
                    cout << -1 << endl;
                }
                else{
                    int src = 0, dest = V - 1;
                    cout << g.shortestPath(src, dest) << endl;
                }
                break;
            }
            case 13:
                if (!g.directed){
                    cout << -1 << endl;
                }
                else{
                    cout << g.maxFlow() << endl;
                }
                break;
            case 14: {
                if(!g.directed){
                    cout << -1 << endl;
                }
                else{
                    vector<vector<bool>> tc = g.transitiveClosure();
                    bool find = false;
                    for(int i = 1; i < V; ++i){
                        if(tc[0][i])
                            cout << i << " ";
                    }
                    cout << endl;
                }
                break;
            }
        }
    }

    return 0;
}
