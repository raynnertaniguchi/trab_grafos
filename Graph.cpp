/*
* Trabalho Final - Algoritmo em Grafos (GCC218)
* Sistemas de Informação - UFLA
* Implementação Grupo R: Turma 14A
* Patrícia Souza Couto, Matrícula: 202210524
* Raynner Gabriel Taniguchi Silva Matrícula:202010167
*/
#include "Graph.h"

Graph::Graph(int V, bool directed) : V(V), directed(directed) {
    adj.resize(V);
    weight.resize(V, vector<int>(V, -1));
    edgeId.resize(V, vector<int>(V, -1));
}

void Graph::addEdge(int id, int u, int v, int w) {
    adj[u].push_back(v);
    if (!directed) adj[v].push_back(u);
    weight[u][v] = w;
    if (!directed) weight[v][u] = w;
    edgeId[u][v] = id;
    if (!directed) edgeId[v][u] = id;
}

bool Graph::hasCycleUtilUndirected(int v, vector<bool>& visited, int parent) {
    visited[v] = true;

    for (int u : adj[v]) {
        // Se o vértice não foi visitado, continue a DFS
        if (!visited[u]) {
            if (hasCycleUtilUndirected(u, visited, v)) {
                return true;
            }
        }
        // Se o vértice foi visitado e não é o pai, encontramos um ciclo
        else if (u != parent) {
            return true;
        }
    }

    return false;
}

bool Graph::hasCycleUtilDirected(int v, vector<int>& state) {
    state[v] = 1; // Marcando o vértice como em processo

    for (int u : adj[v]) {
        if (state[u] == 1) {
            return true; // Encontrou um ciclo dentro do caminho atual
        }
        if (state[u] == 0 && hasCycleUtilDirected(u, state)) {
            return true;
        }
    }

    state[v] = 2; // Marcando o vértice como completamente processado
    return false;
}

void Graph::dfs(int v, vector<bool> &visited) {
    visited[v] = true;
    for (int u : adj[v]) {
        if (!visited[u]) {
            dfs(u, visited);
        }
    }
}

void Graph::dfs(int v, vector<bool>& visited, vector<int> &iDs) {
    visited[v] = true;

    // Ordena os vizinhos lexicograficamente antes de explorar
    vector<int> neighbors = adj[v];
    sort(neighbors.begin(), neighbors.end());

    int id = -1;
    for (int u : neighbors) {
        if (!visited[u]) {
            id = edgeId[v][u];
            if (id != -1){
                iDs.push_back(id);
            }
            dfs(u, visited, iDs);
        }
    }
}

void Graph::dfs(int v, vector<vector<bool>>& reachability, vector<bool>& visited, int start) {
    visited[v] = true;
    reachability[start][v] = true;
    for (int neighbor : adj[v]) {
        if (!visited[neighbor]) {
            dfs(neighbor, reachability, visited, start);
        }
    }
}

void Graph::bfs(int start, vector<int> &Ids) {
    vector<bool> visited(V, false);
    queue<int> q;
    visited[start] = true;
    q.push(start);

    while (!q.empty()) {
        int v = q.front();
        q.pop();

        // Ordena os vizinhos lexicograficamente antes de explorar
        vector<int> neighbors = adj[v];
        sort(neighbors.begin(), neighbors.end());

        for (int u : neighbors) {
            if (!visited[u]) {
                visited[u] = true;
                int id = edgeId[v][u];
                if (id != -1) {
                    Ids.push_back(id);
                }
                q.push(u);
            }
        }
    }
}

// BFS para encontrar o caminho aumentante
bool Graph::bfs(const vector<vector<int>>& capacity, vector<vector<int>>& flow, int source, int sink, vector<int>& parent) {
    int n = capacity.size();
    vector<bool> visited(n, false);
    queue<int> q;
    
    q.push(source);
    visited[source] = true;
    parent[source] = -1;
    
    while (!q.empty()) {
        int u = q.front();
        q.pop();
        
        for (int v = 0; v < n; ++v) {
            if (!visited[v] && capacity[u][v] - flow[u][v] > 0) {  // Verifica a capacidade residual
                q.push(v);
                parent[v] = u;
                visited[v] = true;
                
                if (v == sink) {
                    return true;
                }
            }
        }
    }
    
    return false;
}

Graph Graph::getUndirected() {
    Graph g(V, false); // Cria um grafo não direcionado com o mesmo número de vértices
    for (int u = 0; u < V; ++u) {
        for (int v : adj[u]) {
            // Adiciona arestas bidirecionais
            g.addEdge(edgeId[u][v], u, v, weight[u][v]);
        }
    }
    return g;
}

void Graph::stronglyConnectedComponentsUtil(int u, vector<int>& disc, vector<int>& low, stack<int>& st, vector<bool>& stackMember, int& sccCount) {
    static int time = 0;

    // Inicializa o tempo de descoberta e o valor low
    disc[u] = low[u] = ++time;
    st.push(u);
    stackMember[u] = true;

    // Percorre todos os vértices adjacentes de 'u'
    for (int v : adj[u]) {
        // Se v não foi visitado, faça a recursão para v
        if (disc[v] == -1) {
            stronglyConnectedComponentsUtil(v, disc, low, st, stackMember, sccCount);
            low[u] = min(low[u], low[v]);
        }
        // Atualiza o valor low de 'u' para parent function calls
        else if (stackMember[v]) {
            low[u] = min(low[u], disc[v]);
        }
    }

    // Verifica se 'u' é a raiz de uma SCC
    int w = 0;
    if (low[u] == disc[u]) {
        while (st.top() != u) {
            w = st.top();
            stackMember[w] = false;
            st.pop();
        }
        w = st.top();
        stackMember[w] = false;
        st.pop();

        // Incrementa a contagem de SCCs
        sccCount++;
    }
}

void Graph::articulationPointsUtil(int u, vector<bool>& visited, vector<int>& disc, vector<int>& low, int& time, int parent, vector<bool>& isAP) {
    int children = 0;
    visited[u] = true;
    disc[u] = low[u] = ++time;

    for (int v : adj[u]) {
        if (!visited[v]) {
            children++;
            articulationPointsUtil(v, visited, disc, low, time, u, isAP);
            low[u] = min(low[u], low[v]);

            if (parent != -1 && low[v] >= disc[u])
                isAP[u] = true;
        } else if (v != parent) {
            low[u] = min(low[u], disc[v]);
        }
    }

    if (parent == -1 && children > 1)
        isAP[u] = true;
}

void Graph::bridgeUtil(int u, vector<bool>& visited, vector<int>& disc, vector<int>& low, int& time, int parent, int& bridgeCount) {
    visited[u] = true;
    disc[u] = low[u] = ++time;

    for (int v : adj[u]) {
        if (!visited[v]) {
            bridgeUtil(v, visited, disc, low, time, u, bridgeCount);
            low[u] = min(low[u], low[v]);

            // Se a condição abaixo for verdadeira, a aresta (u, v) é uma ponte
            if (low[v] > disc[u]) {
                bridgeCount++;
            }
        } else if (v != parent) {
            low[u] = min(low[u], disc[v]);
        }
    }
}

int Graph::find(int u, vector<int> &parent) {
    if (u != parent[u])
        parent[u] = find(parent[u], parent);
    return parent[u];
}

// Função para unir dois subconjuntos em um único subconjunto
void Graph::unionSets(int u, int v, vector<int> &parent, vector<int> &rank) {
    int rootU = find(u, parent);
    int rootV = find(v, parent);

    if (rootU != rootV) {
        if (rank[rootU] > rank[rootV]) {
            parent[rootV] = rootU;
        } else if (rank[rootU] < rank[rootV]) {
            parent[rootU] = rootV;
        } else {
            parent[rootV] = rootU;
            rank[rootU]++;
        }
    }
}

// Função auxiliar para realizar a ordenação topológica recursivamente
void Graph::topologicalSortUtil(int v, vector<bool> &visited, stack<int> &Stack) {
    visited[v] = true;

    // Considera todos os vértices adjacentes a este vértice
    for (int i : adj[v]) {
        if (!visited[i]) {
            topologicalSortUtil(i, visited, Stack);
        }
    }

    // Empilha o vértice depois de processar todos os seus vizinhos
    Stack.push(v);
}

// Função para encontrar o conjunto do vértice u
bool Graph::isConnected() {
    // Se o grafo for direcionado, converte para não direcionado
    Graph gToCheck = directed ? getUndirected() : *this;

    vector<bool> visited(V, false);

    // Realiza DFS no grafo original a partir do vértice 0
    gToCheck.dfs(0, visited);

    // Verifica se todos os vértices foram visitados na primeira passagem
    for (bool v : visited) {
        if (!v) {
            return false; // Se algum vértice não foi visitado, o grafo não é conexo
        }
    }

    return true; // O grafo é conexo
}

bool Graph::isBipartite() {
    vector<int> color(V, -1); // -1 significa não colorido, 0 e 1 são as duas cores
    queue<int> q;

    for (int i = 0; i < V; ++i) {
        if (color[i] == -1) { // Se o vértice não foi colorido
            q.push(i);
            color[i] = 0; // Começa com a cor 0

            while (!q.empty()) {
                int u = q.front();
                q.pop();

                for (int v : adj[u]) {
                    if (color[v] == -1) { // Se o vértice v não foi colorido
                        color[v] = 1 - color[u]; // A cor do vértice v é diferente da de u
                        q.push(v);
                    } else if (color[v] == color[u]) { // Se v e u têm a mesma cor
                        return false; // O grafo não é bipartido
                    }
                }
            }
        }
    }

    return true; // O grafo é bipartido
}

bool Graph::isEulerian() {
    // Verifica se o grafo é conexo
    if (!isConnected()) {
        return 0;
    }

    if(directed){
        // Verifica se o grau de entrada é igual ao grau de saída para cada vértice
        vector<int> in_degree(V, 0), out_degree(V, 0);

        for (int u = 0; u < V; ++u) {
            for (int v : adj[u]) {
                out_degree[u]++;
                in_degree[v]++;
            }
        }

        // Comparar os graus de entrada e saída
        for (int i = 0; i < V; ++i) {
            if (in_degree[i] != out_degree[i]) {
                return false; // Se o grau de entrada for diferente do grau de saída, não é euleriano
            }
        }

        return true; // Se todas as condições forem satisfeitas, o grafo direcionado é euleriano
    }

    // Verifica se todos os vértices têm grau par
    for (int i = 0; i < V; ++i) {
        if (adj[i].size() % 2 != 0) {
            return 0; // Se algum vértice tiver grau ímpar, não é possível ter um ciclo euleriano
        }
    }

    return 1; // Todos os vértices têm grau par, então é possível ter um ciclo euleriano
}

bool Graph::hasCycle() {
    if (directed) {
        vector<int> state(V, 0); // 0 = não visitado, 1 = em processo, 2 = totalmente processado

        for (int i = 0; i < V; ++i) {
            if (state[i] == 0) {
                if (hasCycleUtilDirected(i, state)) {
                    return true;
                }
            }
        }
    } else {
        vector<bool> visited(V, false);

        for (int i = 0; i < V; ++i) {
            if (!visited[i]) {
                if (hasCycleUtilUndirected(i, visited, -1)) {
                    return true;
                }
            }
        }
    }

    return false; // Se nenhum ciclo for encontrado
}

int Graph::connectedComponents() {
    if (directed) {
        return -1; // Se o grafo é orientado, retorna -1
    }

    vector<bool> visited(V, false);
    int componentCount = 0;

    for (int i = 0; i < V; ++i) {
        if (!visited[i]) {
            dfs(i, visited); // Inicia uma nova DFS para cada componente
            componentCount++;
        }
    }

    return componentCount; // Retorna a quantidade de componentes conexas
}

int Graph::stronglyConnectedComponents() {
    if (!directed) {
        return -1; // Se o grafo não é orientado, retorna -1
    }
    vector<int> disc(V, -1);
    vector<int> low(V, -1);
    vector<bool> stackMember(V, false);
    stack<int> st;
    int sccCount = 0;

    // Faz a chamada recursiva para encontrar todas as SCCs
    for (int i = 0; i < V; i++) {
        if (disc[i] == -1) {
            stronglyConnectedComponentsUtil(i, disc, low, st, stackMember, sccCount);
        }
    }

    return sccCount;
}

vector<int> Graph::articulationPoints() {
    vector<int> disc(V, -1); // Tempo de descoberta
    vector<int> low(V, -1); // Tempo de menor alcance
    vector<bool> visited(V, false);
    vector<bool> isAP(V, false);
    int time = 0;
    int par = -1;

    for (int u = 0; u < V; u++) {
        if (!visited[u]) {
            articulationPointsUtil(u, visited, disc, low, time, par, isAP);
        }
    }

    vector<int> articulationPointsList;
    for (int u = 0; u < V; u++) {
        if (isAP[u]) {
            articulationPointsList.push_back(u);
        }
    }

    return articulationPointsList;
}

int Graph::bridges() {
    vector<int> disc(V, -1);
    vector<int> low(V, -1);
    vector<bool> visited(V, false);
    int time = 0;
    int bridgeCount = 0;

    for (int u = 0; u < V; u++) {
        if (!visited[u]) {
            bridgeUtil(u, visited, disc, low, time, -1, bridgeCount);
        }
    }

    return bridgeCount;
}

void Graph::printDFSTree() {
    vector<bool> visited(V, false);
    vector<int> edgeIds;

    // Começa a DFS a partir do vértice 0
    dfs(0, visited, edgeIds);

    // Imprime o identificador das arestas da árvore em profundidade
    for (int id : edgeIds) {
        cout << id << " ";
    }
    cout << endl;
}

void Graph::printBFSTree() {
    vector<int> edgeIds;

    // Começa a BFS a partir do vértice 0
    bfs(0, edgeIds);

    // Imprime os IDs das arestas da árvore em largura
    for (int id : edgeIds) {
        cout << id << " ";
    }
    cout << endl;
}

// Função para calcular o valor final da MST usando o algoritmo de Kruskal
int Graph::minimumSpanningTree() {
    vector<pair<int, pair<int, int>>> edges; // Vetor de arestas: (peso, (u, v))

    // Preenche o vetor de arestas
    for (int u = 0; u < V; ++u) {
        for (int v : adj[u]) {
            if (u < v) { // Para evitar duplicidade, considere u < v para grafos não direcionados
                edges.push_back({weight[u][v], {u, v}});
            }
        }
    }

    // Ordena as arestas pelo peso
    sort(edges.begin(), edges.end());

    // Estruturas de dados para o algoritmo de Kruskal
    vector<int> parent(V);
    vector<int> rank(V, 0);

    // Inicializa cada vértice como seu próprio pai (representante)
    for (int i = 0; i < V; ++i) {
        parent[i] = i;
    }

    int mstValue = 0;

    // Processa as arestas em ordem crescente de peso
    for (auto &edge : edges) {
        int w = edge.first;
        int u = edge.second.first;
        int v = edge.second.second;

        int rootU = find(u, parent);
        int rootV = find(v, parent);

        // Se u e v estão em componentes diferentes, adicione a aresta à MST
        if (rootU != rootV) {
            mstValue += w;
            unionSets(rootU, rootV, parent, rank);
        }
    }

    return mstValue;
}

// Função para imprimir a ordem topológica dos vértices
vector<int> Graph::topologicalSort() {

    stack<int> Stack;
    vector<bool> visited(V, false);

    // Chama a função auxiliar recursiva para cada vértice não visitado
    for (int i = 0; i < V; i++) {
        if (!visited[i]) {
            topologicalSortUtil(i, visited, Stack);
        }
    }

    // Armazena a ordem topológica
    vector<int> topologicalOrder;

    // Desempilha os vértices da pilha e adiciona na ordem final
    while (!Stack.empty()) {
        topologicalOrder.push_back(Stack.top());
        Stack.pop();
    }

    return topologicalOrder;
}

int Graph::shortestPath(int src, int dest) {
    // Vetor de distâncias, inicializado com infinito
    vector<int> dist(V, numeric_limits<int>::max());
    dist[src] = 0; // Distância da origem a si mesma é zero

    // Fila de prioridade (min-heap) para armazenar vértices a serem processados
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<>> pq;
    pq.emplace(0, src);

    while (!pq.empty()) {
        int u = pq.top().second;
        int current_dist = pq.top().first;
        pq.pop();

        // Se já alcançou o destino, retorne a distância mínima
        if (u == dest) {
            return current_dist;
        }

        // Processa todos os vértices adjacentes a u
        for (int v : adj[u]) {
            int w = weight[u][v];
            if (w != -1) { // Verifica se existe uma aresta entre u e v
                int new_dist = current_dist + w;
                if (new_dist < dist[v]) {
                    dist[v] = new_dist;
                    pq.emplace(new_dist, v);
                }
            }
        }
    }

    // Se o destino não é alcançável, retorna -1 (indicando que não há caminho)
    return -1;
}

int Graph::maxFlow() {
    int n = weight.size();
    int source = 0;
    int sink = V - 1;
    
    vector<vector<int>> flow(n, vector<int>(n, 0));
    vector<int> parent(n);
    int max_flow = 0;
    
    while (bfs(weight, flow, source, sink, parent)) {
        // Encontra o fluxo máximo através do caminho encontrado pelo BFS
        int path_flow = INT_MAX;
        for (int v = sink; v != source; v = parent[v]) {
            int u = parent[v];
            path_flow = min(path_flow, weight[u][v] - flow[u][v]);
        }
        
        // Atualiza as capacidades residuais das arestas e arestas reversas ao longo do caminho
        for (int v = sink; v != source; v = parent[v]) {
            int u = parent[v];
            flow[u][v] += path_flow;
            flow[v][u] -= path_flow;
        }
        
        max_flow += path_flow;
    }
    
    return max_flow;
}

vector<vector<bool>> Graph::transitiveClosure() {
    vector<vector<bool>> reachability(V, vector<bool>(V, false));

    for (int i = 0; i < V; ++i) {
        vector<bool> visited(V, false);
        dfs(i, reachability, visited, i);
    }
    return reachability;
}
