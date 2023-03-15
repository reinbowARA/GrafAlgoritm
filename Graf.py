import sys


class Graf:
    
    def dfs(visited, graph, node):  #function for dfs 
        if node not in visited:
            print (node)
            visited.add(node)
            for neighbour in graph[node]:
                Graf.dfs(visited, graph, neighbour)

    def bfs(visited, graph, node): #function for BFS
        visited = []
        queue = []
        visited.append(node)
        queue.append(node)

        while queue:          # Creating loop to visit each node
            m = queue.pop(0) 
            print (m, end = " ") 

            for neighbour in graph[m]:
                if neighbour not in visited:
                    visited.append(neighbour)
                    queue.append(neighbour)

    def __init__(self, vertices):
            self.V = vertices
            self.graph = []

    def printSolution(self, dist):
        print("Vertex \t Distance from Source")
        for node in range(self.V):
            print(node, "\t\t", dist[node])

    def minDistance(self, dist, sptSet):
 
        min = 1e7
 
        for v in range(self.V):
            if dist[v] < min and sptSet[v] == False:
                min = dist[v]
                min_index = v
 
        return min_index
 
    def dijkstra(self, src):
 
        dist = [1e7] * self.V
        dist[src] = 0
        sptSet = [False] * self.V
 
        for cout in range(self.V):
 
            u = self.minDistance(dist, sptSet)
 
            sptSet[u] = True
 
            for v in range(self.V):
                if (self.graph[u][v] > 0 and
                   sptSet[v] == False and
                   dist[v] > dist[u] + self.graph[u][v]):
                    dist[v] = dist[u] + self.graph[u][v]
 
        self.printSolution(dist)
    
    def Prims(G, N):
        INF = 9999999
        selected_node = [0]*(N)
        #selected_node[0]= True
        no_edge = 0
        print("Edge : Weight\n")
        while (no_edge < N - 1): 
            minimum = INF
            a = 0
            b = 0
            for m in range(N):
                if selected_node[m]:
                    for n in range(N):
                        if ((not selected_node[n]) and G[m][n]):  
                            # not in selected and there is an edge
                            if minimum > G[m][n]:
                                minimum = G[m][n]
                                a = m
                                b = n
            print(str(a) + "-" + str(b) + ":" + str(G[a][b]))
            selected_node[b] = True
            no_edge += 1

    def printMST(self, parent):
            print("Edge \tWeight")
            for i in range(1, self.V):
                print(parent[i], "-", i, "\t", self.graph[i][parent[i]])
    
    # A utility function to find the vertex with
    # minimum distance value, from the set of vertices
    # not yet included in shortest path tree
    def minKey(self, key, mstSet):
 
        # Initialize min value
        min = sys.maxsize
 
        for v in range(self.V):
            if key[v] < min and mstSet[v] == False:
                min = key[v]
                min_index = v
 
        return min_index
 
    # Function to construct and print MST for a graph
    # represented using adjacency matrix representation
    def primMST(self):
 
        # Key values used to pick minimum weight edge in cut
        key = [sys.maxsize] * self.V
        parent = [None] * self.V  # Array to store constructed MST
        # Make key 0 so that this vertex is picked as first vertex
        key[0] = 0
        mstSet = [False] * self.V
 
        parent[0] = -1  # First node is always the root of
 
        for cout in range(self.V):
 
            # Pick the minimum distance vertex from
            # the set of vertices not yet processed.
            # u is always equal to src in first iteration
            u = self.minKey(key, mstSet)
 
            # Put the minimum distance vertex in
            # the shortest path tree
            mstSet[u] = True
 
            # Update dist value of the adjacent vertices
            # of the picked vertex only if the current
            # distance is greater than new distance and
            # the vertex in not in the shortest path tree
            for v in range(self.V):
 
                # graph[u][v] is non zero only for adjacent vertices of m
                # mstSet[v] is false for vertices not yet included in MST
                # Update the key only if graph[u][v] is smaller than key[v]
                if self.graph[u][v] > 0 and mstSet[v] == False \
                and key[v] > self.graph[u][v]:
                    key[v] = self.graph[u][v]
                    parent[v] = u
 
        self.printMST(parent)

        # Algorithm 
    def floyd(G, nV):

        dist = list(map(lambda p: list(map(lambda q: q, p)), G))
        # Adding vertices individually
        for r in range(nV):
            for p in range(nV):
                for q in range(nV):
                    dist[p][q] = min(dist[p][q], dist[p][r] + dist[r][q])
        Graf.sol(dist,nV)

    # Printing the output
    def sol(dist,nV):
        INF = 999
        for p in range(nV):
            for q in range(nV):
                if(dist[p][q] == INF):
                    print("INF", end=" ")
                else:
                    print(dist[p][q], end="  ")
            print(" ")

    # Крускал алгоритм
    def add_edge(self, u, v, w):
            self.graph.append([u, v, w])

    # Search function

    def find(self, parent, i):
        if parent[i] == i:
            return i
        return self.find(parent, parent[i])

    def apply_union(self, parent, rank, x, y):
        xroot = self.find(parent, x)
        yroot = self.find(parent, y)
        if rank[xroot] < rank[yroot]:
            parent[xroot] = yroot
        elif rank[xroot] > rank[yroot]:
            parent[yroot] = xroot
        else:
            parent[yroot] = xroot
            rank[xroot] += 1
        
    #  Applying Kruskal algorithm
    def kruskal_algo(self):
        result = []
        i, e = 0, 0
        self.graph = sorted(self.graph, key=lambda item: item[2])
        parent = []
        rank = []
        for node in range(self.V):
            parent.append(node)
            rank.append(0)
        while e < self.V - 1:
            u, v, w = self.graph[i]
            i = i + 1
            x = self.find(parent, u)
            y = self.find(parent, v)
            if x != y:
                e = e + 1
                result.append([u, v, w])
                self.apply_union(parent, rank, x, y)
        for u, v, weight in result:
            print("%d - %d: %d" % (u, v, weight))