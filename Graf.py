import sys


class Graf:

    def dfs(visited, graph, node):  # Поиск в глубину
        if node not in visited:
            print(node)
            visited.add(node)
            for neighbour in graph[node]:
                Graf.dfs(visited, graph, neighbour)

    def bfs(visited, graph, node):  # Поиск в ширину
        visited = []
        queue = []
        visited.append(node)
        queue.append(node)

        while queue:          # Создание цикла для посещения каждого узла
            m = queue.pop(0)
            print(m, end=" ")

            for neighbour in graph[m]:
                if neighbour not in visited:
                    visited.append(neighbour)
                    queue.append(neighbour)

    def __init__(self, vertices):
        self.V = vertices
        self.graph = []

    # Алгоритм Дейкстры
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

    # Алгоритм Прима
    def Prims(G, N):
        INF = 9999999
        selected_node = [0]*(N)
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
                            # не выделено, и есть край
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

    # Вспомогательная функция для поиска вершины с
    # минимальное значение расстояния от набора вершин
    # еще не включено в дерево кратчайших путей
    def minKey(self, key, mstSet):

        # Инициализировать минимальное значение
        min = sys.maxsize

        for v in range(self.V):
            if key[v] < min and mstSet[v] == False:
                min = key[v]
                min_index = v

        return min_index

    # Функция для построения и печати MST для графика
    # представлено с использованием представления матрицы смежности
    def primMST(self):

        # Ключевые значения, используемые для выбора кромки минимального веса в разрезе
        key = [sys.maxsize] * self.V
        parent = [None] * self.V  # Массив для хранения сконструированного MST
        # Сделайте ключ 0, чтобы эта вершина была выбрана в качестве первой вершины
        key[0] = 0
        mstSet = [False] * self.V

        parent[0] = -1  # Первый узел всегда является корнем

        for cout in range(self.V):

            # Выберите вершину на минимальном расстоянии от
            # набор вершин, которые еще не обработаны.
            # u всегда равно src на первой итерации
            u = self.minKey(key, mstSet)

            # Поместите вершину минимального расстояния в
            # дерево кратчайшего пути
            mstSet[u] = True

            # Обновить значение dist соседних вершин
            # выбранной вершины, только если текущая
            # расстояние больше, чем новое расстояние, и
            # вершина не находится в дереве кратчайшего пути
            for v in range(self.V):

                # graph[u][v] отличен от нуля только для смежных вершин m
                # должно быть установлено mstSet[v] false для вершин, еще не включенных в MST
                # Обновляйте ключ только в том случае, если graph[u][v] меньше key[v]
                if self.graph[u][v] > 0 and mstSet[v] == False \
                        and key[v] > self.graph[u][v]:
                    key[v] = self.graph[u][v]
                    parent[v] = u

        self.printMST(parent)

    # Алгоритм Флойда-Уоршалла
    def floyd(G, nV):

        dist = list(map(lambda p: list(map(lambda q: q, p)), G))
        # Добавление вершин по отдельности
        for r in range(nV):
            for p in range(nV):
                for q in range(nV):
                    dist[p][q] = min(dist[p][q], dist[p][r] + dist[r][q])
        Graf.sol(dist, nV)

    # Вывод
    def sol(dist, nV):
        INF = 999
        for p in range(nV):
            for q in range(nV):
                if (dist[p][q] == INF):
                    print("INF", end=" ")
                else:
                    print(dist[p][q], end="  ")
            print(" ")

    # Крускал алгоритм
    def add_edge(self, u, v, w):
        self.graph.append([u, v, w])

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
