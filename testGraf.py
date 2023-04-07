from Graf import Graf


graph = {
  '1':['8'],
  '5' : ['3','7'],
  '3' : ['2', '4'],
  '7' : ['8'],
  '2' : ['3'],
  '4' : ['8'],
  '8' : ['2']
}

# Driver Code
print("Поиск в глубину:")
visited = set()
Graf.dfs(visited, graph, '1')

print("\nПоиск в ширину:")
Graf.bfs(visited,graph, '5') 

g = Graf(9)
g.graph = [[0, 4, 0, 0, 0, 0, 0, 8, 0],
           [4, 0, 8, 0, 0, 0, 0, 11, 0],
           [0, 8, 0, 7, 0, 4, 0, 0, 2],
           [0, 0, 7, 0, 9, 14, 0, 0, 0],
           [0, 0, 0, 9, 0, 10, 0, 0, 0],
           [0, 0, 4, 14, 10, 0, 2, 0, 0],
           [0, 0, 0, 0, 0, 2, 0, 1, 6],
           [8, 11, 0, 0, 0, 0, 1, 0, 7],
           [0, 0, 2, 0, 0, 0, 6, 7, 0]
           ]

print("\n\nАлгоритм Дейкстры:")
g.dijkstra(0)
print("\nАлгоритм Прима:")
g.primMST()
g = Graf(4)
INF = 999
graph = [[0, 5, INF, INF],
         [50, 0, 15, 5],
         [30, INF, 0, 15],
         [15, INF, 5, 0]]

print("\nАлгоритм Флойда-Уоршалла:")
Graf.floyd(graph,4)

g = Graf(6)
g.add_edge(0, 1, 4)
g.add_edge(0, 2, 4)
g.add_edge(1, 2, 2)
g.add_edge(1, 0, 4)
g.add_edge(2, 0, 4)
g.add_edge(2, 1, 2)
g.add_edge(2, 3, 3)
g.add_edge(2, 5, 2)
g.add_edge(2, 4, 4)
g.add_edge(3, 2, 3)
g.add_edge(3, 4, 3)
g.add_edge(4, 2, 4)
g.add_edge(4, 3, 3)
g.add_edge(5, 2, 2)
g.add_edge(5, 4, 3)
print("\nАлгоритм Крускала:")
g.kruskal_algo()