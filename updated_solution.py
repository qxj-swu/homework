class Edge:
    def __init__(self, to, rev, cap):
        self.to = to
        self.rev = rev
        self.cap = cap
        self.original_cap = cap  # 记录原始容量用于路径恢复

class Dinic:
    def __init__(self, n):
        self.size = n
        self.graph = [[] for _ in range(n)]
    
    def add_edge(self, fr, to, cap):
        forward = Edge(to, len(self.graph[to]), cap)
        backward = Edge(fr, len(self.graph[fr]), 0)
        self.graph[fr].append(forward)
        self.graph[to].append(backward)
    
    def bfs_level(self, s, t, level):
        q = []
        level[:] = [-1] * self.size
        level[s] = 0
        q.append(s)
        while q:
            v = q.pop(0)
            for edge in self.graph[v]:
                if edge.cap > 0 and level[edge.to] == -1:
                    level[edge.to] = level[v] + 1
                    q.append(edge.to)
                    if edge.to == t:
                        return
    
    def dfs_flow(self, v, t, flow, level, ptr):
        if v == t:
            return flow
        while ptr[v] < len(self.graph[v]):
            edge = self.graph[v][ptr[v]]
            if edge.cap > 0 and level[v] < level[edge.to]:
                min_flow = min(flow, edge.cap)
                result = self.dfs_flow(edge.to, t, min_flow, level, ptr)
                if result > 0:
                    edge.cap -= result
                    self.graph[edge.to][edge.rev].cap += result
                    return result
            ptr[v] += 1
        return 0
    
    def max_flow(self, s, t):
        flow = 0
        level = [-1] * self.size
        while True:
            self.bfs_level(s, t, level)
            if level[t] == -1:
                return flow
            ptr = [0] * self.size
            while True:
                f = self.dfs_flow(s, t, float('inf'), level, ptr)
                if f == 0:
                    break
                flow += f
            level = [-1] * self.size

def solve_escape(n, starts):
    total_nodes = 2 + 2 * n * n
    dinic = Dinic(total_nodes)
    S, T = 0, 1
    
    # 节点ID与网格坐标的映射
    node_to_grid = {}
    start_to_node = {}
    
    def get_in_id(i, j):
        node_id = 2 + 2 * ((i-1) * n + (j-1))
        node_to_grid[node_id] = (i, j, 'in')
        node_to_grid[node_id + 1] = (i, j, 'out')
        return node_id
    
    # 顶点拆分
    for i in range(1, n+1):
        for j in range(1, n+1):
            in_node = get_in_id(i, j)
            dinic.add_edge(in_node, in_node + 1, 1)
    
    # 邻接边
    dirs = [(-1,0), (1,0), (0,-1), (0,1)]
    for i in range(1, n+1):
        for j in range(1, n+1):
            u_out = get_in_id(i, j) + 1
            for di, dj in dirs:
                ni, nj = i + di, j + dj
                if 1 <= ni <= n and 1 <= nj <= n:
                    v_in = get_in_id(ni, nj)
                    dinic.add_edge(u_out, v_in, 1)
    
    # 源点连接起点
    for x, y in starts:
        in_node = get_in_id(x, y)
        dinic.add_edge(S, in_node, 1)
        start_to_node[(x, y)] = in_node
    
    # 边界连接汇点
    boundary_nodes = {}
    for i in range(1, n+1):
        for j in range(1, n+1):
            if i == 1 or i == n or j == 1 or j == n:
                out_node = get_in_id(i, j) + 1
                dinic.add_edge(out_node, T, 1)
                boundary_nodes[out_node] = (i, j)
    
    # 计算最大流
    max_flow = dinic.max_flow(S, T)
    can_escape = max_flow == len(starts)
    
    if not can_escape:
        return False, []
    
    # 提取路径
    paths = []
    for start_pos in starts:
        start_node = start_to_node[start_pos]
        # 检查从源点到起点是否有流
        has_flow = False
        for edge in dinic.graph[S]:
            if edge.to == start_node and edge.cap < edge.original_cap:
                has_flow = True
                break
        
        if has_flow:
            path = [start_pos]
            current = start_node
            # 追踪路径
            while current != T:
                # 如果是入节点，下一个必然是对应的出节点
                if current in node_to_grid and node_to_grid[current][2] == 'in':
                    next_node = current + 1
                    current = next_node
                    continue
                
                # 从出节点找下一个有流经过的入节点
                found_next = False
                for edge in dinic.graph[current]:
                    if edge.to != S and edge.cap < edge.original_cap:
                        next_node = edge.to
                        # 如果到达汇点，结束路径
                        if next_node == T:
                            if current in boundary_nodes:
                                path.append(boundary_nodes[current])
                            found_next = True
                            break
                        # 如果是入节点，添加其坐标到路径
                        if next_node in node_to_grid and node_to_grid[next_node][2] == 'in':
                            i, j, _ = node_to_grid[next_node]
                            path.append((i, j))
                        
                        current = next_node
                        found_next = True
                        break
                
                # 如果已到达汇点或无法找到下一个节点，结束此路径追踪
                if current == T or not found_next:
                    break
            
            paths.append(path)
    
    return can_escape, paths

if __name__ == "__main__":
    n = 6
    starts = [(3,1), (2,2), (3,2), (4,2), (2,4), (3,4), (4,4), (2,6), (3,6), (4,6)]
    可逃离, 路径列表 = solve_escape(n, starts)
    print(f"是否可以逃离: {可逃离}")

    if 可逃离:
        print(f"找到 {len(路径列表)} 条逃离路径:")
        for i, 路径 in enumerate(路径列表):
            print(f"路径 {i+1}: {路径}") 