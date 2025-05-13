class Edge:
    def __init__(self, to, rev, cap):
        self.to = to
        self.rev = rev
        self.cap = cap
        self.original_cap = cap  # 记录原始容量用于路径恢复

class FordFulkerson:
    def __init__(self, n):
        self.size = n
        self.graph = [[] for _ in range(n)]
    
    def add_edge(self, fr, to, cap):
        forward = Edge(to, len(self.graph[to]), cap)
        backward = Edge(fr, len(self.graph[fr]), 0)
        self.graph[fr].append(forward)
        self.graph[to].append(backward)
    
    def dfs(self, s, t, visited, path, parent):
        visited[s] = True
        
        if s == t:
            return True
        
        for i, edge in enumerate(self.graph[s]):
            if edge.cap > 0 and not visited[edge.to]:
                parent[edge.to] = (s, i)
                if self.dfs(edge.to, t, visited, path, parent):
                    return True
        
        return False
    
    def max_flow(self, s, t):
        flow = 0
        parent = {}
        
        while True:
            visited = [False] * self.size
            path = []
            if not self.dfs(s, t, visited, path, parent):
                break
            
            # 计算路径上的最小容量
            min_cap = float('inf')
            v = t
            while v != s:
                u, i = parent[v]
                min_cap = min(min_cap, self.graph[u][i].cap)
                v = u
            
            # 更新残余网络
            v = t
            while v != s:
                u, i = parent[v]
                self.graph[u][i].cap -= min_cap
                self.graph[v][self.graph[u][i].rev].cap += min_cap
                v = u
            
            flow += min_cap
            
        return flow

def solve_escape(n, starts):
    """求解逃离问题，找出顶点不相交的路径"""
    # 构建网络：每个网格点拆分为入节点和出节点
    grid_size = n * n
    total_nodes = 2 + 2 * grid_size  # 源点、汇点、网格点(每个拆成入和出)
    ff = FordFulkerson(total_nodes)
    S, T = 0, 1  # 源点和汇点
    
    # 节点ID映射和反向映射
    node_to_grid = {}
    start_to_node = {}
    
    def get_in_id(i, j):
        """获取位置(i,j)对应的入节点ID"""
        node_id = 2 + 2 * ((i-1) * n + (j-1))
        node_to_grid[node_id] = (i, j, 'in')
        node_to_grid[node_id + 1] = (i, j, 'out')
        return node_id
    
    # 1. 顶点拆分，对应节点容量约束
    for i in range(1, n+1):
        for j in range(1, n+1):
            in_node = get_in_id(i, j)
            ff.add_edge(in_node, in_node + 1, 1)  # 入节点连出节点，容量为1
    
    # 2. 相邻点连边
    dirs = [(-1,0), (1,0), (0,-1), (0,1)]  # 四个方向
    for i in range(1, n+1):
        for j in range(1, n+1):
            u_out = get_in_id(i, j) + 1  # 出节点
            for di, dj in dirs:
                ni, nj = i + di, j + dj
                if 1 <= ni <= n and 1 <= nj <= n:  # 确保邻居在网格内
                    v_in = get_in_id(ni, nj)  # 邻居的入节点
                    ff.add_edge(u_out, v_in, 1)
    
    # 3. 源点连接所有起点
    for x, y in starts:
        in_node = get_in_id(x, y)
        ff.add_edge(S, in_node, 1)
        start_to_node[(x, y)] = in_node
    
    # 4. 边界点连接汇点
    boundary_nodes = {}
    for i in range(1, n+1):
        for j in range(1, n+1):
            if i == 1 or i == n or j == 1 or j == n:  # 边界点
                out_node = get_in_id(i, j) + 1
                ff.add_edge(out_node, T, 1)
                boundary_nodes[out_node] = (i, j)
    
    # 计算最大流
    max_flow = ff.max_flow(S, T)
    can_escape = max_flow == len(starts)
    
    if not can_escape:
        return False, []
    
    # 提取路径
    paths = []
    for start_pos in starts:
        start_node = start_to_node[start_pos]
        # 检查从源点到起点是否有流
        has_flow = False
        for i, edge in enumerate(ff.graph[S]):
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
                for edge in ff.graph[current]:
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