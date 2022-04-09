def testEnergy(Ax,Ay,Bx,By,payload,speed,wind_speed,wind_direction):
    distance = np.sqrt((Ax - Bx) ** 2 + (Ay - By) ** 2)

    # directions among vertex A and vertex B
    direction = int(np.rad2deg(float(sp.atan2(By - Ay, Bx - Ax))))
    direction = direction + 360 if direction<0 else direction
    relative_wind_direction = abs(direction - wind_direction)

    return get_energy(distance, payload, speed, wind_speed, relative_wind_direction)

   
def shortestCycle(graph,source,destination,payload,speed,time=0):
    src = shortestPath(graph, source, destination,payload,speed,time)
    dst = shortestPath(graph, destination, source,payload,speed,time) 
    return src+dst[1:]

def shortestPath(graph, source, destination,payload,speed,time=0):
    # shortest paths is a dict of nodes
    # whose value is a tuple of (previous node, weight)
    shortest_paths = {source: (None, 0)}
    current_node = graph.get_vertex(source)
    destination_node = graph.get_vertex(destination)
    visited = set()
    while current_node.get_id()!= destination:
        visited.add(current_node.get_id())
        destinations = current_node.get_connections()
        weight_to_current_node = shortest_paths[current_node.get_id()][1]

        for next_node in destinations:
            weight = current_node.get_weight(next_node,payload,speed,graph.get_wind_speed(time),graph.get_wind_direction(time)) + weight_to_current_node

            if next_node.get_id() not in shortest_paths:
                shortest_paths[next_node.get_id()] = (current_node.get_id(), weight)
            else:
                current_shortest_weight = shortest_paths[next_node.get_id()][1]
                if current_shortest_weight > weight:
                    shortest_paths[next_node.get_id()] = (current_node.get_id(), weight)
        
        next_destinations = {node: shortest_paths[node] for node in shortest_paths if node not in visited}
        if not next_destinations:
            return "Route Not Possible"
        # next node is the destination with the lowest weight
        current_node = graph.get_vertex(min(next_destinations, key=lambda k: next_destinations[k][1]))

    # # Work back through destinations in shortest path
    path = []
    while current_node is not None:
        path.append(current_node.get_id())
        next_node = graph.get_vertex(shortest_paths[current_node.get_id()][0])
        current_node = next_node
    # # Reverse path
    path = path[::-1]
    return path