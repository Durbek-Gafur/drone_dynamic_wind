from supportingFunctions import shortestPath, shortestCycle
from loadGraph import initDefaultGraph

def offlineShortestPath(graph,Battery,source,destination,payload,speed,wind_speed,wind_direction):
    status = None
    flag = False
    time = 0
    cycle = shortestCycle(graph, source, destination,payload,speed,wind_speed,wind_direction)
    total_energy = graph.get_path_energy(cycle,payload,speed,wind_speed,wind_direction)
    if total_energy <=Battery:
        for i in range(len(cycle)-1):
            # wind_speed, wind_direction is function of time
            edge_energy = graph.get_energy_between_two_nodes(cycle[i],cycle[i+1],payload,speed,wind_speed,wind_direction)
            Battery -= edge_energy 

            if cycle[i+1] == destination:
                payload = 0
                flag = True 
            if cycle[i+1] == source:
                status = "SUCCESS"
                return status
            if Battery < 0:
                if flag == True:
                    status = "DELIVERED"
                    return status
                else:
                    status = "FAILED"
                    return status
            time+= graph.get_distance(cycle[i],cycle[i+1]) 
    else:
        status = "CANCELLED"
        return status

if __name__ == '__main__':
    # drone's parameters

    g = initDefaultGraph()

    payload = 10
    speed = 10

    # wind's parameters (global)
    wind_speed = 5
    wind_direction = 0
    # g.printConnection(payload,speed,wind_speed,wind_direction)
    # start = time
    status = offlineShortestPath(g,840,'a', 'd',payload,speed,wind_speed,wind_direction)
  
    print(status)
