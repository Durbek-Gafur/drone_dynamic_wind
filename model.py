import sympy as sp
import numpy as np
import mpmath

mpmath.mp.dps = 5


# physical model for the drone's energy consumption. Do not touch it!
# It returns the energy required for flying 1 meter
def get_energy(distance, payload_weight, drone_speed, wind_speed, relative_wind_direction):

    # start calculations
    m_package = payload_weight
    m_drone = 7
    m_battery = 10

    num_rotors = 8
    diameter = 0.432

    # s_battery = 540000
    # delta = 0.5
    # f = 1.2

    pressure = 100726  # 50 meters above sea level
    R = 287.058
    temperature = 15 + 273.15  # 15 degrees in Kelvin
    rho = pressure / (R*temperature)

    g = 9.81

    # power efficiency
    eta = 0.7

    drag_coefficient_drone = 1.49
    drag_coefficient_battery = 1
    drag_coefficient_package = 2.2

    projected_area_drone = 0.224
    projected_area_battery = 0.015
    projected_area_package = 0.0929

    v_north = drone_speed - wind_speed*np.cos(np.deg2rad(relative_wind_direction))
    v_east = - wind_speed*np.sin(np.deg2rad(relative_wind_direction))
    v_air = np.sqrt(v_north**2 + v_east**2)

    # Drag force
    F_drag_drone = 0.5 * rho * (v_air**2) * drag_coefficient_drone * projected_area_drone
    F_drag_battery = 0.5 * rho * (v_air**2) * drag_coefficient_battery * projected_area_battery
    F_drag_package = 0.5 * rho * (v_air**2) * drag_coefficient_package * projected_area_package

    F_drag = F_drag_drone + F_drag_battery + F_drag_package

    alpha = np.arctan(F_drag / ((m_drone + m_battery + m_package)*g))

    # Thrust
    T = (m_drone + m_battery + m_package)*g + F_drag

    # # Power min hover
    # P_min_hover = (T**1.5) / (np.sqrt(0.5 * np.pi * num_rotors * (diameter**2) * rho))

    # v_i = Symbol('v_i')
    # f_0 = v_i - (2*T / (np.pi * num_rotors * (diameter**2) * rho * sp.sqrt((drone_speed*sp.cos(alpha))**2 + (drone_speed*sp.sin(alpha) + v_i)**2)))
    # induced_speed = float(nsolve(f_0, v_i, 5))
    # print(induced_speed)

    tmp_a = 2*T
    tmp_b = np.pi * num_rotors * (diameter**2) * rho
    tmp_c = (drone_speed*sp.cos(alpha))**2
    tmp_d = drone_speed*sp.sin(alpha)
    tmp_e = tmp_a / tmp_b

    coeff = [1, (2*tmp_d), (tmp_c+tmp_d**2), 0, -tmp_e**2]
    sol = np.roots(coeff)
    induced_speed = float(max(sol[np.isreal(sol)]).real)
    # print(induced_speed)

    # Power min to go forward
    P_min = T*(drone_speed*np.sin(alpha) + induced_speed)

    # expended power
    P = P_min / eta

    # energy efficiency of travel
    mu = P / drone_speed

    # Energy consumed
    E = mu * distance

    # # Range of a drone
    # R = (m_battery * s_battery * delta) / (e * f)

    return E/1000.

class Vertex:
    def __init__(self, node,point):
        self.id = node # a,b,c,d
        self.x = point[0]
        self.y = point[1]
        self.adjacent = {}

    def __str__(self):
        return str(self.id) + ' adjacent: ' + str([x.id for x in self.adjacent])

    def add_neighbor(self, neighbor):

        self.adjacent[neighbor] = {
        "distance": np.sqrt((self.x - neighbor.x) ** 2 + (self.y - neighbor.y) ** 2),
        "direction": int(np.rad2deg(float(sp.atan2(neighbor.y - self.y, neighbor.x - self.x))))

        }

    def get_connections(self):
        return self.adjacent.keys()  

    def get_id(self):
        return self.id

    def get_direction(self,neighbor):
        return self.adjacent[neighbor]["direction"] 

    def get_weight(self, neighbor,payload,speed,wind_speed,wind_direction):
        direction = self.adjacent[neighbor]["direction"] 
        distance = self.adjacent[neighbor]["distance"] 
        if direction< 0:
            direction = direction + 360

        # relative wind direction (difference between the two directions)
        relative_wind_direction = abs(direction - wind_direction)


        return get_energy(distance, payload, speed, wind_speed, relative_wind_direction)

class Graph:
    def __init__(self):
        self.vert_dict = {}
        self.num_vertices = 0

    def __iter__(self):
        return iter(self.vert_dict.values())

    def add_vertex(self, node,point):
        self.num_vertices = self.num_vertices + 1
        new_vertex = Vertex(node,point)
        self.vert_dict[node] = new_vertex
        return new_vertex

    def get_vertex(self, n):
        if n in self.vert_dict:
            return self.vert_dict[n]
        else:
            return None

    def add_edge(self, frm, to, cost = 0):
        if frm not in self.vert_dict:
            self.add_vertex(frm)
        if to not in self.vert_dict:
            self.add_vertex(to)

        self.vert_dict[frm].add_neighbor(self.vert_dict[to])
        self.vert_dict[to].add_neighbor(self.vert_dict[frm])

    def get_vertices(self):
        return self.vert_dict.keys()

    def printConnection(self,payload,speed,wind_speed,wind_direction):
        for v in self:
            for w in v.get_connections():
                vid = v.get_id()
                wid = w.get_id()
                print('( %s , %s, %f, %f == %f)'  % ( vid, wid, v.get_direction(w),v.get_weight(w,payload,speed,wind_speed,wind_direction),testEnergy(v.x,v.y,w.x,w.y,payload,speed,wind_speed,wind_direction)))

                #["distance"],v.get_weight(w)["direction"]))


        for v in self:
            print ('g.vert_dict[%s]=%s' %(v.get_id(), g.vert_dict[v.get_id()]))

def initDefaultGraph():
    g = Graph()
    g.add_vertex('a',(0,0))
    g.add_vertex('b',(1,1))
    g.add_vertex('c',(1,2))
    g.add_vertex('d',(2,2))
    g.add_vertex('e',(2,1))
    g.add_vertex('f',(-1,1))

    g.add_edge('a', 'b')  
    # g.add_edge('a', 'c')
    g.add_edge('a', 'f')
    g.add_edge('b', 'c')
    g.add_edge('b', 'e')
    g.add_edge('b', 'f')
    g.add_edge('c', 'd')
    g.add_edge('c', 'f')
    g.add_edge('d', 'e')
    # g.add_edge('e', 'f')
    return g

  

def testEnergy(Ax,Ay,Bx,By,payload,speed,wind_speed,wind_direction):
    distance = np.sqrt((Ax - Bx) ** 2 + (Ay - By) ** 2)

    # directions among vertex A and vertex B
    direction = int(np.rad2deg(float(sp.atan2(By - Ay, Bx - Ax))))
    direction = direction + 360 if direction<0 else direction
    relative_wind_direction = abs(direction - wind_direction)

    return get_energy(distance, payload, speed, wind_speed, relative_wind_direction)

   
def shortestCycle(graph,source,destination,payload,speed,wind_speed,wind_direction):
    src = shortestPath(graph, source, destination,payload,speed,wind_speed,wind_direction)
    dst = shortestPath(graph, destination, source,payload,speed,wind_speed,wind_direction) 
    return src+dst[1:]

def shortestPath(graph, source, destination,payload,speed,wind_speed,wind_direction):
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
            weight = current_node.get_weight(next_node,payload,speed,wind_speed,wind_direction) + weight_to_current_node

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

def offlineShortestPath(graph,Battery,destination):
    status = next_node
    flag

if __name__ == '__main__':
    # drone's parameters

    g = initDefaultGraph()

    payload = 0
    speed = 10

    # wind's parameters (global)
    wind_speed = 5
    wind_direction = 40
    # g.printConnection(payload,speed,wind_speed,wind_direction)
    # start = time
    print(shortestCycle(g, 'a', 'd',payload,speed,wind_speed,wind_direction))
  
