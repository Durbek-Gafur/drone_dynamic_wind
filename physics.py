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
    def __init__(self,winds):
        self.vert_dict = {}
        self.num_vertices = 0
        self.winds = winds

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
#big change
    def get_energy_between_two_nodes(self,source,destination,payload,speed,time):
        current_node = self.get_vertex(source)
        next_node = self.get_vertex(destination)
        return current_node.get_weight(next_node,payload,speed,self.get_wind_speed(time),self.get_wind_direction(time))

    def get_path_energy(self,path,payload,speed,time):
        if len(path) == 2:
            return self.get_energy_between_two_nodes(path[i],path[i+1],payload,speed,time)
        energy = 0
        for i in range(len(path)-1):
            
            if len(path)/2<i+1:
                payload = 0
            energy+=self.get_energy_between_two_nodes(path[i],path[i+1],payload,speed,time)

        return energy 

    def get_distance(self,source,destination):
        current_node = self.get_vertex(source)
        next_node = self.get_vertex(destination)
        return current_node.adjacent[next_node]["distance"] 

    def get_wind_speed(self,time):
        return self.winds.get_wind_speed(time)

    def get_wind_direction(self,time):
        return self.winds.get_wind_direction(time)

    def printConnection(self,payload,speed,time):

        for v in self:
            for w in v.get_connections():
                vid = v.get_id()
                wid = w.get_id()

                print('( %s , %s, %f, %f )'  % ( vid, wid, v.get_direction(w),v.get_weight(w,payload,speed,self.get_wind_speed(time),self.get_wind_direction(time)))) #testEnergy(v.x,v.y,w.x,w.y,payload,speed,self.get_wind_speed(time),self.get_wind_direction(time))
        for v in self:
            print ('g.vert_dict[%s]=%s' %(v.get_id(), g.vert_dict[v.get_id()]))


class Wind:
    def __init__(self,wind_speed,wind_direction,timeUpperBound):
        self.wind_speed = wind_speed
        self.wind_direction = wind_direction
        self.time = timeUpperBound

    def get_wind_speed(self):
        return self.wind_speed

    def get_wind_direction(self):
        return self.wind_direction

    def __str__(self):
        return str(self.wind_speed)+" "+str(self.wind_direction)

class Winds():
    def __init__(self):
        self.wind_dict = {}
        self.wind_array = []
        self.add_wind(0,0,0)
        self.add_wind(0,0,float("inf"))

    def add_wind(self,wind_speed,wind_direction,timeUpperBound):
        wind = Wind(wind_speed,wind_direction,timeUpperBound)
        self.wind_dict[timeUpperBound] = wind
        self.wind_array.append(timeUpperBound)
        self.wind_array.sort()

    def get_wind(self,time):

        for i in range(len(self.wind_array)-1):
            if self.wind_array[i]<= time < self.wind_array[i+1]:
                return self.wind_dict[self.wind_array[i+1]]
        return None 

    def get_wind_speed(self,time):
        return self.get_wind(time).wind_speed

    def get_wind_direction(self,time):
        return self.get_wind(time).wind_direction

    def __str__(self):
        r=""
        for key in self.wind_array:
            r+=str(key)+" "+str(self.wind_dict[key])+"\n"
        return r


# winds = Winds()
# winds.add_wind(5,0,10)
# winds.add_wind(15,1,20)

# g = Graph(winds)
# g.add_vertex('a',(0,0))
# g.add_vertex('b',(100,100))
# g.add_vertex('c',(100,200))
# g.add_vertex('d',(200,200))
# g.add_vertex('e',(200,100))
# g.add_vertex('f',(-100,100))

# g.add_edge('a', 'b')  
# # g.add_edge('a', 'c')
# g.add_edge('a', 'f')
# g.add_edge('b', 'c')
# g.add_edge('b', 'e')
# g.add_edge('b', 'f')
# g.add_edge('c', 'd')
# g.add_edge('c', 'f')
# g.add_edge('d', 'e')


# print(g.get_wind_speed(1))
# print(g.get_wind_speed(12))
# # print(graph.vert_dict)
# # g.printConnection(10,5,1)

