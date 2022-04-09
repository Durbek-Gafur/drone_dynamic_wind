from physics import Graph,Winds

def initWinds():
    winds = Winds()
    winds.add_wind(5,0,10)
    winds.add_wind(5,0,20)
    return winds

def initDefaultGraph():
    winds = initWinds()
    g = Graph(winds)
    g.add_vertex('a',(0,0))
    g.add_vertex('b',(100,100))
    g.add_vertex('c',(100,200))
    g.add_vertex('d',(200,200))
    g.add_vertex('e',(200,100))
    g.add_vertex('f',(-100,100))

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