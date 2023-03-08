class Node():
    def __init__(self, position=(-1, -1), index = (-1, -1), name=None):
        self._position = position

        self._name = name

        self._index = index

        self._x = position[0]
        self._y = position[1]

        self._f = None
        self._g = None 
        self._h = None 

    def get_position(self):
        return self._position