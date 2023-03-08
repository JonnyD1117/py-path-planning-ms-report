def px2coord(indices, res=2, dx_0=-9, dy_0=-33):
    x_coord = indices[1]*res + dx_0
    y_coord = indices[0]*res - dy_0
    coord = (x_coord, y_coord)
    return coord

def coords2px(coords, res=2, dx_0=-9, dy_0=-33):

    dj_0 = abs(int(dx_0/res))
    di_0 = abs(int(dy_0/res))

    index_i = int(coords[1]/res)
    index_j = int(coords[0]/res)

    px_coords = (index_i + di_0, index_j + dj_0)

    return px_coords


if __name__ == "__main__":

    # goal_ind: [33, 20]
    # start_ind: [ 279, 96]

    # # SI Based Initial and goal States 
    # origin: [0, 0]            
    # goal_point: [-3, 15]
    # x0_offset: -4.8clear

    # y0_offset: 13.95

    goal_ind = (33, 20)
    start_ind = (279, 96)

    res = .05 
    x_off = -4.8
    y_off = 13.95

    start = px2coord(start_ind, res=res, dx_0=x_off, dy_0=y_off)
    coords = px2coord(goal_ind, res=res, dx_0=x_off, dy_0=y_off)
    print("Goal")
    print(coords)
    print("Start")
    print(start)