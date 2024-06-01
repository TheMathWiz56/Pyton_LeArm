from Constants import square


def compare_points(reference, point_list):
    shortest_distance = get_distance_between_points(reference, point_list[0])
    index = 0
    for i in range(1, len(point_list), 1):
        if get_distance_between_points(reference, point_list[i]) < shortest_distance:
            index = i
            shortest_distance = get_distance_between_points(reference, point_list[i])
    return point_list[index]


def get_distance_between_points(point1, point2):
    distance = 0
    for i in range(len(point1)):
        distance += distance + square(point1[i] - point2[i])
    return distance


print(compare_points([0, 0],[[5, 6], [2, 3], [-1, 0]]))
