import math

def dist(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def main():
    x1, y1 = 2.0, -1.0
    x2, y2 = -2.0, 2.0
    x3, y3 = 4.0, 2.0
    x4, y4 = 3.0, 5.0
    x5, y5 = 4.0, 5.0
    p1 = (x1, y1)
    p2 = (x2, y2)
    p3 = (x3, y3)
    p4 = (x4, y4)
    p5 = (x5, y5)
    v = [p1, p2, p3, p4, p5]

    min_dist = 0
    point1 = (0, 0)

    for point in v:
        d = dist((0, 0), point)
        if d > min_dist:
            min_dist = d
            point1 = point

    print("The farthest point to origin:")
    print(point1[0], point1[1])

    print("Distance of this point to origin:")
    print(min_dist)

    mx = 0
    x3_net, y3_net = 0, 0
    point2 = None

    for point in v:
        if point == point1:
            continue
        else:
            x1, y1 = point1
            x2, y2 = point

            radi1 = -(math.sqrt(x1**2 + y1**2) * (x1**2 - 2 * x1 * x2 + x2**2 + y1**2 - 2 * y1 * y2 + y2**2)) / (2 * (-x1**2 + x2 * x1 - y1**2 + y2 * y1))

            if radi1 >= mx:
                mx = radi1
                x3_net = (x1 * (x1**2 + y1**2 - y2**2 - x2**2)) / (2 * (x1**2 + y1**2 - x2 * x1 - y1 * y2))
                y3_net = (y1 / x1) * x3_net
                point2 = point

    print("The second point is:")
    print(point2[0], point2[1])

    print("New center is:")
    print(x3_net, y3_net)

    if (point1[0] + point2[0]) / 2 == x3_net and (point1[1] + point2[1]) / 2 == y3_net:
        print("No further iteration required, and the center is:")
        print(x3_net, y3_net)
    else:
        a = (point1[0] + point2[0]) / 2
        b = (point1[1] + point2[1]) / 2
        minn = 0
        p1 = None
        d = 0

        for point in v:
            c = dist((a, b), point)
            if c > dist(point1, point2) / 2:
                if c > minn:
                    p1 = point
                    minn = c
                d = 9.0

        if d == 0:
            print("No further iteration required, and the center is:")
            print(a, b)
        else:
            avgp_x = (point1[0] + point2[0]) / 2
            avgp_y = (point1[1] + point2[1]) / 2
            m = (avgp_y - y3_net) / (avgp_x - x3_net)
            c = y3_net - x3_net * m
            x1, y1 = point1
            x5_net = -(-x1**2 + p1[0]**2 - y1**2 + 2 * c * y1 + p1[1]**2 - 2 * c * p1[1]) / (2 * (x1 - p1[0] + m * y1 - m * p1[1]))
            y5_net = m * (x5_net - x3_net) + y3_net
            distt = dist((x5_net, y5_net), (p1[0], p1[1]))
            print("Further iteration was needed, its new min radius is:")
            print(distt)
            print("And the new center is:")
            print(x5_net, y5_net)

if __name__ == "__main__":
    main()

