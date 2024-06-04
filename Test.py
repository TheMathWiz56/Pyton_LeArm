import math as m


def scale_to_range_from_0(n, maxn):
    remainder = n % maxn
    return n % maxn


print(scale_to_range_from_0(7 * m.pi, 2 * m.pi))

