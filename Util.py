import numpy as np


def get_degree_between_vectors(vec1, vec2):
    len_vec1 = np.linalg.norm(vec1)
    len_vec2 = np.linalg.norm(vec2)
    inner_product = np.inner(vec1, vec2)
    degree = np.rad2deg(np.arccos(inner_product / (len_vec1 * len_vec2)))
    return degree


def is_valid_triangle(a, b, c):
    if a + b >= c and b + c >= a and a + c >= b:
        return True
    else:
        return False


def get_degree_between_triangle(a, b, c):
    # Return degree between a and b
    cos_angle_alpha = (np.power(a, 2) + np.power(b, 2) - np.power(c, 2)) / (
            2 * a * b)
    return np.rad2deg(np.arccos(cos_angle_alpha))


def get_normal_vector(vec1, vec2):
    # Get global axis vector by outer product
    # (Vertical vector to a, b, c plane.)
    normal_vector = np.cross(vec1, vec2)
    normal_vector = np.divide(normal_vector, np.linalg.norm(normal_vector))
    normal_vector = np.append(normal_vector, np.array([0]))
    return normal_vector
