import math
import unittest
from objects import Vector3


class VectorTest(unittest.TestCase):

    def test_init(self):
        self.assertEqual(Vector3(0, 0, 0), Vector3([0, 0, 0]))

    def test_setGet(self):
        vec = Vector3(0, 0, 0)
        vec.x = 1
        self.assertEqual(vec.x, 1)
        self.assertEqual(vec[0], 1)
        vec.y = 1
        self.assertEqual(vec.y, 1)
        self.assertEqual(vec[1], 1)
        vec.z = 1
        self.assertEqual(vec.z, 1)
        self.assertEqual(vec[2], 1)

    def test_setGetIndex(self):
        vec = Vector3(0, 0, 0)
        vec[0] = 1
        self.assertEqual(vec.x, 1)
        self.assertEqual(vec[0], 1)
        vec[1] = 1
        self.assertEqual(vec.y, 1)
        self.assertEqual(vec[1], 1)
        vec[2] = 1
        self.assertEqual(vec.z, 1)
        self.assertEqual(vec[2], 1)

    def test_toString(self):
        self.assertEqual(str(Vector3(0, 0, 0)), "[0, 0, 0]")

    def test_equal_list(self):
        self.assertEqual(Vector3(0, 0, 0), [0, 0, 0])
        self.assertEqual(Vector3([0, 0, 0]), [0, 0, 0])

    def test_add_vector(self):
        self.assertEqual(Vector3(0, 0, 0) + Vector3(1, 2, 3), Vector3(1, 2, 3))

    def test_add_value(self):
        self.assertEqual(Vector3(0, 0, 0) + 1, Vector3(1, 1, 1))

    def test_sub_vector(self):
        self.assertEqual(Vector3(0, 0, 0) - Vector3(1, 2, 3), Vector3(-1, -2, -3))

    def test_sub_value(self):
        self.assertEqual(Vector3(0, 0, 0) - 1, Vector3(-1, -1, -1))

    def test_mul_vector(self):
        self.assertEqual(Vector3(1, 1, 1) * Vector3(1, 2, 3), Vector3(1, 2, 3))

    def test_mul_value(self):
        self.assertEqual(Vector3(1, 1, 1) * 2, Vector3(2, 2, 2))

    def test_div_vector(self):
        self.assertEqual(Vector3(2, 4, 6) / Vector3(2, 2, 2), Vector3(1, 2, 3))

    def test_div_value(self):
        self.assertEqual(Vector3(2, 2, 2) / 2, Vector3(1, 1, 1))

    def test_negate(self):
        self.assertEqual(-Vector3(1, 1, 1), Vector3(-1, -1, -1))

    def test_magnitude(self):
        self.assertEqual(Vector3(-2, 1, 2).magnitude(), 3)

    def test_normalize(self):
        self.assertEqual(Vector3(-2, 1, 2).normalize(), Vector3(-2 / 3, 1 / 3, 2 / 3))

    def test_normalize_magnitude(self):
        self.assertEqual(Vector3(-2, 1, 2).normalize(True), (Vector3(-2 / 3, 1 / 3, 2 / 3), 3))

    def test_normalize_zero(self):
        self.assertEqual(Vector3(0, 0, 0).normalize(), Vector3(0, 0, 0))

    def test_normalize_magnitude_zero(self):
        self.assertEqual(Vector3(0, 0, 0).normalize(True), (Vector3(0, 0, 0), 0))

    def test_dot(self):
        self.assertEqual(Vector3(1, 2, 3).dot(Vector3(3, 2, 1)), 10)

    def test_cross(self):
        self.assertEqual(Vector3(2, 3, 4).cross(Vector3(5, 6, 7)), Vector3(-3, 6, -3))

    def test_flatten(self):
        self.assertEqual(Vector3(2, 3, 4).flatten(), Vector3(2, 3, 0))

    def test_copy(self):
        self.assertEqual(Vector3(2, 3, 4).copy(), Vector3(2, 3, 4))

    def test_angle2D(self):
        self.assertEqual(Vector3(1, 0, 4).angle2D(Vector3(0, 1, -1)), math.pi / 2)

    def test_angle3D(self):
        self.assertAlmostEqual(Vector3(1, 4, 2).angle3D(Vector3(-2, -3, -3)), 2.76652874, 4)

    def test_rotate(self):
        self.assertEqual(Vector3(1, 0, 4).rotate(math.pi / 4), Vector3(math.sqrt(2) / 2, math.sqrt(2) / 2, 4))

    def test_clamp_min(self):
        self.assertEqual(Vector3(1, -1, 4).clamp(Vector3(1, 0, 10), Vector3(0, 1, -9)), Vector3(1, 0, 10))

    def test_clamp_max(self):
        self.assertEqual(Vector3(-1, 1, 4).clamp(Vector3(1, 0, -10), Vector3(0, 1, 9)), Vector3(0, 1, 9))

    def test_clamp(self):
        self.assertEqual(Vector3(1, 1, 4).clamp(Vector3(1, 0, -10), Vector3(0, 1, 9)),Vector3(1, 1, 4))

    def test_dist(self):
        self.assertEqual(Vector3(1, 1, 0).dist(Vector3(2, 1, -2)), math.sqrt(5))

    def test_flat_dist(self):
        self.assertEqual(Vector3(1, 1, 0).flat_dist(Vector3(2, 1, -2)), 1)


if __name__ == '__main__':
    unittest.main()
