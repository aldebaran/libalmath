import unittest
import almath



class TestTransform(unittest.TestCase):
    def test_init(self):
        t = almath.Transform()

        self.assertAlmostEqual(t.r1_c1, 1.0)
        self.assertAlmostEqual(t.r1_c2, 0.0)
        self.assertAlmostEqual(t.r1_c3, 0.0)
        self.assertAlmostEqual(t.r1_c4, 0.0)

        self.assertAlmostEqual(t.r2_c1, 0.0)
        self.assertAlmostEqual(t.r2_c2, 1.0)
        self.assertAlmostEqual(t.r2_c3, 0.0)
        self.assertAlmostEqual(t.r2_c4, 0.0)

        self.assertAlmostEqual(t.r3_c1, 0.0)
        self.assertAlmostEqual(t.r3_c2, 0.0)
        self.assertAlmostEqual(t.r3_c3, 1.0)
        self.assertAlmostEqual(t.r3_c4, 0.0)

    def test_isNear(self):
        t = almath.Transform()
        self.assertTrue(t.isNear(t))




class TestPosition6D(unittest.TestCase):
    def test_init(self):
        p = almath.Position6D()
        self.assertAlmostEqual(p.x, 0.0)
        self.assertAlmostEqual(p.y, 0.0)
        self.assertAlmostEqual(p.z, 0.0)
        self.assertAlmostEqual(p.wx, 0.0)
        self.assertAlmostEqual(p.wy, 0.0)
        self.assertAlmostEqual(p.wz, 0.0)

    def test_ops(self):
        p0 = almath.Position6D(0.1, 0.2, 0.4, -0.3, 0.8, 12.0)
        p1 = almath.Position6D(0.3, -0.2, 0.9, -0.1, 0.4, 2.0)
        pAdd = p0 + p1

        pExp = almath.Position6D(p0.x + p1.x,
                                 p0.y + p1.y,
                                 p0.z + p1.z,
                                 p0.wx + p1.wx,
                                 p0.wy + p1.wy,
                                 p0.wz + p1.wz)
        self.assertTrue(pAdd.isNear(pExp))

        factor = 0.2
        pFact = p0 * factor
        pExp = almath.Position6D(p0.x * factor,
                                 p0.y * factor,
                                 p0.z * factor,
                                 p0.wx * factor,
                                 p0.wy * factor,
                                 p0.wz * factor)
        self.assertTrue(pFact.isNear(pExp))



    def test_distance(self):
        p0 = almath.Position6D()
        self.assertAlmostEqual(p0.distance(p0), 0.0)
        p1 = almath.Position6D(0.2, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(p0.distance(p1), p1.x)



class TestPosition3D(unittest.TestCase):
    def test_init(self):
        p = almath.Position3D()
        self.assertAlmostEqual(p.x, 0.0)
        self.assertAlmostEqual(p.y, 0.0)
        self.assertAlmostEqual(p.z, 0.0)

    def test_ops(self):
        p0 = almath.Position3D(0.1, 0.2, 0.4)
        p1 = almath.Position3D(0.3, -0.2, 0.9)
        pAdd = p0 + p1
        pExp = almath.Position3D(p0.x + p1.x,
                                 p0.y + p1.y,
                                 p0.z + p1.z)

        self.assertTrue(pAdd.isNear(pExp))

        factor = 0.2
        pFact = p0 * factor
        pExp = almath.Position3D(p0.x * factor,
                                 p0.y * factor,
                                 p0.z * factor)
        self.assertTrue(pFact.isNear(pExp))


    def test_distance(self):
        p0 = almath.Position3D()
        self.assertAlmostEqual(p0.distance(p0), 0.0)
        p1 = almath.Position3D(0.2, 0.0, 0.0)
        self.assertAlmostEqual(p0.distance(p1), p1.x)




class TestPosition2D(unittest.TestCase):
    def test_init(self):
        p = almath.Position2D()
        self.assertAlmostEqual(p.x, 0.0)
        self.assertAlmostEqual(p.y, 0.0)

    def test_ops(self):
        p0 = almath.Position2D(0.1, 0.2)
        p1 = almath.Position2D(0.3, -0.2)
        pAdd = p0 + p1
        pExp = almath.Position2D(p0.x + p1.x,
                                 p0.y + p1.y)

        self.assertTrue(pAdd.isNear(pExp))

        factor = 0.2
        pFact = p0 * factor
        pExp = almath.Position2D(p0.x * factor,
                                 p0.y * factor)
        self.assertTrue(pFact.isNear(pExp))


    def test_distance(self):
        p0 = almath.Position2D()
        self.assertAlmostEqual(p0.distance(p0), 0.0)
        p1 = almath.Position2D(0.2, 0.0)
        self.assertAlmostEqual(p0.distance(p1), p1.x)




class TestPose2D(unittest.TestCase):
    def test_init(self):
        p = almath.Pose2D()
        self.assertAlmostEqual(p.x, 0.0)
        self.assertAlmostEqual(p.y, 0.0)
        self.assertAlmostEqual(p.theta, 0.0)

    def test_ops(self):
        p0 = almath.Pose2D(0.1, 0.2, 0.2)
        p1 = almath.Pose2D(0.3, -0.2, -0.6)
        pAdd = p0 + p1
        pExp = almath.Pose2D(p0.x + p1.x,
                             p0.y + p1.y,
                             p0.theta + p1.theta)

        self.assertTrue(pAdd.isNear(pExp))

    def test_distance(self):
        p0 = almath.Pose2D()
        self.assertAlmostEqual(p0.distance(p0), 0.0)
        p1 = almath.Pose2D(0.2, 0.0, 0.1)
        self.assertAlmostEqual(p0.distance(p1), p1.x)


    def test_inverse(self):
        p0 = almath.Pose2D(0.1, -0.1, -0.5)
	p1 = p0.inverse()
	pResult = almath.Pose2D(-0.135701, 0.0398157, 0.5)
	self.assertTrue(p1.isNear(pResult))

class TestALInterpolationArticular(unittest.TestCase):
    def test_init(self):
        int = almath.ALInterpolationArticular()
        times = almath.vectorFloat([0.0, 1.5])
        positions = almath.vectorFloat([2.0, 0.0])
        velocities = almath.vectorFloat([1.0, -1.0])
        isHotStart = False
        period = 0.02
        int.Init(times, positions, velocities, isHotStart, period)
        solCurrent = int.getCurrentInterpolation(0.5)
        exp = almath.PositionAndVelocity(1.81481481481481, -1.44444444444444)
        self.assertTrue(exp.isNear(solCurrent))



class TestConvexHull(unittest.TestCase):
    def test_init(self):
        size = 1.0

        points = almath.vectorPosition2D([almath.Position2D(size, size),
                                          almath.Position2D(-size, size),
                                          almath.Position2D(-size, -size),
                                          almath.Position2D(size, -size),
                                          almath.Position2D(0.0, 0.0)])

        hull = almath.getConvexHull(points)

        for i in range(hull.size()):
            print hull[i].x, hull[i].y

        self.assertEqual(hull.size(), 5)



if __name__ == '__main__':
    unittest.main()
