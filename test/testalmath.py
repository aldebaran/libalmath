import almath as m

print 'Check print of almath struct'
print 'Check Pose2D'
pose2D = m.Pose2D(0.0, 1.3, 2.9)
print pose2D.__repr__()
print pose2D
print ''

print 'Check Position2D'
position2D = m.Position2D(0.1, 0.2)
print position2D.__repr__()
print position2D
print ''

print 'Check Position3D'
position3D = m.Position3D(0.1, 0.2, 0.3)
print position3D.__repr__()
print position3D
print ''

print 'Check Position6D'
position6D = m.Position6D(0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
print position6D.__repr__()
print position6D
print ''

print 'Check PositionAndVelocity'
PositionAndVelocity = m.PositionAndVelocity(0.1, 0.2)
print PositionAndVelocity.__repr__()
print PositionAndVelocity
print ''

print 'Check Rotation'
Rotation = m.Rotation.fromRotX(0.1)
print Rotation.__repr__()
print Rotation
print ''

print 'Check Rotation2D'
Rotation2D = m.Rotation2D.fromAngle(0.1)
print Rotation2D.__repr__()
print Rotation2D
print ''

print 'Check Rotation3D'
Rotation3D = m.Rotation3D(0.1, 0.2, 0.3)
print Rotation3D.__repr__()
print Rotation3D
print ''

print 'Check Transform'
Transform = m.Transform.fromRotX(0.1)
print Transform.__repr__()
print Transform
print ''

print 'Check TransformAndVelocity6D'
TransformAndVelocity6D = m.TransformAndVelocity6D()
print TransformAndVelocity6D.__repr__()
print TransformAndVelocity6D
print ''

print 'Check Velocity3D'
Velocity3D = m.Velocity3D(0.1, 0.2, 0.3)
print Velocity3D.__repr__()
print Velocity3D
print ''

print 'Check Velocity6D'
Velocity6D = m.Velocity6D(0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
print Velocity6D.__repr__()
print Velocity6D
print ''

print 'Check Quaternion'
quaternion = m.Quaternion(0.1, 0.2, 0.3, 0.4)
print quaternion.__repr__()
print quaternion
print ''

# Transform stuff.
t = m.TransformFromPosition (0.0, 1.3, 2.9)
t1 = m.TransformFromPosition (0.0, 0.0, 10.0)
t *= t1


print t.__repr__()
print t


p = m.Position3DFromTransform(t)
print p.__repr__()
print p


t2 = m.TransformFromRotZ(0.3)

print t2
print m.Determinant(t2)


p1 = m.Position6D(0.2)
p2 = m.Position6D(0.4)

p3 = p1 + p2
print p3.x

print p3.distance(p1)

print m.norm(p3)


p1 = m.Position3D(0.0)
p2 = m.Position3D(0.4)

p3 = p1 + p2

print p3.norm()



p1 = m.Position2D(0.0, 0.0)
p2 = m.Position2D(0.1, 0.1)

print p2.x, p2.y
p3 = p1 + p2

print p3.norm()


p1 = m.Pose2D(0.1, 0.0, 0.0)
p2 = m.Pose2D(0.1, 0.1, 0.2)

print p2.x, p2.y, p2.theta
p3 = p1 + p2

print p3.x
