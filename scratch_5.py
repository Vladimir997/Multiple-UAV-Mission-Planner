import matplotlib.pyplot as plt
import math

# This code is only for visualising the offset

def rotate_around_point(x_to_rotate, y_to_rotate, around_x, around_y, for_theta):
    x_temp = x_to_rotate - around_x
    y_temp = y_to_rotate - around_y
    new_x = x_temp * math.cos(for_theta) - y_temp * math.sin(for_theta)
    new_y = x_temp * math.sin(for_theta) + y_temp * math.cos(for_theta)
    new_x += around_x
    new_y += around_y
    return new_x, new_y

# 1, 0
# 1, -10
# 0, 10
plt.plot([1,1,0,1],[0,-10,10,0])
x1, y1 = 1, -10
x2, y2 = 0, 10
x3, y3 = 1, 0
r = 0.5
if x2 - x1 == 0:
    x1 += 0.000001
if y2 - y1 == 0:
    y1 += 0.000001
if x3 - x2 == 0:
    x3 += 0.000001
if y3 - y2 == 0:
    y3 += 0.000001
print([x1,x2,x3,y1,y2,y3])
'''
dx21 = x2 - x1
dy21 = y2 - y1
dx31 = x3 - x1
dy31 = y3 - y1
m12 = math.sqrt(dx21*dx21 + dy21*dy21)
m13 = math.sqrt(dx31*dx31 + dy31*dy31)
theta = math.acos((dx21*dx31 + dy21*dy31)/(m12*m13))

m1 = (y2 - y1)/(x2 - x1)
b1 = y1 - m1 * x1
m2 = (y3 - y2)/(x3 - x2)
b2 = y3 - m2 * x3
print([m1,m2,b1,b2])
theta = math.atan2(m1-m2,1+m1*m2)
if (x1<x2<x3 or x3<x2<x1) and (y1<y2<y3 or y3<y2<y1) and theta < math.pi/2:
    theta = math.pi - theta
theta_half = theta/2
if theta < 0:
    print('check theta')
    theta_half = -theta_half
'''
a = math.sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1))
b = math.sqrt((x3 - x2)*(x3 - x2) + (y3 - y2)*(y3 - y2))
c = math.sqrt((x1 - x3)*(x1 - x3) + (y1 - y3)*(y1 - y3))
theta = math.acos((a*a + b*b - c*c)/(2*a*b))
theta_half = theta/2

new_x_angle = math.atan2(y2-y1,x2-x1)


x_final, y_final = rotate_around_point(x2 + r, y2, x2, y2, new_x_angle + theta_half)

'''''
x_line = x2 + r * math.cos(theta)
y_line = y2 + m1 * x2 + b1
y_final = y_line + r * math.sin(theta)
x_final = x_line - (y_final - b1)/m1
'''
print(math.degrees(theta))
print(math.degrees(theta_half))
print(math.degrees(new_x_angle))
print(x_final)
print(y_final)
plt.plot([0, x_final], [10, y_final])
plt.show()
