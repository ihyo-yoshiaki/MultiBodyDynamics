import numpy as np
import matplotlib.pyplot as plt
from Joint import RevJoint
from Body import Body, Bar, Rectangle
from body_joint_sysytem import MultiBodyDynamics2D

body1 = Bar(m=1., x=0., y=0., theta=0., l=1.)
body2 = Bar(m=1., x=1., y=1., theta=np.pi/3, l=1.)
body3 = Rectangle(m=1., x=2., y=1., theta=np.pi*1.5, a=1., b=2.)
loc1 = np.array([[0.], [0.]])
loc2 = np.array([[1.], [1.]])

rev_joint = RevJoint(body1=body1, body2=body2, loc1=loc1, loc2=loc2)
C, Cs = rev_joint.get_constraint()

system = MultiBodyDynamics2D(body1, body2)
system.set_joint(i=0, j=1, joint=RevJoint, 
                 loc_i=np.array([[1.], [0.]], dtype="float"),
                 loc_j=np.array([[1.], [0.]], dtype="float"))
system.draw_ax()
system.add_body(body3)
system.draw_ax()