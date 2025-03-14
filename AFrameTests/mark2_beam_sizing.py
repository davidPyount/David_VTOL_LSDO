# This code optimizes the diameter for the wing booms in Mark 2, assuming a 5g load distributed evenly between two spars
# Lift distribution is obtained from an XFLR5 analysis on the wing at cruise conditions 

import csdl_alpha as csdl
import numpy as np
import aframe as af
from modopt import CSDLAlphaProblem
from modopt import SLSQP
import matplotlib.pyplot as plt

# start recorder
recorder = csdl.Recorder(inline=True)
recorder.start()

# create a 1D beam 1 mesh
num_nodes_1 = 15

beam_len = 2*0.3048 # [m]
beam_lens = np.linspace(0,beam_len,num_nodes_1)

beam_1_mesh = np.zeros((num_nodes_1, 3))
beam_1_mesh[:, 1] = np.linspace(0, beam_len, num_nodes_1)
beam_1_mesh = csdl.Variable(value=beam_1_mesh)

# create beam 1 loads

SF = 1.5
lift_dist = np.array([-1.0284, 0, -0.6833, 0, 0.4557]) * 5/2 # 5g turn distributed evenly between each spar

rho = 1.225
v = 70*.3048
c = 9/12*.3048
delta = beam_len/num_nodes_1

beam_1_loads = np.zeros((num_nodes_1, 6))
Cls = np.zeros((num_nodes_1, 1))
for i in range(num_nodes_1 - 1):
    Cls[i] = lift_dist[0]*beam_lens[i]**4 + lift_dist[1]*beam_lens[i]**3 + lift_dist[2]*beam_lens[i]**2 + lift_dist[3]*beam_lens[i] + lift_dist[4]
    beam_1_loads[i,2] = Cls[i] * 1/2*rho*v**2*c*delta

beam_1_loads = csdl.Variable(value=beam_1_loads)

# create a material
# from https://www.clearwatercomposites.com/resources/properties-of-carbon-fiber/
# and https://www.matweb.com/search/datasheet_print.aspx?matguid=39e40851fc164b6c9bda29d798bf3726
carbon_fiber = af.Material(name='carbon_fiber', E=96.2E9/SF, G = 3.16E9/SF, density = 1420)
# flex_yield = 973 # [MPa]
# flex_mod = 98000; # [MPa]
# strain_yield = flex_yield/flex_mod


# create cs properties for beam 1
# beam_1_radius = csdl.Variable(value=np.ones(num_nodes_1 - 1) * 0.5)
beam_1_radius = csdl.Variable(value=0.1)
beam_1_radius.set_as_design_variable(lower=0.0015875, scaler=1E3) # needs to be larger than 1/8" for wiring purposes
beam_1_radius_expanded = csdl.expand(beam_1_radius, (num_nodes_1 - 1,))
# beam_1_thickness = csdl.Variable(value=np.ones(num_nodes_1 - 1) * 0.001)
beam_1_thickness = csdl.Variable(value=0.0015875) # beam_1_cs = af.CSTube(radius=beam_1_radius, thickness=beam_1_thickness)
beam_1_thickness_expanded = csdl.expand(beam_1_thickness, (num_nodes_1 - 1,))
beam_1_cs = af.CSTube(radius=beam_1_radius_expanded, thickness=beam_1_thickness_expanded)

# create beam 1 with boundary conditions and loads
beam_1 = af.Beam(name='beam_1', mesh=beam_1_mesh, material=carbon_fiber, cs=beam_1_cs)
beam_1.fix(node=0)
beam_1.add_load(beam_1_loads)

# acceleration (optional)
acc = csdl.Variable(value=np.array([0, 0, -9.81, 0, 0, 0]))

# instantiate the frame model and add all beams and joints and accelerations
frame = af.Frame(beams=[beam_1], acc=acc)

# solve the linear system
frame.solve()

# get the displacement
beam_1_displacement = frame.displacement[beam_1.name]

# displaced mesh

displacement_limit = beam_len*0.05

beam_1_def_mesh = beam_1_mesh + beam_1_displacement
beam_1_displacement.set_as_constraint(lower=-displacement_limit,upper=displacement_limit)

# get the cg of the beam
cg = beam_1.cg

# set a min mass objective
mass = beam_1.mass
mass.set_as_objective()

# finish up
recorder.stop()

# sim = csdl.experimental.PySimulator(recorder)
sim = csdl.experimental.JaxSimulator(recorder=recorder)
prob = CSDLAlphaProblem(problem_name='single_beam_opt', simulator=sim)
optimizer = SLSQP(prob, solver_options={'maxiter': 300, 'ftol': 1e-6, 'disp': True}, turn_off_outputs=True)
optimizer.solve()
optimizer.print_results()
recorder.execute()

print(cg.value)
print(beam_1_displacement.value)

plt.figure()
plt.grid()
plt.plot(beam_1_def_mesh.value[:, 1], beam_1_def_mesh.value[:, 2], color='black', linewidth=2)
plt.scatter(beam_1_def_mesh.value[:, 1], beam_1_def_mesh.value[:, 2], zorder=10, edgecolor='black', s=50, color='green')
plt.xlabel('y (m)')
plt.ylabel('z (m)')
plt.title('Optimized Beam Displacement')

plt.figure()
plt.grid()
plt.plot(np.linspace(0, beam_len, num_nodes_1)[:-1], beam_1_radius.value*np.ones((num_nodes_1))[:-1], color='black', linewidth=2)
plt.scatter(np.linspace(0, beam_len, num_nodes_1)[:-1], beam_1_radius.value*np.ones((num_nodes_1))[:-1], zorder=10, edgecolor='black', s=50, color='green')
# plt.plot(np.linspace(0, 10, num_nodes_1)[:-1], beam_1_radius.value, color='black', linewidth=2)
# plt.scatter(np.linspace(0, 10, num_nodes_1)[:-1], beam_1_radius.value, zorder=10, edgecolor='black', s=50, color='green')
plt.xlabel('beam length (m)')
plt.ylabel('radius (m)')
plt.title('Optimized Beam Radius')
plt.show()