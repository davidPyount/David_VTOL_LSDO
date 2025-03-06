import numpy as np 
import matplotlib.pyplot as plt 
import csdl_alpha as csdl
import aframe as af
from modopt import CSDLAlphaProblem
from modopt import SLSQP
from VortexAD.core.geometry.gen_vlm_mesh import gen_vlm_mesh
from VortexAD.core.vlm.vlm_solver import vlm_solver

# flow parameters
frame = 'caddee'
vnv_scaler =  1.
num_nodes = 1
alpha = np.array([5.,]) * np.pi/180.
V_inf = np.array([-60, 0., 0.])
if frame == 'caddee':
    V_inf *= -1.
    vnv_scaler = -1.

# grid setup
ns = 11 #Node numbers
nc = 3
b = 10
c = 1
# nc, ns = 11, 15

# generating mesh
mesh_orig = gen_vlm_mesh(ns, nc, b, c, frame=frame)
mesh = np.zeros((num_nodes,) + mesh_orig.shape)
for i in range(num_nodes):
    mesh[i,:,:,:] = mesh_orig

# setting up mesh velocity
V_rot_mat = np.zeros((3,3))
V_rot_mat[1,1] = 1.
V_rot_mat[0,0] = V_rot_mat[2,2] = np.cos(alpha)
V_rot_mat[2,0] = np.sin(alpha)
V_rot_mat[0,2] = -np.sin(alpha)
V_inf_rot = np.matmul(V_rot_mat, V_inf)

mesh_velocity = np.zeros_like(mesh)
mesh_velocity[:,:,:,0] = V_inf_rot[0]
mesh_velocity[:,:,:,2] = V_inf_rot[2]

# solver input setup
recorder = csdl.Recorder(inline=True)
recorder.start()

mesh = csdl.Variable(value=mesh)
mesh_velocity = csdl.Variable(value=mesh_velocity)
mesh_list = [mesh]
mesh_velocity_list = [mesh_velocity]

# alpha_ML = np.ones((num_nodes, ns-1)) * -5*np.pi/180.
# alpha_ML = None

# THIS IS WHERE WE GET THE FORCES I THINK!!!!!!!!
output_vg = vlm_solver(mesh_list, mesh_velocity_list)
wing_CL = output_vg.surface_CL[0]


# print("Lenth")
# print(len(mesh_list))
summedCW = csdl.Variable(value = np.zeros((ns-1,3)))
#first is 0?, second is 0?, 3rd is which panel in chord, which panel in span, that is force vector

#we want to go through and add all cordwise forces, so a[0][0][1][:] + a[0][0][2][:]
for i in range(ns-1):
    for j in range(nc-1): #THIS IS BROKEN RN FIX THIS LOOK AT SET IN DOCKS
        summedCW = summedCW.set[i,:] += output_vg.surface_panel_forces[0][0][j][i]

## A FRAME ===============================================================
# start recorder
# recorder = csdl.Recorder(inline=True)
# recorder.start()

# create a 1D beam 1 mesh
#ns = 11, so num_nodes_1 = 10
num_nodes_1 = ns - 1
beam_1_mesh = np.zeros((num_nodes_1, nc)) #Something here with nc is causing shape errors if nc isnt 3 :(
beam_1_mesh[:, 1] = np.linspace(0, b, num_nodes_1)
beam_1_mesh = csdl.Variable(value=beam_1_mesh)

# num_nodes_1 = 21
# beam_1_mesh = np.zeros((num_nodes_1, 3))
# beam_1_mesh[:, 1] = np.linspace(0, 10, num_nodes_1)
# beam_1_mesh = csdl.Variable(value=beam_1_mesh)

# create beam 1 loads
beam_1_loads = np.zeros((num_nodes_1, 6))
beam_1_loads[:,:3] = summedCW
print(beam_1_loads)
print("test3")
beam_1_loads = csdl.Variable(value=beam_1_loads)

# create a material
aluminum = af.Material(name='aluminum', E=69E9, G=26E9, density=2700)

# create cs properties for beam 1
beam_1_radius = csdl.Variable(value=np.ones(num_nodes_1 - 1) * 0.5)
beam_1_radius.set_as_design_variable(lower=1E-3, scaler=1E1)
beam_1_thickness = csdl.Variable(value=np.ones(num_nodes_1 - 1) * 0.001)
beam_1_cs = af.CSTube(radius=beam_1_radius, thickness=beam_1_thickness)

# create beam 1 with boundary conditions and loads
beam_1 = af.Beam(name='beam_1', mesh=beam_1_mesh, material=aluminum, cs=beam_1_cs)
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
beam_1_def_mesh = beam_1_mesh + beam_1_displacement
beam_1_displacement.set_as_constraint(upper=0.1)

# get the cg of the beam
cg = beam_1.cg

# set a min mass objective
mass = beam_1.mass
mass.set_as_objective()

recorder.stop()

# py_sim = csdl.experimental.PySimulator(
#         recorder=recorder,
#     )   
#     py_sim.check_totals(ofs=csdl.average(vlm_outputs.AIC_force_eval_pts), wrts=elevator_deflection)

from csdl_alpha.experimental import PySimulator

py_sim = PySimulator(
    recorder=recorder
)
py_sim.check_totals(ofs=wing_CL, wrts=mesh)

# recorder.print_graph_structure()
# recorder.visualize_graph(filename='ex1_oas_graph')

wing_CL = output_vg.surface_CL[0].value
wing_CDi = output_vg.surface_CDi[0].value

wing_CL_OAS = np.array([0.4426841725811703]) * vnv_scaler
wing_CDi_OAS = np.array([0.005878842561184834]) * vnv_scaler

CL_error = (wing_CL_OAS - wing_CL)/(wing_CL_OAS) * 100
CDi_error = (wing_CDi_OAS - wing_CDi)/(wing_CDi_OAS) * 100

print('======== ERROR PERCENTAGES (OAS - VortexAD) ========')
print(f'CL error (%): {CL_error}')
print(f'CDi error (%): {CDi_error}')

print('======  PRINTING TOTAL OUTPUTS ======')
print('Total force (N): ', output_vg.total_force.value)
print('Total Moment (Nm): ', output_vg.total_moment.value)
print('Total lift (N): ', output_vg.total_lift.value)
print('Total drag (N): ', output_vg.total_drag.value)

print('======  PRINTING OUTPUTS PER SURFACE ======')
for i in range(len(mesh_list)): # LOOPING THROUGH NUMBER OF SURFACES
    print('======  SURFACE 1 ======')
    print('Surface total force (N): ', output_vg.surface_force[i].value)
    print('Surface total moment (Nm): ', output_vg.surface_moment[i].value)
    print('Surface total lift (N): ', output_vg.surface_lift[i].value)
    print('Surface total drag (N): ', output_vg.surface_drag[i].value)
    print('Surface CL: ', output_vg.surface_CL[i].value)
    print('Surface CDi : ', output_vg.surface_CDi[i].value)

    print('Surface panel forces (N): ', output_vg.surface_panel_forces[i].value)
    print('Surface sectional center of pressure (m): ', output_vg.surface_sectional_cop[i].value)
    print('Surface total center of pressure (m): ', output_vg.surface_cop[i].value)


## A FRAME POST RECORDER STOP
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
plt.plot(np.linspace(0, 10, num_nodes_1)[:-1], beam_1_radius.value, color='black', linewidth=2)
plt.scatter(np.linspace(0, 10, num_nodes_1)[:-1], beam_1_radius.value, zorder=10, edgecolor='black', s=50, color='green')
plt.xlabel('beam length (m)')
plt.ylabel('radius (m)')
plt.title('Optimized Beam Radius')
plt.show()