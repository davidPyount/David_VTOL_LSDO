import csdl_alpha as csdl
import numpy as np
import aframe as af
from modopt import CSDLAlphaProblem
from modopt import SLSQP

recorder = csdl.Recorder(inline=True)
recorder.start()

num_beam_nodes = 9

# Define the beam node coordinates
beam_nodes = csdl.Variable(
    value=np.array([
        [-3.53927E+00,2.96057E-05,-2.63734E+00],
        [-3.55440E+00,9.24183E-01,-2.60233E+00],
        [-3.57151E+00,1.88572E+00,-2.56586E+00],
        [-3.58867E+00,2.84852E+00,-2.52935E+00],
        [-3.60592E+00,3.81277E+00,-2.49278E+00],
        [-3.62204E+00,4.77378E+00,-2.45601E+00],
        [-3.64281E+00,5.73453E+00,-2.41846E+00],
        [-3.67890E+00,6.69733E+00,-2.38051E+00],
        [-3.92518E+00,7.69578E+00,-2.32876E+00],
    ])
)

# Define the box beam cross-section thicknesses per element
tbot = csdl.Variable(value=np.array([
    4.7226865E-03,
    4.3909194E-03,
    3.9121813E-03,
    3.3915973E-03,
    2.9390926E-03,
    2.0629110E-03,
    1.3926143E-03,
    3.0000000E-04,
]))
ttop = csdl.Variable(value=np.array([
    8.298922E-03,
    7.689502E-03,
    6.751313E-03,
    5.704943E-03,
    4.375919E-03,
    2.971346E-03,
    1.925665E-03,
    3.000000E-04,
]))
tweb = csdl.Variable(value=3.00E-04 * np.ones((num_beam_nodes-1, )))
tbot.set_as_design_variable(lower=1E-5, scaler=1E3)
ttop.set_as_design_variable(lower=1E-5, scaler=1E3)

# The outer cross-sectional box dimensions
width = csdl.Variable(value=np.array([
    8.087064E-01,
    7.670528E-01,
    7.242612E-01,
    6.813902E-01,
    6.374276E-01,
    5.865347E-01,
    5.257780E-01,
    2.467123E-01,
]))

height = csdl.Variable(value=np.array([
    2.6502895E-01,
    2.5133992E-01,
    2.3728960E-01,
    2.2321557E-01,
    2.0880246E-01,
    1.9208410E-01,
    1.7181877E-01,
    1.0655984E-01,
]))

# Nodal forces
beam_forces = np.array([
    [0,0,-4.005064E+03],
    [0,0,-6.741551E+03],
    [0,0,-6.970990E+03],
    [0,0,-6.610402E+03],
    [0,0,-6.139581E+03],
    [0,0,-5.558692E+03],
    [0,0,-4.848959E+03],
    [0,0,-3.685410E+03],
    [0,0,-1.475417E+03],
])

beam_loads = np.zeros((num_beam_nodes, 6))
beam_loads[:, 0:3] = beam_forces

beam_loads_csdl = csdl.Variable(value=beam_loads)

beam_cs = af.CSBox(height=height, width=width, ttop=ttop, tbot=tbot, tweb=tweb)

aluminum = af.Material(name='aluminum', E=69E9, G=26E9, density=2700)
beam = af.Beam(name='beam', mesh=beam_nodes, material=aluminum, cs=beam_cs)
beam.fix(node=0)
beam.add_load(beam_loads_csdl)

frame = af.Frame(beams=[beam])


frame.solve()

# Mass
mass = beam.mass
mass.set_as_objective()

# displacement
beam_displacement = frame.displacement[beam.name]
max_displacement = csdl.maximum(csdl.absolute(beam_displacement), rho=100.0)
max_displacement.set_as_constraint(upper=0.2)

# stress
stress = frame.compute_stress()
beam_stress = stress[beam.name]
print(beam_stress.value)

recorder.stop()
recorder.execute()
old_beam_displacement = beam_displacement.value.copy()
old_beam_thickness_top = ttop.value.copy()
old_beam_thickness_bot = tbot.value.copy()

# Optimize:
sim = csdl.experimental.JaxSimulator(recorder=recorder)
prob = CSDLAlphaProblem(problem_name='box_beam_opt', simulator=sim)
optimizer = SLSQP(prob, solver_options={'maxiter': 3000, 'ftol': 1e-6, 'disp': True}, turn_off_outputs=True)
optimizer.solve()
optimizer.print_results()
recorder.execute()

print("new displacement", beam_displacement.value)
print("old displacement", old_beam_displacement)
print("max stress", np.max(beam_stress.value, axis=1))

# Plot Displacements
import matplotlib.pyplot as plt
plt.plot(old_beam_displacement[:,0],label = 'original x', color='red', linestyle='dashed')
plt.plot(old_beam_displacement[:,1],label = 'original y', color='green', linestyle='dashed')
plt.plot(old_beam_displacement[:,2],label = 'original z', color='blue', linestyle='dashed')
plt.plot(beam_displacement.value[:,0],label = 'optimized x', color='red')
plt.plot(beam_displacement.value[:,1],label = 'optimized y', color='green')
plt.plot(beam_displacement.value[:,2],label = 'optimized z', color='blue')
plt.legend()
plt.title('Beam Displacement')

# 3D plot of the beam
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(beam_nodes.value[:, 0], beam_nodes.value[:, 1], beam_nodes.value[:, 2], label='Original Beam', color='black')
ax.plot(beam_nodes.value[:, 0] + old_beam_displacement[:, 0], beam_nodes.value[:, 1] + old_beam_displacement[:, 1], beam_nodes.value[:, 2] + old_beam_displacement[:, 2], label='Original Beam Displaced', color='red', linestyle='dashed')
ax.plot(beam_nodes.value[:, 0] + beam_displacement.value[:, 0], beam_nodes.value[:, 1] + beam_displacement.value[:, 1], beam_nodes.value[:, 2] + beam_displacement.value[:, 2], label='Optimized Beam Displaced', color='red')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Beam Displacement')
ax.legend()
ax.set_xlim(-0, -10)
ax.set_ylim(0, 10)

# Plot the thicknesses
fig, ax = plt.subplots()
ax.plot(np.linspace(0, 1, num_beam_nodes-1), old_beam_thickness_bot, label='Bottom', color='red', linestyle='dashed')
ax.plot(np.linspace(0, 1, num_beam_nodes-1), old_beam_thickness_top, label='Top', color='green', linestyle='dashed')
ax.plot(np.linspace(0, 1, num_beam_nodes-1), tbot.value, label='Bottom Optimized', color='red')
ax.plot(np.linspace(0, 1, num_beam_nodes-1), ttop.value, label='Top Optimized', color='green')
ax.set_xlabel('Beam Length (normalized)')
ax.set_ylabel('Thickness (m)')
ax.set_title('Beam Thicknesses')
ax.legend()
plt.show()
