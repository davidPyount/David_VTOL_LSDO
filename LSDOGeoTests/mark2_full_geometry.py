import time
import lsdo_geo
import lsdo_function_spaces as lfs
lfs.num_workers = 1
# t01 = time.time()
import csdl_alpha as csdl
import numpy as np
# from python_csdl_backend import Simulator
import lsdo_geo as lg

# t02 = time.time()
# print(t02-t01)
recorder = csdl.Recorder(inline=True)
recorder.start()

import_file_path = 'c:/Users/seth3/David_VTOL_LSDO/LSDOGeoTests/'
import_file = 'mark2_full_geometry.stp'
geometry = lg.import_geometry(import_file_path + import_file, parallelize=False)

# geometry.plot()

# Specify the names that are given to each component within OpenVSP
wing_name = 'Wing'
h_tail_name = 'HStab'
v_tail_name = 'VStab'
fuselage_name = 'Fuselage'
nose_name = 'Nosecone'
cruise_prop_name = 'Main Propeller'
front_left_prop_name = 'Front Left Lift Rotor'
rear_left_prop_name = 'Rear Left Lift Rotor'
front_right_prop_name = 'Front Right Lift Rotor'
rear_right_prop_name = 'Rear Right Lift Rotor'

left_boom_name = 'Left Boom'
right_boom_name = 'RightBoom'

# Specify the coordinates of the components

# Do the x,y,z need to be incorporate incidence on the wing that results in the TE and LE not being in the same plane?

x_wing_LE = 0.25
y_wing_LE = 0
z_wing_LE = 0.25
wing_chord = 0.75
wing_span = 4
x_wing_TE = x_wing_LE + wing_chord
x_wing_qc = x_wing_LE + wing_chord/4

x_h_tail_LE = 2.375
y_h_tail_LE = 0
z_h_tail_LE = 1/12
h_tail_chord = 5/12
h_tail_span = 1.25
x_h_tail_TE = x_h_tail_LE + h_tail_chord
x_h_tail_qc = x_h_tail_LE + h_tail_chord/4

x_v_tail_LE = 2.375
y_v_tail_LE = 0
z_v_tail_LE = 1/12
v_tail_chord = 5/12
v_tail_span = 1.08
x_v_tail_TE = x_v_tail_LE + v_tail_chord
x_v_tail_qc = x_v_tail_LE + v_tail_chord/4

boom_length = 3
x_boom_front = (x_wing_qc) - boom_length/2
x_boom_rear = x_boom_front + 3
y_boom = 1.5
z_boom = 0.2

lift_rotor_d = 14/12
z_lift_rotor = z_boom + 0.1 # approximate

x_nose_tip = -0.5
z_nose_tip = 0.125

fuselage_length = 1.25

# region Declaring all components
# Wing, tails, fuselage
wing = geometry.declare_component(function_search_names=[wing_name], name='wing')
# wing.plot()
h_tail = geometry.declare_component(function_search_names=[h_tail_name], name='h_tail')
# h_tail.plot()
v_tail = geometry.declare_component(function_search_names=[v_tail_name], name='v_tail')
# v_tail.plot()
fuselage = geometry.declare_component(function_search_names=[fuselage_name], name='fuselage')
# fuselage.plot()

# Nose hub
nose_hub = geometry.declare_component(name='nose', function_search_names=[nose_name])
# nose_hub.plot()

# ? Cannot get OpenVSP stp to export disk and blades separately or distinguish between blades

cruise_disk = geometry.declare_component(name='cruise_disk', function_search_names=[cruise_prop_name])
cruise_components = [cruise_disk]

rl_disk = geometry.declare_component(name='rl_disk', function_search_names=[rear_left_prop_name])
rl_components = [rl_disk]

rr_disk = geometry.declare_component(name='rr_disk', function_search_names=[rear_right_prop_name])
rr_components = [rr_disk]

fl_disk = geometry.declare_component(name='fl_disk', function_search_names=[front_left_prop_name])
fl_components = [fl_disk]

fr_disk = geometry.declare_component(name='fr_disk', function_search_names=[front_right_prop_name])
fr_components = [fr_disk]

# Cruise prop
# cruise_disk = geometry.declare_component(name='cruise_disk', function_search_names=[cruise_prop_name + '-disk'])
# cruise_disk.plot()
# cruise_blade_1 = geometry.declare_component(name='cruise_blade_1', function_search_names=[cruise_prop_name + '_blades, 0'])
# cruise_blade_1.plot()
# cruise_blade_2 = geometry.declare_component(name='cruise_blade_2', function_search_names=[cruise_prop_name + '_blades, 1'])
# cruise_blade_2.plot()

# Rotor: rear left
# rl_disk = geometry.declare_component(name='rl_disk', function_search_names=[rear_left_prop_name + '_disk'])
# rlo_disk.plot()
# rl_blade_1 = geometry.declare_component(name='rl_blade_1', function_search_names=[rear_left_prop_name + '_blades, 0'])
# rlo_blade_1.plot()
# rl_blade_2 = geometry.declare_component(name='rl_blade_2', function_search_names=[rear_left_prop_name + '_blades, 1'])
# rl_components = [rl_disk, rl_blade_1, rl_blade_2]

# Rotor: rear right
# rr_disk = geometry.declare_component(name='rr_disk', function_search_names=[rear_right_prop_name + '_disk'])
# rri_disk.plot()
# rr_blade_1 = geometry.declare_component(name='rr_blade_1', function_search_names=[rear_right_prop_name + '_blades, 0'])
# rri_blade_1.plot()
# rr_blade_2 = geometry.declare_component(name='rr_blade_2', function_search_names=[rear_right_prop_name + 'blades, 1'])
# rri_blade_2.plot()
# rr_components = [rr_disk, rr_blade_1, rr_blade_2]

# Rotor: front left
# fl_disk = geometry.declare_component(name='fl_disk', function_search_names=[front_left_prop_name + '_disk'])
# flo_disk.plot()
# fl_blade_1 = geometry.declare_component(name='fl_blade_1', function_search_names=[front_left_prop_name + '_blades, 0'])
# flo_blade_1.plot()
# fl_blade_2 = geometry.declare_component(name='fl_blade_2', function_search_names=[front_left_prop_name + '_blades, 1'])
# flo_blade_2.plot()

# Rotor: front right
# fri_disk.plot()
# fr_blade_1 = geometry.declare_component(name='fr_blade_1', function_search_names=[front_right_prop_name + '_blades, 0'])
# fri_blade_1.plot()
# fr_blade_2 = geometry.declare_component(name='fr_blade_2', function_search_names=[front_right_prop_name + '_blades, 1'])
# fri_blade_2.plot()

lift_rotor_related_components = [rl_components, rr_components, 
                                 fl_components, fr_components]

# left boom
left_boom = geometry.declare_component(name='left_boom', function_search_names=[left_boom_name])

# right boom
right_boom = geometry.declare_component(name='right_boom', function_search_names=right_boom_name)

boom_components = [left_boom, right_boom]

# endregion

# region Defining key points
# wing
wing_le_left = wing.project(np.array([x_wing_LE, -y_wing_LE, z_wing_LE]), plot=False)
wing_le_right = wing.project(np.array([x_wing_LE, y_wing_LE, z_wing_LE]), plot=False)
wing_le_center = wing.project(np.array([x_wing_LE, 0, z_wing_LE]), plot=False)
wing_te_left = wing.project(np.array([x_wing_TE, -y_wing_LE, z_wing_LE]), plot=False)
wing_te_right = wing.project(np.array([x_wing_TE, y_wing_LE, z_wing_LE]), plot=False)
wing_te_center = wing.project(np.array([x_wing_TE, 0, z_wing_LE]), plot=False)
wing_qc = wing.project(np.array([x_wing_qc, 0, z_wing_LE]), plot=False)

# h-tail
h_tail_le_left = h_tail.project(np.array([x_h_tail_LE, -y_h_tail_LE, z_h_tail_LE]), plot=False)
h_tail_le_right = h_tail.project(np.array([x_h_tail_LE, y_h_tail_LE, z_h_tail_LE]), plot=False)
h_tail_le_center = h_tail.project(np.array([x_h_tail_LE, 0., z_h_tail_LE]), plot=False)
h_tail_te_left = h_tail.project(np.array([x_h_tail_LE, -y_h_tail_LE, z_h_tail_LE]), plot=False)
h_tail_te_right = h_tail.project(np.array([x_h_tail_LE, y_h_tail_LE, z_h_tail_LE]), plot=False)
h_tail_te_center = h_tail.project(np.array([x_h_tail_LE, 0., z_h_tail_LE]), plot=False)
h_tail_qc = h_tail.project(np.array([x_h_tail_qc, 0., z_h_tail_LE]), plot=False)

# v-tail
v_tail_le_right = v_tail.project(np.array([x_v_tail_LE, y_v_tail_LE, z_v_tail_LE]), plot=False)
v_tail_le_left = v_tail.project(np.array([x_v_tail_LE, -y_v_tail_LE, z_v_tail_LE]), plot=False)
v_tail_le_center = v_tail.project(np.array([x_v_tail_LE, 0., z_v_tail_LE]), plot=False)
v_tail_te_right = v_tail.project(np.array([x_v_tail_LE, y_v_tail_LE, z_v_tail_LE]), plot=False)
v_tail_te_left = v_tail.project(np.array([x_v_tail_LE, -y_v_tail_LE, z_v_tail_LE]), plot=False)
v_tail_te_center = v_tail.project(np.array([x_v_tail_LE, 0., z_v_tail_LE]), plot=False)
v_tail_qc = v_tail.project(np.array([x_v_tail_qc, 0., z_v_tail_LE]), plot=False)

# Do we need these?
fuselage_wing_qc = fuselage.project(np.array([10.25, 0., 8.5]), plot=False)
fuselage_wing_te_center = fuselage.project(np.array([14.332, 0., 8.439]), plot=False)
fuselage_tail_qc = fuselage.project(np.array([24.15, 0., 8.]), plot=False)
fuselage_tail_te_center = fuselage.project(np.array([31.187, 0., 8.009]), plot=False)

# booms
# left_boom_front_tip = left_boom.project(np.array([x_boom_front,-y_boom,z_boom]), plot=False)
# left_boom_rear_tip = left_boom.project(np.array([x_boom_rear,-y_boom,z_boom]), plot=False)
# right_boom_front_tip = right_boom.project(np.array([x_boom_front,y_boom,z_boom]), plot=False)
# right_boom_rear_tip = right_boom.project(np.array([x_boom_rear,y_boom,z_boom]), plot=False)
left_boom_front_tip = np.array([x_boom_front,-y_boom,z_boom])
left_boom_rear_tip = np.array([x_boom_rear,-y_boom,z_boom])
right_boom_front_tip = np.array([x_boom_front,y_boom,z_boom])
right_boom_rear_tip = np.array([x_boom_rear,y_boom,z_boom])


# currently defining these points in terms of the boom instead of giving them their own values (except z)
# pt means point?
rl_disk_pt = np.array([x_boom_rear, -y_boom, z_lift_rotor])
rr_disk_pt = np.array([x_boom_rear, y_boom, z_lift_rotor])
fl_disk_pt = np.array([x_boom_front, -y_boom, z_lift_rotor])
fr_disk_pt = np.array([x_boom_front, y_boom, z_lift_rotor])

# rl_disk_center = rlo_disk.project(rlo_disk_pt)
# rr_disk_center = rri_disk.project(rri_disk_pt)
# fl_disk_center = flo_disk.project(flo_disk_pt)
# fr_disk_center = fri_disk.project(fri_disk_pt)

# rl_disk_center_on_wing = wing.project(rl_disk_pt)
# rr_disk_center_on_wing = wing.project(rr_disk_pt)
# fl_disk_center_on_wing = wing.project(fl_disk_pt)
# fr_disk_center_on_wing = wing.project(fr_disk_pt)

boom_fl = left_boom.project(left_boom_front_tip)
boom_rl = left_boom.project(left_boom_rear_tip)
boom_fr = right_boom.project(right_boom_front_tip)
boom_rr = right_boom.project(right_boom_rear_tip)

wing_boom_fl = wing.project(left_boom_front_tip)
wing_boom_fr = wing.project(right_boom_front_tip)
wing_boom_rl = wing.project(left_boom_rear_tip)
wing_boom_rr = wing.project(right_boom_rear_tip)

# Do we need these?
# fuselage_nose = np.array([2.464, 0., 5.113])
# fuselage_rear = np.array([31.889, 0., 7.798])
# fuselage_nose_points_parametric = fuselage.project(fuselage_nose, grid_search_density_parameter=20)
# fuselage_rear_points_parametric = fuselage.project(fuselage_rear)
# fuselage_rear_point_on_pusher_disk_parametric = cruise_disk.project(fuselage_rear)
fuselage_nose = np.array([x_nose_tip, 0., z_nose_tip])
fuselage_rear = np.array([0, 0., fuselage_length])
fuselage_nose_points_parametric = fuselage.project(fuselage_nose, grid_search_density_parameter=20)
fuselage_rear_points_parametric = fuselage.project(fuselage_rear)
fuselage_nose_point_on_cruise_propeller_disk_parametric = cruise_disk.project(fuselage_nose)

# endregion

rl_disk_y1_para = rl_disk.project(np.array([x_boom_rear, -y_boom + lift_rotor_d/2, z_lift_rotor]))
rl_disk_y2_para = rl_disk.project(np.array([x_boom_rear, -y_boom - lift_rotor_d/2, z_lift_rotor]))

rr_disk_y1_para = rr_disk.project(np.array([x_boom_rear, y_boom + lift_rotor_d/2, z_lift_rotor]))
rr_disk_y2_para = rr_disk.project(np.array([x_boom_rear, y_boom - lift_rotor_d/2, z_lift_rotor]))

fl_disk_y1_para = fl_disk.project(np.array([x_boom_front, -y_boom + lift_rotor_d/2, z_lift_rotor]))
fl_disk_y2_para = fl_disk.project(np.array([x_boom_front, -y_boom - lift_rotor_d/2, z_lift_rotor]))

fr_disk_y1_para = fr_disk.project(np.array([x_boom_front, y_boom + lift_rotor_d/2, z_lift_rotor]))
fr_disk_y2_para = fr_disk.project(np.array([x_boom_front, y_boom - lift_rotor_d/2, z_lift_rotor]))

rotor_edges = [(rl_disk_y1_para, rl_disk_y2_para), (rl_disk_y1_para, rl_disk_y2_para),
                (rr_disk_y1_para, rr_disk_y2_para), (rr_disk_y1_para, rr_disk_y2_para)]
# # endregion

# region Projection for meshes
# region Wing camber mesh
wing_num_spanwise_vlm = 23
wing_num_chordwise_vlm = 5

# Is it OK for the line to be defined using the points I have chosen?
leading_edge_line_parametric = wing.project(np.linspace(np.array([x_wing_LE, -y_wing_LE, z_wing_LE + 1]), np.array([x_wing_LE, y_wing_LE, z_wing_LE + 1]), wing_num_spanwise_vlm), 
                                 direction=np.array([0., 0., -1.]), grid_search_density_parameter=20.)
trailing_edge_line_parametric = wing.project(np.linspace(np.array([x_wing_TE, -y_wing_LE, z_wing_LE + 1]), np.array([x_wing_TE, y_wing_LE, z_wing_LE + 1]), wing_num_spanwise_vlm), 
                                  direction=np.array([0., 0., -1.]), grid_search_density_parameter=20.)
leading_edge_line = geometry.evaluate(leading_edge_line_parametric)
trailing_edge_line = geometry.evaluate(trailing_edge_line_parametric)
chord_surface = csdl.linear_combination(leading_edge_line, trailing_edge_line, wing_num_chordwise_vlm)
wing_upper_surface_wireframe_parametric = wing.project(chord_surface.value.reshape((wing_num_chordwise_vlm,wing_num_spanwise_vlm,3))+np.array([0., 0., 1.]), 
                                       direction=np.array([0., 0., -1.]), plot=False, grid_search_density_parameter=10.)
wing_lower_surface_wireframe_parametric = wing.project(chord_surface.value.reshape((wing_num_chordwise_vlm,wing_num_spanwise_vlm,3))+np.array([0., 0., -1.]), 
                                       direction=np.array([0., 0., 1.]), plot=False, grid_search_density_parameter=10.)
upper_surface_wireframe = geometry.evaluate(wing_upper_surface_wireframe_parametric)
lower_surface_wireframe = geometry.evaluate(wing_lower_surface_wireframe_parametric)
wing_camber_surface = csdl.linear_combination(upper_surface_wireframe, lower_surface_wireframe, 1).reshape((wing_num_chordwise_vlm, wing_num_spanwise_vlm, 3))
# endregion Wing camber mesh

# region Htail camber mesh
h_tail_num_spanwise_vlm = 11
h_tail_num_chordwise_vlm = 4
leading_edge_line_parametric = h_tail.project(np.linspace(np.array([x_h_tail_LE, -y_h_tail_LE, z_h_tail_LE]), np.array([x_h_tail_LE, y_h_tail_LE, z_h_tail_LE]), h_tail_num_spanwise_vlm), 
                                 direction=np.array([0., 0., -1.]), grid_search_density_parameter=20.)
trailing_edge_line_parametric = h_tail.project(np.linspace(np.array([x_h_tail_TE, -y_h_tail_LE, z_h_tail_LE]), np.array([x_h_tail_TE, -y_h_tail_LE, z_h_tail_LE]), h_tail_num_spanwise_vlm), 
                                  direction=np.array([0., 0., -1.]), grid_search_density_parameter=20.)
leading_edge_line = geometry.evaluate(leading_edge_line_parametric)
trailing_edge_line = geometry.evaluate(trailing_edge_line_parametric)
chord_surface = csdl.linear_combination(leading_edge_line, trailing_edge_line, h_tail_num_chordwise_vlm)
h_tail_upper_surface_wireframe_parametric = h_tail.project(chord_surface.value.reshape((h_tail_num_chordwise_vlm,h_tail_num_spanwise_vlm,3))+np.array([0., 0., 1.]), 
                                       direction=np.array([0., 0., -1.]), plot=False, grid_search_density_parameter=20.)
h_tail_lower_surface_wireframe_parametric = h_tail.project(chord_surface.value.reshape((h_tail_num_chordwise_vlm,h_tail_num_spanwise_vlm,3))+np.array([0., 0., -1.]), 
                                       direction=np.array([0., 0., 1.]), plot=False, grid_search_density_parameter=20.)
upper_surface_wireframe = geometry.evaluate(h_tail_upper_surface_wireframe_parametric)
lower_surface_wireframe = geometry.evaluate(h_tail_lower_surface_wireframe_parametric)
h_tail_camber_surface = csdl.linear_combination(upper_surface_wireframe, lower_surface_wireframe, 1).reshape((h_tail_num_chordwise_vlm, h_tail_num_spanwise_vlm, 3))
# endregion Htail camber mesh

# region Wing beam mesh
num_beam_nodes = 13
# wing_qc_right_physical = np.array([12.617, 25.250, 7.5])
# wing_qc_left_physical = np.array([12.617, -25.250, 7.5])
# wing_qc_center_physical = np.array([10.25, 0., 8.5])
wing_qc_right_physical = np.array([12.517, 25.250, 7.5])
wing_qc_left_physical = np.array([12.517, -25.250, 7.5])
wing_qc_center_physical = np.array([10.5, 0., 8.5])

left_physical = np.linspace(wing_qc_left_physical, wing_qc_center_physical, num_beam_nodes//2, endpoint=False)
right_physical = np.linspace(wing_qc_center_physical, wing_qc_right_physical, num_beam_nodes//2+1, endpoint=True)
beam_mesh_physical = np.concatenate((left_physical, right_physical), axis=0)
beam_top_parametric = wing.project(beam_mesh_physical+np.array([0., 0., 1.]), plot=False)
beam_bottom_parametric = wing.project(beam_mesh_physical+np.array([0., 0., -1.]), plot=False)
beam_tops = geometry.evaluate(beam_top_parametric)
beam_bottoms = geometry.evaluate(beam_bottom_parametric)
wing_beam_mesh = csdl.linear_combination(beam_tops, beam_bottoms, 1).reshape((num_beam_nodes, 3))
beam_heights = csdl.norm(beam_tops - beam_bottoms, axes=(1,))
# endregion Wing beam mesh

# # Figure plotting the meshes
# plotting_elements = geometry.plot_meshes([wing_camber_surface, h_tail_camber_surface], function_opacity=0.5, mesh_color='#FFCD00', show=False)
# plotting_elements = geometry.plot_meshes([wing_beam_mesh], mesh_line_width=10, function_opacity=0., additional_plotting_elements=plotting_elements, show=False)
# import vedo
# plotter = vedo.Plotter()
# plotter.show(plotting_elements, axes=0, viewup='z')
# endregion

# region Parameterization

constant_b_spline_curve_1_dof_space = lfs.BSplineSpace(num_parametric_dimensions=1, degree=0, coefficients_shape=(1,))
linear_b_spline_curve_2_dof_space = lfs.BSplineSpace(num_parametric_dimensions=1, degree=1, coefficients_shape=(2,))
linear_b_spline_curve_3_dof_space = lfs.BSplineSpace(num_parametric_dimensions=1, degree=1, coefficients_shape=(3,))
cubic_b_spline_curve_5_dof_space = lfs.BSplineSpace(num_parametric_dimensions=1, degree=3, coefficients_shape=(5,))

# region Parameterization Setup
parameterization_solver = lsdo_geo.ParameterizationSolver()
parameterization_design_parameters = lsdo_geo.GeometricVariables()

# region Wing Parameterization setup
wing_ffd_block = lsdo_geo.construct_ffd_block_around_entities(name='wing_ffd_block', entities=wing, num_coefficients=(2,11,2), degree=(1,3,1))
wing_ffd_block_sectional_parameterization = lsdo_geo.VolumeSectionalParameterization(name='wing_sectional_parameterization',
                                                                            parameterized_points=wing_ffd_block.coefficients,
                                                                            principal_parametric_dimension=1)

wing_chord_stretch_coefficients = csdl.Variable(name='wing_chord_stretch_coefficients', value=np.array([0., 0., 0.]))
wing_chord_stretch_b_spline = lfs.Function(name='wing_chord_stretch_b_spline', space=linear_b_spline_curve_3_dof_space, 
                                          coefficients=wing_chord_stretch_coefficients)

wing_wingspan_stretch_coefficients = csdl.Variable(name='wing_wingspan_stretch_coefficients', value=np.array([-0., 0.]))
wing_wingspan_stretch_b_spline = lfs.Function(name='wing_wingspan_stretch_b_spline', space=linear_b_spline_curve_2_dof_space, 
                                          coefficients=wing_wingspan_stretch_coefficients)

wing_twist_coefficients = csdl.Variable(name='wing_twist_coefficients', value=np.array([0., 0., 0., 0., 0.]))
wing_twist_b_spline = lfs.Function(name='wing_twist_b_spline', space=cubic_b_spline_curve_5_dof_space,
                                          coefficients=wing_twist_coefficients)

wing_translation_x_coefficients = csdl.Variable(name='wing_translation_x_coefficients', value=np.array([0.]))
wing_translation_x_b_spline = lfs.Function(name='wing_translation_x_b_spline', space=constant_b_spline_curve_1_dof_space,
                                          coefficients=wing_translation_x_coefficients)

wing_translation_z_coefficients = csdl.Variable(name='wing_translation_z_coefficients', value=np.array([0.]))
wing_translation_z_b_spline = lfs.Function(name='wing_translation_z_b_spline', space=constant_b_spline_curve_1_dof_space,
                                          coefficients=wing_translation_z_coefficients)

parameterization_solver.add_parameter(parameter=wing_chord_stretch_coefficients)
parameterization_solver.add_parameter(parameter=wing_wingspan_stretch_coefficients, cost=1.e3)
parameterization_solver.add_parameter(parameter=wing_translation_x_coefficients)
parameterization_solver.add_parameter(parameter=wing_translation_z_coefficients)
# endregion Wing Parameterization setup

# region Horizontal Stabilizer setup
h_tail_ffd_block = lsdo_geo.construct_ffd_block_around_entities(name='h_tail_ffd_block', entities=h_tail, num_coefficients=(2,11,2), degree=(1,3,1))
h_tail_ffd_block_sectional_parameterization = lsdo_geo.VolumeSectionalParameterization(name='h_tail_sectional_parameterization',
                                                                            parameterized_points=h_tail_ffd_block.coefficients,
                                                                            principal_parametric_dimension=1)

h_tail_chord_stretch_coefficients = csdl.Variable(name='h_tail_chord_stretch_coefficients', value=np.array([0., 0., 0.]))
h_tail_chord_stretch_b_spline = lfs.Function(name='h_tail_chord_stretch_b_spline', space=linear_b_spline_curve_3_dof_space, 
                                          coefficients=h_tail_chord_stretch_coefficients)

h_tail_span_stretch_coefficients = csdl.Variable(name='h_tail_span_stretch_coefficients', value=np.array([-0., 0.]))
h_tail_span_stretch_b_spline = lfs.Function(name='h_tail_span_stretch_b_spline', space=linear_b_spline_curve_2_dof_space, 
                                          coefficients=h_tail_span_stretch_coefficients)

# h_tail_twist_coefficients = csdl.Variable(name='h_tail_twist_coefficients', value=np.array([0., 0., 0., 0., 0.]))
# h_tail_twist_b_spline = lfs.Function(name='h_tail_twist_b_spline', space=cubic_b_spline_curve_5_dof_space,
#                                           coefficients=h_tail_twist_coefficients)

h_tail_translation_x_coefficients = csdl.Variable(name='h_tail_translation_x_coefficients', value=np.array([0.]))
h_tail_translation_x_b_spline = lfs.Function(name='h_tail_translation_x_b_spline', space=constant_b_spline_curve_1_dof_space,
                                          coefficients=h_tail_translation_x_coefficients)
h_tail_translation_z_coefficients = csdl.Variable(name='h_tail_translation_z_coefficients', value=np.array([0.]))
h_tail_translation_z_b_spline = lfs.Function(name='h_tail_translation_z_b_spline', space=constant_b_spline_curve_1_dof_space,
                                          coefficients=h_tail_translation_z_coefficients)

parameterization_solver.add_parameter(parameter=h_tail_chord_stretch_coefficients)
parameterization_solver.add_parameter(parameter=h_tail_span_stretch_coefficients)
parameterization_solver.add_parameter(parameter=h_tail_translation_x_coefficients)
parameterization_solver.add_parameter(parameter=h_tail_translation_z_coefficients)
# endregion Horizontal Stabilizer setup

# # region Boom setup
# # Why are num_coefficients and degree set to the chosen values?
# # How to couple this value to the length of the boom?
# boom_ffd_block = lsdo_geo.construct_ffd_block_around_entities(name='boom_ffd_block', entities=left_boom, num_coefficients=(2,11,2), degree=(1,3,1))
# boom_ffd_block_sectional_parameterization = lsdo_geo.VolumeSectionalParameterization(name='boom_sectional_parameterization',
#                                                                             parameterized_points=boom_ffd_block.coefficients,
#                                                                             principal_parametric_dimension=1)

# boom_translation_y_coefficients = csdl.Variable(name='boom_translation_y_coefficients', value=np.array([0.]))
# boom_translation_y_b_spline = lfs.Function(name='boom_translation_y_b_spline', space=constant_b_spline_curve_1_dof_space,
#                                           coefficients=boom_translation_y_coefficients)

# parameterization_solver.add_parameter(parameter=boom_translation_y_coefficients)

# region Fuselage setup
# fuselage_ffd_block = lsdo_geo.construct_ffd_block_around_entities(name='fuselage_ffd_block', entities=[fuselage, nose_hub], num_coefficients=(2,2,2), degree=(1,1,1))
# fuselage_ffd_block_sectional_parameterization = lsdo_geo.VolumeSectionalParameterization(name='fuselage_sectional_parameterization',
#                                                                             parameterized_points=fuselage_ffd_block.coefficients,
#                                                                             principal_parametric_dimension=0)
# # fuselage_ffd_block_sectional_parameterization.add_sectional_translation(name='sectional_fuselage_stretch', axis=0)

# fuselage_stretch_coefficients = csdl.Variable(name='fuselage_stretch_coefficients', shape=(2,), value=np.array([0., -0.]))
# fuselage_stretch_b_spline = lfs.Function(name='fuselage_stretch_b_spline', space=linear_b_spline_curve_2_dof_space, 
#                                           coefficients=fuselage_stretch_coefficients)

# parameterization_solver.add_parameter(parameter=fuselage_stretch_coefficients)
# endregion

# region Lift Rotors setup
lift_rotor_ffd_blocks = []
lift_rotor_sectional_parameterizations = []
lift_rotor_parameterization_b_splines = []
for i, component_set in enumerate(lift_rotor_related_components):
    rotor_ffd_block = lsdo_geo.construct_ffd_block_around_entities(name=f'{component_set[0].name[:3]}_rotor_ffd_block', entities=component_set, num_coefficients=(2,2,2), degree=(1,1,1))
    rotor_ffd_block_sectional_parameterization = lsdo_geo.VolumeSectionalParameterization(name=f'{component_set[0].name[:3]}_rotor_sectional_parameterization',
                                                                                parameterized_points=rotor_ffd_block.coefficients,
                                                                                principal_parametric_dimension=2)
    
    rotor_stretch_coefficient = csdl.Variable(name=f'{component_set[0].name[:3]}_rotor_stretch_coefficient', shape=(1,), value=0.)
    lift_rotor_sectional_stretch_b_spline = lfs.Function(name=f'{component_set[0].name[:3]}_rotor_sectional_stretch_x_b_spline', space=constant_b_spline_curve_1_dof_space,
                                                coefficients=rotor_stretch_coefficient)
    
    lift_rotor_ffd_blocks.append(rotor_ffd_block)
    lift_rotor_sectional_parameterizations.append(rotor_ffd_block_sectional_parameterization)
    lift_rotor_parameterization_b_splines.append(lift_rotor_sectional_stretch_b_spline)                 

    parameterization_solver.add_parameter(parameter=rotor_stretch_coefficient)
# endregion Lift Rotors setup

# # region Plot parameterization
# plotting_elements = []
# plotting_elements = geometry.plot(color='#00629B', additional_plotting_elements=plotting_elements, show=False)
# plotting_elements = wing_ffd_block.plot(opacity=0.25, color='#B6B1A9', plot_embedded_points=False, additional_plotting_elements=plotting_elements, show=False)
# plotting_elements = h_tail_ffd_block.plot(opacity=0.25, color='#B6B1A9', plot_embedded_points=False, additional_plotting_elements=plotting_elements, show=False)
# plotting_elements = fuselage_ffd_block.plot(opacity=0.25, color='#B6B1A9', plot_embedded_points=False, additional_plotting_elements=plotting_elements, show=False)
# for rotor_ffd_block in lift_rotor_ffd_blocks:
#     plotting_elements = rotor_ffd_block.plot(opacity=0.25, color='#B6B1A9', plot_embedded_points=False, additional_plotting_elements=plotting_elements, show=False)
# plotting_elements = wing_ffd_block_sectional_parameterization.plot(opacity=0.5, color='#182B49', additional_plotting_elements=plotting_elements, show=False)
# plotting_elements = h_tail_ffd_block_sectional_parameterization.plot(opacity=0.5, color='#182B49', additional_plotting_elements=plotting_elements, show=False)
# plotting_elements = fuselage_ffd_block_sectional_parameterization.plot(opacity=0.5, color='#182B49', additional_plotting_elements=plotting_elements, show=False)
# for rotor_ffd_block_sectional_parameterization in lift_rotor_sectional_parameterizations:
#     plotting_elements = rotor_ffd_block_sectional_parameterization.plot(opacity=0.5, color='#182B49', additional_plotting_elements=plotting_elements, show=False)

# import vedo
# plotter = vedo.Plotter()
# plotter.show(plotting_elements, axes=0, viewup='z')
# exit()

# # endregion Plot parameterization

# endregion Parameterization Setup

# region Parameterization Solver Setup Evaluations

# region Wing Parameterization Evaluation for Parameterization Solver
section_parametric_coordinates = np.linspace(0., 1., wing_ffd_block_sectional_parameterization.num_sections).reshape((-1,1))
sectional_wing_chord_stretch = wing_chord_stretch_b_spline.evaluate(section_parametric_coordinates)
sectional_wing_wingspan_stretch = wing_wingspan_stretch_b_spline.evaluate(section_parametric_coordinates)
# sectional_wing_twist = wing_twist_b_spline.evaluate(section_parametric_coordinates)
sectional_wing_translation_x = wing_translation_x_b_spline.evaluate(section_parametric_coordinates)
sectional_wing_translation_z = wing_translation_z_b_spline.evaluate(section_parametric_coordinates)

sectional_parameters = lsdo_geo.VolumeSectionalParameterizationInputs(
    stretches={0: sectional_wing_chord_stretch},
    translations={1: sectional_wing_wingspan_stretch, 0: sectional_wing_translation_x, 2: sectional_wing_translation_z}
)

wing_ffd_block_coefficients = wing_ffd_block_sectional_parameterization.evaluate(sectional_parameters, plot=False)
wing_coefficients = wing_ffd_block.evaluate(wing_ffd_block_coefficients, plot=False)
wing.set_coefficients(wing_coefficients)
# geometry.plot()

# endregion Wing Parameterization Evaluation for Parameterization Solver

# region Horizontal Stabilizer Parameterization Evaluation for Parameterization Solver
section_parametric_coordinates = np.linspace(0., 1., h_tail_ffd_block_sectional_parameterization.num_sections).reshape((-1,1))
sectional_h_tail_chord_stretch = h_tail_chord_stretch_b_spline.evaluate(section_parametric_coordinates)
sectional_h_tail_span_stretch = h_tail_span_stretch_b_spline.evaluate(section_parametric_coordinates)
# sectional_h_tail_twist = h_tail_twist_b_spline.evaluate(section_parametric_coordinates)
sectional_h_tail_translation_x = h_tail_translation_x_b_spline.evaluate(section_parametric_coordinates)
sectional_h_tail_translation_z = h_tail_translation_z_b_spline.evaluate(section_parametric_coordinates)

# sectional_parameters = {
#     'sectional_h_tail_chord_stretch':sectional_h_tail_chord_stretch,
#     'sectional_h_tail_span_stretch':sectional_h_tail_span_stretch,
#     # 'sectional_h_tail_twist':sectional_h_tail_twist,
#     'sectional_h_tail_translation_x':sectional_h_tail_translation_x,
#     'sectional_h_tail_translation_z':sectional_h_tail_translation_z
#                         }
sectional_parameters = lsdo_geo.VolumeSectionalParameterizationInputs(
    stretches={0: sectional_h_tail_chord_stretch},
    translations={1: sectional_h_tail_span_stretch, 0: sectional_h_tail_translation_x, 2: sectional_h_tail_translation_z}
)

h_tail_ffd_block_coefficients = h_tail_ffd_block_sectional_parameterization.evaluate(sectional_parameters, plot=False)
h_tail_coefficients = h_tail_ffd_block.evaluate(h_tail_ffd_block_coefficients, plot=False)
h_tail.set_coefficients(coefficients=h_tail_coefficients)
# geometry.plot()
# endregion

# # region Fuselage Parameterization Evaluation for Parameterization Solver
# section_parametric_coordinates = np.linspace(0., 1., fuselage_ffd_block_sectional_parameterization.num_sections).reshape((-1,1))
# sectional_fuselage_stretch = fuselage_stretch_b_spline.evaluate(section_parametric_coordinates)

# # sectional_parameters = {'sectional_fuselage_stretch':sectional_fuselage_stretch}
# sectional_parameters = lsdo_geo.VolumeSectionalParameterizationInputs(
#     translations={0: sectional_fuselage_stretch}
# )

# fuselage_ffd_block_coefficients = fuselage_ffd_block_sectional_parameterization.evaluate(sectional_parameters, plot=False)
# fuselage_and_nose_hub_coefficients = fuselage_ffd_block.evaluate(fuselage_ffd_block_coefficients, plot=False)
# fuselage_coefficients = fuselage_and_nose_hub_coefficients[0]
# nose_hub_coefficients = fuselage_and_nose_hub_coefficients[1]

# fuselage.set_coefficients(coefficients=fuselage_coefficients)
# nose_hub.set_coefficients(coefficients=nose_hub_coefficients)
# geometry.plot()

# endregion

# # region Boom Parameterization Evaluation for Parameterization Solver
# section_parametric_coordinates = np.linspace(0., 1., boom_ffd_block_sectional_parameterization.num_sections).reshape((-1,1))

# sectional_boom_translation_y = boom_translation_y_b_spline.evaluate(section_parametric_coordinates)

# sectional_parameters = lsdo_geo.VolumeSectionalParameterizationInputs(
#     translations={1: sectional_boom_translation_y}
# )

# boom_ffd_block_coefficients = boom_ffd_block_sectional_parameterization.evaluate(sectional_parameters, plot=False)
# boom_coefficients = boom_ffd_block.evaluate(boom_ffd_block_coefficients, plot=False)
# boom.set_coefficients(coefficients=boom_coefficients)

# # endregion

# region Lift Rotors Parameterization Evaluation for Parameterization Solver
for i, component_set in enumerate(lift_rotor_related_components):
    rotor_ffd_block = lift_rotor_ffd_blocks[i]
    rotor_ffd_block_sectional_parameterization = lift_rotor_sectional_parameterizations[i]
    rotor_stretch_b_spline = lift_rotor_parameterization_b_splines[i]

    section_parametric_coordinates = np.linspace(0., 1., rotor_ffd_block_sectional_parameterization.num_sections).reshape((-1,1))
    sectional_stretch = rotor_stretch_b_spline.evaluate(section_parametric_coordinates)

    sectional_parameters = lsdo_geo.VolumeSectionalParameterizationInputs(
        stretches={0: sectional_stretch, 1:sectional_stretch}
    )

    rotor_ffd_block_coefficients = rotor_ffd_block_sectional_parameterization.evaluate(sectional_parameters, plot=False)
    rotor_coefficients = rotor_ffd_block.evaluate(rotor_ffd_block_coefficients, plot=False)
    for i, component in enumerate(component_set):
        component.set_coefficients(rotor_coefficients[i])
        # component.set_coefficients(coefficients=rotor_coefficients[i])
        # ? besides line 498 with the wing, this line is the only instance of set_coefficients where
        # 'coefficients=' does not precede the argument 
    # geometry.plot()

# endregion Lift Rotors Parameterization Evaluation for Parameterization Solver

# region Lift Rotors rigid body translation
for i, component_set in enumerate(lift_rotor_related_components):
    # disk = component_set[0]
    # blade_1 = component_set[1]
    # blade_2 = component_set[2]
    # hub = component_set[3]

    boom = boom_components[i]

    # Add rigid body translation
    rigid_body_translation = csdl.Variable(shape=(3,), value=0., name=f'{component_set[0].name[:3]}_rotor_rigid_body_translation')

    for component in component_set:
        for function in component.functions.values():
            function.coefficients = function.coefficients + csdl.expand(rigid_body_translation, function.coefficients.shape, action='k->ijk')

    for function in boom.functions.values():
        function.coefficients = function.coefficients + csdl.expand(rigid_body_translation, function.coefficients.shape, action='k->ijk')

    parameterization_solver.add_parameter(parameter=rigid_body_translation)
# endregion Lift Rotors rigid body translation

# region pusher rigid body translation
rigid_body_translation = csdl.Variable(shape=(3,), value=0., name='cruise_rotor_rigid_body_translation')
for component in cruise_components:
    for function in component.functions.values():
        function.coefficients = function.coefficients + csdl.expand(rigid_body_translation, function.coefficients.shape, action='k->ijk')

parameterization_solver.add_parameter(parameter=rigid_body_translation)
# endregion pusher rigid body translation

# region Vertical Stabilizer rigid body translation
rigid_body_translation = csdl.Variable(shape=(3,), value=0., name='cruise_rotor_rigid_body_translation')
for function in v_tail.functions.values():
    function.coefficients = function.coefficients + csdl.expand(rigid_body_translation, function.coefficients.shape, action='k->ijk')

parameterization_solver.add_parameter(parameter=rigid_body_translation)
# endregion Vertical Stabilizer rigid body translation

# # region Boom rigid body translation
# # not sure if this is necessary since the boom movement is tied into the lifting rotor movement above
# rigid_body_translation = csdl.Variable(shape=(3,), value=0., name='boom_rigid_body_translation')
# for function in boom.functions.values():
#     function.coefficients = function.coefficients + csdl.expand(rigid_body_translation, function.coefficients.shape, action='k->ijk')

# parameterization_solver.add_parameter(parameter=rigid_body_translation)
# # endregion Boom rigid body translation

# endregion Parameterization Solver Setup Evaluations

# region Define Design Parameters

# region wing design parameters
wing_span_computed = csdl.norm(geometry.evaluate(wing_le_right) - geometry.evaluate(wing_le_left))
wing_root_chord_computed = csdl.norm(geometry.evaluate(wing_te_center) - geometry.evaluate(wing_le_center))
wing_tip_chord_left_computed = csdl.norm(geometry.evaluate(wing_te_left) - geometry.evaluate(wing_le_left))
wing_tip_chord_right_computed = csdl.norm(geometry.evaluate(wing_te_right) - geometry.evaluate(wing_le_right))

wing_span = csdl.Variable(name='wing_span', value=np.array([wing_span]))
wing_root_chord = csdl.Variable(name='wing_root_chord', value=np.array([wing_chord]))
wing_tip_chord = csdl.Variable(name='wing_tip_chord_left', value=np.array([wing_chord]))

parameterization_design_parameters.add_variable(computed_value=wing_span_computed, desired_value=wing_span)
parameterization_design_parameters.add_variable(computed_value=wing_root_chord_computed, desired_value=wing_root_chord)
parameterization_design_parameters.add_variable(computed_value=wing_tip_chord_left_computed, desired_value=wing_tip_chord)
parameterization_design_parameters.add_variable(computed_value=wing_tip_chord_right_computed, desired_value=wing_tip_chord)
# endregion wing design parameters

# region h_tail design parameterization inputs
h_tail_span_computed = csdl.norm(geometry.evaluate(h_tail_le_right) - geometry.evaluate(h_tail_le_left))
h_tail_root_chord_computed = csdl.norm(geometry.evaluate(h_tail_te_center) - geometry.evaluate(h_tail_le_center))
h_tail_tip_chord_left_computed = csdl.norm(geometry.evaluate(h_tail_te_left) - geometry.evaluate(h_tail_le_left))
h_tail_tip_chord_right_computed = csdl.norm(geometry.evaluate(h_tail_te_right) - geometry.evaluate(h_tail_le_right))

h_tail_span = csdl.Variable(name='h_tail_span', value=np.array([h_tail_span]))
h_tail_root_chord = csdl.Variable(name='h_tail_root_chord', value=np.array([h_tail_chord]))
h_tail_tip_chord = csdl.Variable(name='h_tail_tip_chord_left', value=np.array([h_tail_chord]))

parameterization_design_parameters.add_variable(computed_value=h_tail_span_computed, desired_value=h_tail_span)
parameterization_design_parameters.add_variable(computed_value=h_tail_root_chord_computed, desired_value=h_tail_root_chord)
parameterization_design_parameters.add_variable(computed_value=h_tail_tip_chord_left_computed, desired_value=h_tail_tip_chord)
parameterization_design_parameters.add_variable(computed_value=h_tail_tip_chord_right_computed, desired_value=h_tail_tip_chord)
# endregion h_tail design parameterization inputs

# region tail moment arm variables
tail_moment_arm_computed = csdl.norm(geometry.evaluate(h_tail_qc) - geometry.evaluate(wing_qc))
tail_moment_arm = csdl.Variable(name='tail_moment_arm', value=np.array([2.125]))
parameterization_design_parameters.add_variable(computed_value=tail_moment_arm_computed, desired_value=tail_moment_arm)

wing_fuselage_connection = geometry.evaluate(wing_te_center) - geometry.evaluate(fuselage_wing_te_center)
h_tail_fuselage_connection = geometry.evaluate(h_tail_te_center) - geometry.evaluate(fuselage_tail_te_center)
parameterization_design_parameters.add_variable(computed_value=wing_fuselage_connection, desired_value=wing_fuselage_connection.value)
parameterization_design_parameters.add_variable(computed_value=h_tail_fuselage_connection, desired_value=h_tail_fuselage_connection.value)

# endregion tail moment arm variables

# region v-tail connection, is this OK?
vtail_fuselage_connection_point = geometry.evaluate(v_tail.project(np.array([x_v_tail_LE, 0., z_h_tail_LE])))
vtail_fuselage_connection = geometry.evaluate(fuselage_rear_points_parametric) - vtail_fuselage_connection_point
parameterization_design_parameters.add_variable(computed_value=vtail_fuselage_connection, desired_value=vtail_fuselage_connection.value)

# endregion v-tail connection

# region lift + pusher rotor parameterization inputs
# pusher_fuselage_connection = geometry.evaluate(fuselage_rear_points_parametric) - geometry.evaluate(fuselage_rear_point_on_pusher_disk_parametric)
cruise_prop_fuselage_connection = geometry.evaluate(fuselage_rear_points_parametric) - geometry.evaluate(fuselage_nose_point_on_cruise_propeller_disk_parametric)
parameterization_design_parameters.add_variable(computed_value=cruise_prop_fuselage_connection, desired_value=cruise_prop_fuselage_connection.value)

# These values are not being optimized
# flo_radius = fro_radius = front_outer_radius = csdl.Variable(name='front_outer_radius', value=10/2)
# fli_radius = fri_radius = front_inner_radius = csdl.Variable(name='front_inner_radius', value=10/2)
# rlo_radius = rro_radius = rear_outer_radius = csdl.Variable(name='rear_outer_radius', value=10/2)
# rli_radius = rri_radius = rear_inner_radius = csdl.Variable(name='rear_inner_radius', value=10/2)
# dv_radius_list = [rlo_radius, rli_radius, rri_radius, rro_radius, flo_radius, fli_radius, fri_radius, fro_radius]

boom_points = [boom_rl, boom_rr, boom_fl, boom_fr]
boom_points_on_wing = [wing_boom_rl, wing_boom_rr, wing_boom_fr, wing_boom_fl]
rotor_prefixes = ['rl','rr','fr','fl']

 
for i in range(len(boom_points)):
    boom_connection = geometry.evaluate(boom_points[i]) - geometry.evaluate(boom_points_on_wing[i])

    parameterization_design_parameters.add_variable(computed_value=boom_connection, desired_value=boom_connection.value)
    
    # ? commenting this out because we are not changing rotor radius for now
    # component_rotor_edges = rotor_edges[i]
    # radius_computed = csdl.norm(geometry.evaluate(component_rotor_edges[0]) - geometry.evaluate(component_rotor_edges[1]))/2
    # parameterization_design_parameters.add_variable(computed_value=radius_computed, desired_value=dv_radius_list[i])

# endregion lift + pusher rotor parameterization inputs

# endregion Define Design Parameters

# geometry.plot()
parameterization_solver.evaluate(parameterization_design_parameters)
geometry.plot()

# endregion

# region Mesh Evaluation
upper_surface_wireframe = geometry.evaluate(wing_upper_surface_wireframe_parametric)
lower_surface_wireframe = geometry.evaluate(wing_lower_surface_wireframe_parametric)
wing_vlm_mesh = csdl.linear_combination(upper_surface_wireframe, lower_surface_wireframe, 1).reshape((wing_num_chordwise_vlm, wing_num_spanwise_vlm, 3))

upper_surface_wireframe = geometry.evaluate(h_tail_upper_surface_wireframe_parametric)
lower_surface_wireframe = geometry.evaluate(h_tail_lower_surface_wireframe_parametric)
h_tail_vlm_mesh = csdl.linear_combination(upper_surface_wireframe, lower_surface_wireframe, 1).reshape((h_tail_num_chordwise_vlm, h_tail_num_spanwise_vlm, 3))

beam_tops = wing.evaluate(beam_top_parametric)
beam_bottoms = wing.evaluate(beam_bottom_parametric)
wing_beam_mesh = csdl.linear_combination(beam_tops, beam_bottoms, 1).reshape((num_beam_nodes, 3))
beam_heights = csdl.norm(beam_tops - beam_bottoms, axes=(1,))
# endregion Mesh Evaluation