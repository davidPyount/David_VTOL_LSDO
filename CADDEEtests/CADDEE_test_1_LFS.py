import CADDEE_alpha as cd
import csdl_alpha as csdl
import numpy as np
from lsdo_airfoil.core.three_d_airfoil_aero_model import ThreeDAirfoilMLModelMaker
from VortexAD.core.vlm.vlm_solver import vlm_solver
from modopt import CSDLAlphaProblem, SLSQP
from CADDEE_alpha.utils.units import Units
import lsdo_function_spaces as lfs
import aframe as af
import os #for relative path import
from BladeAD.core.airfoil.ml_airfoil_models.NACA_4412.naca_4412_model import NACA4412MLAirfoilModel
from BladeAD.utils.parameterization import BsplineParameterization
from BladeAD.core.BEM.bem_model import BEMModel
from BladeAD.core.pitt_peters.pitt_peters_model import PittPetersModel
from BladeAD.utils.var_groups import RotorAnalysisInputs, RotorMeshParameters
from CADDEE_alpha.core.mesh.meshers import CamberSurface
import aeroelastic_coupling_utils as acu
units = Units()

#This is bogus dont use it.


#TO DO
#Static stability constraints
#Make it converge

ForwardRunOnly = True #Setting this to true will bypass base_config.setup_geometry, which takes the most time

# Start the CSDL recorder
recorder = csdl.Recorder(inline=True, expand_ops=True)
recorder.start()

# import geometry
dirname = os.path.dirname(__file__) #relative path
filename = os.path.join(dirname, 'mark2.stp')
mark2_geom = cd.import_geometry(filename)
plotting_elements = mark2_geom.plot(show=False, opacity=0.5, color='#FFCD00')

g = 9.81

# define INITIAL values for design parameters
w_total = 6 * units.mass.pound_to_kg * g

# fuselage
fuselage_length = 1.25 * units.length.foot_to_m
fuse_perim = 1.5 * units.length.foot_to_m
fuse_h = 5/12 * units.length.foot_to_m
fuse_w = 5/12 * units.length.foot_to_m
fuse_t = 3/16 * units.length.inch_to_m
fuse_CG = np.array([-0.32, 0, 0.034]) # [m]

# wing
span = 4 * units.length.foot_to_m
wingchord = 9/12 * units.length.foot_to_m
wing_S = span * wingchord
wing_AR = span/wingchord
wing_tc = 0.12
wing_taper = 1
x_wing_LE = -0.203 # [m]
y_wing_LE = span/2
z_wing_LE = -0.055
x_wing_TE = x_wing_LE - wingchord

# h-stab
h_stab_span = 1.25 * units.length.foot_to_m
h_stab_chord = 5/12 * units.length.foot_to_m
h_stab_AR = h_stab_span/h_stab_chord
h_stab_S = h_stab_span * h_stab_chord
h_stab_taper = 1
h_stab_tc = 0.12
x_h_tail_LE = -0.851 # [m]
y_h_tail_LE = h_stab_span/2
z_h_tail_LE = 0.016
x_h_tail_TE = x_h_tail_LE - h_stab_chord

# v-stab
v_stab_span = 1.08/2 * units.length.foot_to_m
v_stab_chord = 5/12 * units.length.foot_to_m
v_stab_AR = v_stab_span/v_stab_chord
v_stab_S = v_stab_span * v_stab_chord
v_stab_taper = 1
v_stab_tc = 0.12
v_stab_area = v_stab_span * v_stab_chord

# lift propulsion assembly (rotors, motors, booms, mounts)
lift_rotor_d = 14 * units.length.inch_to_m
lift_motor_mass_val = 0.117 # kg
lift_motor_mount_mass_val = 0.0545 # kg
wing_boom_mount_mass_val = 0.026 # kg
lift_rotor_mass_val = 0.030 # [kg] from https://www.apcprop.com/product/14x5-5mr/?v=7516fd43adaa
wing_boom_y = 0.457  # [m]
wing_boom_z = -0.038   

# cruise propulsion (rotor and motor)
cruise_prop_d = 8 * units.length.inch_to_m
cruise_motor_mass_val = 0.072 # [kg]
cruise_motor_CG = np.array([0.0165, 0, 0]) # [m]

# main spar
main_spar_len = 34 * units.length.inch_to_m
main_spar_OD_val = 0.75 * units.length.inch_to_m
main_spar_ID_val = 0.625 * units.length.inch_to_m

# internal wing spars
wing_spar_density = 1435 # [kg/m^3]

# wing booms
wing_boom_len = 30 * units.length.inch_to_m
wing_boom_OD = 0.75 * units.length.inch_to_m
wing_boom_ID = 0.625 * units.length.inch_to_m

# battery
m_battery = 0.372 # [kg]
l_battery = 0.140 # [m]
w_battery = 0.044 # [m]
h_battery = 0.033 # [m]
battery_x_val = -0.1859 # [m]

# skeleton
skeleton_mass_val = 0.689
skeleton_CG = np.array([-0.201, 0, 0.055]) # [m]

# fuselage-wing mount
wing_fuse_mount_mass_val = 0.0567 # kg
wing_fuse_mount_CG = np.array([-0.256, 0, -0.053]) # [m]

# tail mount
tail_mount_mass_val = 0.061 # kg
tail_mount_CG = np.array([-0.904, 0, -0.006]) # [m]
tail_mount_length_val = 0.117 # [m]

# cruise conditions
alt = 0
dist = 7000
pitch = 0
cruise_v = 70 * units.length.foot_to_m
sos = 1100 * units.length.foot_to_m
mach =cruise_v/sos

# https://www.dupont.com/content/dam/dupont/amer/us/en/
# performance-building-solutions/public/documents/en/
# styrofoam-panel-core-20-xps-pis-43-d100943-enus.pdf
# density_foam = 24 # kg/m3
density_foam = 36 # kg/m^3 according to updated weight from Madison

# VLM nodes
wing_chordwise_panels = 5
wing_spanwise_panels = 3
h_tail_chordwise_panels = 13
h_tail_spanwise_panels = 13

if wing_spanwise_panels % 2 == 0:
    raise ValueError('sowwy! wing_spanwise_panels must be odd until beam mesh is decoupled from VLM mesh 3:')
if wing_chordwise_panels % 2 == 0:
    raise ValueError('sowwy! wing_chordwise_panels must be odd until beam mesh is decoupled from VLM mesh 3:')

wing_spanwise_nodes = wing_spanwise_panels + 1
wing_chordwise_nodes = wing_chordwise_panels + 1
tail_spanwise_nodes = h_tail_spanwise_panels + 1
tail_chordwise_nodes = h_tail_chordwise_panels + 1

# make instance of CADDEE class
caddee = cd.CADDEE()

def define_base_config(caddee : cd.CADDEE):
    """Build the system configuration and define meshes."""

    # Make aircraft component and pass in the geometry
    aircraft = cd.aircraft.components.Aircraft(geometry=mark2_geom, compute_surface_area=False)

    # instantiation configuration object and pass in system component (aircraft)
    base_config = cd.Configuration(system=aircraft)

    # why isn't the fuselage component assigned to the aircraft?
    fuselage_geometry = aircraft.create_subgeometry(search_names=["Fuselage"])
    fuselage = cd.aircraft.components.Fuselage(
    length=fuselage_length, 
    max_height= fuse_h,
    max_width= fuse_w,
    geometry=fuselage_geometry,
    skip_ffd = True)
    # fuselage = cd.aircraft.components.Fuselage(length=fuselage_length, geometry=fuselage_geometry)
    fuselage.quantities.drag_parameters.characteristic_length = fuselage_length #WHY THE HELL IS THIS ALWAYS NONE!!!! #I think CADDEE is supposed to do this by default.
    aircraft.comps["fuselage"] = fuselage
    
    # Treating the main spar like a funky fresh fuselage. Its dimensions will change but those of the main fuselage will not
    main_spar_geometry = aircraft.create_subgeometry(search_names=["MainSpar"])
    # !! Adding main spar length as a design variable
    main_spar_length = csdl.Variable(name="main_spar_length", value=main_spar_len)
    main_spar_length.set_as_design_variable(lower=0.75*main_spar_len, upper=1.25*main_spar_len, scaler=1)
    main_spar = cd.aircraft.components.Fuselage(
        length=main_spar_length, 
        max_height= 0.75 * units.length.inch_to_m,
        max_width= 0.75 * units.length.inch_to_m,
        geometry=main_spar_geometry,
        )
    main_spar.quantities.drag_parameters.characteristic_length = main_spar_length #Refer to above.
    # assign main spar component to aircraft
    aircraft.comps["main_spar"] = main_spar

    # Make wing geometry from aircraft component and instantiate wing component
    wing_geometry = aircraft.create_subgeometry(search_names=["Wing"])
    aspect_ratio = csdl.Variable(name="wing_aspect_ratio", value = wing_AR)
    # wing_span = csdl.Variable(name="wing_span", value = span)
    # wing_chord = csdl.Variable(name="wing_chord", value=wingchord)
    wing_area = csdl.Variable(name="wing_area", value = wing_S)
    wing_root_twist = csdl.Variable(name="wing_root_twist", value=np.deg2rad(1.5))
    wing_tip_twist = csdl.Variable(name="wing_tip_twist", value=np.deg2rad(1.5))
    
    # Set design variables for wing
    aspect_ratio.set_as_design_variable(upper=1.5 * wing_AR, lower=0.75 * wing_AR, scaler=1/5)
    wing_area.set_as_design_variable(upper=1.5 * wing_S, lower=0.75 * wing_S, scaler=4)
    wing_root_twist.set_as_design_variable(upper=np.deg2rad(10), lower=np.deg2rad(-10), scaler=4)
    wing_tip_twist.set_as_design_variable(upper=np.deg2rad(10), lower=np.deg2rad(-10), scaler=4)
    
    wing = cd.aircraft.components.Wing(
        AR=aspect_ratio, S_ref=wing_area, 
        taper_ratio=wing_taper, root_twist_delta=wing_root_twist,
        tip_twist_delta=wing_tip_twist, geometry=wing_geometry
    )

    # Pressure space for Wing
    pressure_function_space = lfs.IDWFunctionSpace(num_parametric_dimensions=2, order=6, grid_size=(120, 20), conserve=False, n_neighbors=10)
    indexed_pressue_function_space = wing.geometry.create_parallel_space(pressure_function_space)
    wing.quantities.pressure_space = indexed_pressue_function_space

    # Assign wing component to aircraft
    aircraft.comps["wing"] = wing    

    #Making hstab parameters changeable.
    h_tail_AR = csdl.Variable(name="hstab_aspect_ratio", value=h_stab_AR)
    h_tail_area = csdl.Variable(name="h_tail_area", value=h_stab_S)
    h_tail_root_twist = csdl.Variable(name="h_stab_root_twist", value=np.deg2rad(-1.5))

    # Set design variables for wing
    h_tail_AR.set_as_design_variable(upper=1.5 * wing_AR, lower=0.6 * wing_AR, scaler=1/3)
    h_tail_area.set_as_design_variable(lower=0.5 * h_stab_S, upper=1.5 * h_stab_S, scaler=20)
    h_tail_root_twist.set_as_design_variable(upper=np.deg2rad(5), lower=np.deg2rad(-5), scaler=10)

    # Make horizontal tail geometry & component
    h_tail_geometry = aircraft.create_subgeometry(search_names=["HStab"])
    h_tail = cd.aircraft.components.Wing(
        AR=h_tail_AR, S_ref=h_tail_area, taper_ratio=h_stab_taper, 
        geometry=h_tail_geometry, root_twist_delta=h_tail_root_twist, tip_twist_delta=h_tail_root_twist
    )

    # Assign tail component to aircraft
    aircraft.comps["h_tail"] = h_tail

    # Make vertical tail geometry & component
    v_tail_geometry = aircraft.create_subgeometry(search_names=["VStab"])

    # these shouldn't need to be design variables if we are not varying v-tail size
    # v_tail_AR = csdl.Variable(name="v_tail_AR", value=v_stab_AR)
    # v_tail_area = csdl.Variable(name="v_tail_area", value=v_stab_S)
    v_tail_AR = v_stab_AR
    v_tail_area = v_stab_area
    
    v_tail = cd.aircraft.components.Wing(
        AR=v_tail_AR, S_ref=v_tail_area, geometry=v_tail_geometry, 
        skip_ffd=True, orientation="vertical"
    )

    # Assign v-tail component to aircraft
    aircraft.comps["v_tail"] = v_tail

    nosecone_geom = aircraft.create_subgeometry(search_names=["Nosecone"])
    nosecone = cd.Component(geometry=nosecone_geom)
    aircraft.comps["nosecone"] = nosecone

    #Generic caddee componets that are used later in define_mass_properties
    #Battery
    battery = cd.Component()
    aircraft.comps["battery"] = battery

    #Wing spars?
    wing_spars = cd.Component()
    aircraft.comps["wing_spars"] = wing_spars

    #Skeleton
    skeleton = cd.Component()
    aircraft.comps["skeleton"] = skeleton

    left_boom_assembly = cd.Component()
    aircraft.comps["left_boom_assembly"] = left_boom_assembly

    right_boom_assembly = cd.Component()
    aircraft.comps["right_boom_assembly"] = right_boom_assembly

    cruise_motor = cd.Component()
    aircraft.comps["cruise_motor"] = cruise_motor

    wing_fuse_mount = cd.Component()
    aircraft.comps["wing_fuse_mount"] = wing_fuse_mount

    tail_mount = cd.Component()
    aircraft.comps["tail_mount"] = tail_mount

    # Connect wing to fuselage at the quarter chord
    base_config.connect_component_geometries(fuselage, wing, 0.75 * wing.LE_center + 0.25 * wing.TE_center)
    # Connect h-tail to spar
    base_config.connect_component_geometries(main_spar, h_tail, main_spar.tail_point)
    # Connect main spar to fuselage
    base_config.connect_component_geometries(main_spar, fuselage, main_spar.nose_point)
    # connect fuselage to nosecone
    base_config.connect_component_geometries(fuselage, nosecone, fuselage.nose_point)

    # Meshing
    mesh_container = base_config.mesh_container

    #Seths meshing code
    # # wing mesh
    # plotting = False
    
    # leading_edge_line_parametric = wing_geometry.project(np.linspace(np.array([x_wing_LE, -y_wing_LE, z_wing_LE + 0.1]), 
    #                                np.array([x_wing_LE, y_wing_LE, z_wing_LE + 0.1]), wing_spanwise_nodes), 
    #                                direction=np.array([0., 0., -1.]), grid_search_density_parameter=20., plot=plotting)
    # trailing_edge_line_parametric = wing_geometry.project(np.linspace(np.array([x_wing_TE, -y_wing_LE, z_wing_LE + 0.1]),
    #                                 np.array([x_wing_TE, y_wing_LE, z_wing_LE + 0.1]), wing_spanwise_nodes), 
    #                                 direction=np.array([0., 0., -1.]), grid_search_density_parameter=20., plot=plotting)
    # leading_edge_line = wing_geometry.evaluate(leading_edge_line_parametric)
    # trailing_edge_line = wing_geometry.evaluate(trailing_edge_line_parametric)
    # chord_surface = csdl.linear_combination(leading_edge_line, trailing_edge_line, wing_chordwise_nodes)
    # wing_geometry_upper_surface_wireframe_parametric = wing_geometry.project(chord_surface.value.reshape(
    #                                                   (wing_chordwise_nodes,wing_spanwise_nodes,3))+np.array([0., 0., -0.1]), 
    #                                                    direction=np.array([0., 0., 1.]), plot=plotting, grid_search_density_parameter=10.)
    # wing_geometry_lower_surface_wireframe_parametric = wing_geometry.project(chord_surface.value.reshape(
    #                                                   (wing_chordwise_nodes,wing_spanwise_nodes,3))+np.array([0., 0., 0.1]), 
    #                                                    direction=np.array([0., 0., -1.]), plot=plotting, grid_search_density_parameter=10.)
    # upper_surface_wireframe = wing_geometry.evaluate(wing_geometry_upper_surface_wireframe_parametric)
    # lower_surface_wireframe = wing_geometry.evaluate(wing_geometry_lower_surface_wireframe_parametric)
    # wing_camber_surface = csdl.linear_combination(upper_surface_wireframe, lower_surface_wireframe, 1).reshape(
    #                     (wing_chordwise_nodes, wing_spanwise_nodes, 3))
    # # wing_camber_surface = csdl.expand(wing_camber_surface, (1, wing_chordwise_nodes, wing_spanwise_nodes, 3), 'ijk->aijk')
    # # wing_geometry.plot_meshes([wing_camber_surface])

    # # h-tail mesh
    # leading_edge_line_parametric = h_tail_geometry.project(np.linspace(np.array([x_h_tail_LE, -y_h_tail_LE, z_h_tail_LE]), 
    #                                np.array([x_h_tail_LE, y_h_tail_LE, z_h_tail_LE]), tail_spanwise_nodes), direction=np.array([0., 0., -1.]), 
    #                                grid_search_density_parameter=20.,plot=plotting)
    # trailing_edge_line_parametric = h_tail_geometry.project(np.linspace(np.array([x_h_tail_TE, -y_h_tail_LE, z_h_tail_LE + 0.1]), 
    #                                 np.array([x_h_tail_TE, y_h_tail_LE, z_h_tail_LE + 0.1]), tail_spanwise_nodes), 
    #                                 direction=np.array([0., 0., -1.]), grid_search_density_parameter=20.,plot=plotting)
    # leading_edge_line = h_tail_geometry.evaluate(leading_edge_line_parametric)
    # trailing_edge_line = h_tail_geometry.evaluate(trailing_edge_line_parametric)
    # chord_surface = csdl.linear_combination(leading_edge_line, trailing_edge_line, tail_chordwise_nodes)
    # h_tail_geometry_upper_surface_wireframe_parametric = h_tail_geometry.project(chord_surface.value.reshape(
    #                                                     (tail_chordwise_nodes,tail_spanwise_nodes,3))+np.array([0., 0., -0.1]), 
    #                                                     direction=np.array([0., 0., 1.]), plot=plotting, grid_search_density_parameter=10.)
    # h_tail_geometry_lower_surface_wireframe_parametric = h_tail_geometry.project(chord_surface.value.reshape(
    #                                                     (tail_chordwise_nodes,tail_spanwise_nodes,3))+np.array([0., 0., 0.1]), 
    #                                                     direction=np.array([0., 0., -1.]), plot=plotting, grid_search_density_parameter=10.)
    # upper_surface_wireframe = h_tail_geometry.evaluate(h_tail_geometry_upper_surface_wireframe_parametric)
    # lower_surface_wireframe = h_tail_geometry.evaluate(h_tail_geometry_lower_surface_wireframe_parametric)
    # h_tail_camber_surface = csdl.linear_combination(upper_surface_wireframe, lower_surface_wireframe, 1).reshape(
    #                         (tail_chordwise_nodes, tail_spanwise_nodes, 3))
    # # h_tail_camber_surface = csdl.expand(h_tail_camber_surface, (1, tail_chordwise_nodes, tail_spanwise_nodes, 3), 'ijk->aijk')
    # # h_tail_geometry.plot_meshes([h_tail_camber_surface])

    # # nodal coords look good here (no extra row of zeros)
    # wing_discretization = CamberSurface(nodal_coordinates = wing_camber_surface)
    # wing_discretization._upper_wireframe_para = wing_geometry_upper_surface_wireframe_parametric
    # wing_discretization._lower_wireframe_para = wing_geometry_lower_surface_wireframe_parametric
    # wing_discretization._geom = wing_geometry
    # wing_discretization._num_chord_wise = wing_chordwise_panels
    # wing_discretization._num_spanwise = wing_spanwise_panels

    # h_tail_discretization = CamberSurface(nodal_coordinates = h_tail_camber_surface)
    # h_tail_discretization._upper_wireframe_para = h_tail_geometry_upper_surface_wireframe_parametric
    # h_tail_discretization._lower_wireframe_para = h_tail_geometry_lower_surface_wireframe_parametric
    # h_tail_discretization._geom = h_tail_geometry
    # h_tail_discretization._num_chord_wise = h_tail_chordwise_panels
    # h_tail_discretization._num_spanwise = h_tail_spanwise_panels

    vlm_mesh = cd.mesh.VLMMesh()
    wing_chord_surface = cd.mesh.make_vlm_surface(
        wing, 26, 1, LE_interp="ellipse", TE_interp="ellipse", 
        spacing_spanwise="cosine", ignore_camber=True, plot=False,
    )
    wing_chord_surface.project_airfoil_points()
    vlm_mesh.discretizations["wing_camber_surface"] = wing_chord_surface

    tail_surface = cd.mesh.make_vlm_surface(
        h_tail, 8, 1, ignore_camber=True
    )
    vlm_mesh.discretizations["h_tail_camber_surface"] = tail_surface
    

    #ROTOR MESHING FOR BEM
    num_radial = 25
    cruise_prop_geom = aircraft.create_subgeometry(search_names=["Main Propeller"]) #get geo from openvsp
    cruise_prop = cd.aircraft.components.Rotor(radius=4*units.length.inch_to_m, geometry=cruise_prop_geom, compute_surface_area=False, skip_ffd=True) #make CADDEE component
    cruise_prop_mesh = cd.mesh.make_rotor_mesh( #made bladeAD rotor mesh (not CADDEE mesh)
        cruise_prop, num_radial=num_radial, num_azimuthal=5, num_blades=2
    )
    rotor_meshes = cd.mesh.RotorMeshes() #make caddee-compatible mesh holder
    cruise_prop_mesh.twist_profile = csdl.Variable(shape=(num_radial, ), value=np.deg2rad(np.linspace(20., 50., num_radial))) #These are taken from other code, make them accurate for us.
    cruise_prop_mesh.chord_profile = csdl.Variable(shape=(num_radial, ), value=np.linspace(1*units.length.inch_to_m, 0.2*units.length.inch_to_m, num_radial))
    rotor_meshes.discretizations["cruise_prop_mesh"] = cruise_prop_mesh #Assign bladeAD rotor mesh to caddee rotor mesh container

    # Assign mesh to mesh container
    mesh_container["vlm_mesh"] = vlm_mesh
    mesh_container["rotor_meshes"] = rotor_meshes

    # Set up the geometry: this will run the inner optimization
    # !! uncomment this during real run, comment out to check initial setup
    if ForwardRunOnly:
        base_config.setup_geometry(plot=False)
          
    # Assign base configuration to CADDEE instance
    caddee.base_configuration = base_config

    # I had to add these lines to fix the issue I was having with VLM solver (nodal coordinates had extra row of zeros)
    # wing_nodes = csdl.expand(wing_camber_surface, (1, wing_chordwise_nodes, wing_spanwise_nodes, 3), 'ijk->aijk')
    # h_tail_nodes = csdl.expand(h_tail_camber_surface, (1, tail_chordwise_nodes, tail_spanwise_nodes, 3), 'ijk->aijk')
    # wing_camber_surface = csdl.expand(wing_camber_surface, (1, wing_chordwise_nodes, wing_spanwise_nodes, 3), 'ijk->aijk')
    # h_tail_camber_surface = csdl.expand(h_tail_camber_surface, (1, tail_chordwise_nodes, tail_spanwise_nodes, 3), 'ijk->aijk')

    # return wing_nodes, h_tail_nodes
    # return wing_camber_surface, h_tail_camber_surface

def define_conditions(caddee: cd.CADDEE):
    conditions = caddee.conditions
    base_config = caddee.base_configuration

    cruise = cd.aircraft.conditions.CruiseCondition(
        altitude=0,
        range=10e3,
        pitch_angle=0,
        speed=cruise_v,
    )
    cruise.configuration = base_config.copy()
    conditions["cruise"] = cruise

def define_mass_properties(caddee : cd.CADDEE,vlm_output, force_coords, force_vectors):
    """Define the mass properties of the aircraft."""
    base_config = caddee.base_configuration
    aircraft = base_config.system

    conditions = caddee.conditions
    cruise : cd.aircraft.conditions.CruiseCondition = conditions["cruise"]
    # dynamic_pressure = 0.5 * cruise.quantities.atmos_states.density * cruise.parameters.speed**2

    # design_gross_weight = csdl.Variable(name="design_gross_weight", value=w_total)
    fuel_weight = 0
    
    battery = aircraft.comps["battery"]
    battery_mass = csdl.Variable(name="battery_mass", value=m_battery)
    # position pulled from CAD
    battery.quantities.mass_properties.mass = battery_mass
    battery_x = csdl.Variable(name="battery_x", value=battery_x_val)
    battery_position = csdl.Variable(name="battery_position", value=np.zeros(3))
    battery_position = battery_position.set(csdl.slice[0],battery_x)
    battery.quantities.mass_properties.cg_vector = battery_position 
    battery_x.set_as_design_variable(lower=-15*cd.Units.length.inch_to_m, upper=-5*cd.Units.length.inch_to_m, scaler=5*cd.Units.length.inch_to_m)

    wing : cd.aircraft.components.Wing = aircraft.comps["wing"]
    wing_qc = 0.75 * wing.LE_center + 0.25 * wing.TE_center
    # these values need to be defined in terms of values passed into the wing component
    wing_span = csdl.sqrt(wing.parameters.AR * wing.parameters.S_ref)
    wing_span.set_as_constraint(upper = 6 * cd.Units.length.foot_to_m, scaler=0.5) ########## Look into this

    beam_radius, beam_ID_radius = run_beam(caddee:=caddee, vlm_output=vlm_output, force_coords = force_coords, force_vectors = force_vectors)

    wing_spar_area = np.pi * (beam_radius**2 - beam_ID_radius**2) # [m^2]
    wing_spar_volume = wing_spar_area * wing_span
    wing_spar_mass = wing_spar_volume * wing_spar_density # [m^3]
    wing_spars_mass = wing_spar_mass * 2

    wing_spars = aircraft.comps["wing_spars"]
    wing_spars.quantities.mass_properties.mass = wing_spars_mass
    wing_spars.quantities.mass_properties.cg_vector = wing_qc
    
    # approximate the wing cross-section area as an ellipse, rectangle, and triangle.
    # TO DO: Replace this with CSDL integrator
    seth_wing_volume = True
    if seth_wing_volume:
        wing_chord = csdl.sqrt(wing.parameters.S_ref / wing.parameters.AR)
        # wing_thickness_to_chord = 0.12 * wing_chord

        wing_max_t = wing_chord*wing_tc
        wing_ellipse_a = 0.3*wing_chord
        wing_ellipse_b = 1/2 * wing_max_t
        wing_ellipse_area = np.pi * wing_ellipse_a * wing_ellipse_b / 2

        wing_rectangle_area = 0.2*wing_chord * wing_max_t

        wing_triangle_area = 2 * (1/2 * wing_max_t/2 * wing_chord/2)

        wing_cross_section_area = wing_ellipse_area + wing_rectangle_area + wing_triangle_area

        wing_volume = wing_cross_section_area * wing_span - 2*np.pi*(beam_radius**2)*wing_span

        wing_mass = wing_volume * density_foam
        wing.quantities.mass_properties.mass = wing_mass + fuel_weight
        wing.quantities.mass_properties.cg_vector = 0.56 * wing.LE_center + 0.44 * wing.TE_center # CG is around 44.4% of chord for 4412
    else:
        beam_radius_ft = beam_radius/units.length.foot_to_m
        wing_mass = wing_mass_model(wing.parameters.AR,wing.parameters.S_ref,4,4,12,beam_radius_ft)*units.mass.pound_to_kg
        wing.quantities.mass_properties.mass = wing_mass
        wing.quantities.mass_properties.cg_vector = 0.56 * wing.LE_center + 0.44 * wing.TE_center # CG is around 44.4% of chord for 4412


    fuselage : cd.aircraft.components.Fuselage = aircraft.comps["fuselage"]
    # density from https://forum.flitetest.com/index.php?threads/foam-board-strength-a-scientific-test.62313/
    density_fuse_foam = 59 # [kg/m^3] 
    fuse_volume = fuselage_length * ((fuse_h*fuse_w) - (fuse_h-2*fuse_t)*(fuse_w-2*fuse_t)) + fuselage_length*(fuse_w/2)*fuse_t # include extra bit on bottom
    fuselage_mass = fuse_volume * density_fuse_foam
    fuselage_mass = csdl.Variable(name="fuselage_mass", value = fuselage_mass)
    fuselage.quantities.mass_properties.mass = fuselage_mass
    fuselage_position = csdl.Variable(name="fuselage_position", value=fuse_CG)
    fuselage.quantities.mass_properties.cg_vector = fuselage_position

    h_tail : cd.aircraft.components.Wing = aircraft.comps["h_tail"]
    
    # approximate the cross-sectional area of the h-stab as an ellipse, rectangle, and triangle

    h_tail = aircraft.comps["h_tail"]
    if seth_wing_volume:
        h_tail_span = csdl.sqrt(h_tail.parameters.AR * h_tail.parameters.S_ref)
        h_tail_chord = csdl.sqrt(h_tail.parameters.S_ref / h_tail.parameters.AR)
        # h_tail_thickness_to_chord = 0.12 * h_tail_chord

        h_tail_max_t = h_tail_chord*h_stab_tc
        h_tail_ellipse_a = 0.3*h_tail_chord
        h_tail_ellipse_b = 1/2 * h_tail_max_t
        h_tail_ellipse_area = np.pi * h_tail_ellipse_a * h_tail_ellipse_b / 2

        h_tail_rectangle_area = 0.2*h_tail_chord * h_tail_max_t

        h_tail_triangle_area = 2 * (1/2 * h_tail_max_t/2 * 0.5*h_tail_chord)

        h_tail_cross_section_area = h_tail_ellipse_area + h_tail_rectangle_area + h_tail_triangle_area 

        h_tail_volume = h_tail_cross_section_area * h_tail_span
        
        h_tail_mass = density_foam * h_tail_volume
    else:
        h_tail_mass = wing_mass_model(h_tail.parameters.AR,h_tail.parameters.S_ref,0,0,12,0)

    h_tail.quantities.mass_properties.mass = h_tail_mass
    h_tail.quantities.mass_properties.cg_vector = 0.575 * h_tail.LE_center + 0.425 * h_tail.TE_center
    

    v_tail : cd.aircraft.components.Wing = aircraft.comps["v_tail"]

    v_tail_span = csdl.sqrt(v_tail.parameters.AR * v_tail.parameters.S_ref)
    v_tail_chord = csdl.sqrt(v_tail.parameters.S_ref / v_tail.parameters.AR)
    # v_tail_thickness_to_chord = 0.12 * v_tail_chord

    v_tail_max_t = v_tail_chord*v_stab_tc
    v_tail_ellipse_a = 0.3*v_tail_chord
    v_tail_ellipse_b = 1/2 * v_tail_max_t
    v_tail_ellipse_area = np.pi * v_tail_ellipse_a * v_tail_ellipse_b / 2

    v_tail_rectangle_area = 0.2*v_tail_chord * v_tail_max_t

    v_tail_triangle_area = 2* (1/2 * v_tail_max_t/2 * 0.5*v_tail_chord)

    v_tail_cross_section_area = v_tail_ellipse_area + v_tail_rectangle_area + v_tail_triangle_area

    v_tail_volume = v_tail_cross_section_area * v_tail_span

    v_tail_mass = v_tail_volume * density_foam
    v_tail.quantities.mass_properties.mass = v_tail_mass
    v_tail.quantities.mass_properties.cg_vector = 0.575 * h_tail.LE_center + 0.425 * h_tail.TE_center

    skeleton = aircraft.comps["skeleton"]

    skeleton_mass = csdl.Variable(name = "skeleton_mass", value = skeleton_mass_val)
    skeleton.quantities.mass_properties.mass = skeleton_mass
    skeleton.quantities.mass_properties.cg_vector = csdl.Variable(name="skeleton_cg", value = skeleton_CG) 

    cruise_motor = aircraft.comps["cruise_motor"]
    # from https://www.cobramotorsusa.com/motors-2217-20.html
    cruise_motor_mass = csdl.Variable(name="cruise_motor_mass", value=cruise_motor_mass_val) 
    cruise_motor.quantities.mass_properties.mass = cruise_motor_mass
    cruise_motor.quantities.mass_properties.cg_vector = csdl.Variable(name="cruise_motor_cg", value = cruise_motor_CG)

    # boom assemblies are centered at CG. Consist of boom, wing mount, motor, and motor mount. Treat as single object on each side
    # fl_boom : cd.aircraft.components.Fuselage = aircraft.comps["fl_boom"]
    # fr_boom : cd.aircraft.components.Fuselage = aircraft.comps["fr_boom"]
    # rl_boom : cd.aircraft.components.Fuselage = aircraft.comps["rl_boom"]
    # rr_boom : cd.aircraft.components.Fuselage = aircraft.comps["rr_boom"]

    # half_boom_OD = csdl.Variable(value=.75 * cd.Units.length.inch_to_m)
    # half_boom_ID = csdl.Variable(value=.625 * cd.Units.length.inch_to_m) 
    # half_boom_length = csdl.norm(fl_boom.nose_point - fl_boom.tail_point)
    half_boom_length = wing_boom_len/2
    # half_boom_volume = np.pi*((half_boom_OD/2)**2 - (half_boom_ID/2)**2) * half_boom_length
    half_boom_volume = np.pi*((wing_boom_OD/2)**2 - (wing_boom_ID/2)**2) * half_boom_length
    # !! from Madison: 3/4 inch x 36in length carbon fiber tube = 86.4 g
    # Volume = pi/4 * (0.75^2 - 0.5^2) * 0.0254^2 * 0.9144 = 1.4479e-4 m^3
    # Density = 0.0864 kg / 1.4479e-4 m^3 = 596.7 kg/m^3
    # note: a lot less than the other tube used in the wings
    # half_boom_density = 0.054 * ftin3_2_kgm3 # ?? get from AFL
    half_boom_density = 596.7 # kg/m^3
    half_boom_mass = half_boom_volume * half_boom_density


    left_boom_assembly = aircraft.comps["left_boom_assembly"]
    left_boom_assembly_mass_val = wing_boom_mount_mass_val + (half_boom_mass + lift_motor_mass_val + lift_motor_mount_mass_val + lift_rotor_mass_val)*2
    left_boom_assembly_mass = csdl.Variable(name="left_boom_assembly_mass", value = left_boom_assembly_mass_val) # kg
    left_boom_assembly.quantities.mass_properties.mass = left_boom_assembly_mass
    left_boom_assembly_CG = csdl.Variable(name="left_boom_assembly_CG", value = np.array([0, -wing_boom_y, wing_boom_z]))
    left_boom_assembly_CG = left_boom_assembly_CG.set(csdl.slice[0], wing_qc[0])
    left_boom_assembly.quantities.mass_properties.cg_vector = left_boom_assembly_CG

    right_boom_assembly = aircraft.comps["right_boom_assembly"]
    right_boom_assembly_mass = left_boom_assembly_mass
    right_boom_assembly.quantities.mass_properties.mass = right_boom_assembly_mass
    right_boom_assembly_CG = csdl.Variable(name="right_boom_assembly_CG", value = np.array([0, wing_boom_y, wing_boom_z]))
    right_boom_assembly_CG = right_boom_assembly_CG.set(csdl.slice[0], wing_qc[0])
    right_boom_assembly.quantities.mass_properties.cg_vector = right_boom_assembly_CG

    main_spar_OD = csdl.Variable(name="main_spar_OD", value=main_spar_OD_val)
    main_spar_ID = csdl.Variable(name="main_spar_ID", value=main_spar_ID_val)
    main_spar_density = half_boom_density
    main_spar : cd.aircraft.components.Fuselage = aircraft.comps["main_spar"]
    main_spar_length = csdl.norm(main_spar.nose_point - main_spar.tail_point)
    main_spar_volume = np.pi*((main_spar_OD/2)**2 - (main_spar_ID/2)**2) * main_spar_length
    main_spar_mass = main_spar_density * main_spar_volume
    # main_spar_mass = csdl.Variable(name="main_spar_mass", value = main_spar_mass)
    main_spar.quantities.mass_properties.mass = main_spar_mass
    main_spar.quantities.mass_properties.cg_vector = (main_spar.nose_point + main_spar.tail_point)/2

    tail_mount = aircraft.comps["tail_mount"]
    tail_mount_mass = csdl.Variable(name = "tail_mount_mass", value = tail_mount_mass_val)
    tail_mount.quantities.mass_properties.mass = tail_mount_mass
    tail_mount_half_length =csdl.Variable(name="tail_mount_half_length", value = np.array([tail_mount_length_val/2, 0, 0]))
    # this is approximate but expresses the CG of the tail mount as a function of the main spar length
    tail_mount_CG = main_spar.tail_point + tail_mount_half_length
    # tail_mount_CG = tail_mount_CG.set(csdl.slice[0], main_spar_length)
    tail_mount.quantities.mass_properties.cg_vector = tail_mount_CG

    wing_fuse_mount = aircraft.comps["wing_fuse_mount"]
    wing_fuse_mount_mass = csdl.Variable(name="wing_fuse_mount_mass", value = wing_fuse_mount_mass_val)
    # mass and CG pulled from CAD
    wing_fuse_mount.quantities.mass_properties.mass = wing_fuse_mount_mass
    wing_fuse_mount.quantities.mass_properties.cg_vector = csdl.Variable(name="wing_fuse_mount_cg", value = wing_fuse_mount_CG)

    base_config.assemble_system_mass_properties(update_copies=True)

    total_aircraft_mass = base_config.system.quantities.mass_properties.mass
    total_aircraft_mass.name = "total_aircraft_mass"
    total_aircraft_mass.set_as_constraint(upper=6*cd.Units.mass.pound_to_kg, scaler=.3) #This will probably fail optimization still.
    # total_aircraft_mass.set_as_constraint(upper=8*cd.Units.mass.pound_to_kg, scaler=.3) #This will probably fail optimization still.

    print(base_config.system.quantities.mass_properties.inertia_tensor.value)

def define_vlm_analysis(caddee: cd.CADDEE):
    """Define the analysis of performed on the aircraft."""
    cruise : cd.aircraft.conditions.CruiseCondition = caddee.conditions["cruise"]
    cruise_config = cruise.configuration
    mesh_container = cruise_config.mesh_container
    wing = cruise_config.system.comps['wing']
    tail = cruise_config.system.comps["h_tail"]

    # Re-evaluate meshes and compute nodal velocities
    cruise.finalize_meshes()

    # Set up VLM analysis
    vlm_mesh = mesh_container["vlm_mesh"]
    wing_lattice = vlm_mesh.discretizations["wing_camber_surface"]
    tail_lattice = vlm_mesh.discretizations["h_tail_camber_surface"]
    airfoil_upper_nodes = wing_lattice._airfoil_upper_para
    airfoil_lower_nodes = wing_lattice._airfoil_lower_para
    pressure_indexed_space : lfs.FunctionSetSpace = wing.quantities.pressure_space

    # run vlm solver
    lattice_coordinates = [wing_lattice.nodal_coordinates, tail_lattice.nodal_coordinates]
    lattice_nodal_velocitiies = [wing_lattice.nodal_velocities, tail_lattice.nodal_velocities]


    nasa_langley_airfoil_maker = ThreeDAirfoilMLModelMaker(
        airfoil_name="ls417",
        aoa_range=np.linspace(-12, 16, 50), 
        reynolds_range=[1e5, 2e5, 5e5, 1e6, 2e6, 4e6, 7e6, 10e6], 
        mach_range=[0., 0.2, 0.3, 0.4, 0.5, 0.6],
        num_interp=120,
    )
    Cl_model = nasa_langley_airfoil_maker.get_airfoil_model(quantities=["Cl"])
    Cd_model = nasa_langley_airfoil_maker.get_airfoil_model(quantities=["Cd"])
    Cp_model = nasa_langley_airfoil_maker.get_airfoil_model(quantities=["Cp"])
    alpha_stall_model = nasa_langley_airfoil_maker.get_airfoil_model(quantities=["alpha_Cl_min_max"])

    vlm_outputs = vlm_solver(
        mesh_list=[lattice_coordinates],
        mesh_velocity_list=[lattice_nodal_velocitiies],
        atmos_states=cruise.quantities.atmos_states,
        airfoil_alpha_stall_models=[alpha_stall_model],
        airfoil_Cd_models=[Cd_model],
        airfoil_Cl_models=[Cl_model],
        airfoil_Cp_models=[Cp_model], 
    )

    
    vlm_forces = vlm_outputs.total_force
    vlm_moments = vlm_outputs.total_moment

    if True:
        V_inf = cruise.parameters.speed
        rho_inf = cruise.quantities.atmos_states.density
        spanwise_Cp = vlm_outputs.surface_spanwise_Cp[0]
        spanwise_pressure = spanwise_Cp * 0.5 * rho_inf * V_inf**2
        spanwise_pressure = csdl.blockmat([[spanwise_pressure[0, :, 0:120].T()], [spanwise_pressure[0, :, 120:].T()]])
        
        pressure_function = pressure_indexed_space.fit_function_set(
            values=spanwise_pressure.reshape((-1, 1)), parametric_coordinates=airfoil_upper_nodes+airfoil_lower_nodes,
            regularization_parameter=1e-4,
        )

        if recorder.inline is True:
            wing.geometry.plot_but_good(color=pressure_function)
        
        # box_beam_mesh = mesh_container["beam_mesh"]
        # box_beam = box_beam_mesh.discretizations["wing_box_beam"]
        # beam_nodes = box_beam.nodal_coordinates

        right_wing_inds = list(wing.quantities.right_wing_geometry.functions)
        force_magnitudes, force_para_coords = pressure_function.integrate(wing.geometry, grid_n=30, indices=right_wing_inds)
        force_magnitudes:csdl.Variable = force_magnitudes.flatten()
        force_coords = wing.geometry.evaluate(force_para_coords)
        force_normals = wing.geometry.evaluate_normals(force_para_coords)
        force_vectors = force_normals*csdl.expand(force_magnitudes, force_normals.shape, 'i->ij')

        #Cut off here
        return vlm_outputs, force_coords, force_vectors


    # vlm_mesh = mesh_container["vlm_mesh"]
    # wing_camber_surface = vlm_mesh.discretizations["wing_camber_surface"]
    # h_tail_camber_surface = vlm_mesh.discretizations["h_tail_camber_surface"]

    # # lattice_coordinates = [wing_chord_surface.nodal_coordinates]
    # lattice_coordinates = [wing_camber_surface.nodal_coordinates, h_tail_camber_surface.nodal_coordinates]
    # # lattice_coordinates = [wing_nodes, h_tail_nodes]
    # # lattice_nodal_velocities = [wing_chord_surface.nodal_velocities]
    # lattice_nodal_velocities = [wing_camber_surface.nodal_velocities, h_tail_camber_surface.nodal_velocities]
    
    # # top one is the glitchy one
    # mark2_geom.plot_meshes(meshes=[wing_camber_surface.nodal_coordinates.value, h_tail_camber_surface.nodal_coordinates.value])
    # # mark2_geom.plot_meshes(meshes=[wing_nodes.value, h_tail_nodes.value])

    # vlm_output = vlm_solver(
    #     lattice_coordinates, 
    #     lattice_nodal_velocities, 
    #     atmos_states=cruise.quantities.atmos_states,
    #     airfoil_Cd_models=[None, None],
    #     airfoil_Cl_models=[None, None],
    #     airfoil_Cp_models=[None, None],
    #     airfoil_alpha_stall_models=[None, None],
    # )

    # return vlm_output

def run_beam(caddee: cd.CADDEE, vlm_output, force_coords, force_vectors):
    base_config = caddee.base_configuration
    aircraft = base_config.system
    wing = aircraft.comps["wing"] #changed this it might cause issues.
    # cruise : cd.aircraft.conditions.CruiseCondition = conditions["cruise"]
    # vlm_mesh = mesh_container["vlm_mesh"]
    # wing_lattice = vlm_mesh.discretizations["wing_chord_surface"]
    wing_span = csdl.sqrt(wing.parameters.AR * wing.parameters.S_ref)

    # # this sums the chordwise forces
    # # num_chordwise_panels = vlm_output.surface_panel_force_points[0][0].shape[0]
    # wing_spanwise_vlm_forces = csdl.Variable(value=np.zeros((wing_spanwise_panels,3)))
    # # this loop combines forces chordwise
    # for i in csdl.frange(wing_chordwise_panels):
    #     wing_spanwise_vlm_forces += vlm_output.surface_panel_forces[0][0][i]
    # # wing_spanwise_vlm_forces = np.sum(vlm_output.surface_panel_forces[0][0].value, axis=1)

    # 
    # num_chordwise_panels = vlm_output.surface_panel_force_points[0][0].shape[0]
    if wing_chordwise_panels % 2 == 0:
        middle_index = int(wing_chordwise_panels/2 - 1) # there is a better way to do this
    else:
        middle_index = int((wing_chordwise_panels - 1)/2)

    force_points = vlm_output.surface_panel_force_points[0][0][middle_index]

    num_nodes = force_points.shape[0]
    # beam_mesh = np.zeros((num_nodes, 3))
 
    beam_mesh = csdl.Variable(value=np.zeros((num_nodes, 3)))
    
    # beam_mesh = beam_mesh.set(csdl.slice[:,1], csdl.linear_combination(-wing_span/2, wing_span/2, num_nodes))
    beam_points = csdl.linear_combination(-wing_span/2, wing_span/2, num_nodes)

    for i in range(beam_points.shape[0]):
        beam_mesh = beam_mesh.set(csdl.slice[i,1], beam_points[i])

    # beam_mesh[:, 1]  = csdl.linear_combination(-wing_span/2, wing_span/2, num_nodes)

    # loads = csdl.Variable(value=np.zeros((wing_spanwise_vlm_forces.shape[0], 6)))

    # for j in range(3):
    #     for i in csdl.frange(loads[:,0].shape[0]):
    #         loads = loads.set(csdl.slice[i,j], wing_spanwise_vlm_forces[i,j])

    # loads = 5 * loads / 2

    SF = 1.5

    mapper = acu.NodalMap()
    force_map = mapper.evaluate(force_coords, beam_points.reshape((-1, 3)))
    beam_forces = force_map.T() @ force_vectors

    beam_forces_plus_moments = csdl.Variable(shape=(beam_forces.shape[0], 6), value=0)
    beam_forces_plus_moments = beam_forces_plus_moments.set(
        csdl.slice[:, 0:3], beam_forces
    )

    # !! From Madison, 4ft tube with 3/8in OD, 1/4in ID weighs 69.3g
    # Volume = (pi/4) * (OD^2 - ID^2) * L = (pi/4) * (0.375^2 - 0.25^2) * (0.0254)^2 * 1.2192 = 4.826E-5 m^3
    # Density = mass/volume = 0.0693kg / 4.826E-5 m^3 = 1.435 kg/m^3 (very close to original value of 1420)
    carbon_fiber = af.Material(name='carbon_fiber', E=96.2E9/SF, G = 3.16E9/SF, density = 1435)
    beam_radius = csdl.Variable(name="wing_spar_radius", value=0.0047)
    beam_radius.set_as_design_variable(lower=0.0015875, scaler=1E3) # needs to be larger than 1/8" for wiring purposes
    beam_radius_expanded = csdl.expand(beam_radius, (num_nodes - 1,))
    # beam_1_thickness = csdl.Variable(value=np.ones(num_nodes_1 - 1) * 0.001)
    beam_thickness = csdl.Variable(value=0.0015875) # beam_1_cs = af.CSTube(radius=beam_1_radius, thickness=beam_1_thickness)
    beam_thickness_expanded = csdl.expand(beam_thickness, (num_nodes - 1,))
    beam_cs = af.CSTube(radius=beam_radius_expanded, thickness=beam_thickness_expanded)
    beam = af.Beam(name='beam', mesh=beam_mesh, material=carbon_fiber, cs=beam_cs)
    beam.fix(node=int((beam.num_nodes - 1)/2))
    beam.add_boundary_condition(node=0, dof=[1, 1, 1, 1, 1, 1])
    beam.add_load(beam_forces_plus_moments)

    acc = csdl.Variable(value=np.array([0, 0, 9.81, 0, 0, 0]))
    frame = af.Frame(beams=[beam], acc=acc)

    frame.solve()

    beam_displacement = frame.displacement[beam.name]
    max_disp = csdl.norm(beam_displacement[-1])
    displacement_limit = wing_span*0.025
    r = displacement_limit - max_disp
    r.set_as_constraint(equals=0, scaler = 1)
    beam_ID_radius = beam_radius - beam_thickness

    return beam_radius, beam_ID_radius

def wing_mass_model(AR,S,m,p,t,spar_outer_diameter):
    #All inputs need to be CSDL variables!
    b = csdl.sqrt(AR*S) #ft #This should be a csdl variable at this point, check that.
    c = b/AR #ft #This should be a csdl variable at this point, check that.

    m_adjusted = 0.01*m  # maximum camber in % of chord # These should be CSDL variables as well
    p_adjusted = 0.10*p  # maximum camber position in tenths of chord
    t_adjusted = 0.01*t  # thickness in % of chord

    # Coefficients for 4 digit series
    a0 =  csdl.Variable(value=1.4845)
    a1 = csdl.Variable(value=-0.6300)
    a2 = csdl.Variable(value=-1.7580)
    a3 =  csdl.Variable(value=1.4215)
    a4 = csdl.Variable(value=-0.5075)

    n = 8 # number of points along the chord
    x_vals = np.linspace(0,c.value,n) # x coordinate of points along the chord
    x = csdl.Variable(value=x_vals,name="Wing_Weight_Model_X_Values")
    yc_vals  = np.zeros(n) # y coordinate of the camber line
    yc = csdl.Variable(shape =(n,),value=yc_vals,name="Wing_Weight_Model_YC_Values")
    dyc_vals = np.zeros(n) # gradient of the camber line
    dyc = csdl.Variable(shape =(n,),value=dyc_vals,name="Wing_Weight_Model_DYC_Values")
    yt_vals  = np.zeros(n) # thickness distribution
    yt = csdl.Variable(shape =(n,),value=yt_vals,name="Wing_Weight_Model_YT_Values")
    xu_vals  = np.zeros(n) # x coordinate of the upper surface
    xu = csdl.Variable(shape =(n,),value=xu_vals,name="Wing_Weight_Model_XU_Values")
    yu_vals  = np.zeros(n) # y coordinate of the upper surface
    yu = csdl.Variable(shape =(n,),value=yu_vals,name="Wing_Weight_Model_YU_Values")
    xl_vals  = np.zeros(n) # x coordinate of the lower surface
    xl = csdl.Variable(shape =(n,),value=xl_vals,name="Wing_Weight_Model_XL_Values")
    yl_vals  = np.zeros(n) # y coordinate of the lower surface
    yl = csdl.Variable(shape =(n,),value=yl_vals,name="Wing_Weight_Model_YL_Values")
    for i in csdl.frange(n):
        if  (x[i].value/c.value < p):
            yc  = yc.set(csdl.slice[i],(c*m/p**2)*(2*p*(x[i]/c)-(x[i]/c)**2))
            dyc = dyc.set(csdl.slice[i],((2*m)/p**2)*(p-(x[i]/c)))
        else:
            yc  = yc.set(csdl.slice[i],(c*m/(1-p)**2)*(1-2*p+2*p*(x[i]/c)-(x[i]/c)**2))
            dyc = dyc.set(csdl.slice[i],((2*m)/(1-p)**2)*(p-(x[i]/c)))
    for i in csdl.frange(n):
        yt = yt.set(csdl.slice[i],(t*c)*(a0*csdl.sqrt(x[i]/c)+a1*(x[i]/c)+a2*(x[i]/c)**2+a3*(x[i]/c)**3+a4*(x[i]/c)**4))
        teta  = csdl.arctan(dyc[i])
        xu = xu.set(csdl.slice[i],x[i]  - yt[i]*csdl.sin(teta))
        xl = xl.set(csdl.slice[i],x[i]  + yt[i]*csdl.sin(teta))
        yu = yu.set(csdl.slice[i],yc[i]  + yt[i]*csdl.cos(teta))
        yl = yl.set(csdl.slice[i],yc[i]  - yt[i]*csdl.cos(teta))

    # plot.xlim(-0.2,c+0.2)
    # plot.ylim(-c/3,c/3)
    # plot.plot(xu,yu,color='deepskyblue')   
    # plot.plot(xl,yl,color='deepskyblue')
    # plot.plot(x,yc,'g--') 

    # upper = np.trapz(yu,xu)
    # lower = np.trapz(yl,xl)
    upper = csdlTrapIntegrator(xu,yu)
    lower = csdlTrapIntegrator(xl,yl)

    # upperNP = np.trapz(yu,xu)
    # lowerNP = n.trapz(yl,xl)

    total_area = upper + -(lower)

    foam_density = csdl.Variable(value=1.5) #lb/ft**3 #This is an aproximation to get the code working
    volume_wing = total_area * b
    volume_spars = 2*np.pi*(spar_outer_diameter/2/12)**2*b #Volume of both spars in ft^3, spar outer diamter in in^3
    volume_total = volume_wing-volume_spars
    mass = volume_total*foam_density
    return mass

def csdlTrapIntegrator(x,y):
    if x.shape != y.shape:
        raise ValueError("X data and Y data must have same length")
    #integral = csdl.Variable(shape = (1,0) name = 'Area of Airfoil', value = 0)
    integral = 0
    integral = csdl.Variable(shape=(1,),name="Airfoil_Area",value = integral)
    for i in csdl.frange(x.shape[0] - 1):
        integral = integral + (x[i+1] - x[i]) * (y[i] + y[i+1]) / 2.0
    return integral


def define_analysis(caddee: cd.CADDEE, vlm_output):
    conditions = caddee.conditions
    cruise = conditions["cruise"]
    cruise_config = cruise.configuration
    mesh_container = cruise_config.mesh_container
    base_config = caddee.base_configuration
    aircraft = base_config.system
    vlm_forces = vlm_output.total_force
    vlm_moments = vlm_output.total_moment

    ###################################### BEM STUFF, not using qst (quasi-steady transition)
    # Drag build-up
    drag_build_up_model = cd.aircraft.models.aero.compute_drag_build_up

    wing = aircraft.comps["wing"]
    fuselage = aircraft.comps["fuselage"]
    h_tail = aircraft.comps["h_tail"]
    v_tail = aircraft.comps['v_tail']
    # booms = [aircraft.comps["boom_FR"], aircraft.comps["boom_FL"], aircraft.comps["boom_BR"], aircraft.comps["boom_BL"]]

    #This fails for weird reasons having to do with none values for certain drag parameters.
    parasitic_drag_buildup = drag_build_up_model(cruise.quantities.ac_states, cruise.quantities.atmos_states,
                                        wing.parameters.S_ref, [wing, fuselage, h_tail, v_tail])
    
    # !! change here
    Cd0 = 0.0429 
    parasite_drag = -0.5 * cruise.quantities.atmos_states.density * cruise.parameters.speed**2 * wing.parameters.S_ref * Cd0
    drag = csdl.Variable(name="drag", value=np.zeros((1,3)))
    drag = drag.set(csdl.slice[0,0],parasite_drag)

    cruise_power = {}

    # BEM solver
    # rotor_meshes = mesh_container["rotor_meshes"]
    # cruise_rotor_mesh = rotor_meshes.discretizations["cruise_prop_mesh"]
    # mesh_vel = cruise_rotor_mesh.nodal_velocities
    # cruise_rpm = csdl.Variable(name="cruise_pusher_rpm", shape=(1, ), value=14000)
    # cruise_rpm.set_as_design_variable(upper=14238, lower=1200, scaler=1/14238) #,
    # bem_inputs = RotorAnalysisInputs(mesh_parameters = cruise_rotor_mesh, mesh_velocity = mesh_vel, rpm = cruise_rpm)
    # bem_model = BEMModel(num_nodes=1, airfoil_model=NACA4412MLAirfoilModel())
    # bem_outputs = bem_model.evaluate(bem_inputs)
    # cruise_power = bem_outputs.total_power
    # cruise.quantities.power = cruise_power
    # cruise_power.name = "Cruise Power [W]"

    # Discretization
    num_nodes = 1 # Number of evaluation points
    num_radial = 35 # Number of radial sections
    num_azimuthal = 1 # Number of azimuthal sections (can be 1 for axisymmetric flow)

    num_blades = 2

    # Simple 1D airfoil model
    # Specify polar parameters

    # Create airfoil model
    airfoil_model = NACA4412MLAirfoilModel()

    # Set up rotor analysis inputs
    # 1) thrust vector and origin (origin is the rotor hub location and only needed for computing moments)
    thrust_vector=csdl.Variable(name="thrust_vector", value=np.array([1, 0, 0])) # Thrust vector in the x-direction
    thrust_origin=csdl.Variable(name="thrust_origin", value=np.array([0. ,0., 0.]))

    # 2) Rotor geometry 
    # chord and twist profiles (linearly varying from root to tip)
    chord_profile=csdl.Variable(name="chord_profile", value=np.linspace(0.025, 0.005, num_radial))
    twist_profile=csdl.Variable(name="twist_profile", value=np.linspace(np.deg2rad(50), np.deg2rad(20), num_radial)) # Twist in RADIANS
    # Radius of the rotor
    radius = csdl.Variable(name="radius", value=8/2*units.length.inch_to_m)

    # 3) Mesh velocity: vector of shape (num_nodes, 3) where each row is the 
    # free streamvelocity vector (u, v, w) at the rotor center
    mesh_vel_np = np.zeros((num_nodes, 3))
    mesh_vel_np[:, 0] = 21 # Free stream velocity in the x-direction in m/s (cruise velocity)
    mesh_velocity = csdl.Variable(value=mesh_vel_np)
    # Rotor speed in RPM
    rpm = csdl.Variable(name="rpm", value=14000 * np.ones((num_nodes,)))
    rpm.set_as_design_variable(upper=14328,lower=999,scaler=1/14000)

    # 4) Assemble inputs
    # mesh parameters
    bem_mesh_parameters = RotorMeshParameters(
        thrust_vector=thrust_vector,
        thrust_origin=thrust_origin,
        chord_profile=chord_profile,
        twist_profile=twist_profile, 
        radius=radius,
        num_radial=num_radial,
        num_azimuthal=num_azimuthal,
        num_blades=num_blades,
        norm_hub_radius=0.2,
    )
    # rotor analysis inputs
    inputs = RotorAnalysisInputs(
        rpm=rpm,
        mesh_parameters=bem_mesh_parameters,
        mesh_velocity=mesh_velocity,
    )
    # Instantiate and run BEM model
    bem_model = BEMModel(
        num_nodes=num_nodes,
        airfoil_model=airfoil_model,
        integration_scheme='trapezoidal',
    )
    bem_outputs = bem_model.evaluate(inputs=inputs) 

    cruise_power = bem_outputs.total_power
    thrust = bem_outputs.total_thrust
    cruise.quantities.power = cruise_power
    cruise_power.name = "Cruise Power [W]"

    # bem_outputs.forces = bem_outputs.forces/3
    # vlm_forces = vlm_forces.set(csdl.slice[0,2],vlm_forces[0][2] * 0.82)

    # total forces and moments
    # !! added drag back in
    total_forces_cruise, total_moments_cruise = cruise.assemble_forces_and_moments(
        [vlm_forces, bem_outputs.forces, drag], [vlm_moments, bem_outputs.moments] #removed drag_build_up
    )

    # eom
    eom_model = cd.aircraft.models.eom.SixDofEulerFlatEarthModel()
    accel_cruise = eom_model.evaluate(
        total_forces=total_forces_cruise,
        total_moments=total_moments_cruise,
        ac_states=cruise.quantities.ac_states,
        ac_mass_properties=cruise_config.system.quantities.mass_properties
    )
    accel_norm_cruise = accel_cruise.accel_norm
    accel_norm_cruise.name = "cruise_trim"

    #This is how the trim residual is set.
    # accel_norm_cruise.set_as_constraint(upper=0.1, lower=-0.1, scaler=10)
        # Setting force equilibrium constraints
    force_norm = csdl.norm(total_forces_cruise)
    moment_norm = csdl.norm(total_moments_cruise)

    force_norm.name = "total_forces_norm"
    moment_norm.name = "total_moments_norm"

    force_norm.set_as_constraint(upper=0.1, lower=-0.1, scaler=10)
    moment_norm.set_as_constraint(upper=0.1, lower=-0.1, scaler=10)
    force_norm.set_as_constraint(equals=0, scaler=1e-4)
    moment_norm.set_as_constraint(equals=0., scaler=1e-4)

    # Performing linearized stability analysis
    long_stability_results = cruise.perform_linear_stability_analysis(
        total_forces=total_forces_cruise,
        total_moments=total_moments_cruise,
        ac_states=cruise.quantities.ac_states,
        mass_properties=cruise_config.system.quantities.mass_properties,
    )

    #Static Margin
    #assume neutral point at c/4 OR find the aerodynamic center and use that.
    # constraining CG might be unnecessary
    # CG = aircraft.quantities.mass_properties.cg_vector[0]
    # wing_qc = 0.75 * wing.LE_center + 0.25 * wing.TE_center
    # CG.set_as_constraint(upper=wing_qc[0].value*0.95, lower=wing_qc[0].value*1.1, scaler=4)

    # wing_cop = vlm_output.surface_cop[0][0][0]
    # h_stab_cop = vlm_output.surface_cop[1][0][0]
    # total_area = wing.parameters.S_ref + h_tail.parameters.S_ref
    # total_cop = wing_cop * wing.parameters.S_ref / total_area + h_stab_cop * h_tail.parameters.S_ref / total_area
    # b = csdl.sqrt(wing.parameters.AR*wing.parameters.S_ref) #ft #This should be a csdl variable at this point, check that.
    # c = b/wing.parameters.AR #ft #This should be a csdl variable at this point, check that.
    # # NP = c/4
    # static_margin = (total_cop-CG)/c
    # static_margin.set_as_constraint(upper=0.18,lower=0.2,scaler=5)
    
    # #Longitudinal dynamic stability THIS CAUSES ERROR
    # t2d = long_stability_results.time_2_double_phugoid
    # t2d.name = "time2double"
    # #t2d.set_as_constraint()
    # drph = long_stability_results.damping_ratio_phugoid
    # drph.set_as_constraint(upper=0.045,lower=0.04,scaler=1/0.04)
    # drsp = long_stability_results.damping_ratio_short_period
    # drsp.set_as_constraint(lower=0.7,upper=2,scaler=1/0.7)
    
    # mil 8785C find values for those things.

    ########### Mission Power Analysis
    cruise_velocity = cruise.parameters.speed
    R = csdl.Variable(value=10e3) #m
    cruise_time = R/cruise_velocity #s
    energy = cruise.quantities.power * cruise_time #Ws
    energy.set_as_constraint(lower=0,upper = 131868, scaler = 1/131868)
    energy.name = ("Energy in Ws (J)")
    
    ER = energy/R
    ER.name = "E/R"

    #Davids wing weight model testing stuff
    # aircraft = caddee.base_configuration.system
    # wing = aircraft.comps["wing"]
    #weight = wing_weight_model(wing.parameters.AR,wing.parameters.S_ref,csdl.Variable(value=4),csdl.Variable(value=4),csdl.Variable(value=12),csdl.Variable(value=0.2))
    #print(f"The calculated wing weight is {weight}")

    #SET AS OBJECTIVE
    ER.set_as_objective(scaler=1/21) 
    #cruise.quantities.power.set_as_objective()
    #Equivalent bc cruise speed is fixed

if __name__ == "__main__": #I like doing this because it makes it clear where the main executiom begins and also I can collapse it : )
    # Run the code (forward evaluation)

    # make instance of CADDEE class
    caddee = cd.CADDEE()

    #Define configuration in CADDEE
    # wing_nodes, h_tail_nodes = define_base_config(caddee=caddee)
    # wing_camber_surface, h_tail_camber_surface = define_base_config(caddee=caddee)
    define_base_config(caddee=caddee)
    # define_base_config(caddee=caddee)

    #Define flight regimes in CADDEE
    define_conditions(caddee=caddee)

    #Run cruise vlm analysis
    vlm_output, force_coords, force_vectors = define_vlm_analysis(caddee=caddee)

    #Define masses and set up mass models (how do masses change with change in parameters)
    define_mass_properties(caddee=caddee,vlm_output=vlm_output, force_coords = force_coords, force_vectors = force_vectors)

    #What analysis are we performing, this calls the other define configuration functions
    define_analysis(caddee=caddee,vlm_output=vlm_output)

    # Run optimization (Now actually do the optimization using whats been recorded)
    jax_sim = csdl.experimental.JaxSimulator(
        recorder=recorder, # Turn off gpu if none available
    )

    # Check analytical derivatives against finite difference
    jax_sim.check_optimization_derivatives()

    # Make CSDLAlphaProblem and initialize optimizer
    
    # !! comment out to check for issues with setup
    problem = CSDLAlphaProblem(problem_name="er_minimization", simulator=jax_sim)
    optimizer = SLSQP(problem, solver_options={'maxiter': 3000, 'ftol': 1E-6}, turn_off_outputs=True)
    #optimizer = SLSQP(problem=problem)
    # Solve optimization problem
    optimizer.solve()
    optimizer.print_results()
    recorder.execute()

    # Plot geometry after optimization
    mark2_geom.plot(additional_plotting_elements=plotting_elements, opacity=0.5, color="#00629B")

    # Print design variables, constraints, objectives after optimization
    for dv in recorder.design_variables.keys():
        print(dv.name, dv.value)

    for c in recorder.constraints.keys():
        print(c.name, c.value)

    for obj in recorder.objectives.keys():
        print(obj.name, obj.value)