import CADDEE_alpha as cd
import csdl_alpha as csdl
import numpy as np
from lsdo_airfoil.core.three_d_airfoil_aero_model import ThreeDAirfoilMLModelMaker
from VortexAD.core.vlm.vlm_solver import vlm_solver
from modopt import CSDLAlphaProblem, SLSQP
from CADDEE_alpha.utils.units import Units
import lsdo_function_spaces as lfs
import aframe as af
import aeroelastic_coupling_utils as acu
import os #for relative path import
import math as mth
from BladeAD.core.airfoil.ml_airfoil_models.NACA_4412.naca_4412_model import NACA4412MLAirfoilModel
from BladeAD.utils.parameterization import BsplineParameterization
from BladeAD.core.BEM.bem_model import BEMModel
from BladeAD.core.pitt_peters.pitt_peters_model import PittPetersModel
from BladeAD.utils.var_groups import RotorAnalysisInputs, RotorMeshParameters
units = Units()

#TO DO
#Updated weights model -> create static stability criteria (both vtol and FW) -> informs spar length (tail arm), boom placement

# Start the CSDL recorder
recorder = csdl.Recorder(inline=True, expand_ops=True)
recorder.start()

# import geometry
dirname = os.path.dirname(__file__) #relative path
filename = os.path.join(dirname, 'mark2.stp')
mark2_geom = cd.import_geometry(filename)
plotting_elements = mark2_geom.plot(show=False, opacity=0.5, color='#FFCD00')

ft2m = 0.3048
N2lb = 4.44822
g = 9.81
ftin3_2_kgm3 = 1494.7149

# define INITIAL values for design parameters
weight = 6 * 4.44822
# fuselage
fuselage_length = 1.25 * ft2m
# fuselage seth version
fuse_len = 1.25 * ft2m
fuse_perim = 1.5 * ft2m
fuse_t = 3/16/12 * ft2m
# wing
span = 4* ft2m
wingchord = 9/12 * ft2m
wing_S = span * wingchord
wing_AR = span/wingchord
wing_taper = 1
# h-stab
h_stab_span = 1.25 * ft2m
h_stab_chord = 5/12 * ft2m
h_stab_AR = h_stab_span/h_stab_chord
h_stab_S = h_stab_span * h_stab_chord
h_stab_taper = 1
h_stab_tc = 0.12
h_stab_area = h_stab_span * h_stab_chord
# v-stab
v_stab_span = 1.08/2 * ft2m
v_stab_chord = 5/12 * ft2m
v_stab_AR = v_stab_span/v_stab_chord
v_stab_S = v_stab_span * v_stab_chord
v_stab_taper = 1
v_stab_tc = 0.12
v_stab_area = v_stab_span * v_stab_chord
# lift rotors
lift_rotor_d = 14/12 * ft2m
# cruise propeller
cruise_prop_d = 8/12 * ft2m
# main spar
main_spar_len = 3 * ft2m
# wing booms
wing_boom_len = 30/12 * ft2m
wing_boom_y = 0.457

# cruise conditions
alt = 0
dist = 7000
pitch = 0
cruise_v = 70 * ft2m
sos = 1100 * ft2m
mach =cruise_v/sos

# weights
w_total = 6 * cd.Units.mass.pound_to_kg * g
m_battery = 0.372
l_battery = 0.155 # [m]
w_battery = 0.048 # [m]
h_battery = 0.033 # [m]

# https://www.dupont.com/content/dam/dupont/amer/us/en/performance-building-solutions/public/documents/en/styrofoam-panel-core-20-xps-pis-43-d100943-enus.pdf
density_foam = 24 # kg/m3

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
    fuselage = cd.aircraft.components.Fuselage(length=fuselage_length, geometry=fuselage_geometry)
    fuselage.quantities.drag_parameters.characteristic_length = fuselage_length #WHY THE HELL IS THIS ALWAYS NONE!!!! #I think CADDEE is supposed to do this by default.
    aircraft.comps["fuselage"] = fuselage
    
    # Treating the boom like a funky fresh fuselage. Its dimensions will change but those of the main fuselage will not
    main_spar_geometry = aircraft.create_subgeometry(search_names=["MainSpar"])
    main_spar = cd.aircraft.components.Fuselage(
        length=main_spar_len, 
        geometry=main_spar_geometry,
        )
    main_spar.quantities.drag_parameters.characteristic_length = main_spar_len #Refer to above.
    # assign main spar component to aircraft
    aircraft.comps["main_spar"] = main_spar

    # Make wing geometry from aircraft component and instantiate wing component
    wing_geometry = aircraft.create_subgeometry(search_names=["Wing"])
    aspect_ratio = csdl.Variable(name="wing_aspect_ratio", value = wing_AR)
    wing_span = csdl.Variable(name="wing_span", value = span)
    wing_chord = csdl.Variable(name="wing_chord", value=wingchord)
    wing_area = csdl.Variable(name="wing_area", value = wing_S)
    wing_root_twist = csdl.Variable(name="wing_root_twist", value=0)
    wing_tip_twist = csdl.Variable(name="wing_tip_twist", value=0)
    
    # Set design variables for wing
    # aspect_ratio.set_as_design_variable(upper=1.2 * wing_AR, lower=0.8 * wing_AR, scaler=1/8)
    # wing_area.set_as_design_variable(upper=1.2 * wing_S, lower=0.8 * wing_S, scaler=1/16)
    wing_span.set_as_design_variable(upper=1.5 * span, lower = 0.8 * span, scaler=1/8)
    wing_chord.set_as_design_variable(upper = 1.2 * wingchord, lower = 0.8 * wingchord, scaler = 1/16)
    wing_root_twist.set_as_design_variable(upper=np.deg2rad(5), lower=np.deg2rad(-5), scaler=4)
    wing_tip_twist.set_as_design_variable(upper=np.deg2rad(10), lower=np.deg2rad(-10), scaler=2)
    
    wing = cd.aircraft.components.Wing(
        AR=aspect_ratio, S_ref=wing_area, 
        taper_ratio=wing_taper, root_twist_delta=wing_root_twist,
        tip_twist_delta=wing_tip_twist, geometry=wing_geometry
    )

    wing.quantities.drag_parameters.characteristic_length = wing_chord
    # Assign wing component to aircraft
    aircraft.comps["wing"] = wing

    # Connect wing to fuselage at the quarter chord
    base_config.connect_component_geometries(fuselage, wing, 0.75 * wing.LE_center + 0.25 * wing.TE_center)
    # base_config.connect_component_geometries(main_spar, h_tail, h_tail.TE_center)

    #Making hstab parameters changeable.
    h_stab_AR = h_stab_span/h_stab_chord #why
    h_stab_AR = csdl.Variable(name="hstab_aspect_ratio", value=h_stab_AR)
    h_stab_root_twist = csdl.Variable(name="h_stab_root_twist", value=np.deg2rad(0))
    h_stab_tip_twist = csdl.Variable(name="h_stab_tip_twist", value=np.deg2rad(0))

    # Set design variables for wing
    h_stab_AR.set_as_design_variable(upper=1.5 * wing_AR, lower=0.5 * wing_AR, scaler=1/8)
    h_stab_root_twist.set_as_design_variable(upper=np.deg2rad(5), lower=np.deg2rad(-5), scaler=4)
    h_stab_tip_twist.set_as_design_variable(upper=np.deg2rad(10), lower=np.deg2rad(-10), scaler=2)

    # Make horizontal tail geometry & component
    h_tail_geometry = aircraft.create_subgeometry(search_names=["HStab"])
    h_tail = cd.aircraft.components.Wing(
        AR=h_stab_AR, S_ref=h_stab_area, taper_ratio=h_stab_taper, 
        geometry=h_tail_geometry, root_twist_delta=h_stab_root_twist, tip_twist_delta=h_stab_tip_twist
    )

    # Assign tail component to aircraft
    aircraft.comps["h_tail"] = h_tail
    #base_config.connect_component_geometries(fuselage, h_tail, h_tail.TE_center) #.TE_Center doesnt work for some reason

    # Make vertical tail geometry & componen
    v_tail_geometry = aircraft.create_subgeometry(search_names=["VStab"])

    v_tail_AR = csdl.Variable(name="v_tail_AR", value=v_stab_AR)
    v_tail_area = csdl.Variable(name="v_tail_area", value=v_stab_S)

    # v_tail_AR.set_as_design_variable(lower=0.8 * v_stab_AR, upper=1.5 * v_stab_AR, scaler=1/4)
    # v_tail_area.set_as_design_variable(lower=0.8 * v_stab_S, upper=1.2 * v_stab_S, scaler=1/4)
    
    v_tail = cd.aircraft.components.Wing(
        AR=v_tail_AR, S_ref=v_tail_area, geometry=v_tail_geometry, 
        skip_ffd=True, orientation="vertical"
    )

    # Assign v-tail component to aircraft
    aircraft.comps["v_tail"] = v_tail

    # Connect h-tail to spar?
    base_config.connect_component_geometries(main_spar, h_tail, h_tail.TE_center)

    #Booms
    wing_boom_length = csdl.Variable(value=wing_boom_len,name='Wing Boom Length')
    wing_boom_length.set_as_constraint(upper = 50/12*ft2m, lower = 1*ft2m, scaler=1e-1) #these limits are currently arbitrary.

    fl_boom_geom = aircraft.create_subgeometry(search_names = ['FrontLeftBoom'])
    fl_boom = cd.aircraft.components.Fuselage(length=wing_boom_length/2, geometry=fl_boom_geom)
    aircraft.comps["fl_boom"] = fl_boom

    rl_boom_geom = aircraft.create_subgeometry(search_names = ['BackLeftBoom'])
    rl_boom = cd.aircraft.components.Fuselage(length=wing_boom_length/2, geometry=rl_boom_geom)
    aircraft.comps["rl_boom"] = rl_boom

    fr_boom_geom = aircraft.create_subgeometry(search_names = ['FrontRightBoom'])
    fr_boom = cd.aircraft.components.Fuselage(length=wing_boom_length/2, geometry=fr_boom_geom)
    aircraft.comps["fr_boom"] = fr_boom

    rr_boom_geom = aircraft.create_subgeometry(search_names = ['BackRightBoom'])
    rr_boom = cd.aircraft.components.Fuselage(length=wing_boom_length/2, geometry=rr_boom_geom)
    aircraft.comps["rr_boom"] = rr_boom

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

    # # lift rotors to lift booms
    # base_config.connect_component_geometries(fr_boom, fr_prop, fr_boom.nose_point)
    # base_config.connect_component_geometries(fl_boom, fl_prop, fl_boom.nose_point)
    # base_config.connect_component_geometries(rr_boom, rr_prop, rr_boom.tail_point)
    # base_config.connect_component_geometries(rl_boom, rl_prop, rl_boom.tail_point)
    # main spar to fuselage
    base_config.connect_component_geometries(main_spar, fuselage, main_spar.nose_point)
    # wing booms to wing
    base_config.connect_component_geometries(fr_boom, wing, fr_boom.tail_point)
    base_config.connect_component_geometries(fl_boom, wing, fl_boom.tail_point)
    base_config.connect_component_geometries(rr_boom, wing, rr_boom.nose_point)
    base_config.connect_component_geometries(rl_boom, wing, rl_boom.nose_point)


    ## MAKE MESHES ###################################################################
    # Meshing
    mesh_container = base_config.mesh_container

    # H-Tail 
    tail_chord_surface = cd.mesh.make_vlm_surface(
        wing_comp=h_tail,
        num_chordwise=1, 
        num_spanwise=4, # ? decreased for speed, bump this up later
    )

    # Wing chord surface (lifting line)
    wing_chord_surface = cd.mesh.make_vlm_surface(
        wing_comp=wing,
        num_chordwise=3, # ? decreased for speed, bump this up later
        num_spanwise=4,
    )
    vlm_mesh = cd.mesh.VLMMesh()
    vlm_mesh.discretizations["wing_chord_surface"] = wing_chord_surface
    vlm_mesh.discretizations["h_tail_chord_surface"] = tail_chord_surface
    # vlm_mesh.discretizations["v_tail_chord_surface"] = wing_chord_surface

    num_radial = 5
    cruise_prop_geom = aircraft.create_subgeometry(search_names=["Main Propeller"]) #get geo from openvsp
    cruise_prop = cd.aircraft.components.Rotor(radius=6, geometry=cruise_prop_geom, compute_surface_area=False, skip_ffd=True) #make CADDEE component
    cruise_prop_mesh = cd.mesh.make_rotor_mesh( #made bladeAD rotor mesh (not CADDEE mesh)
        cruise_prop, num_radial=num_radial, num_azimuthal=1, num_blades=2
    )
    rotor_meshes = cd.mesh.RotorMeshes() #make caddee-compatible mesh holder
    cruise_prop_mesh.twist_profile = csdl.Variable(shape=(num_radial, ), value=np.deg2rad(np.linspace(50., 20., num_radial))) #These are taken from other code, make them accurate for us.
    cruise_prop_mesh.chord_profile = csdl.Variable(shape=(num_radial, ), value=np.linspace(0.24, 0.08, num_radial))
    rotor_meshes.discretizations["cruise_prop_mesh"] = cruise_prop_mesh #Assign bladeAD rotor mesh to caddee rotor mesh container

    #Connect cruise prop
    base_config.connect_component_geometries(fuselage, cruise_prop, connection_point=fuselage.nose_point)

    # plot meshes
    # mark2_geom.plot_meshes(meshes=[wing_chord_surface.nodal_coordinates.value, tail_chord_surface.nodal_coordinates.value])
    
    # Assign mesh to mesh container
    mesh_container["vlm_mesh"] = vlm_mesh
    mesh_container["rotor_meshes"] = rotor_meshes

    # Set up the geometry: this will run the inner optimization
    base_config.setup_geometry(plot=False)

    # tail moment arm
    wing_qc = 0.75 * wing.LE_center + 0.25 * wing.TE_center
    h_tail_qc = 0.75 * h_tail.LE_center + 0.25 * h_tail.TE_center
    tail_moment_arm = csdl.norm(wing_qc - h_tail_qc)
    print("tail moment arm", tail_moment_arm.value)

    # Assign base configuration to CADDEE instance
    caddee.base_configuration = base_config

def define_conditions(caddee: cd.CADDEE):
    conditions = caddee.conditions
    base_config = caddee.base_configuration

    pitch_angle = csdl.Variable(name="pitch_angle", value=0)
    pitch_angle.set_as_design_variable(upper=np.deg2rad(5), lower=np.deg2rad(-5), scaler=4)
    cruise = cd.aircraft.conditions.CruiseCondition(
        altitude=0,
        range=100,
        pitch_angle=pitch_angle,
        speed=21.336,
    )
    cruise.configuration = base_config.copy()
    conditions["cruise"] = cruise

def define_mass_properties(caddee : cd.CADDEE):
    """Define the mass properties of the aircraft."""

    base_config = caddee.base_configuration
    aircraft = base_config.system

    conditions = caddee.conditions
    cruise : cd.aircraft.conditions.CruiseCondition = conditions["cruise"]
    #dynamic_pressure = 0.5 * cruise.quantities.atmos_states.density * cruise.parameters.speed**2

    design_gross_weight = csdl.Variable(name="design_gross_weight", value=w_total)
    # fuel_weight = csdl.Variable(name="fuel_weight", value=250*cd.Units.mass.pound_to_kg)
    fuel_weight = 0
    
    battery = aircraft.comps["battery"]
    battery_mass = csdl.Variable(name="battery_mass", value=m_battery)
    # position pulled from CAD
    battery_x = 7.32*cd.Units.length.inch_to_m
    battery.quantities.mass_properties.mass = battery_mass
    battery_x = csdl.Variable(name="battery_x", value=battery_x)
    battery_position = csdl.Variable(name="battery_position", value=np.zeros((3)))
    battery_position = battery_position.set(csdl.slice[0],battery_x)
    battery.quantities.mass_properties.cg_vector = battery_position # csdl.Variable(name="battery_cg", shape=(3,), value = np.array([battery_x, 0, 0]))
    battery_x.set_as_design_variable(lower=5*cd.Units.length.inch_to_m, upper=10*cd.Units.length.inch_to_m, scaler=1)

    wing : cd.aircraft.components.Wing = aircraft.comps["wing"]
    wing_center = (wing.LE_center + wing.TE_center) / 2
    wing_qc = 0.75 * wing.LE_center + 0.25 * wing.TE_center

    # approximate the wing cross-section area as an ellipse, rectangle, and triangle

    # these values need to be defined in terms of values passed into the wing component
    wing_span = csdl.sqrt(wing.parameters.AR * wing.parameters.S_ref)
    wing_chord = csdl.sqrt(wing.parameters.S_ref / wing.parameters.AR)
    wing_thickness_to_chord = 0.12 * wing_chord

    wing_max_t = wing_chord*wing_thickness_to_chord
    wing_ellipse_a = 0.3*wing_chord
    wing_ellipse_b = 1/2 * wing_max_t
    wing_ellipse_area = np.pi * wing_ellipse_a * wing_ellipse_b

    wing_rectangle_area = 0.2*wing_chord * wing_max_t

    wing_triangle_area = 1/2 * wing_max_t * 0.5*wing_chord

    wing_cross_section_area = wing_ellipse_area + wing_rectangle_area + wing_triangle_area

    wing_volume = wing_cross_section_area * wing_span

    wing_mass = wing_volume * density_foam
    wing.quantities.mass_properties.mass = wing_mass + fuel_weight
    wing.quantities.mass_properties.cg_vector = 0.56 * wing.LE_center + 0.44 * wing.TE_center # CG is around 44.4% of chord for 4412

    #AFRAME INTEGRATED HERE!!!!!!!!
    
    #might need to save VLM outputs globally

    #beam_radius, beam_ID_radius = run_beam(caddee=caddee, vlm_outputs=vlm_outputs)



    beam_radius = csdl.Variable(value=0.25)
    beam_ID_radius = csdl.Variable(value = 0.22)

    # wing_spar_OD = 0.375 * cd.Units.length.inch_to_m
    # wing_spar_ID = 0.25 * cd.Units.length.inch_to_m
    wing_spar_volume = np.pi * ( beam_radius**2 - beam_ID_radius**2 ) * wing_span

    wing_spar_density = 0.054 * ftin3_2_kgm3

    wing_spars_mass = wing_spar_density * wing_spar_volume * 2 # two spars

    wing_spars = aircraft.comps["wing_spars"]
    wing_spars.quantities.mass_properties.mass = wing_spars_mass
    # CG pulled from CAD of skeleton assembly (battery not included)
    wing_spars.quantities.mass_properties.cg_vector = wing_qc 

    # Even though the fuselage geometry includes the nosecone, this mass calculation does not! The nosecone mass is part of the skeleton
    fuselage : cd.aircraft.components.Fuselage = aircraft.comps["fuselage"]
    density_fuse_foam = 11 # [kg/m^3] double check this
    fuse_volume = fuse_len * fuse_perim * fuse_t + 2.5/12/16 * fuse_len + fuse_t # include extra bit on bottom
    fuselage_mass = fuse_volume * density_fuse_foam
    fuselage_mass = csdl.Variable(name="fuselage_mass", value = fuselage_mass)
    fuselage.quantities.mass_properties.mass = fuselage_mass
    # fuselage.quantities.mass_properties.cg_vector = wing_center + np.array([0., 0., 0.5])
    fuselage.quantities.mass_properties.cg_vector = wing_qc

    h_tail : cd.aircraft.components.Wing = aircraft.comps["h_tail"]
    
    # approximate the cross-sectional area of the h-stab as an ellipse meeting a triangle at quarter chord

    # since planform area and AR are the design variables, do I need to recalculate span, chord in terms of them or express cross sectional area in terms of them?
    # instead I set span and chord to the design variables (for now)
    # I think it also fine to just rely on h_tail.parameters
    # approximate the NACA 0012 as an ellipse and a triangle

    h_tail_span = csdl.sqrt(h_tail.parameters.AR * h_tail.parameters.S_ref)
    h_tail_chord = csdl.sqrt(h_tail.parameters.S_ref / h_tail.parameters.AR)
    h_tail_thickness_to_chord = 0.12 * h_tail_chord

    h_tail_max_t = h_tail_chord*h_tail_thickness_to_chord
    h_tail_ellipse_a = 0.3*h_tail_chord
    h_tail_ellipse_b = 1/2 * h_tail_max_t
    h_tail_ellipse_area = np.pi * h_tail_ellipse_a * h_tail_ellipse_b

    h_tail_triangle_area = 1/2 * h_tail_max_t * 0.7*h_tail_chord

    h_tail_cross_section_area = h_tail_ellipse_area + h_tail_triangle_area

    h_tail_volume = h_tail_cross_section_area * h_tail_span
    
    h_tail_mass = density_foam * h_tail_volume
    # h_tail_mass = ga_aviation_weights.evaluate_horizontal_tail_weight(
    #     S_ref=h_tail.parameters.S_ref,
    # )
    h_tail.quantities.mass_properties.mass = h_tail_mass
    h_tail.quantities.mass_properties.cg_vector = 0.6 * h_tail.LE_center + 0.4 * h_tail.TE_center
    
    v_tail : cd.aircraft.components.Wing = aircraft.comps["v_tail"]

    v_tail_span = csdl.sqrt(v_tail.parameters.AR * v_tail.parameters.S_ref)
    v_tail_chord = csdl.sqrt(v_tail.parameters.S_ref / v_tail.parameters.AR)
    v_tail_thickness_to_chord = 0.12 * v_tail_chord

    v_tail_max_t = v_tail_chord*v_tail_thickness_to_chord
    v_tail_ellipse_a = 0.3*v_tail_chord
    v_tail_ellipse_b = 1/2 * v_tail_max_t
    v_tail_ellipse_area = np.pi * v_tail_ellipse_a * v_tail_ellipse_b

    v_tail_triangle_area = 1/2 * v_tail_max_t * 0.7*v_tail_chord

    v_tail_cross_section_area = v_tail_ellipse_area + v_tail_triangle_area

    v_tail_volume = v_tail_cross_section_area * v_tail_span

    v_tail_mass = v_tail_volume * density_foam
    # v_tail_mass = ga_aviation_weights.evaluate_vertical_tail_weight(
    #     S_ref=v_tail.parameters.S_ref,
    #     AR=v_tail.parameters.AR,
    #     thickness_to_chord=0.1,
    #     sweep_c4=np.deg2rad(20),
    # )
    v_tail.quantities.mass_properties.mass = v_tail_mass
    v_tail.quantities.mass_properties.cg_vector = 0.6 * h_tail.LE_center + 0.4 * h_tail.TE_center
    # trying to get v_tail.LE_center fails because vtail does not have an FFD? Use h-tail as reference instead
    # v_tail.quantities.mass_properties.cg_vector = 0.6 * v_tail.LE_center + 0.4 * v_tail.TE_center
    
    skeleton = aircraft.comps["skeleton"]

    skeleton_mass = 0.7 * cd.Units.mass.pound_to_kg
    skeleton_mass = csdl.Variable(name = "skeleton_mass", value = skeleton_mass)
    skeleton.quantities.mass_properties.mass = skeleton_mass
    # CG pulled from CAD of skeleton assembly (battery not included)
    skeleton.quantities.mass_properties.cg_vector = fuselage.nose_point + np.array([8.65, 0, 2.04])*cd.Units.length.inch_to_m

    cruise_motor = aircraft.comps["cruise_motor"]
    cruise_motor_mass = csdl.Variable(name="cruise_motor_mass", value=72/1000) # from https://www.cobramotorsusa.com/motors-2217-20.html
    cruise_motor.quantities.mass_properties.mass = cruise_motor_mass
    cruise_motor.quantities.mass_properties.cg_vector = fuselage.nose_point + np.array([-1.299*cd.Units.length.inch_to_m, 0.,0.])

    # boom assemblies are centered at CG. Consist of boom, wing mount, motor, and motor mount. Treat as single object on each side
    fl_boom = cd.aircraft.components.Fuselage = aircraft.comps["fl_boom"]
    fr_boom = cd.aircraft.components.Fuselage = aircraft.comps["fr_boom"]
    rl_boom = cd.aircraft.components.Fuselage = aircraft.comps["rl_boom"]
    rr_boom = cd.aircraft.components.Fuselage = aircraft.comps["rr_boom"]

    half_boom_OD = 0.75 * cd.Units.length.inch_to_m
    half_boom_ID = 0.625 * cd.Units.length.inch_to_m # double check this
    half_boom_length = csdl.norm(fl_boom.nose_point - fl_boom.tail_point)
    half_boom_volume = np.pi*((half_boom_OD/2)**2 - (half_boom_ID/2)**2) * half_boom_length.value 
    half_boom_density = 0.054 * ftin3_2_kgm3 # ?? get from AFL
    half_boom_mass = half_boom_volume * half_boom_density

    left_boom_assembly = aircraft.comps["left_boom_assembly"]
    lift_motor_mass = 117/1000 # kg
    lift_motor_mount_mass = 0.06*cd.Units.mass.pound_to_kg
    left_wing_boom_mount_mass = 0.12*cd.Units.mass.pound_to_kg
    left_boom_assembly_mass = csdl.Variable(name="left_boom_assembly_mass", value = left_wing_boom_mount_mass + (half_boom_mass + lift_motor_mass + lift_motor_mount_mass)*2)
    left_boom_assembly.quantities.mass_properties.mass = left_boom_assembly_mass
    left_boom_assembly.quantities.mass_properties.cg_vector = wing_qc + np.array([0, -18, 0])*cd.Units.length.inch_to_m

    right_boom_assembly = aircraft.comps["right_boom_assembly"]
    lift_motor_mass = 117/1000 # kg
    right_wing_boom_mount_mass = 0.12*cd.Units.mass.pound_to_kg
    right_boom_assembly_mass = csdl.Variable(name="right_boom_assembly_mass", value = right_wing_boom_mount_mass + (half_boom_mass + lift_motor_mass + lift_motor_mount_mass)*2)
    right_boom_assembly.quantities.mass_properties.mass = right_boom_assembly_mass
    right_boom_assembly.quantities.mass_properties.cg_vector = wing_qc + np.array([0, 18, 0])*cd.Units.length.inch_to_m

    main_spar_OD = half_boom_OD
    main_spar_ID = half_boom_ID
    main_spar_density = half_boom_density
    main_spar : cd.aircraft.components.Fuselage = aircraft.comps["main_spar"]
    main_spar_length = csdl.norm(main_spar.nose_point - main_spar.tail_point)
    main_spar_volume = np.pi*((main_spar_OD/2)**2 - (main_spar_ID/2)**2) * main_spar_length.value
    main_spar_mass = main_spar_density * main_spar_volume
    main_spar_mass = csdl.Variable(name="main_spar_mass", value = main_spar_mass)
    main_spar.quantities.mass_properties.mass = main_spar_mass
    main_spar.quantities.mass_properties.cg_vector = main_spar.nose_point - main_spar.tail_point

    # fl_rotor: cd.aircraft.components.Rotor = aircraft.comps["fl_rotor"]
    # fl_rotor_mass = csdl.Variable(name="fl_rotor_mass", value = 0.06625*cd.Units.mass.pound_to_kg)
    # fl_rotor.quantities.mass_properties.mass = fl_rotor_mass
    # fl_rotor.quantities.mass_properties.cg_vector = fl_boom.nose_point
    
    # rl_rotor : cd.aircraft.components.Rotor = aircraft.comps["rl_rotor"]
    # rl_rotor = aircraft.comps["rl_rotor"]
    # rl_rotor_mass = fl_rotor_mass
    # rl_rotor.quantities.mass_properties.mass = fl_rotor_mass
    # rl_rotor.quantities.mass_properties.cg_vector = rl_boom.tail_point

    # fr_boom : cd.aircraft.components.Fuselage = aircraft.comps["fr_boom"]
    # fr_rotor = aircraft.comps["fr_rotor"]
    # fr_rotor_mass = fl_rotor_mass
    # fr_rotor.quantities.mass_properties.mass = fr_rotor_mass
    # fr_rotor.quantities.mass_properties.cg_vector = fr_boom.nose_point

    # rr_boom : cd.aircraft.components.Fuselage = aircraft.comps["rr_boom"]
    # rr_rotor = aircraft.comps["rr_rotor"]
    # rr_rotor_mass = fl_rotor_mass
    # rr_rotor.quantities.mass_properties.mass = rr_rotor_mass
    # rr_rotor.quantities.mass_properties.cg_vector = rr_boom.tail_point

    wing_fuse_mount = aircraft.comps["wing_fuse_mount"]
    wing_fuse_mount_mass = csdl.Variable(name="wing_fuse_mount_mass", value = 0.12*cd.Units.mass.pound_to_kg)
    # mass and CG pulled from CAD
    wing_fuse_mount.quantities.mass_properties.mass = wing_fuse_mount_mass
    wing_fuse_mount.quantities.mass_properties.cg_vector = np.array([10.18, 0, -3.16])*cd.Units.length.inch_to_m

    tail_mount = aircraft.comps["tail_mount"]
    tail_mount_mass = csdl.Variable(name = "tail_mount_mass", value = 0.13*cd.Units.mass.pound_to_kg)
    tail_mount.quantities.mass_properties.mass = tail_mount_mass
    tail_mount.quantities.mass_properties.cg_vector = np.array([37.69, 0, -1.34])*cd.Units.length.inch_to_m

    weights_solver = cd.aircraft.models.weights.WeightsSolverModel()
    weights_solver.evaluate(
        design_gross_weight, 
        battery_mass, wing_mass, main_spar_mass, wing_spars_mass, fuselage_mass, 
        h_tail_mass, v_tail_mass, skeleton_mass, left_boom_assembly_mass, right_boom_assembly_mass,
        wing_fuse_mount_mass, tail_mount_mass, cruise_motor_mass)

    base_config.assemble_system_mass_properties(update_copies=True)

    total_aircraft_mass = base_config.system.quantities.mass_properties.mass
    total_aircraft_mass.name = "total_aircraft_mass"
    total_aircraft_mass.set_as_constraint(upper=6*cd.Units.mass.pound_to_kg, scaler=1e-3)
    # total_aircraft_mass.set_as_objective(scaler=1e-3)

    print(base_config.system.quantities.mass_properties.inertia_tensor.value)
    
    #Very old very broken define mass properties function
    # """Define vehicle-level mass properties of the base configuration."""
    # lpc = False #We need to do aframe before the rest like this has it.
    # if lpc:
    #     base_config = caddee.base_configuration
    #     conditions = caddee.conditions

    #     if do_cruise:
    #         cruise = conditions["cruise"]
    #         cruise_speed = cruise.parameters.speed[0]
    #     else:
    #         cruise_speed = csdl.Variable(shape=(1, ), value=20)

    #     # Get system component
    #     aircraft = base_config.system
        
    #     # battery
    #     battery = aircraft.comps["battery"] #Dont have a battery in here atm
    #     battery_cg = csdl.Variable(shape=(3, ), value=np.array([-2.85, 0., -1.]))
    #     battery_mass = 1.2 #double check that
    #     battery.quantities.mass_properties.mass = battery_mass
    #     battery.quantities.mass_properties.cg_vector = battery_cg
        
    #     # Wing
    #     wing = aircraft.comps["wing"]
    #     wing_area = wing.parameters.S_ref
    #     wing_AR = wing.parameters.AR
        
    #     beam_mesh = base_config.mesh_container["beam_mesh"] #didnt mesh this yet
    #     wing_box = beam_mesh.discretizations["wing_box_beam"] #again
    #     carbon_fiber = wing.quantities.material_properties.material

    #     box_cs = af.CSBox(
    #         ttop=wing_box.top_skin_thickness,
    #         tbot=wing_box.bottom_skin_thickness,
    #         tweb=wing_box.shear_web_thickness,
    #         height=wing_box.beam_height,
    #         width=wing_box.beam_width,
    #     )
    #     beam_plus_5g = af.Beam(
    #         name="wing_beam", 
    #         mesh=wing_box.nodal_coordinates, 
    #         cs=box_cs,
    #         material=carbon_fiber,
    #     )

    #     beam_minus_3g = af.Beam(
    #         name="wing_beam", 
    #         mesh=wing_box.nodal_coordinates, 
    #         cs=box_cs,
    #         material=carbon_fiber,
    #     )
    #     wing_mass_model = af.FrameMass()
    #     wing_mass_model.add_beam(beam_plus_5g)
    #     wing_mps = wing_mass_model.evaluate()
    #     wing_cg = wing_mps.cg
    #     wing_cg = wing_cg.set(csdl.slice[1], 0)
    #     wing_mass = wing_mps.mass * 2
    #     wing_mass.name = "wing_mass"
    #     wing.quantities.mass_properties.mass = wing_mass
    #     wing.quantities.mass_properties.cg_vector = wing_cg

    #     if do_structural_sizing:
    #         aircraft_in_3g = conditions["plus_5g"].configuration.system
    #         aircraft_in_m1g = conditions["minus_3g"].configuration.system

    #         wing_in_3g = aircraft_in_3g.comps["aircraft"].comps["wing"]
    #         wing_in_m1g = aircraft_in_m1g.comps["aircraft"].comps["wing"]

    #         wing_in_3g.quantities.beam = beam_plus_5g
    #         wing_in_m1g.quantities.beam = beam_minus_3g


    #     # Fuselage
    #     fuselage = aircraft.comps["fuselage"] #also not a thing rn causes as shape error when I load it in define base config
    #     fuselage_length = fuselage.parameters.length

    #     h_tail = aircraft.comps["h_tail"]
    #     h_tail_area = h_tail.parameters.S_ref
    #     v_tail = aircraft.comps["v_tail"]
    #     v_tail_area =  v_tail.parameters.S_ref
        
    #     # Booms
    #     booms = aircraft.comps["booms"]

    #     fuselage_mps.mass = fuselage_mps.mass * scaler
    #     fuselage.quantities.mass_properties.mass = fuselage_mps.mass
    #     fuselage.quantities.mass_properties.cg_vector = fuselage_mps.cg_vector
    #     fuselage.quantities.mass_properties.inertia_tensor = fuselage_mps.inertia_tensor

    #     boom_mps.mass = boom_mps.mass * scaler
    #     booms.quantities.mass_properties.mass = boom_mps.mass
    #     booms.quantities.mass_properties.cg_vector = boom_mps.cg_vector
    #     booms.quantities.mass_properties.inertia_tensor = boom_mps.inertia_tensor

    #     # payload
    #     payload = aircraft.comps["payload"]
    #     payload_mass = csdl.Variable(shape=(1, ), value=540+800)
    #     payload_cg = csdl.Variable(shape=(3, ), value=np.array([-3., 0., -1.5]))
    #     payload.quantities.mass_properties.mass = payload_mass
    #     payload.quantities.mass_properties.cg_vector = payload_cg

    #     # systems
    #     systems = aircraft.comps["systems"]
    #     systems_mass = csdl.Variable(shape=(1, ), value=244)
    #     systems_cg = csdl.Variable(shape=(3, ), value=np.array([-1., 0., -1.5]))
    #     systems.quantities.mass_properties.mass = systems_mass
    #     systems.quantities.mass_properties.cg_vector = systems_cg

    #     # Assemble system mass properties
    #     base_config.assemble_system_mass_properties(update_copies=True)

    #     aircraft_mass = base_config.system.quantities.mass_properties.mass
    #     aircraft_mass.name = "aircraft_mass"
    # induced_drag_example = True
    # if induced_drag_example:
    #     base_config = caddee.base_configuration
    #     conditions = caddee.conditions

    #     # get some operational variables from the cruise condition
    #     cruise = conditions["cruise"]
    #     rho_imperial = cruise.quantities.atmos_states.density * (1 / units.mass.slug_to_kg) / (1 / units.length.foot_to_m)**3
    #     speed_imperial = cruise.parameters.speed * (1 / units.speed.ftps_to_mps)
    #     q_cruise = 0.5 * rho_imperial * speed_imperial**2
    #     range_imperial = cruise.parameters.range * (1/ units.length.nautical_mile_to_m)

    #     #Access the base config and the its components
    #     aircraft = base_config.system

    #     wing = aircraft.comps["wing"]
    #     fuselage = aircraft.comps["fuselage"]
    #     h_tail = aircraft.comps["h_tail"]
    #     v_tail = aircraft.comps["v_tail"]
    #     payload = fuselage.comps["payload"]

    #     # design gross weight estimate dont need because we dont need it for statistical sizing?
    #     #dg_est = csdl.ImplicitVariable(shape=(1, ), value=6)
    #     dg_est = 6

    #     # wing mass
    #     ## WRITE CUSTOM WING MASS MODEL AS FUNCTION OF CSDL VARIABLES ()
    #     S_ref=wing.parameters.S_ref #Are these being treated correctly as CSDL variables?
    #     AR=wing.parameters.AR

    #     spar_OD = 0.25 #??? I think I need to make a spar componet in CADDEE first?
    #     wing_weight = wing_weight_model(AR,S_ref,csdl.Variable(value=4),csdl.Variable(value=4),csdl.Variable(value=12),spar_outer_diameter=spar_OD)

    #     # fuselage mass CONST
    #     fuselage_weight = 3

    #     # h tail mass
    #     h_tail_weight = wing_weight_model(h_tail.AR,h_tail.S,csdl.Variable(value=0),csdl.Variable(value=0),csdl.Variable(value = 12),csdl.Variable(value=0))

    #     # v tail mass
    #     v_tail_weight = 0.1

    #     # avionics mass CONST
    #     avionics_weight = 0.5

    #     # instruments mass CONST
    #     instruments_weight = 0.5

    #     # Landing gear mass CONST
    #     landing_gear_weight = 0.2

    #     # Battery mass CONST but moveable CG!
    #     battery_weight = 1   

    #     #You should probably find a more accurate CG for these things
    #     wing.quantities.mass_properties.mass = wing_weight * units.mass.pound_to_kg
    #     wing.quantities.mass_properties.cg_vector = np.array([1 * units.length.foot_to_m, 0. , 2. * units.length.foot_to_m])

    #     fuselage.quantities.mass_properties.mass = fuselage_weight * units.mass.pound_to_kg
    #     fuselage.quantities.mass_properties.cg_vector = np.array([1 * units.length.foot_to_m, 0. , 0.])

    #     h_tail.quantities.mass_properties.mass = h_tail_weight * units.mass.pound_to_kg
    #     h_tail.quantities.mass_properties.cg_vector = np.array([1 * units.length.foot_to_m, 0., 0.051 * units.length.foot_to_m])

    #     v_tail.quantities.mass_properties.mass = h_tail_weight * units.mass.pound_to_kg
    #     v_tail.quantities.mass_properties.cg_vector = np.array([1 * units.length.foot_to_m, 0., 1.51 * units.length.foot_to_m])

    #     battery.quantities.mass_properties.mass = battery_weight * units.mass.pound_to_kg
    #     battery.quantities.mass_properties.cg_vector = np.array([1 * units.length.foot_to_m, 0. , 0.]) #Not true, figure this out

    #     weights_solver = cd.aircraft.models.weights.WeightsSolverModel()
    #     weights_solver.evaluate(
    #         dg_est, wing_weight, fuselage_weight, h_tail_weight, v_tail_weight, avionics_weight, instruments_weight, landing_gear_weight
    #     )
    #     base_config.assemble_system_mass_properties(update_copies=True)

    #     total_aircraft_mass = base_config.system.quantities.mass_properties.mass
    #     total_aircraft_mass.name = "total_aircraft_mass"
    #     total_aircraft_mass.set_as_constraint(upper=6, scaler=1e-3)

    #     print(aircraft.quantities.mass_properties.mass)

    #     print(id(payload.quantities.mass_properties))
    #     print(id(aircraft.quantities.mass_properties))

    #     base_config.assemble_system_mass_properties()

    #     print(aircraft.quantities.mass_properties.mass.value)
    #     print(base_config.system.quantities.mass_properties)

    #     ## THE LPC WAY ################################################################
    #         # Get base config and conditions

def wing_weight_model(AR,S,m,p,t,spar_outer_diameter):
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

    n = 15 # number of points along the chord
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
    for i in range(n):
        if  (x[i].value/c.value < p.value):
            yc[i]  = (c*m/p**2)*(2*p*(x[i]/c)-(x[i]/c)**2)
            dyc[i] = ((2*m)/p**2)*(p-(x[i]/c))
        else:
            yc[i]  = (c*m/(1-p)**2)*(1-2*p+2*p*(x[i]/c)-(x[i]/c)**2)
            dyc[i] = ((2*m)/(1-p)**2)*(p-(x[i]/c))
            
    for i in range(n):
        yt[i] = (t*c)*(a0*m.sqrt(x[i]/c)+a1*(x[i]/c)+a2*(x[i]/c)**2+a3*(x[i]/c)**3+a4*(x[i]/c)**4)
        teta  = m.atan(dyc[i])
        xu[i] = x[i]  - yt[i]*m.sin(teta)
        xl[i] = x[i]  + yt[i]*m.sin(teta)
        yu[i] = yc[i] + yt[i]*m.cos(teta)
        yl[i] = yc[i] - yt[i]*m.cos(teta)

    # for i in range(n):
    #     if ()


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
    volume_spars = 2*m.pi*(spar_outer_diameter/2/12)**2*b #Volume of both spars in ft^3, spar outer diamter in in^3
    volume_total = volume_wing-volume_spars
    weight = volume_total*foam_density
    return weight

def csdlTrapIntegrator(x,y): #I have no idea if this works correctly
    print("test")
    if len(x) != len(y):
        raise ValueError("X data and Y data must have same length")
    #integral = csdl.Variable(shape = (1,0) name = 'Area of Airfoil', value = 0)
    integral = 0
    integral = csdl.Variable(shape=(1,),name="Airfoil_Area",value = integral)
    for i in range(len(x) - 1):
        integral = integral + (x[i+1] - x[i]) * (y[i] + y[i+1]) / 2.0
    return integral

def define_analysis(caddee: cd.CADDEE):
    conditions = caddee.conditions
    cruise = conditions["cruise"]
    cruise_config = cruise.configuration
    mesh_container = cruise_config.mesh_container
    base_config = caddee.base_configuration
    aircraft = base_config.system

    # Cruise stuff  
    # Re-evaluate meshes and compute nodal velocities
    cruise.finalize_meshes()

    # Make an instance of an airfoil model
    vlm_mesh_0 = mesh_container["vlm_mesh"]
    wing_chord_surface = vlm_mesh_0.discretizations["wing_chord_surface"]
    h_tail_chord_surface = vlm_mesh_0.discretizations["h_tail_chord_surface"]

    lattice_coordinates = [wing_chord_surface.nodal_coordinates, h_tail_chord_surface.nodal_coordinates]
    lattice_nodal_velocities = [wing_chord_surface.nodal_velocities, h_tail_chord_surface.nodal_velocities]

    vlm_outputs_1 = vlm_solver(
        lattice_coordinates, 
        lattice_nodal_velocities, 
        atmos_states=cruise.quantities.atmos_states,
        airfoil_Cd_models=[None, None],
        airfoil_Cl_models=[None, None],
        airfoil_Cp_models=[None, None],
        airfoil_alpha_stall_models=[None, None],
    )

    vlm_forces = vlm_outputs_1.total_force
    vlm_moments = vlm_outputs_1.total_moment

    # # We multiply by (-1) since the lift and drag are w.r.t. the flight-dynamics reference frame
    # total_induced_drag = vlm_outputs_1.total_drag * -1
    # total_lift = vlm_outputs_1.total_lift * -1

    #Do strucutal sizing and weights model? Update structure and weights?

    #WOULD AFRAME GO HERE???????


    ###################################### BEM STUFF, not using qst (quasi-steady transition)
    # Drag build-up
    drag_build_up_model = cd.aircraft.models.aero.compute_drag_build_up

    wing = aircraft.comps["wing"]
    fuselage = aircraft.comps["fuselage"]
    h_tail = aircraft.comps["h_tail"]
    v_tail = aircraft.comps['v_tail']
    # booms = [aircraft.comps["boom_FR"], aircraft.comps["boom_FL"], aircraft.comps["boom_BR"], aircraft.comps["boom_BL"]]
    #This fails for reasons I couldnt tell you why
    # drag_build_up = drag_build_up_model(cruise.quantities.ac_states, cruise.quantities.atmos_states,
    #                                     wing.parameters.S_ref, [wing, fuselage, h_tail, v_tail] + booms)
    
    cruise_power = {}

    # BEM solver
    rotor_meshes = mesh_container["rotor_meshes"]
    cruise_rotor_mesh = rotor_meshes.discretizations["cruise_prop_mesh"]
    mesh_vel = cruise_rotor_mesh.nodal_velocities
    cruise_rpm = csdl.Variable(name="cruise_pusher_rpm", shape=(1, ), value=1200) #check this
    cruise_rpm.set_as_design_variable(upper=2500, lower=1200, scaler=1e-3) #and this
    bem_inputs = RotorAnalysisInputs(mesh_parameters = cruise_rotor_mesh, mesh_velocity = mesh_vel, rpm = cruise_rpm)
    bem_model = BEMModel(num_nodes=1, airfoil_model=NACA4412MLAirfoilModel())
    bem_outputs = bem_model.evaluate(bem_inputs)
    cruise_power = bem_outputs.total_power
    cruise.quantities.power = cruise_power

    # total forces and moments
    total_forces_cruise, total_moments_cruise = cruise.assemble_forces_and_moments(
        [vlm_forces, bem_outputs.forces], [vlm_moments, bem_outputs.moments] #removed drag_build_up
    )

    youGotMassPropertiesWorking = True #The EOM wont wowrk until mass properties are configured correctly.
    if youGotMassPropertiesWorking:
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
        accel_norm_cruise.set_as_constraint(upper=0, lower=0, scaler=1)

    ########### Mission Power Analysis
    cruise_veloicty = cruise.parameters.speed
    R = csdl.Variable(value=10e3) #m
    cruise_time = R/cruise_veloicty #s
    capacity = cruise.quantities.power * cruise_time #Ws
    capacity.set_as_constraint(lower=0,upper = 131868, scaler = 1e3)
    
    ER = capacity/R
    ER.name = "E/R"

    #Davids wing weight model testing stuff
    # aircraft = caddee.base_configuration.system
    # wing = aircraft.comps["wing"]
    #weight = wing_weight_model(wing.parameters.AR,wing.parameters.S_ref,csdl.Variable(value=4),csdl.Variable(value=4),csdl.Variable(value=12),csdl.Variable(value=0.2))
    #print(f"The calculated wing weight is {weight}")

    #SET AS OBJECTIVE
    ER.set_as_objective()

if __name__ == "__main__": #I like doing this because it makes it clear where the main executiom begins and also I can collapse it : )
    # Run the code (forward evaluation)

    # make instance of CADDEE class
    caddee = cd.CADDEE()

    #Define configuration in CADDEE
    define_base_config(caddee=caddee)

    #Define flight regimes in CADDEE
    define_conditions(caddee=caddee)

    #Define masses and set up mass models (how do masses change with change in parameters)
    define_mass_properties(caddee=caddee)

    #What analysis are we performing, this calls the other define configuration functions
    define_analysis(caddee=caddee)

    # Run optimization (Now actually do the optimization using whats been recorded)
    jax_sim = csdl.experimental.JaxSimulator(
        recorder=recorder, # Turn off gpu if none available
    )

    # Check analytical derivatives against finite difference
    # jax_sim.check_optimization_derivatives()

    # Make CSDLAlphaProblem and initialize optimizer
    problem = CSDLAlphaProblem(problem_name="induced_drag_minimization", simulator=jax_sim)
    optimizer = SLSQP(problem=problem)

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