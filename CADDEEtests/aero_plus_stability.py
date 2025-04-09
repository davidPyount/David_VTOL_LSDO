'''Handling qualities optimization example'''
import CADDEE_alpha as cd
import csdl_alpha as csdl
import numpy as np
from VortexAD.core.vlm.vlm_solver import vlm_solver
from CADDEE_alpha.utils.coordinate_transformations import perform_local_to_body_transformation
from modopt import CSDLAlphaProblem, SLSQP

# Start the CSDL recorder
recorder = csdl.Recorder(inline=True, expand_ops=True)
recorder.start()

ft2m = 0.3048
N2lb = 4.44822
g = 9.81
ftin3_2_kgm3 = 1494.7149

# define initial values for design parameters
weight = 6 * N2lb
# fuselage
fuse_len = 1.25 * ft2m
fuse_perim = 1.5 * ft2m
fuse_t = 3/16/12 * ft2m
# wing
wingspan = 4* ft2m
wingchord = 9/12 * ft2m
wing_S = wingspan * wingchord
wing_AR = wingspan/wingchord
wing_taper = 1
# h-stab
h_stab_span = 1.25 * ft2m
h_stab_chord = 5/12 * ft2m
h_stab_AR = h_stab_span/h_stab_chord
h_stab_S = h_stab_span * h_stab_chord
h_stab_taper = 1
h_stab_tc = 0.12
# v-stab
v_stab_span = 1.08/2 * ft2m
v_stab_chord = 5/12 * ft2m
v_stab_AR = v_stab_span/v_stab_chord
v_stab_S = v_stab_span * v_stab_chord
v_stab_taper = 1
v_stab_tc = 0.12
# lift rotors
lift_rotor_d = 14/12 * ft2m
# cruise propeller
cruise_prop_d = 8/12 * ft2m
# main spar
main_spar_len = 3 * ft2m
# wing booms
wing_boom_length = 30/12 * ft2m
wing_boom_y = 0.457
# nosecone

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

# import geometry
mark2_geom = cd.import_geometry("C:/Users/seth3/David_VTOL_LSDO/CADDEEtests/mark2.stp")
plotting_elements = mark2_geom.plot(show=False, opacity=0.5, color='#FFCD00')

# make instance of CADDEE class
caddee = cd.CADDEE()

def define_base_config(caddee : cd.CADDEE):
    """Build the system configuration and define meshes."""

    # Make aircraft component and pass in the geometry
    aircraft = cd.aircraft.components.Aircraft(geometry=mark2_geom, compute_surface_area=False)

    # instantiation configuration object and pass in system component (aircraft)
    base_config = cd.Configuration(system=aircraft)

    fuselage_geometry = aircraft.create_subgeometry(search_names=["Fuselage","Nosecone"])
    fuselage_length = csdl.Variable(name="fuselage_length", value=fuse_len)
    # fuselage_length.set_as_design_variable(lower=0.8*7.5, upper=1.2*7.5)
    fuselage = cd.aircraft.components.Fuselage(
        length=fuselage_length, 
        max_height= 5/12 * ft2m,
        max_width= 5/12 * ft2m,
        geometry=fuselage_geometry,
        skip_ffd = True)
    
    # Assign fuselage component to aircraft
    aircraft.comps["fuselage"] = fuselage
    
    # Treating the boom like a funky fresh fuselage. Its dimensions will change but those of the main fuselage will not
    main_spar_geometry = aircraft.create_subgeometry(search_names=["MainSpar"])
    main_spar_length = csdl.Variable(name="fuselage_length", value=main_spar_len)
    main_spar_length.set_as_design_variable(lower=0.8*main_spar_len, upper=1.2*main_spar_len)
    main_spar = cd.aircraft.components.Fuselage(
        length=main_spar_length, 
        # max_height= 5/12 * ft2m,
        # max_width= 5/12 * ft2m,
        geometry=main_spar_geometry,
        )

    # assign main spar component to aircraft
    aircraft.comps["main_spar"] = main_spar

    # Make wing geometry from aircraft component and instantiate wing component
    wing_geometry = aircraft.create_subgeometry(search_names=["Wing"])
    aspect_ratio = csdl.Variable(name="wing_aspect_ratio", value = wing_AR)
    wing_span = csdl.Variable(name="wing_span", value = wingspan)
    wing_chord = csdl.Variable(name="wing_chord", value=wingchord)
    wing_area = csdl.Variable(name="wing_area", value = wing_S)
    wing_root_twist = csdl.Variable(name="wing_root_twist", value=0)
    wing_tip_twist = csdl.Variable(name="wing_tip_twist", value=0)
    
    # Set design variables for wing
    # aspect_ratio.set_as_design_variable(upper=1.2 * wing_AR, lower=0.8 * wing_AR, scaler=1/8)
    # wing_area.set_as_design_variable(upper=1.2 * wing_S, lower=0.8 * wing_S, scaler=1/16)
    wing_span.set_as_design_variable(upper=1.5 * wingspan, lower = 0.8 * wingspan, scaler=1/8)
    wing_chord.set_as_design_variable(upper = 1.2 * wingchord, lower = 0.8 * wingchord, scaler = 1/16)
    wing_root_twist.set_as_design_variable(upper=np.deg2rad(5), lower=np.deg2rad(-5), scaler=4)
    wing_tip_twist.set_as_design_variable(upper=np.deg2rad(10), lower=np.deg2rad(-10), scaler=2)
    
    wing = cd.aircraft.components.Wing(
        AR=aspect_ratio, S_ref=wing_area, 
        taper_ratio=wing_taper, root_twist_delta=wing_root_twist,
        tip_twist_delta=wing_tip_twist, geometry=wing_geometry
    )

    # Assign wing component to aircraft
    aircraft.comps["wing"] = wing

    # Make horizontal tail geometry & component
    h_tail_geometry = aircraft.create_subgeometry(search_names=["HStab"])
    h_tail_AR = csdl.Variable(name="h_tail_AR", value=h_stab_AR)
    h_tail_area = csdl.Variable(name="h_tail_area", value=h_stab_S)
    h_tail_span = csdl.Variable(name="wing_span", value = h_stab_span)
    h_tail_chord = csdl.Variable(name="wing_chord", value=h_stab_chord)

    h_tail_span.set_as_design_variable(lower=0.8 * h_stab_span, upper=1.5 * h_stab_span, scaler=1/4)
    h_tail_chord.set_as_design_variable(lower=0.8 * h_stab_chord, upper=1.2 * h_stab_chord, scaler=1/4)
    # h_tail_AR.set_as_design_variable(lower=0.8 * h_stab_AR, upper=1.5 * h_stab_AR, scaler=1/4)
    # h_tail_area.set_as_design_variable(lower=0.8 * h_stab_S, upper=1.2 * h_stab_S, scaler=1/4)
    h_tail = cd.aircraft.components.Wing(AR=h_tail_AR, S_ref=h_tail_area, taper_ratio=h_stab_taper, geometry=h_tail_geometry)

    # Assign tail component to aircraft
    aircraft.comps["h_tail"] = h_tail

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

    # lifting rotors
    # import separately
    fl_prop_geom = aircraft.create_subgeometry(search_names=["FrontLeftLiftRotor"])
    fl_prop = cd.aircraft.components.Rotor(
        radius=lift_rotor_d, geometry= fl_prop_geom, compute_surface_area=False, skip_ffd=True,
    )
    aircraft.comps["fl_rotor"] = fl_prop

    rl_prop_geom = aircraft.create_subgeometry(search_names=["BackLeftLiftRotor"])
    rl_prop = cd.aircraft.components.Rotor(
        radius=lift_rotor_d, geometry= rl_prop_geom, compute_surface_area=False, skip_ffd=True,
    )
    aircraft.comps["rl_rotor"] = rl_prop

    fr_prop_geom = aircraft.create_subgeometry(search_names=["FrontRightLiftRotor"])
    fr_prop = cd.aircraft.components.Rotor(
        radius=lift_rotor_d, geometry= fr_prop_geom, compute_surface_area=False, skip_ffd=True,
    )
    aircraft.comps["fr_rotor"] = fr_prop

    rr_prop_geom = aircraft.create_subgeometry(search_names=["BackRightLiftRotor"])
    rr_prop = cd.aircraft.components.Rotor(
        radius=lift_rotor_d, geometry= rr_prop_geom, compute_surface_area=False, skip_ffd=True,
    )
    aircraft.comps["rr_rotor"] = rr_prop

    lift_rotors = [fl_prop, fr_prop, rl_prop, rr_prop]

    cruise_prop_geom = aircraft.create_subgeometry(search_names=["Propeller"])
    cruise_prop = cd.aircraft.components.Rotor(
        radius=cruise_prop_d, geometry= cruise_prop_geom, compute_surface_area=False, skip_ffd=True,
    )
    aircraft.comps["cruise_propeller"] = cruise_prop

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

    # battery_geom = aircraft.create_subgeometry(search_names='Battery')
    # battery = cd.aircraft.components.Fuselage(length=l_battery, geometry=battery_geom)

    # Connect component geometries 
    # look for examples of component connection in ex_lpc.py 
    # wing to fuselage
    base_config.connect_component_geometries(fuselage, wing, 0.75 * wing.LE_center + 0.25 * wing.TE_center)
    # h-tail to spar
    base_config.connect_component_geometries(main_spar, h_tail, h_tail.TE_center)
    # v-tail to spar
    base_config.connect_component_geometries(main_spar, v_tail, v_tail.TE_root)
    # cruise propeller to fuselage nose
    base_config.connect_component_geometries(fuselage, cruise_prop, connection_point=fuselage.nose_point)
    # lift rotors to lift booms
    base_config.connect_component_geometries(fr_boom, fr_prop, fr_boom.nose_point)
    base_config.connect_component_geometries(fl_boom, fl_prop, fl_boom.nose_point)
    base_config.connect_component_geometries(rr_boom, rr_prop, rr_boom.tail_point)
    base_config.connect_component_geometries(rl_boom, rl_prop, rl_boom.tail_point)
    # main spar to fuselage
    base_config.connect_component_geometries(main_spar, fuselage, main_spar.nose_point)
    # wing booms to wing
    base_config.connect_component_geometries(fr_boom, wing, fr_boom.tail_point)
    base_config.connect_component_geometries(fl_boom, wing, fl_boom.tail_point)
    base_config.connect_component_geometries(rr_boom, wing, rr_boom.nose_point)
    base_config.connect_component_geometries(rl_boom, wing, rl_boom.nose_point)

    # geometry and positioning of the "Skeleton Assembly" (which includes all landing gear, nosecone, and all avionics except battery)
    # will not change between models so we can treat it all as one point mass

    skeleton = cd.Component()
    aircraft.comps["skeleton"] = skeleton

    left_boom_assembly = cd.Component()
    aircraft.comps["left_boom_assembly"] = left_boom_assembly

    right_boom_assembly = cd.Component()
    aircraft.comps["right_boom_assembly"] = right_boom_assembly

    wing_spars = cd.Component()
    aircraft.comps["wing_spars"] = wing_spars

    cruise_motor = cd.Component()
    aircraft.comps["cruise_motor"] = cruise_motor

    wing_fuse_mount = cd.Component()
    aircraft.comps["wing_fuse_mount"] = wing_fuse_mount

    tail_mount = cd.Component()
    aircraft.comps["tail_mount"] = tail_mount

    battery = cd.Component()
    aircraft.comps["battery"] = battery

    # Meshing
    mesh_container = base_config.mesh_container

    # H-Tail 
    tail_chord_surface = cd.mesh.make_vlm_surface(
        wing_comp=h_tail,
        num_chordwise=1, 
        num_spanwise=4, # ? decreased for speed, bump this up later
    )

    # V-Tail
    # ? This code does NOT work for vertical tails
    # tail_chord_surface = cd.mesh.make_vlm_surface(
    #     wing_comp=v_tail,
    #     num_chordwise=1, 
    #     num_spanwise=10,
    # )

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

    num_radial = 5 # ? do we even need a propeller discretization
    cruise_prop_mesh = cd.mesh.RotorMeshes()
    cruise_prop_discretization = cd.mesh.make_rotor_mesh(
        cruise_prop, num_radial=num_radial, num_azimuthal=1, num_blades=2
    )
    cruise_prop_mesh.discretizations["propeller_discretization"] = cruise_prop_discretization 

    # lift rotors
    lift_rotor_meshes = cd.mesh.RotorMeshes()
    for i in range(len(lift_rotors)):
        rotor_discretization = cd.mesh.make_rotor_mesh(
            lift_rotors[i], num_radial=num_radial, num_azimuthal=1, num_blades=2)
        lift_rotor_meshes.discretizations[f"rotor_{i}_mesh"] = rotor_discretization

    # plot meshes
    # mark2_geom.plot_meshes(meshes=[wing_chord_surface.nodal_coordinates.value, tail_chord_surface.nodal_coordinates.value])
    
    # Assign mesh to mesh container
    mesh_container["vlm_mesh"] = vlm_mesh
    mesh_container["cruise_prop_mesh"] = cruise_prop_mesh
    mesh_container["lift_rotor_meshes"] = lift_rotor_meshes

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
    """Define the operating conditions of the aircraft."""
    conditions = caddee.conditions
    base_config = caddee.base_configuration

    # pitch_angle = csdl.Variable(name="pitch_angle", value=0.)
    # pitch_angle.set_as_design_variable(upper=np.deg2rad(2.5), lower=np.deg2rad(-2), scaler=4)
    cruise = cd.aircraft.conditions.CruiseCondition(
        altitude= alt,
        range= dist,
        pitch_angle= pitch,
        mach_number= mach
    )
    cruise.configuration = base_config.copy()
    conditions["cruise"] = cruise

def define_mass_properties(caddee: cd.CADDEE):
    """Define the mass properties of the aircraft."""

    base_config = caddee.base_configuration
    aircraft = base_config.system

    conditions = caddee.conditions
    cruise : cd.aircraft.conditions.CruiseCondition = conditions["cruise"]
    dynamic_pressure = 0.5 * cruise.quantities.atmos_states.density * cruise.parameters.speed**2

    design_gross_weight = csdl.Variable(name="design_gross_weight", value=w_total)
    # fuel_weight = csdl.Variable(name="fuel_weight", value=250*cd.Units.mass.pound_to_kg)
    fuel_weight = 0
    
    battery = aircraft.comps["battery"]
    battery_mass = csdl.Variable(name="battery_mass", value=m_battery)
    # position pulled from CAD
    battery_x = 7.32*cd.Units.length.inch_to_m
    battery.quantities.mass_properties.mass = battery_mass
    battery.quantities.mass_properties.cg_vector = csdl.Variable(name="battery_cg", shape=(3,), value = np.array([battery_x, 0, 0]))
    # ? this is breaking the jax section because it is not an independent variable
    battery.quantities.mass_properties.cg_vector[0].set_as_design_variable(lower=5*cd.Units.length.inch_to_m, upper=10*cd.Units.length.inch_to_m, scaler=1)
    
    # battery_x.set_as_design_variable(lower=5*cd.Units.length.inch_to_m, upper=10*cd.Units.length.inch_to_m, scaler=1)

    # ga_aviation_weights = cd.aircraft.models.weights.general_aviation_weights.GeneralAviationWeights(
    #     design_gross_weight=design_gross_weight,
    #     dynamic_pressure=dynamic_pressure,
    # )

    wing : cd.aircraft.components.Wing = aircraft.comps["wing"]
    wing_center = (wing.LE_center + wing.TE_center) / 2
    wing_qc = 0.75 * wing.LE_center + 0.25 * wing.TE_center

 # approximate the wing cross-section area as an ellipse, rectangle, and triangle

    wing_max_t = wing.parameters.MAC*wing.parameters.thickness_to_chord
    wing_ellipse_a = 0.3*wing.parameters.MAC
    wing_ellipse_b = 1/2 * wing_max_t
    wing_ellipse_area = np.pi * wing_ellipse_a * wing_ellipse_b

    wing_rectangle_area = 0.2*wing.parameters.MAC * wing_max_t

    wing_triangle_area = 1/2 * wing_max_t * 0.5*wing.parameters.MAC

    wing_cross_section_area = wing_ellipse_area + wing_triangle_area

    wing_volume = wing_cross_section_area * wing.parameters.span

    wing_mass = wing_volume * density_foam
    wing.quantities.mass_properties.mass = wing_mass + fuel_weight
    wing.quantities.mass_properties.cg_vector = 0.56 * wing.LE_center + 0.44 * wing.TE_center # CG is around 44.4% of chord for 4412

# need to get expression for wing weight as function of cross-sectional airfoil area, foam density, and spar radius
# so this step is probably after wing spar sizing in Aframe

# A-FRAME GOES HERE or somewhere even earlier to replace what is below here
    
    wing_spar_OD = 0.375 * cd.Units.length.inch_to_m
    wing_spar_ID = 0.25 * cd.Units.length.inch_to_m
    wing_spar_volume = np.pi * ( (wing_spar_OD/2)**2 - (wing_spar_ID/2)**2 ) * wing.parameters.span

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

    h_tail_max_t = h_tail.parameters.MAC*h_tail.parameters.thickness_to_chord
    h_tail_ellipse_a = 0.3*h_tail.parameters.MAC
    h_tail_ellipse_b = 1/2 * h_tail_max_t
    h_tail_ellipse_area = np.pi * h_tail_ellipse_a * h_tail_ellipse_b

    h_tail_triangle_area = 1/2 * h_tail_max_t * 0.7*h_tail.parameters.MAC

    h_tail_cross_section_area = h_tail_ellipse_area + h_tail_triangle_area

    h_tail_volume = h_tail_cross_section_area * h_tail.parameters.span
    
    h_tail_mass = density_foam * h_tail_volume
    # h_tail_mass = ga_aviation_weights.evaluate_horizontal_tail_weight(
    #     S_ref=h_tail.parameters.S_ref,
    # )
    h_tail.quantities.mass_properties.mass = h_tail_mass
    h_tail.quantities.mass_properties.cg_vector = 0.6 * h_tail.LE_center + 0.4 * h_tail.TE_center
    
    v_tail : cd.aircraft.components.Wing = aircraft.comps["v_tail"]

    v_tail_max_t = v_tail.parameters.MAC*v_tail.parameters.thickness_to_chord
    v_tail_ellipse_a = 0.3*v_tail.parameters.MAC
    v_tail_ellipse_b = 1/2 * v_tail_max_t
    v_tail_ellipse_area = np.pi * v_tail_ellipse_a * v_tail_ellipse_b

    v_tail_triangle_area = 1/2 * v_tail_max_t * 0.7*v_tail.parameters.MAC

    v_tail_cross_section_area = v_tail_ellipse_area + v_tail_triangle_area

    v_tail_volume = v_tail_cross_section_area * v_tail.parameters.span

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
    fl_boom : cd.aircraft.components.Fuselage = aircraft.comps["fl_boom"]
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

    fl_rotor: cd.aircraft.components.Rotor = aircraft.comps["fl_rotor"]
    fl_rotor_mass = csdl.Variable(name="fl_rotor_mass", value = 0.06625*cd.Units.mass.pound_to_kg)
    fl_rotor.quantities.mass_properties.mass = fl_rotor_mass
    fl_rotor.quantities.mass_properties.cg_vector = fl_boom.nose_point
    
    rl_rotor : cd.aircraft.components.Rotor = aircraft.comps["rl_rotor"]
    rl_rotor = aircraft.comps["rl_rotor"]
    rl_rotor_mass = fl_rotor_mass
    rl_rotor.quantities.mass_properties.mass = fl_rotor_mass
    rl_rotor.quantities.mass_properties.cg_vector = rl_boom.tail_point

    fr_boom : cd.aircraft.components.Fuselage = aircraft.comps["fr_boom"]
    fr_rotor = aircraft.comps["fr_rotor"]
    fr_rotor_mass = fl_rotor_mass
    fr_rotor.quantities.mass_properties.mass = fr_rotor_mass
    fr_rotor.quantities.mass_properties.cg_vector = fr_boom.nose_point

    rr_boom : cd.aircraft.components.Fuselage = aircraft.comps["rr_boom"]
    rr_rotor = aircraft.comps["rr_rotor"]
    rr_rotor_mass = fl_rotor_mass
    rr_rotor.quantities.mass_properties.mass = rr_rotor_mass
    rr_rotor.quantities.mass_properties.cg_vector = rr_boom.tail_point

    wing_fuse_mount = aircraft.comps["wing_fuse_mount"]
    wing_fuse_mount_mass = csdl.Variable(name="wing_fuse_mount_mass", value = 0.12*cd.Units.mass.pound_to_kg)
    # mass and CG pulled from CAD
    wing_fuse_mount.quantities.mass_properties.mass = wing_fuse_mount_mass
    wing_fuse_mount.quantities.mass_properties.cg_vector = np.array([10.18, 0, -3.16])*cd.Units.length.inch_to_m

# rolled these into boom assembly

    # left_wing_boom_mount = aircraft.comps(["left_wing_boom_mount"])
    # left_wing_boom_mount_mass = csdl.Variable(name = "left_wing_boom_mount_mass", value = 0.12/32.174*cd.Units.mass.pound_to_kg)
    #   # mass and CG pulled from CAD
    # left_wing_boom_mount.quantities.mass_properties.mass = left_wing_boom_mount_mass
    # left_wing_boom_mount.quantities.mass_properties.cg_vector = np.array([10.37 -18 3])*cd.Units.length.inch_to_m

    # right_wing_boom_mount = aircraft.comps(["right_wing_boom_mount"])
    # right_wing_boom_mount_mass = left_wing_boom_mount_mass
    #   # mass and CG pulled from CAD
    # right_wing_boom_mount.quantities.mass_properties.mass = right_wing_boom_mount_mass
    # right_wing_boom_mount.quantities.mass_properties.cg_vector = np.array([10.37 18 3])*cd.Units.length.inch_to_m


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

def define_analysis(caddee: cd.CADDEE):
    """Define the analysis of performed on the aircraft."""
    cruise : cd.aircraft.conditions.CruiseCondition = caddee.conditions["cruise"]
    cruise_config = cruise.configuration
    mesh_container = cruise_config.mesh_container

    tail = cruise_config.system.comps["h_tail"]
    v_tail = cruise_config.system.comps["v_tail"]
    wing = cruise_config.system.comps["wing"]
    fuselage = cruise_config.system.comps["fuselage"]
    # elevator_deflection = csdl.Variable(name="elevator", value=0)
    # elevator_deflection.set_as_design_variable(lower=np.deg2rad(-10), upper=np.deg2rad(10), scaler=2)
    # tail.actuate(elevator_deflection)

    # Re-evaluate meshes and compute nodal velocities
    cruise.finalize_meshes()

    vlm_mesh = mesh_container["vlm_mesh"]
    wing_chord_surface = vlm_mesh.discretizations["wing_chord_surface"]
    h_tail_chord_surface = vlm_mesh.discretizations["h_tail_chord_surface"]

    lattice_coordinates = [wing_chord_surface.nodal_coordinates, h_tail_chord_surface.nodal_coordinates]
    lattice_nodal_velocities = [wing_chord_surface.nodal_velocities, h_tail_chord_surface.nodal_velocities]

    vlm_outputs = vlm_solver(
        lattice_coordinates, 
        lattice_nodal_velocities, 
        atmos_states=cruise.quantities.atmos_states,
        airfoil_Cd_models=[None, None],
        airfoil_Cl_models=[None, None],
        airfoil_Cp_models=[None, None],
        airfoil_alpha_stall_models=[None, None],
    )

    vlm_forces = vlm_outputs.total_force
    vlm_moments = vlm_outputs.total_moment

    # rotor analysis
    thrust_coefficient =  0.0204
    # rotor_meshes = mesh_container["rotor_meshes"]
    rotor_meshes = mesh_container["lift_rotor_meshes"]
    propeller_discretization = rotor_meshes.discretizations["propeller_discretization"]
    # ? need to return to this
    cruise_rpm = csdl.Variable(name="cruise_pusher_rpm", shape=(1, ), value=1500)
    # cruise_rpm.set_as_design_variable(scaler=1e-3, lower=500, upper=2000)
    cruise_omega = cruise_rpm / 60 * 2 * np.pi
    radius = propeller_discretization.radius
    thrust_vector = propeller_discretization.thrust_vector
    thrust_origin = propeller_discretization.thrust_origin.reshape((-1, 3))
    rho = cruise.quantities.atmos_states.density
    thrust = thrust_coefficient * rho * np.pi * radius**2 * (cruise_omega * radius)**2
    thrust_forces = thrust * thrust_vector

    # To capture the effect of perturbations in the aircraft states, we need to rotate thrust vector into body-fixed frame
    # NOTE: We only do this for thrust in this example because other solvers automatically do this.
    thrust_forces_body = perform_local_to_body_transformation(
        cruise.quantities.ac_states.phi,
        cruise.quantities.ac_states.theta,
        cruise.quantities.ac_states.psi,
        thrust_forces
    )
    thrust_moments_body = csdl.cross(thrust_origin, thrust_forces_body, axis=1)

    # Parasite drag build up
    drag_build_up_model = cd.aircraft.models.aero.compute_drag_build_up
    parasite_drag = drag_build_up_model(cruise.quantities.ac_states, cruise.quantities.atmos_states,
                                        wing.parameters.S_ref, [wing, fuselage, tail, v_tail])
    
    # Summing up the total forces and moments
    total_forces, total_moments = cruise.assemble_forces_and_moments(
        [vlm_forces, thrust_forces_body, parasite_drag], [vlm_moments, thrust_moments_body]
    )

    # Setting force equilibrium constraints
    force_norm = csdl.norm(total_forces)
    moment_norm = csdl.norm(total_moments)

    force_norm.name = "total_forces_norm"
    moment_norm.name = "total_moments_norm"

    force_norm.set_as_constraint(equals=0, scaler=1e-4)
    moment_norm.set_as_constraint(equals=0., scaler=1e-4)

    # Performing linearized stability analysis
    long_stability_results = cruise.perform_linear_stability_analysis(
        total_forces=total_forces,
        total_moments=total_moments,
        ac_states=cruise.quantities.ac_states,
        mass_properties=cruise_config.system.quantities.mass_properties,
    )

    t2d = long_stability_results.time_2_double_phugoid
    t2d.name = "time2double"
    t2d.set_as_objective(scaler=-4e-2)
    print("time to double", t2d.value)

# Run the code (forward evaluation)
define_base_config(caddee=caddee)

define_conditions(caddee=caddee)

define_mass_properties(caddee=caddee)

define_analysis(caddee=caddee)

# Run optimization
jax_sim = csdl.experimental.JaxSimulator(
    recorder=recorder, # Turn off gpu if none available
    gpu=False,
    derivatives_kwargs= {
        "concatenate_ofs" : True # Turn off
    }
)

# Check analytical derivatives against finite difference
jax_sim.check_optimization_derivatives()

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