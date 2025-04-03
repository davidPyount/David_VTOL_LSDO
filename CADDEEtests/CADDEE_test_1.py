'''Induced drag minimization example'''
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
units = Units()

# Start the CSDL recorder
recorder = csdl.Recorder(inline=True, expand_ops=True)
recorder.start()

# import geometry
mark2_geom = cd.import_geometry("C:/Users/david/OneDrive - UC San Diego/GitHub/LSDO/David_VTOL_LSDO/CADDEEtests/mark2.stp")
plotting_elements = mark2_geom.plot(show=False, opacity=0.5, color='#FFCD00')

ft2m = 0.3048
N2lb = 4.44822
# define INITIAL values for design parameters
weight = 6 * 4.44822
# fuselage
fuselage_length = 1.25 * ft2m
# wing
wing_span = 4* ft2m
wing_chord = 9/12 * ft2m
wing_area = wing_span * wing_chord
wing_AR = wing_span/wing_chord
wing_taper = 1
# h-stab
h_stab_span = 1.25 * ft2m
h_stab_chord = 5/12 * ft2m
h_stab_AR = h_stab_span/h_stab_chord
h_stab_area = h_stab_span * h_stab_chord
h_stab_taper = 1
# v-stab
v_stab_span = 1.08/2 * ft2m
v_stab_chord = 5/12 * ft2m
v_stab_AR = v_stab_span/v_stab_chord
v_stab_area = v_stab_span * v_stab_chord
v_stab_taper = 1

# main spar
main_spar_length = 36 * ft2m

# wing booms
wing_boom_length = 36.2 * ft2m

do_cruise = True
do_trim_optimization = True
do_structural_sizing = True
run_optimization = True
run_ffd = True

def define_base_config(caddee : cd.CADDEE):
    """Build the system configuration and define meshes."""

    # Make aircraft component and pass in the geometry
    aircraft = cd.aircraft.components.Aircraft(geometry=mark2_geom, compute_surface_area=False)

    # instantiation configuration object and pass in system component (aircraft)
    base_config = cd.Configuration(system=aircraft)

    # why isn't the fuselage component assigned to the aircraft?
    fuselage_geometry = aircraft.create_subgeometry(search_names=["Fuselage"])
    fuselage = cd.aircraft.components.Fuselage(length=fuselage_length, geometry=fuselage_geometry)

    # Make wing geometry from aircraft component and instantiate wing component
    wing_geometry = aircraft.create_subgeometry(
        search_names=["Wing"],
    )
    aspect_ratio = csdl.Variable(name="wing_aspect_ratio", value=wing_AR)
    wing_root_twist = csdl.Variable(name="wing_root_twist", value=np.deg2rad(0))
    wing_tip_twist = csdl.Variable(name="wing_tip_twist", value=np.deg2rad(0))

    # Set design variables for wing
    aspect_ratio.set_as_design_variable(upper=1.5 * wing_AR, lower=0.5 * wing_AR, scaler=1/8)
    wing_root_twist.set_as_design_variable(upper=np.deg2rad(5), lower=np.deg2rad(-5), scaler=4)
    wing_tip_twist.set_as_design_variable(upper=np.deg2rad(10), lower=np.deg2rad(-10), scaler=2)
    
    # Are taper ratio and area held constant here? I think so they're not CSDL variables -David
    wing = cd.aircraft.components.Wing(
        AR=aspect_ratio, S_ref=wing_area, 
        taper_ratio=wing_taper, root_twist_delta=wing_root_twist,
        tip_twist_delta=wing_tip_twist, 
        geometry=wing_geometry
    )

    # Wing spar material
    carbon_fiber = af.Material(name='carbon_fiber', E=96.2E9, G = 3.16E9, density = 1420)

    # Function spaces DO WE NEED THESE?
    # Thickness
    thickness_space = wing_geometry.create_parallel_space(lfs.ConstantSpace(2))
    thickness_var, thickness_function = thickness_space.initialize_function(1, value=0.005)
    wing.quantities.material_properties.set_material(carbon_fiber, thickness=None)

    # Pressure space
    pressure_function_space = lfs.IDWFunctionSpace(num_parametric_dimensions=2, order=6, grid_size=(120, 20), conserve=False, n_neighbors=10)
    indexed_pressue_function_space = wing.geometry.create_parallel_space(pressure_function_space)
    wing.quantities.pressure_space = indexed_pressue_function_space

    # Aerodynamic parameters for drag build up
    wing.quantities.drag_parameters.percent_laminar = 70
    wing.quantities.drag_parameters.percent_turbulent = 30

    # Assign wing component to aircraft
    aircraft.comps["wing"] = wing

    # Connect wing to fuselage at the quarter chord
    base_config.connect_component_geometries(fuselage, wing, 0.75 * wing.LE_center + 0.25 * wing.TE_center)
    # base_config.connect_component_geometries(main_spar, h_tail, h_tail.TE_center)

    # Make horizontal tail geometry & component
    h_tail_geometry = aircraft.create_subgeometry(search_names=["HStab"])
    h_tail = cd.aircraft.components.Wing(
        AR=h_stab_AR, S_ref=h_stab_area, taper_ratio=h_stab_taper, 
        geometry=h_tail_geometry
    )

    # Assign tail component to aircraft
    aircraft.comps["h_tail"] = h_tail
    #base_config.connect_component_geometries(fuselage, h_tail, h_tail.TE_center) #.TE_Center doesnt work for some reason

    # Make vertical tail geometry & component
    v_tail_geometry = aircraft.create_subgeometry(search_names=["VStab"])
    v_tail = cd.aircraft.components.Wing(
        AR=v_stab_AR, S_ref=v_stab_area, geometry=v_tail_geometry, 
        skip_ffd=True, orientation="vertical"
    )

    # Assign v-tail component to aircraft
    aircraft.comps["v_tail"] = v_tail

    m_spar_length = csdl.Variable(name="Main Spar Length", value = main_spar_length)
    m_spar_length.set_as_design_variable(upper= 13.716, lower=9,scaler=1e-1)
    #Main spar
    main_spar_geometry = aircraft.create_subgeometry(
        search_names=["MainSpar"],
    )
    main_spar = cd.Component(main_spar_geometry,lenght=main_spar_length)
    aircraft.comps["main_spar"] = main_spar


    # # Connect h-tail to spar?
    # base_config.connect_component_geometries(main_spar, h_tail, h_tail.TE_center) TE_center doesnt work for some reason
    # # Connect v-tail to spar?
    # base_config.connect_component_geometries(main_spar, v_tail, v_tail.TE_center)

    #Booms
    #These dont have rotors at the moment. Not sure if we need, not using rotorAD
    #Front Right
    boomFR_geometry = aircraft.create_subgeometry(
        search_names=["FrontRightBoom"]
    )
    boomFR = cd.Componet(boomFR_geometry,length=wing_boom_length/2)
    aircraft.comps["boom_FR"] = boomFR
    base_config.connect_component_geometries(boomFR,wing,connection_point=boomFR.ffd_block_face1)

    #Back Right
    boomBR_geometry = aircraft.create_subgeometry(
        search_names=["BackRightBoom"]
    )
    boomBR = cd.Componet(boomBR_geometry,length=wing_boom_length/2)
    aircraft.comps["boom_BR"] = boomBR
    base_config.connect_component_geometries(boomBR,wing,connection_point=boomFR.ffd_block_face1)

    #Front Left
    boomFL_geometry = aircraft.create_subgeometry(
        search_names=["FrontLeftBoom"]
    )
    boomFL = cd.Componet(boomFL_geometry,length=wing_boom_length/2)
    aircraft.comps["boom_FL"] = boomFL
    base_config.connect_component_geometries(boomFL,wing,connection_point=boomFR.ffd_block_face1)

    #Back Left
    boomBL_geometry = aircraft.create_subgeometry(
        search_names=["BackLeftBoom"]
    )
    boomBL = cd.Componet(boomBL_geometry,length=wing_boom_length/2)
    aircraft.comps["boom_BL"] = boomBL
    base_config.connect_component_geometries(boomBL,wing,connection_point=boomFR.ffd_block_face1)


    ## MAKE MESHES
    # Meshing
    mesh_container = base_config.mesh_container

    # Tail 
    tail_chord_surface = cd.mesh.make_vlm_surface(
        wing_comp=h_tail,
        num_chordwise=1, 
        num_spanwise=10,
    )

    # Wing chord surface (lifting line)
    wing_chord_surface = cd.mesh.make_vlm_surface(
        wing_comp=wing,
        num_chordwise=16,
        num_spanwise=30,
    )
    vlm_mesh_0 = cd.mesh.VLMMesh()
    vlm_mesh_0.discretizations["wing_chord_surface"] = wing_chord_surface
    vlm_mesh_0.discretizations["h_tail_chord_surface"] = tail_chord_surface

    #Do we need to do any VLM stuff for drag buildup on the fuselage?

    #I think we need to make a mesh for A-Frame? Do that here?

    # plot meshes
    mark2_geom.plot_meshes(meshes=[wing_chord_surface.nodal_coordinates.value, tail_chord_surface.nodal_coordinates.value])
    # Assign mesh to mesh container
    mesh_container["vlm_mesh_0"] = vlm_mesh_0

    # Set up the geometry: this will run the inner optimization
    base_config.setup_geometry()

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
        mach_number=0.06,
    )
    cruise.configuration = base_config.copy()
    conditions["cruise"] = cruise

    do_struct_sizing = True
    if do_struct_sizing:
        #+5g
        pitch_angle5g = csdl.Variable(name="5g_pitch",shape=(1,),value=np.deg2rad(10))
        pitch_angle.set_as_design_variable(upper=np.deg2rad(15),lower=0,scaler=10)
        flight_path_angle5g = csdl.Variable(shape=(1,0),value=np.deg2rad(5))
        plus_5g = cd.aircraft.conditions.ClimbCondition(
            initial_altitiude = 50,
            final_altitude=150,
            pitch_angle=pitch_angle5g,
            fligth_path_angle=flight_path_angle5g,
            mach_number=0.04,
        )
        plus_5g.configuration = base_config.copy()
        conditions["plus_5g"] = plus_5g

        #-3
        pitch_angle3g = csdl.Variable(name="3g_pitch",shape=(1,),value=np.deg2rad(-8))
        pitch_angle3g.set_as_design_variable(upper=0,lower=np.deg2rad(-15),scaler=10)
        flight_path_angle3g = csdl.Variable(shape=(1,0),value=np.deg2rad(-1))
        plus_5g = cd.aircraft.conditions.ClimbCondition(
            initial_altitiude = 150,
            final_altitude=50,
            pitch_angle=pitch_angle3g,
            fligth_path_angle=flight_path_angle3g,
            mach_number=0.04,
        )
        plus_5g.configuration = base_config.copy()
        conditions["minus_3g"] = plus_5g

def define_mass_properties(caddee : cd.CADDEE):
    """Define vehicle-level mass properties of the base configuration."""
    induced_drag_example = True
    if induced_drag_example:
        base_config = caddee.base_configuration
        conditions = caddee.conditions

        # get some operational variables from the cruise condition
        cruise = conditions["cruise"]
        rho_imperial = cruise.quantities.atmos_states.density * (1 / units.mass.slug_to_kg) / (1 / units.length.foot_to_m)**3
        speed_imperial = cruise.parameters.speed * (1 / units.speed.ftps_to_mps)
        q_cruise = 0.5 * rho_imperial * speed_imperial**2
        range_imperial = cruise.parameters.range * (1/ units.length.nautical_mile_to_m)

        #Access the base config and the its components
        aircraft = base_config.system

        wing = aircraft.comps["wing"]
        fuselage = aircraft.comps["fuselage"]
        h_tail = aircraft.comps["h_tail"]
        v_tail = aircraft.comps["v_tail"]
        #main_landing_gear = aircraft.comps["main_landing_gear"]
        #avionics = fuselage.comps["avionics"]
        #instruments = fuselage.comps["instruments"]
        #engine = aircraft.comps["engine"]
        #fuel = wing.comps["fuel"]
        payload = fuselage.comps["payload"]

        # design gross weight estimate dont need because we dont need it for statistical sizing?
        #dg_est = csdl.ImplicitVariable(shape=(1, ), value=6)
        dg_est = 6

        #fuel_weight = csdl.Variable(shape=(1,), value=500)
        #engine_weight = csdl.Variable(shape=(1,), value=250)
        #payload_weight =  csdl.Variable(shape=(1,), value=0.3)

        #I think add battery weight and make battery CG a design variable to help meet static marge
        # systems
        # systems = aircraft.comps["systems"]
        # systems_mass = csdl.Variable(shape=(1, ), value=244)
        # systems_cg = csdl.Variable(shape=(3, ), value=np.array([-1., 0., -1.5]))
        # systems.quantities.mass_properties.mass = systems_mass
        # systems.quantities.mass_properties.cg_vector = systems_cg

        # wing mass
        ## WRITE CUSTOM WING MASS MODEL AS FUNCTION OF CSDL VARIABLES ()
        S_ref=wing.parameters.S_ref #Are these being treated correctly as CSDL variables?
        AR=wing.parameters.AR

        spar_OD = 0.25 #??? I think I need to make a spar componet in CADDEE first?
        wing_weight = wing_weight_model(AR,S_ref,4,4,12,spar_OD)

        # fuselage mass CONST
        fuselage_weight = 3

        # h tail mass
        h_tail_weight = 0.2

        # v tail mass
        v_tail_weight = 0.1

        # avionics mass CONST
        avionics_weight = 1

        # instruments mass CONST
        instruments_weight = 0.5

        # Landing gear mass CONST
        landing_gear_weight = 0.2    

        #You should probably find a more accurate CG for these things
        wing.quantities.mass_properties.mass = wing_weight * units.mass.pound_to_kg
        wing.quantities.mass_properties.cg_vector = np.array([1 * units.length.foot_to_m, 0. , 2. * units.length.foot_to_m])

        fuselage.quantities.mass_properties.mass = fuselage_weight * units.mass.pound_to_kg
        fuselage.quantities.mass_properties.cg_vector = np.array([1 * units.length.foot_to_m, 0. , 0.])

        h_tail.quantities.mass_properties.mass = h_tail_weight * units.mass.pound_to_kg
        h_tail.quantities.mass_properties.cg_vector = np.array([1 * units.length.foot_to_m, 0., 0.051 * units.length.foot_to_m])

        v_tail.quantities.mass_properties.mass = h_tail_weight * units.mass.pound_to_kg
        v_tail.quantities.mass_properties.cg_vector = np.array([1 * units.length.foot_to_m, 0., 1.51 * units.length.foot_to_m])

        weights_solver = cd.aircraft.models.weights.WeightsSolverModel()
        weights_solver.evaluate(
            dg_est, wing_weight, fuselage_weight, h_tail_weight, v_tail_weight, avionics_weight, instruments_weight, landing_gear_weight
        )
        base_config.assemble_system_mass_properties(update_copies=True)

        total_aircraft_mass = base_config.system.quantities.mass_properties.mass
        total_aircraft_mass.name = "total_aircraft_mass"
        total_aircraft_mass.set_as_constraint(upper=6, scaler=1e-3)

        print(aircraft.quantities.mass_properties.mass)

        print(id(payload.quantities.mass_properties))
        print(id(aircraft.quantities.mass_properties))

        # aircraft.quantities.mass_properties.mass = 1100
        # aircraft.quantities.mass_properties.cg_vector = np.array([2.2513916, 0., 0.216399])
        # aircraft.quantities.mass_properties.inertia_tensor = np.zeros((3, 3))

        base_config.assemble_system_mass_properties()

        print(aircraft.quantities.mass_properties.mass.value)
        print(base_config.system.quantities.mass_properties)

        ## THE LPC WAY ################################################################
            # Get base config and conditions
    lpc = False #This way includes A-frame but may be way more complicated than we need.
    if lpc:
        base_config = caddee.base_configuration
        conditions = caddee.conditions

        if do_cruise:
            cruise = conditions["cruise"]
            cruise_speed = cruise.parameters.speed[0]
        else:
            cruise_speed = csdl.Variable(shape=(1, ), value=20)

        # Get system component
        aircraft = base_config.system
        
        # battery
        battery = aircraft.comps["battery"] #Dont have a battery in here atm
        battery_cg = csdl.Variable(shape=(3, ), value=np.array([-2.85, 0., -1.]))
        battery_mass = 1.2 #double check that
        battery.quantities.mass_properties.mass = battery_mass
        battery.quantities.mass_properties.cg_vector = battery_cg
        
        # Wing
        wing = aircraft.comps["wing"]
        wing_area = wing.parameters.S_ref
        wing_AR = wing.parameters.AR
        
        beam_mesh = base_config.mesh_container["beam_mesh"] #didnt mesh this yet
        wing_box = beam_mesh.discretizations["wing_box_beam"] #again
        carbon_fiber = wing.quantities.material_properties.material

        box_cs = af.CSBox(
            ttop=wing_box.top_skin_thickness,
            tbot=wing_box.bottom_skin_thickness,
            tweb=wing_box.shear_web_thickness,
            height=wing_box.beam_height,
            width=wing_box.beam_width,
        )
        beam_plus_5g = af.Beam(
            name="wing_beam", 
            mesh=wing_box.nodal_coordinates, 
            cs=box_cs,
            material=carbon_fiber,
        )

        beam_minus_3g = af.Beam(
            name="wing_beam", 
            mesh=wing_box.nodal_coordinates, 
            cs=box_cs,
            material=carbon_fiber,
        )
        wing_mass_model = af.FrameMass()
        wing_mass_model.add_beam(beam_plus_5g)
        wing_mps = wing_mass_model.evaluate()
        wing_cg = wing_mps.cg
        wing_cg = wing_cg.set(csdl.slice[1], 0)
        wing_mass = wing_mps.mass * 2
        wing_mass.name = "wing_mass"
        wing.quantities.mass_properties.mass = wing_mass
        wing.quantities.mass_properties.cg_vector = wing_cg

        if do_structural_sizing:
            aircraft_in_3g = conditions["plus_5g"].configuration.system
            aircraft_in_m1g = conditions["minus_3g"].configuration.system

            wing_in_3g = aircraft_in_3g.comps["aircraft"].comps["wing"]
            wing_in_m1g = aircraft_in_m1g.comps["aircraft"].comps["wing"]

            wing_in_3g.quantities.beam = beam_plus_5g
            wing_in_m1g.quantities.beam = beam_minus_3g


        # Fuselage
        fuselage = aircraft.comps["fuselage"]
        fuselage_length = fuselage.parameters.length

        # Empennage
        empennage = aircraft.comps["empennage"]
        h_tail = empennage.comps["h_tail"]
        h_tail_area = h_tail.parameters.S_ref
        v_tail = empennage.comps["v_tail"]
        v_tail_area =  v_tail.parameters.S_ref
        
        # Booms
        booms = aircraft.comps["booms"]

        # M4-regression mass models (scaled to match better with NDARC)
        nasa_lpc_weights = cd.aircraft.models.weights.nasa_lpc
        scaler = 1.3
        # wing_mps = nasa_lpc_weights.compute_wing_mps(
        #     wing_area=wing_area,
        #     wing_AR=wing_AR,
        #     fuselage_length=fuselage_length,
        #     battery_mass=battery_mass, 
        #     cruise_speed=cruise_speed,
        # )
        # wing_mps.mass = wing_mps.mass * scaler
        # wing.quantities.mass_properties.mass = wing_mps.mass
        # wing.quantities.mass_properties.cg_vector = wing_mps.cg_vector
        # wing.quantities.mass_properties.inertia_tensor = wing_mps.inertia_tensor

        fuselage_mps = nasa_lpc_weights.compute_fuselage_mps(
            wing_area=wing_area,
            wing_AR=wing_AR,
            fuselage_length=fuselage_length,
            battery_mass=battery_mass,
            cruise_speed=cruise_speed,
        )
        fuselage_mps.mass = fuselage_mps.mass * scaler
        fuselage.quantities.mass_properties.mass = fuselage_mps.mass
        fuselage.quantities.mass_properties.cg_vector = fuselage_mps.cg_vector
        fuselage.quantities.mass_properties.inertia_tensor = fuselage_mps.inertia_tensor

        boom_mps = nasa_lpc_weights.compute_boom_mps(
            wing_area=wing_area,
            wing_AR=wing_AR,
            fuselage_length=fuselage_length,
            battery_mass=battery_mass,
            cruise_speed=cruise_speed,
        )
        boom_mps.mass = boom_mps.mass * scaler
        booms.quantities.mass_properties.mass = boom_mps.mass
        booms.quantities.mass_properties.cg_vector = boom_mps.cg_vector
        booms.quantities.mass_properties.inertia_tensor = boom_mps.inertia_tensor

        empennage_mps = nasa_lpc_weights.compute_empennage_mps(
            h_tail_area=h_tail_area,
            v_tail_area=v_tail_area,
        )
        empennage_mps.mass = empennage_mps.mass * scaler
        empennage.quantities.mass_properties = empennage_mps

        # Motors
        motor_group = aircraft.comps["motors"]
        motors = list(motor_group.comps.values())
        
            # get rotor meshes to obtain thrust origin (i.e., motor cg)
        mesh_container = base_config.mesh_container
        rotor_meshes = mesh_container["rotor_meshes"]
        
            # Loop over all motors to assign mass properties
        for i, rotor_mesh in enumerate(rotor_meshes.discretizations.values()):
            motor_comp = motors[i]
            motor_mass = csdl.Variable(name=f"motor_{i}_mass", shape=(1, ), value=25)
            if run_optimization:
                if do_structural_sizing is True and run_ffd is False:
                    pass
                elif do_trim_optimization:
                    pass
                else:
                    motor_mass.set_as_design_variable(upper=50, lower=5, scaler=5e-2)
            motor_cg = rotor_mesh.thrust_origin
            motor_comp.quantities.mass_properties.mass = motor_mass
            motor_comp.quantities.mass_properties.cg_vector = motor_cg

        # payload
        payload = aircraft.comps["payload"]
        payload_mass = csdl.Variable(shape=(1, ), value=540+800)
        payload_cg = csdl.Variable(shape=(3, ), value=np.array([-3., 0., -1.5]))
        payload.quantities.mass_properties.mass = payload_mass
        payload.quantities.mass_properties.cg_vector = payload_cg

        # systems
        systems = aircraft.comps["systems"]
        systems_mass = csdl.Variable(shape=(1, ), value=244)
        systems_cg = csdl.Variable(shape=(3, ), value=np.array([-1., 0., -1.5]))
        systems.quantities.mass_properties.mass = systems_mass
        systems.quantities.mass_properties.cg_vector = systems_cg

        # Assemble system mass properties
        base_config.assemble_system_mass_properties(update_copies=True)

        aircraft_mass = base_config.system.quantities.mass_properties.mass
        aircraft_mass.name = "aircraft_mass"

def wing_weight_model(AR,S,m,p,t,spar_outer_diameter):
    #This is not CSDL'd at the moment bc im hoping to simplify it.
    b = m.sqrt(AR*S) #ft
    c = b/AR #ft

    m = 0.01*m  # maximum camber in % of chord
    p = 0.10*p  # maximum camber position in tenths of chord
    t = 0.01*t  # thickness in % of chord

    # Coefficients for 4 digit series
    a0 =  1.4845
    a1 = -0.6300
    a2 = -1.7580
    a3 =  1.4215
    a4 = -0.5075

    n = 1000 # number of points along the chord
    x = np.linspace(0,c,n) # x coordinate of points along the chord
    y   = np.zeros(n) # x coordinate of points along the chord
    yc  = np.zeros(n) # y coordinate of the camber line
    dyc = np.zeros(n) # gradient of the camber line
    yt  = np.zeros(n) # thickness distribution
    xu  = np.zeros(n) # x coordinate of the upper surface
    yu  = np.zeros(n) # y coordinate of the upper surface
    xl  = np.zeros(n) # x coordinate of the lower surface
    yl  = np.zeros(n) # y coordinate of the lower surface
    for i in range(n):
        if  (x[i]/c < p):
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


    # plot.xlim(-0.2,c+0.2)
    # plot.ylim(-c/3,c/3)
    # plot.plot(xu,yu,color='deepskyblue')   
    # plot.plot(xl,yl,color='deepskyblue')
    # plot.plot(x,yc,'g--') 

    upper = np.trapz(yu,xu)
    lower = np.trapz(yl,xl)

    total_area = upper + -(lower)

    foam_density = 1.5 #lb/ft**3
    volume_wing = total_area * b
    volume_spars = 2*m.pi*(spar_outer_diameter/2/12)**2*b #Volume of both spars in ft^3, spar outer diamter in in^3
    volume_total = volume_wing-volume_spars
    weight = volume_total*foam_density
    return weight

def define_analysis(caddee: cd.CADDEE):
    # #Below is the more regirmented, ex_lpc.py method of doing analysis
    # conditions = caddee.conditions
    # base_config = caddee.base_configuration
    # base_mps = base_config.system.quantities.mass_properties

    # trim_norm_list = []

    # if do_cruise:
    #     cruise = conditions["cruise"]
    #     accel_cruise, total_forces_cruise, total_moments_cruise = define_cruise(cruise)
    #     if do_trim_optimization:
    #         trim_norm_list.append(accel_cruise.accel_norm)


    # if do_structural_sizing:
    #     plus_5g = conditions["plus_5g"]
    #     accel_plus_5g, total_forces_plus_5g, total_moments_plus_5g = define_plus_5g(plus_5g)
        
    #     minus_3g = conditions["minus_3g"]
    #     accel_minus_3g, total_forces_minus_3g, total_moments_minus_3g = define_minus_3g(minus_3g)


    #Below is the less regimented way of doing analysis
    cruise = caddee.conditions["cruise"]
    cruise_config = cruise.configuration
    mesh_container = cruise_config.mesh_container
    base_config = caddee.base_configuration
    aircraft = base_config.system

    # Re-evaluate meshes and compute nodal velocities
    cruise.finalize_meshes()

    # Make an instance of an airfoil model
    nasa_langley_airfoil_maker = ThreeDAirfoilMLModelMaker(
        airfoil_name="ls417",
            aoa_range=np.linspace(-12, 16, 50), 
            reynolds_range=[1000, 1500, 2450, 5450, 10450, 110450, 210450, 310450], 
            mach_range=[0., 0.01, 0.02, 0.03, 0.04, 0.06],
    )
    Cl_model = nasa_langley_airfoil_maker.get_airfoil_model(quantities=["Cl"])

    vlm_mesh_0 = mesh_container["vlm_mesh_0"]
    wing_chord_surface = vlm_mesh_0.discretizations["wing_chord_surface"]
    h_tail_chord_surface = vlm_mesh_0.discretizations["h_tail_chord_surface"]

    lattice_coordinates = [wing_chord_surface.nodal_coordinates, h_tail_chord_surface.nodal_coordinates]
    lattice_nodal_velocities = [wing_chord_surface.nodal_velocities, h_tail_chord_surface.nodal_velocities]

    vlm_outputs_1 = vlm_solver(
        lattice_coordinates, 
        lattice_nodal_velocities, 
        atmos_states=cruise.quantities.atmos_states,
        airfoil_Cd_models=[None, None],
        airfoil_Cl_models=[Cl_model, None],
        airfoil_Cp_models=[None, None],
        airfoil_alpha_stall_models=[None, None],
    )

    # We multiply by (-1) since the lift and drag are w.r.t. the flight-dynamics reference frame
    total_induced_drag = vlm_outputs_1.total_drag * -1
    total_lift = vlm_outputs_1.total_lift * -1
    
    mark2_weight = aircraft.quantities.mass_properties.mass

    lift_constraint = total_lift - mark2_weight
    lift_constraint.name = "lift_equals_weight_constraint"
    lift_constraint.set_as_constraint(equals=0., scaler=1e-3)

    # set objectives and constraints
    total_induced_drag.name = "total_induced_drag"
    total_induced_drag.set_as_objective(scaler=1e-2)

def define_plus_5g(plus_5g):
    plus_5g_config = plus_5g.configuration
    mesh_container = plus_5g_config.mesh_container
    aircraft = plus_5g_config.system
    wing = aircraft.comps["wing"]
    fuselage = aircraft.comps["fuselage"]
    v_tail = aircraft.comps["empennage"].comps["v_tail"]
    rotors = aircraft.comps["rotors"]
    booms = list(aircraft.comps["booms"].comps.values())

    # Actuate tail
    tail = aircraft.comps["empennage"].comps["h_tail"]
    elevator_deflection = csdl.Variable(name="plus_5g_elevator", shape=(1, ), value=0)
    elevator_deflection.set_as_design_variable(lower=np.deg2rad(-20), upper=np.deg2rad(20), scaler=10)
    tail.actuate(elevator_deflection)

    # Re-evaluate meshes and compute nodal velocities
    plus_5g.finalize_meshes()

    # Set up VLM analysis
    vlm_mesh = mesh_container["vlm_mesh"]
    wing_lattice = vlm_mesh.discretizations["wing_chord_surface"]
    tail_lattice = vlm_mesh.discretizations["tail_chord_surface"]
    airfoil_upper_nodes = wing_lattice._airfoil_upper_para
    airfoil_lower_nodes = wing_lattice._airfoil_lower_para
    pressure_indexed_space : lfs.FunctionSetSpace = wing.quantities.pressure_space

    # run vlm solver
    lattice_coordinates = [wing_lattice.nodal_coordinates, tail_lattice.nodal_coordinates]
    lattice_nodal_velocitiies = [wing_lattice.nodal_velocities, tail_lattice.nodal_velocities]
    
     # Add an airfoil model
    nasa_langley_airfoil_maker = ThreeDAirfoilMLModelMaker(
        airfoil_name="ls417",
            aoa_range=np.linspace(-12, 16, 50), 
            reynolds_range=[1e5, 2e5, 5e5, 1e6, 2e6, 4e6, 7e6, 10e6], 
            mach_range=[0., 0.2, 0.3, 0.4, 0.5, 0.6],
    )
    Cl_model = nasa_langley_airfoil_maker.get_airfoil_model(quantities=["Cl"])
    Cd_model = nasa_langley_airfoil_maker.get_airfoil_model(quantities=["Cd"])
    Cp_model = nasa_langley_airfoil_maker.get_airfoil_model(quantities=["Cp"])
    alpha_stall_model = nasa_langley_airfoil_maker.get_airfoil_model(quantities=["alpha_Cl_min_max"])
    
    vlm_outputs = vlm_solver(
        lattice_coordinates, 
        lattice_nodal_velocitiies, 
        atmos_states=plus_5g.quantities.atmos_states,
        airfoil_Cd_models=[None, None],#=airfoil_Cd_models,
        airfoil_Cl_models=[Cl_model, None],
        airfoil_Cp_models=[Cp_model, None],
        airfoil_alpha_stall_models=[alpha_stall_model, None],
    )
    
    vlm_forces = vlm_outputs.total_force
    vlm_moments = vlm_outputs.total_moment
    
    if True:
        V_inf = plus_5g.parameters.speed
        rho_inf = plus_5g.quantities.atmos_states.density
        spanwise_Cp = vlm_outputs.surface_spanwise_Cp[0]
        spanwise_pressure = spanwise_Cp * 0.5 * rho_inf * V_inf**2
        spanwise_pressure = csdl.blockmat([[spanwise_pressure[0, :, 0:120].T()], [spanwise_pressure[0, :, 120:].T()]])
        
        pressure_function = pressure_indexed_space.fit_function_set(
            values=spanwise_pressure.reshape((-1, 1)), parametric_coordinates=airfoil_upper_nodes+airfoil_lower_nodes,
            regularization_parameter=1e-4,
        )

        if recorder.inline is True:
            wing.geometry.plot_but_good(color=pressure_function)
        box_beam_mesh = mesh_container["beam_mesh"]
        box_beam = box_beam_mesh.discretizations["wing_box_beam"]
        beam_nodes = box_beam.nodal_coordinates

        right_wing_inds = list(wing.quantities.right_wing_geometry.functions)
        force_magnitudes, force_para_coords = pressure_function.integrate(wing.geometry, grid_n=30, indices=right_wing_inds)
        force_magnitudes:csdl.Variable = force_magnitudes.flatten()
        force_coords = wing.geometry.evaluate(force_para_coords)
        force_normals = wing.geometry.evaluate_normals(force_para_coords)
        force_vectors = force_normals*csdl.expand(force_magnitudes, force_normals.shape, 'i->ij')

        mapper = acu.NodalMap()
        force_map = mapper.evaluate(force_coords, beam_nodes.reshape((-1, 3)))
        beam_forces = force_map.T() @ force_vectors

        beam_forces_plus_moments = csdl.Variable(shape=(beam_forces.shape[0], 6), value=0)
        beam_forces_plus_moments = beam_forces_plus_moments.set(
            csdl.slice[:, 0:3], beam_forces
        )

        # set up beam analysis
        beam: af.Beam = wing.quantities.beam
        beam.add_boundary_condition(node=0, dof=[1, 1, 1, 1, 1, 1])
        beam.add_load(beam_forces_plus_moments)

        frame = af.Frame()
        frame.add_beam(beam)

        struct_solution = frame.evaluate()

        beam_displacement = struct_solution.get_displacement(beam)
        beam_bkl_top = struct_solution.get_bkl(beam)["top"]
        beam_bkl_bot = struct_solution.get_bkl(beam)["bot"]
        beam_bkl_bot.name = "bottom_buckling_plus_5g"
        beam_bkl_top.name = "top_buckling_plus_5g"
        beam_bkl_bot.set_as_constraint(upper=1.)
        beam_bkl_top.set_as_constraint(upper=1.)
        # beam_stress = csdl.maximum(struct_solution.get_stress(beam))
        # # max_stress_csdl = csdl.maximum(beam_stress)
        # beam_stress.name = "max_stress"
        # beam_stress.set_as_constraint(upper=max_stress, scaler=1e-8)

    # Drag build-up
    drag_build_up_model = cd.aircraft.models.aero.compute_drag_build_up

    drag_build_up = drag_build_up_model(plus_5g.quantities.ac_states, plus_5g.quantities.atmos_states,
                                        wing.parameters.S_ref, [wing, fuselage, tail, v_tail, rotors] + booms)
    
    
    plus_5g_power = {}

    # # BEM solver
    # rotor_meshes = mesh_container["rotor_meshes"]
    # pusher_rotor_mesh = rotor_meshes.discretizations["pusher_prop_mesh"]
    # mesh_vel = pusher_rotor_mesh.nodal_velocities
    # plus_5g_rpm = csdl.Variable(name="plus_5g_pusher_rpm", shape=(1, ), value=1883.73389999)
    # plus_5g_rpm.set_as_design_variable(upper=3000, lower=1200, scaler=1e-3)
    # bem_inputs = RotorAnalysisInputs()
    # bem_inputs.ac_states = plus_5g.quantities.ac_states
    # bem_inputs.atmos_states =  plus_5g.quantities.atmos_states
    # bem_inputs.mesh_parameters = pusher_rotor_mesh
    # bem_inputs.mesh_velocity = mesh_vel
    # bem_inputs.rpm = plus_5g_rpm
    # bem_model = BEMModel(num_nodes=1, airfoil_model=NACA4412MLAirfoilModel())
    # bem_outputs = bem_model.evaluate(bem_inputs)
    # plus_5g_power["pusher_prop"] = bem_outputs.total_power
    # plus_5g.quantities.rotor_power_dict = plus_5g_power

    # total forces and moments
    total_forces_plus_5g, total_moments_plus_5g = plus_5g.assemble_forces_and_moments(
        aero_propulsive_forces=[vlm_forces, drag_build_up,], 
        aero_propulsive_moments=[vlm_moments], 
        load_factor=3,
    )

    # eom
    eom_model = cd.aircraft.models.eom.SixDofEulerFlatEarthModel()
    accel_plus_5g = eom_model.evaluate(
        total_forces=total_forces_plus_5g,
        total_moments=total_moments_plus_5g,
        ac_states=plus_5g.quantities.ac_states,
        ac_mass_properties=plus_5g_config.system.quantities.mass_properties
    )
    accel_norm_plus_5g = accel_plus_5g.accel_norm
    accel_norm_plus_5g.name = "plus_5g_trim"
    if do_trim_optimization:
        pass
    else:
        accel_norm_plus_5g.set_as_constraint(upper=0, lower=0, scaler=4)
    
    return accel_plus_5g, total_forces_plus_5g, total_moments_plus_5g

def define_minus_3g(minus_3g):
    minus_3g_config = minus_3g.configuration
    mesh_container = minus_3g_config.mesh_container
    aircraft = minus_3g_config.system
    wing = aircraft.comps["wing"]
    fuselage = aircraft.comps["fuselage"]
    v_tail = aircraft.comps["empennage"].comps["v_tail"]
    rotors = aircraft.comps["rotors"]
    booms = list(aircraft.comps["booms"].comps.values())

    # Actuate tail
    tail = aircraft.comps["empennage"].comps["h_tail"]
    elevator_deflection = csdl.Variable(name="minus_3g_elevator", shape=(1, ), value=0.)
    elevator_deflection.set_as_design_variable(lower=np.deg2rad(-20), upper=np.deg2rad(20), scaler=10)
    tail.actuate(elevator_deflection)

    # Re-evaluate meshes and compute nodal velocities
    minus_3g.finalize_meshes()

    # Set up VLM analysis
    vlm_mesh = mesh_container["vlm_mesh"]
    wing_lattice = vlm_mesh.discretizations["wing_chord_surface"]
    tail_lattice = vlm_mesh.discretizations["tail_chord_surface"]
    airfoil_upper_nodes = wing_lattice._airfoil_upper_para
    airfoil_lower_nodes = wing_lattice._airfoil_lower_para
    pressure_indexed_space : lfs.FunctionSetSpace = wing.quantities.pressure_space

    # run vlm solver
    lattice_coordinates = [wing_lattice.nodal_coordinates, tail_lattice.nodal_coordinates]
    lattice_nodal_velocitiies = [wing_lattice.nodal_velocities, tail_lattice.nodal_velocities]
    
     # Add an airfoil model
    nasa_langley_airfoil_maker = ThreeDAirfoilMLModelMaker(
        airfoil_name="ls417",
            aoa_range=np.linspace(-12, 16, 50), 
            reynolds_range=[1e5, 2e5, 5e5, 1e6, 2e6, 4e6, 7e6, 10e6], 
            mach_range=[0., 0.2, 0.3, 0.4, 0.5, 0.6],
    )
    Cl_model = nasa_langley_airfoil_maker.get_airfoil_model(quantities=["Cl"])
    Cd_model = nasa_langley_airfoil_maker.get_airfoil_model(quantities=["Cd"])
    Cp_model = nasa_langley_airfoil_maker.get_airfoil_model(quantities=["Cp"])
    alpha_stall_model = nasa_langley_airfoil_maker.get_airfoil_model(quantities=["alpha_Cl_min_max"])
    
    vlm_outputs = vlm_solver(
        lattice_coordinates, 
        lattice_nodal_velocitiies, 
        atmos_states=minus_3g.quantities.atmos_states,
        airfoil_Cd_models=[None, None],#=airfoil_Cd_models,
        airfoil_Cl_models=[Cl_model, None],
        airfoil_Cp_models=[Cp_model, None],
        airfoil_alpha_stall_models=[alpha_stall_model, None],
    )
    
    vlm_forces = vlm_outputs.total_force
    vlm_moments = vlm_outputs.total_moment
    
    if True:
        V_inf = minus_3g.parameters.speed
        rho_inf = minus_3g.quantities.atmos_states.density
        spanwise_Cp = vlm_outputs.surface_spanwise_Cp[0]
        spanwise_pressure = spanwise_Cp * 0.5 * rho_inf * V_inf**2
        spanwise_pressure = csdl.blockmat([[spanwise_pressure[0, :, 0:120].T()], [spanwise_pressure[0, :, 120:].T()]])
        
        pressure_function = pressure_indexed_space.fit_function_set(
            values=spanwise_pressure.reshape((-1, 1)), parametric_coordinates=airfoil_upper_nodes+airfoil_lower_nodes,
            regularization_parameter=1e-4,
        )

        # wing.geometry.plot_but_good(color=pressure_function)

        box_beam_mesh = mesh_container["beam_mesh"]
        box_beam = box_beam_mesh.discretizations["wing_box_beam"]
        beam_nodes = box_beam.nodal_coordinates

        right_wing_inds = list(wing.quantities.right_wing_geometry.functions)
        force_magnitudes, force_para_coords = pressure_function.integrate(wing.geometry, grid_n=30, indices=right_wing_inds)
        force_magnitudes:csdl.Variable = force_magnitudes.flatten()
        force_coords = wing.geometry.evaluate(force_para_coords)
        force_normals = wing.geometry.evaluate_normals(force_para_coords)
        force_vectors = force_normals*csdl.expand(force_magnitudes, force_normals.shape, 'i->ij')

        mapper = acu.NodalMap()
        force_map = mapper.evaluate(force_coords, beam_nodes.reshape((-1, 3)))
        beam_forces = force_map.T() @ force_vectors

        beam_forces_plus_moments = csdl.Variable(shape=(beam_forces.shape[0], 6), value=0)
        beam_forces_plus_moments = beam_forces_plus_moments.set(
            csdl.slice[:, 0:3], beam_forces
        )

        # set up beam analysis
        beam: af.Beam = wing.quantities.beam
        beam.add_boundary_condition(node=0, dof=[1, 1, 1, 1, 1, 1])
        beam.add_load(beam_forces_plus_moments)

        frame = af.Frame()
        frame.add_beam(beam)

        struct_solution = frame.evaluate()

        beam_displacement = struct_solution.get_displacement(beam)
        beam_bkl_top = struct_solution.get_bkl(beam)["top"]
        beam_bkl_bot = struct_solution.get_bkl(beam)["bot"]
        beam_bkl_top.name = "top_buckling_minus_3g"
        beam_bkl_bot.name = "bottom_buckling_minus_3g"
        beam_bkl_top.set_as_constraint(upper=1.)
        beam_bkl_bot.set_as_constraint(upper=1.)
        
        # beam_stress = csdl.maximum(struct_solution.get_stress(beam))
        # # max_stress_csdl = csdl.maximum(beam_stress)
        # beam_stress.name = "max_stress_minus_3g"
        # beam_stress.set_as_constraint(upper=max_stress, scaler=1e-8)

    # Drag build-up
    drag_build_up_model = cd.aircraft.models.aero.compute_drag_build_up

    drag_build_up = drag_build_up_model(minus_3g.quantities.ac_states, minus_3g.quantities.atmos_states,
                                        wing.parameters.S_ref, [wing, fuselage, tail, v_tail, rotors] + booms)
    
    
    minus_3g_power = {}

    # # BEM solver
    # rotor_meshes = mesh_container["rotor_meshes"]
    # pusher_rotor_mesh = rotor_meshes.discretizations["pusher_prop_mesh"]
    # mesh_vel = pusher_rotor_mesh.nodal_velocities
    # minus_3g_rpm = csdl.Variable(name="minus_3g_pusher_rpm", shape=(1, ), value=2000)
    # minus_3g_rpm.set_as_design_variable(upper=3000, lower=1200, scaler=1e-3)
    # bem_inputs = RotorAnalysisInputs()
    # bem_inputs.ac_states = minus_3g.quantities.ac_states
    # bem_inputs.atmos_states =  minus_3g.quantities.atmos_states
    # bem_inputs.mesh_parameters = pusher_rotor_mesh
    # bem_inputs.mesh_velocity = mesh_vel
    # bem_inputs.rpm = minus_3g_rpm
    # bem_model = BEMModel(num_nodes=1, airfoil_model=NACA4412MLAirfoilModel())
    # bem_outputs = bem_model.evaluate(bem_inputs)
    # minus_3g_power["pusher_prop"] = bem_outputs.total_power
    # minus_3g.quantities.rotor_power_dict = minus_3g_power

    # total forces and moments
    total_forces_minus_3g, total_moments_minus_3g = minus_3g.assemble_forces_and_moments(
        aero_propulsive_forces=[vlm_forces, drag_build_up], 
        aero_propulsive_moments=[vlm_moments], 
        load_factor=-1,
    )

    # eom
    eom_model = cd.aircraft.models.eom.SixDofEulerFlatEarthModel()
    accel_minus_3g = eom_model.evaluate(
        total_forces=total_forces_minus_3g,
        total_moments=total_moments_minus_3g,
        ac_states=minus_3g.quantities.ac_states,
        ac_mass_properties=minus_3g_config.system.quantities.mass_properties
    )
    accel_norm_minus_3g = accel_minus_3g.accel_norm
    accel_norm_minus_3g.name = "minus_3g_trim"
    if do_trim_optimization:
        pass
    else:
        accel_norm_minus_3g.set_as_constraint(upper=0, lower=0, scaler=4)
    
    return accel_minus_3g, total_forces_minus_3g, total_moments_minus_3g

def define_cruise(cruise):
    cruise_config = cruise.configuration
    mesh_container = cruise_config.mesh_container
    aircraft = cruise_config.system #This might throw errors??? Go check
    wing = aircraft.comps["wing"]
    fuselage = aircraft.comps["fuselage"]
    v_tail = aircraft.comps["empennage"].comps["v_tail"]
    boomFR = aircraft.comps["boom_FR"]
    boomBR = aircraft.comps["boom_BR"]
    boomFL = aircraft.comps["boom_FL"]
    boomBL = aircraft.comps["boom_BL"]
    #modfy
    #booms = list(aircraft.comps["booms"].comps.values())

    # Actuate tail
    tail = aircraft.comps["empennage"].comps["h_tail"]
    elevator_deflection = csdl.Variable(name="cruise_elevator", shape=(1, ), value=0)
    elevator_deflection.set_as_design_variable(lower=np.deg2rad(-10), upper=np.deg2rad(10), scaler=10)
    tail.actuate(elevator_deflection)

    # Re-evaluate meshes and compute nodal velocities
    cruise.finalize_meshes()

    # Set up VLM analysis
    vlm_mesh = mesh_container["vlm_mesh"]
    wing_lattice = vlm_mesh.discretizations["wing_chord_surface"]
    tail_lattice = vlm_mesh.discretizations["tail_chord_surface"]

    # run vlm solver
    lattice_coordinates = [wing_lattice.nodal_coordinates, tail_lattice.nodal_coordinates]
    lattice_nodal_velocitiies = [wing_lattice.nodal_velocities, tail_lattice.nodal_velocities]
    
     # Add an airfoil model
    nasa_langley_airfoil_maker = ThreeDAirfoilMLModelMaker(
        airfoil_name="ls417",
            aoa_range=np.linspace(-12, 16, 50), 
            reynolds_range=[1e5, 2e5, 5e5, 1e6, 2e6, 4e6, 7e6, 10e6], 
            mach_range=[0., 0.2, 0.3, 0.4, 0.5, 0.6],
    )
    Cl_model = nasa_langley_airfoil_maker.get_airfoil_model(quantities=["Cl"])
    
    vlm_outputs = vlm_solver(
        lattice_coordinates, 
        lattice_nodal_velocitiies, 
        atmos_states=cruise.quantities.atmos_states,
        airfoil_Cd_models=[None, None],#=airfoil_Cd_models,
        airfoil_Cl_models=[Cl_model, None],
        airfoil_Cp_models=[None, None],
        airfoil_alpha_stall_models=[None, None],
    )
    
    vlm_forces = vlm_outputs.total_force
    vlm_moments = vlm_outputs.total_moment
    
    # Drag build-up
    drag_build_up_model = cd.aircraft.models.aero.compute_drag_build_up

    drag_build_up = drag_build_up_model(cruise.quantities.ac_states, cruise.quantities.atmos_states,
                                        wing.parameters.S_ref, [wing, fuselage, tail, v_tail] + boomFR+boomBR+boomFL+boomBL)
    
    
    cruise_power = {}

    # # BEM solver
    # rotor_meshes = mesh_container["rotor_meshes"]
    # pusher_rotor_mesh = rotor_meshes.discretizations["pusher_prop_mesh"]
    # mesh_vel = pusher_rotor_mesh.nodal_velocities
    # cruise_rpm = csdl.Variable(name="cruise_pusher_rpm", shape=(1, ), value=1200)
    # cruise_rpm.set_as_design_variable(upper=2500, lower=1200, scaler=1e-3)
    # bem_inputs = RotorAnalysisInputs()
    # bem_inputs.ac_states = cruise.quantities.ac_states
    # bem_inputs.atmos_states =  cruise.quantities.atmos_states
    # bem_inputs.mesh_parameters = pusher_rotor_mesh
    # bem_inputs.mesh_velocity = mesh_vel
    # bem_inputs.rpm = cruise_rpm
    # bem_model = BEMModel(num_nodes=1, airfoil_model=NACA4412MLAirfoilModel())
    # bem_outputs = bem_model.evaluate(bem_inputs)
    # cruise_power["pusher_prop"] = bem_outputs.total_power
    # cruise.quantities.rotor_power_dict = cruise_power

    # total forces and moments
    total_forces_cruise, total_moments_cruise = cruise.assemble_forces_and_moments(
        [vlm_forces, drag_build_up], [vlm_moments]
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
    if do_trim_optimization:
        pass
    else:
        accel_norm_cruise.set_as_constraint(upper=0, lower=0, scaler=4)
    
    return accel_cruise, total_forces_cruise, total_moments_cruise

if __name__ == "__main__": #I like doing this because it makes it clear where the main executiom begins and also I can collapse it : )
    # Run the code (forward evaluation)

    # make instance of CADDEE class
    caddee = cd.CADDEE()

    #Define configuration in CADDEE
    define_base_config(caddee=caddee)

    #Define flight regimes in CADDEE
    define_conditions(caddee=caddee)

    #Define masses and set up mass models (how do masses change with change in parameters)
    #define_mass_properties(caddee=caddee)

    #define_sub_configurations(caddee)? Unsure if this is needed, figure out what its really doing.

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