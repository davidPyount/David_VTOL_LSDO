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
#Determine why objective function isnt really working : (
#Get structural sizing working

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
do_structural_sizing = False
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
    # fuselage_length.set_as_design_variable(lower=0.8*7.5, upper=1.2*7.5)
    fuselage_length_var = csdl.Variable(fuselage_length)
    fuselage = cd.aircraft.components.Fuselage(
        length=fuselage_length_var, 
        max_height= 5/12 * ft2m,
        max_width= 5/12 * ft2m,
        geometry=fuselage_geometry,
        skip_ffd = True)
    
    # Assign fuselage component to aircraft
    aircraft.comps["fuselage"] = fuselage

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
    carbon_fiber = af.Material(name='carbon_fiber',E=96.2E9, G = 3.16E9, density = 1420)

    # Aerodynamic parameters for drag build up
    wing.quantities.drag_parameters.percent_laminar = 70
    wing.quantities.drag_parameters.percent_turbulent = 30

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
    main_spar = cd.Component(main_spar_geometry,length=main_spar_length)
    aircraft.comps["main_spar"] = main_spar

    # Connect h-tail to spar?
    base_config.connect_component_geometries(main_spar, h_tail, h_tail.TE_center) #TE_center doesnt work for some reason
    # Connect v-tail to spar?
    #base_config.connect_component_geometries(main_spar, v_tail, v_tail.TE_center) vtail skips ffd so can we connect???

    #Booms
    #Front Right
    boom_connection_point_initial = 18/12*ft2m
    boom_connection_point = csdl.Variable(name = 'Boom Connection Point',shape = (1,),value = boom_connection_point_initial)
    boom_connection_point.set_as_design_variable(upper = wing_span *1.2, lower = 0.5, scaler = 1e-1) #Bounds kind of arbitrary.

    boomFR_geometry = aircraft.create_subgeometry(
        search_names=["FrontRightBoom"]
    )
    boomFR = cd.Component(boomFR_geometry,length=wing_boom_length/2)
    aircraft.comps["boom_FR"] = boomFR
    base_config.connect_component_geometries(boomFR,wing,connection_point=(wing.LE_center+wing.TE_center)/2+boom_connection_point)

    #Back Right
    boomBR_geometry = aircraft.create_subgeometry(
        search_names=["BackRightBoom"]
    )
    boomBR = cd.Component(boomBR_geometry,length=wing_boom_length/2)
    aircraft.comps["boom_BR"] = boomBR
    base_config.connect_component_geometries(boomBR,wing,connection_point=(wing.LE_center+wing.TE_center)/2+boom_connection_point)

    #Front Left
    boomFL_geometry = aircraft.create_subgeometry(
        search_names=["FrontLeftBoom"]
    )
    boomFL = cd.Component(boomFL_geometry,length=wing_boom_length/2)
    aircraft.comps["boom_FL"] = boomFL
    base_config.connect_component_geometries(boomFL,wing,connection_point=(wing.LE_center+wing.TE_center)/2-boom_connection_point)

    #Back Left
    boomBL_geometry = aircraft.create_subgeometry(
        search_names=["BackLeftBoom"]
    )
    boomBL = cd.Component(boomBL_geometry,length=wing_boom_length/2)
    aircraft.comps["boom_BL"] = boomBL
    base_config.connect_component_geometries(boomBL,wing,connection_point=(wing.LE_center+wing.TE_center)/2-boom_connection_point)

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
        speed=21.336,
    )
    cruise.configuration = base_config.copy()
    conditions["cruise"] = cruise

def define_mass_properties(caddee : cd.CADDEE):
    """Define vehicle-level mass properties of the base configuration."""
    lpc = False #We need to do aframe before the rest like this has it.
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
        fuselage = aircraft.comps["fuselage"] #also not a thing rn causes as shape error when I load it in define base config
        fuselage_length = fuselage.parameters.length

        h_tail = aircraft.comps["h_tail"]
        h_tail_area = h_tail.parameters.S_ref
        v_tail = aircraft.comps["v_tail"]
        v_tail_area =  v_tail.parameters.S_ref
        
        # Booms
        booms = aircraft.comps["booms"]

        fuselage_mps.mass = fuselage_mps.mass * scaler
        fuselage.quantities.mass_properties.mass = fuselage_mps.mass
        fuselage.quantities.mass_properties.cg_vector = fuselage_mps.cg_vector
        fuselage.quantities.mass_properties.inertia_tensor = fuselage_mps.inertia_tensor

        boom_mps.mass = boom_mps.mass * scaler
        booms.quantities.mass_properties.mass = boom_mps.mass
        booms.quantities.mass_properties.cg_vector = boom_mps.cg_vector
        booms.quantities.mass_properties.inertia_tensor = boom_mps.inertia_tensor

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
        payload = fuselage.comps["payload"]

        # design gross weight estimate dont need because we dont need it for statistical sizing?
        #dg_est = csdl.ImplicitVariable(shape=(1, ), value=6)
        dg_est = 6

        # wing mass
        ## WRITE CUSTOM WING MASS MODEL AS FUNCTION OF CSDL VARIABLES ()
        S_ref=wing.parameters.S_ref #Are these being treated correctly as CSDL variables?
        AR=wing.parameters.AR

        spar_OD = 0.25 #??? I think I need to make a spar componet in CADDEE first?
        wing_weight = wing_weight_model(AR,S_ref,csdl.Variable(value=4),csdl.Variable(value=4),csdl.Variable(value=12),spar_outer_diameter=spar_OD)

        # fuselage mass CONST
        fuselage_weight = 3

        # h tail mass
        h_tail_weight = wing_weight_model(h_tail.AR,h_tail.S,csdl.Variable(value=0),csdl.Variable(value=0),csdl.Variable(value = 12),csdl.Variable(value=0))

        # v tail mass
        v_tail_weight = 0.1

        # avionics mass CONST
        avionics_weight = 0.5

        # instruments mass CONST
        instruments_weight = 0.5

        # Landing gear mass CONST
        landing_gear_weight = 0.2

        # Battery mass CONST but moveable CG!
        battery_weight = 1   

        #You should probably find a more accurate CG for these things
        wing.quantities.mass_properties.mass = wing_weight * units.mass.pound_to_kg
        wing.quantities.mass_properties.cg_vector = np.array([1 * units.length.foot_to_m, 0. , 2. * units.length.foot_to_m])

        fuselage.quantities.mass_properties.mass = fuselage_weight * units.mass.pound_to_kg
        fuselage.quantities.mass_properties.cg_vector = np.array([1 * units.length.foot_to_m, 0. , 0.])

        h_tail.quantities.mass_properties.mass = h_tail_weight * units.mass.pound_to_kg
        h_tail.quantities.mass_properties.cg_vector = np.array([1 * units.length.foot_to_m, 0., 0.051 * units.length.foot_to_m])

        v_tail.quantities.mass_properties.mass = h_tail_weight * units.mass.pound_to_kg
        v_tail.quantities.mass_properties.cg_vector = np.array([1 * units.length.foot_to_m, 0., 1.51 * units.length.foot_to_m])

        battery.quantities.mass_properties.mass = battery_weight * units.mass.pound_to_kg
        battery.quantities.mass_properties.cg_vector = np.array([1 * units.length.foot_to_m, 0. , 0.]) #Not true, figure this out

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

        base_config.assemble_system_mass_properties()

        print(aircraft.quantities.mass_properties.mass.value)
        print(base_config.system.quantities.mass_properties)

        ## THE LPC WAY ################################################################
            # Get base config and conditions

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
    cruise = caddee.conditions["cruise"]
    cruise_config = cruise.configuration
    mesh_container = cruise_config.mesh_container
    base_config = caddee.base_configuration
    aircraft = base_config.system

    # Cruise stuff
    # Re-evaluate meshes and compute nodal velocities
    cruise.finalize_meshes()

    # Make an instance of an airfoil model
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
        airfoil_Cl_models=[None, None],
        airfoil_Cp_models=[None, None],
        airfoil_alpha_stall_models=[None, None],
    )


    vlm_forces = vlm_outputs_1.total_force
    vlm_moments = vlm_outputs_1.total_moment

    # We multiply by (-1) since the lift and drag are w.r.t. the flight-dynamics reference frame
    total_induced_drag = vlm_outputs_1.total_drag * -1
    total_lift = vlm_outputs_1.total_lift * -1

    #Do strucutal sizing and weights model? Update structure and weights?
    
    #BEM STUFF, not using qst (quasi-steady transition)
    # Drag build-up
    drag_build_up_model = cd.aircraft.models.aero.compute_drag_build_up

    wing = aircraft.comps["wing"]
    fuselage = aircraft.comps["fuselage"]
    tail = aircraft.comps["tail"]
    v_tail = aircraft.comps['v_tail']
    booms = aircraft.comps["booms"]
    drag_build_up = drag_build_up_model(cruise.quantities.ac_states, cruise.quantities.atmos_states,
                                        wing.parameters.S_ref, [wing, fuselage, tail, v_tail] + booms)
    
    cruise_power = {}

    # BEM solver
    rotor_meshes = mesh_container["rotor_meshes"] #add these
    pusher_rotor_mesh = rotor_meshes.discretizations["pusher_prop_mesh"] #add these
    mesh_vel = pusher_rotor_mesh.nodal_velocities
    cruise_rpm = csdl.Variable(name="cruise_pusher_rpm", shape=(1, ), value=1200) #check this
    cruise_rpm.set_as_design_variable(upper=2500, lower=1200, scaler=1e-3) #and this
    bem_inputs = RotorAnalysisInputs()
    bem_inputs.ac_states = cruise.quantities.ac_states
    bem_inputs.atmos_states =  cruise.quantities.atmos_states
    bem_inputs.mesh_parameters = pusher_rotor_mesh
    bem_inputs.mesh_velocity = mesh_vel
    bem_inputs.rpm = cruise_rpm
    bem_model = BEMModel(num_nodes=1, airfoil_model=NACA4412MLAirfoilModel())
    bem_outputs = bem_model.evaluate(bem_inputs)
    cruise_power["pusher_prop"] = bem_outputs.total_power
    cruise.quantities.power = cruise_power

    # total forces and moments
    total_forces_cruise, total_moments_cruise = cruise.assemble_forces_and_moments(
        [vlm_forces, drag_build_up, bem_outputs.forces], [vlm_moments, bem_outputs.moments]
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

    #This is how the trim residual is set I think????
    accel_norm_cruise.set_as_constraint(upper=0, lower=0, scaler=4)
    
    # #OLD BAD TRIM RESIDUAL
    # #mark2_weight = aircraft.quantities.mass_properties.mass
    # mark2_weight = csdl.Variable(value=6,name="Gross Weight",shape = (1,))

    # #This is like a very basic trim residual, do a better one!!!!
    # lift_constraint = total_lift - mark2_weight
    # lift_constraint.name = "lift_equals_weight_constraint"
    # lift_constraint.set_as_constraint(equals=0., scaler=1e-3)

    # # set objectives and constraints
    # total_induced_drag.name = "total_induced_drag"
    # #thrust = -total_induced_drag
    
    # total_induced_drag.set_as_objective(scaler=1e-2)

    ########### Mission Power Analysis
    cruise_veloicty = cruise.speed
    R = csdl.Variable(value=10e3) #m
    cruise_time = R/cruise_veloicty #s

    cruise_pusher_rotor_power = csdl.Variable(name="cruise_pusher_power",shape=(1,), value = 223) #W
    cruise_pusher_rotor_power.set_as_constraint(upper=355, lower=0, scaler=1e-3) #is this a good scaler?
    #Should this be a constraint or a design variable?

    total_power = cruise_pusher_rotor_power / 0.95 #effciency factor?
    mission_time = cruise_time
    mission_energy = total_power*mission_time #W-seconds
    #mission_energy.set_as_design_variable(upper = 2300,lower = 0, scaler = 1e-3)
    ER = mission_energy/R #Is this properly a csdl variable? I feel like I did this wrong lmaooo

    aircraft = caddee.base_configuration.system
    wing = aircraft.comps["wing"]

    weight = wing_weight_model(wing.parameters.AR,wing.parameters.S_ref,csdl.Variable(value=4),csdl.Variable(value=4),csdl.Variable(value=12),csdl.Variable(value=0.2))
    print(f"The calculated wing weight is {weight}")

    ## We have constant mission range. Shoulnd't vortexAD inform power required for cruise at specified velocity?

    #SET AS OBJECTIVVEEEEE
    #ER.set_as_objective()

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