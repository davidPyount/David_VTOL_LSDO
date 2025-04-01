'''Induced drag minimization example'''
import CADDEE_alpha as cd
import csdl_alpha as csdl
import numpy as np
from lsdo_airfoil.core.three_d_airfoil_aero_model import ThreeDAirfoilMLModelMaker
from VortexAD.core.vlm.vlm_solver import vlm_solver
from modopt import CSDLAlphaProblem, SLSQP
from CADDEE_alpha.utils.units import Units
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
    
    #Are we fixing chord? I guess by fixing AR and wing area determined from VLM we necessitate certain bounds on AR

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

    # Assign wing component to aircraft
    aircraft.comps["wing"] = wing

    # Make horizontal tail geometry & component
    h_tail_geometry = aircraft.create_subgeometry(search_names=["HStab"])
    h_tail = cd.aircraft.components.Wing(
        AR=h_stab_AR, S_ref=h_stab_area, taper_ratio=h_stab_taper, 
        geometry=h_tail_geometry
    )

    # Assign tail component to aircraft
    aircraft.comps["h_tail"] = h_tail

    # Make vertical tail geometry & component
    v_tail_geometry = aircraft.create_subgeometry(search_names=["VStab"])
    v_tail = cd.aircraft.components.Wing(
        AR=v_stab_AR, S_ref=v_stab_area, geometry=v_tail_geometry, 
        skip_ffd=True, orientation="vertical"
    )

    # Assign v-tail component to aircraft
    aircraft.comps["v_tail"] = v_tail

    # Main spar
    # main_spar_geometry = aircraft.create_subgeometry(
    #     search_names=["Boom"],
    # )

    # # I don't know how to define a component with custom parameters 
    # main_spar = Component(length=3)
    # aircraft.comps["main_spar"] = main_spar


    # Connect wing to fuselage at the quarter chord
    base_config.connect_component_geometries(fuselage, wing, 0.75 * wing.LE_center + 0.25 * wing.TE_center)
    base_config.connect_component_geometries(fuselage, h_tail, h_tail.TE_center)
    # base_config.connect_component_geometries(main_spar, h_tail, h_tail.TE_center)

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

    #Do we need to do any VLM stuff for drag buildup on the fuselage???

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





def define_mass_properties(caddee : cd.CADDEE):
    """Define vehicle-level mass properties of the base configuration."""
    base_config = caddee.base_configuration
    conditions = caddee.conditions

    # get some operational variables from the cruise condition
    cruise = conditions["cruise_1"]
    fast_cruise = conditions["cruise_2"]
    rho_imperial = cruise.quantities.atmos_states.density * (1 / units.mass.slug_to_kg) / (1 / units.length.foot_to_m)**3
    speed_imperial = cruise.parameters.speed * (1 / units.speed.ftps_to_mps)
    q_cruise = 0.5 * rho_imperial * speed_imperial**2
    range_imperial = cruise.parameters.range * (1/ units.length.nautical_mile_to_m)
    max_mach = fast_cruise.parameters.mach_number

    #Access the base config and the its components
    aircraft = base_config.system
    airframe = aircraft.comps["airframe"]

    wing = airframe.comps["wing"]
    fuselage = airframe.comps["fuselage"]
    h_tail = airframe.comps["h_tail"]
    v_tail = airframe.comps["v_tail"]
    #main_landing_gear = airframe.comps["main_landing_gear"]
    #avionics = fuselage.comps["avionics"]
    #instruments = fuselage.comps["instruments"]
    #engine = aircraft.comps["engine"]
    #fuel = wing.comps["fuel"]
    payload = fuselage.comps["payload"]

    # design gross weight estimate
    dg_est = csdl.ImplicitVariable(shape=(1, ), value=2200)

    #fuel_weight = csdl.Variable(shape=(1,), value=500)
    #engine_weight = csdl.Variable(shape=(1,), value=250)
    payload_weight =  csdl.Variable(shape=(1,), value=500)

    # wing mass
    ## WRITE CUSTOM WING MASS MODEL AS FUNCTION OF CSDL VARIABLES ()
    S_ref=wing.parameters.S_ref
    AR=wing.parameters.AR
    #taper_ratio=wing.parameters.taper_ratio
    #thickness_to_chord=wing.parameters.thickness_to_chord

    spar_OD = 0.5 #??? I think I need to make a spar componet in CADDEE first?
    wing_weight = wing_weight_model(AR,S_ref,4,4,12,spar_OD)

    # fuselage mass CONST
    fuselage_weight = 3

    # h tail mass
    h_tail_weight_inputs = GAHorizontalTailInputs(
        S_ref=h_tail.parameters.S_ref,
        W_gross_design=dg_est,
        q_cruise=q_cruise,
    )
    h_tail_weight_model = GAHorizontalTailWeigthModel()
    h_tail_weight = h_tail_weight_model.evaluate(h_tail_weight_inputs)

    # v tail mass
    v_tail_weight_inputs = GAVerticalTailInputs(
        S_ref=v_tail.parameters.S_ref,
        AR=v_tail.parameters.AR,
        W_gross_design=dg_est,
        q_cruise=q_cruise,
        t_o_c=0.12,
        sweep_c4=np.deg2rad(30),
    )
    v_tail_weight_model = GAVerticalTailWeigthModel()
    v_tail_weight = v_tail_weight_model.evaluate(v_tail_weight_inputs)

    # avionics mass CONST
    avionics_weight = 1

    # instruments mass CONST
    instruments_weight = 0.5

    # Landing gear mass CONST
    landing_gear_weight = 0.2    

    #You should probably find a more accurate CG for these things

    wing.quantities.mass_properties.mass = wing_weight * units.mass.pound_to_kg
    wing.quantities.mass_properties.cg_vector = np.array([9.649 * units.length.foot_to_m, 0. , 2. * units.length.foot_to_m])

    fuselage.quantities.mass_properties.mass = fuselage_weight * units.mass.pound_to_kg
    fuselage.quantities.mass_properties.cg_vector = np.array([9.649 * units.length.foot_to_m, 0. , 0.])

    h_tail.quantities.mass_properties.mass = h_tail_weight * units.mass.pound_to_kg
    h_tail.quantities.mass_properties.cg_vector = np.array([25.137 * units.length.foot_to_m, 0., 0.051 * units.length.foot_to_m])

    v_tail.quantities.mass_properties.mass = h_tail_weight * units.mass.pound_to_kg
    v_tail.quantities.mass_properties.cg_vector = np.array([25.137 * units.length.foot_to_m, 0., 1.51 * units.length.foot_to_m])

    payload.quantities.mass_properties.mass = payload_weight * units.mass.pound_to_kg
    payload.quantities.mass_properties.cg_vector = np.array([9.649 * units.length.foot_to_m, 0. , 0.75 * units.length.foot_to_m])

    weights_solver = cd.aircraft.models.weights.WeightsSolverModel()
    weights_solver.evaluate(
        dg_est, wing_weight, fuselage_weight, h_tail_weight, v_tail_weight, avionics_weight, instruments_weight, engine_weight, fuel_weight, payload_weight, landing_gear_weight
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

def wing_weight_model(AR,S,m,p,t,spar_outer_diameter):
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
    cruise = caddee.conditions["cruise"]
    cruise_config = cruise.configuration
    mesh_container = cruise_config.mesh_container

    # Re-evaluate meshes and compute nodal velocities
    cruise.finalize_meshes()

    ## DO WE NEED AN AIRFOIL MODEL?
    # Make an instance of an airfoil model
    nasa_langley_airfoil_maker = ThreeDAirfoilMLModelMaker(
        airfoil_name="ls417",
            aoa_range=np.linspace(-12, 16, 50), 
            reynolds_range=[1e5, 2e5, 5e5, 1e6, 2e6, 4e6, 7e6, 10e6], 
            mach_range=[0., 0.2, 0.3, 0.4, 0.5, 0.6],
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
    
    mark2_weight = weight ##I GOTTA DO SOMETHING HERE I GUESS

    lift_constraint = total_lift - mark2_weight
    lift_constraint.name = "lift_equals_weight_constraint"
    lift_constraint.set_as_constraint(equals=0., scaler=1e-3)

    # set objectives and constraints
    total_induced_drag.name = "total_induced_drag"
    total_induced_drag.set_as_objective(scaler=1e-2)


if __name__ == "__main__": #I like doing this because it makes it clear where the main executiom begins
    # Run the code (forward evaluation)

    # make instance of CADDEE class
    caddee = cd.CADDEE()

    #Define configuration in CADDEE
    define_base_config(caddee=caddee)

    #Define flight regimes in CADDEE
    define_conditions(caddee=caddee)

    #Define masses and set up mass models (how do masses change with change is parameters)
    define_mass_properties(caddee=caddee)

    #define_sub_configurations(caddee)? Unsure if this is needed, figure out what its really doing.

    #What analysis are we performing, I beleive this is where aero,stability,strucutral requirements come in
    define_analysis(caddee=caddee)

    # Run optimization (Now actually do the optimization)
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