'''Induced drag minimization example'''
import CADDEE_alpha as cd
import csdl_alpha as csdl
import numpy as np
from lsdo_airfoil.core.three_d_airfoil_aero_model import ThreeDAirfoilMLModelMaker
from VortexAD.core.vlm.vlm_solver import vlm_solver
from modopt import CSDLAlphaProblem, SLSQP


# Start the CSDL recorder
recorder = csdl.Recorder(inline=True, expand_ops=True)
recorder.start()

# import C172 geometry
mark2_geom = cd.import_geometry("C:/Users/seth3/David_VTOL_LSDO/LSDOGeoTests/mark2.stp")
plotting_elements = mark2_geom.plot(show=False, opacity=0.5, color='#FFCD00')

# make instance of CADDEE class
caddee = cd.CADDEE()

ft2m = 0.3048
N2lb = 4.44822
# define initial values for design parameters
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
    
    # Set design variables for wing
    aspect_ratio.set_as_design_variable(upper=1.5 * wing_AR, lower=0.5 * wing_AR, scaler=1/8)
    wing_root_twist.set_as_design_variable(upper=np.deg2rad(5), lower=np.deg2rad(-5), scaler=4)
    wing_tip_twist.set_as_design_variable(upper=np.deg2rad(10), lower=np.deg2rad(-10), scaler=2)
    
    # Are taper ratio and area held constant here?
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

def define_analysis(caddee: cd.CADDEE):
    cruise = caddee.conditions["cruise"]
    cruise_config = cruise.configuration
    mesh_container = cruise_config.mesh_container

    # Re-evaluate meshes and compute nodal velocities
    cruise.finalize_meshes()

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
    mark2_weight = weight

    lift_constraint = total_lift - mark2_weight
    lift_constraint.name = "lift_equals_weight_constraint"
    lift_constraint.set_as_constraint(equals=0., scaler=1e-3)

    # set objectives and constraints
    total_induced_drag.name = "total_induced_drag"
    total_induced_drag.set_as_objective(scaler=1e-2)

# Run the code (forward evaluation)
define_base_config(caddee=caddee)

define_conditions(caddee=caddee)

define_analysis(caddee=caddee)

# Run optimization
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