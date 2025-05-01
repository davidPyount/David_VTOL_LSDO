






base_config = caddee.base_configuration
    mesh_container = base_config.mesh_container
    conditions = caddee.conditions
    cruise = conditions["cruise"]
    aircraft = base_config.system


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