// quadrocketbuilder setup for FluidX3D (https://github.com/ProjectPhysX/FluidX3D, non-commercial license): one aircraft
// in a uniform free stream at an angle of attack; writes force and moment about the CG (SI, geometry frame: x aft,
// y right, z up) to a CSV file. The case comes from the key/value file named by the QRB_CASE environment variable,
// written by cfd_fluidx3d.py, which also installs this file as FluidX3D/src/quadrocket_setup.cpp.
// Required extensions in defines.hpp: FP16S, FORCE_FIELD, EQUILIBRIUM_BOUNDARIES, SUBGRID (BENCHMARK off).
#include "setup.hpp"
#ifndef BENCHMARK
#include <cstdlib>
#include <fstream>
#include <map>

static std::map<string, string> read_case(const string& path) {
	std::ifstream file(path);
	if(file.fail()) print_error("QRB case file \""+path+"\" does not exist");
	std::map<string, string> values;
	string key, value;
	while(file>>key>>value) values[key] = value;
	return values;
}

static string case_text(const std::map<string, string>& values, const string& key) {
	const auto entry = values.find(key);
	if(entry==values.end()) print_error("QRB case file has no \""+key+"\"");
	return entry->second;
}

static float case_float(const std::map<string, string>& values, const string& key) {
	return std::stof(case_text(values, key));
}

void main_setup() {
	const char* case_path = std::getenv("QRB_CASE");
	if(case_path==nullptr) print_error("set QRB_CASE to a quadrocketbuilder case file");
	const std::map<string, string> values = read_case(case_path);
	const float si_length = case_float(values, "length_m"); // aircraft length along x (nose tip at x = 0)
	const float si_width = case_float(values, "width_m"); // largest y/z extent
	const float si_u = case_float(values, "airspeed_m_s"), si_nu = case_float(values, "nu_m2_s"), si_rho = case_float(values, "rho_kg_m3");
	const float alpha = radians(case_float(values, "alpha_deg"));
	const float upstream = case_float(values, "upstream_lengths"), downstream = case_float(values, "downstream_lengths"), lateral = case_float(values, "lateral_lengths");
	const float lbm_u = case_float(values, "lbm_u");
	const float flow_throughs = case_float(values, "flow_throughs");
	const float3 cg_mm = float3(case_float(values, "cg_x_mm"), case_float(values, "cg_y_mm"), case_float(values, "cg_z_mm"));
	const string output = case_text(values, "output_csv");

	const float si_Lx = (upstream+1.0f+downstream)*si_length, si_Lyz = si_width+2.0f*lateral*si_length;
	const uint3 lbm_N = resolution(float3(si_Lx, si_Lyz, si_Lyz), (uint)case_float(values, "vram_mb"));
	units.set_m_kg_s((float)lbm_N.x*si_length/si_Lx, lbm_u, 1.0f, si_length, si_u, si_rho);
	LBM lbm(lbm_N, units.nu(si_nu));
	const float cells_per_mm = units.x(0.001f);
	const float3 nose = float3(units.x(upstream*si_length), 0.5f*(float)lbm_N.y, 0.5f*(float)lbm_N.z); // geometry origin in grid coordinates
	Mesh* mesh = read_stl(case_text(values, "stl"), cells_per_mm, float3x3(1.0f), nose);
	lbm.voxelize_mesh_on_device(mesh, TYPE_S|TYPE_X);
	const float3 lbm_wind = lbm_u*float3(cosf(alpha), 0.0f, sinf(alpha)); // air moves aft and up past a nose-up aircraft
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if((lbm.flags[n]&TYPE_S)==0u) {
			lbm.u.x[n] = lbm_wind.x;
			lbm.u.z[n] = lbm_wind.z;
		}
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_E; // free stream on every face
	});
	const float3 lbm_cg = nose+cells_per_mm*cg_mm;
	const ulong lbm_flow_through = units.t(si_Lx/si_u);
	const ulong lbm_report = max(lbm_flow_through/(ulong)case_float(values, "reports_per_flow_through"), (ulong)1u);
	const ulong lbm_T = (ulong)(flow_throughs*(float)lbm_flow_through);
	print_info("QRB: Re_length = "+to_string(to_uint(units.si_Re(si_length, si_u, si_nu)))+", cells per length = "+to_string(units.x(si_length), 1u)
		+", flow-through = "+to_string(lbm_flow_through)+" steps, total "+to_string(lbm_T));
	write_file(output, "t_s,Fx_N,Fy_N,Fz_N,Mx_Nm,My_Nm,Mz_Nm\n");
	lbm.run(0u, lbm_T);
	while(lbm.get_t()<lbm_T) {
		lbm.run(lbm_report, lbm_T);
		const float3 force = lbm.object_force(TYPE_S|TYPE_X);
		const float3 torque = lbm.object_torque(lbm_cg, TYPE_S|TYPE_X);
		write_line(output, to_string(units.si_t(lbm.get_t()), 6u)+","+to_string(units.si_F(force.x), 6u)+","+to_string(units.si_F(force.y), 6u)+","+to_string(units.si_F(force.z), 6u)+","
			+to_string(units.si_M(torque.x), 6u)+","+to_string(units.si_M(torque.y), 6u)+","+to_string(units.si_M(torque.z), 6u)+"\n");
	}
}
#endif // BENCHMARK
