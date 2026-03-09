#include "core/ConfigParser.hpp"
#include "core/Logger.hpp"

#include <pugixml.hpp>
#include <stdexcept>
#include <cmath>

namespace sim::core {
    // Helpers: read an attribute with a default value
    static double attr_double(const pugi::xml_node& node, const char* name, double def) {
        auto attr = node.attribute(name);
        return attr ? attr.as_double(def) : def;
    }
    static int attr_int(const pugi::xml_node& node, const char* name, int def) {
        auto attr = node.attribute(name);
        return attr ? attr.as_int(def) : def;
    }
    static std::string attr_string(const pugi::xml_node& node, const char* name,
                                const std::string& def) {
        auto attr = node.attribute(name);
        return attr ? std::string(attr.as_string(def.c_str())) : def;
    }

    // Helper: read a child element's text as double

    static double child_double(const pugi::xml_node& parent, const char* name, double def) {
        auto child = parent.child(name);
        return child ? child.text().as_double(def) : def;
    }

    static std::string child_string(const pugi::xml_node& parent, const char* name,
                                    const std::string& def) {
        auto child = parent.child(name);
        return child ? std::string(child.text().as_string(def.c_str())) : def;
    }
    
    // Helper: read a Vec3d from attributes
    static math::Vec3d read_vec3(const pugi::xml_node& node,
                                const char* x_attr, const char* y_attr, const char* z_attr,
                                const math::Vec3d& def = math::Vec3d::zero())
    {
        return {
            attr_double(node, x_attr, def.x()),
            attr_double(node, y_attr, def.y()),
            attr_double(node, z_attr, def.z())
        };
    }

    // Helper: read a diagonal inertia tensor
    static math::Mat3d read_inertia(const pugi::xml_node& node, const math::Mat3d& def) {
        auto inertia_node = node.child("inertia");
        if (!inertia_node) return def;

        double Ixx = attr_double(inertia_node, "Ixx", def(0, 0));
        double Iyy = attr_double(inertia_node, "Iyy", def(1, 1));
        double Izz = attr_double(inertia_node, "Izz", def(2, 2));
        double Ixy = attr_double(inertia_node, "Ixy", 0.0);
        double Ixz = attr_double(inertia_node, "Ixz", 0.0);
        double Iyz = attr_double(inertia_node, "Iyz", 0.0);

        // Aerospace sign convention: off-diagonals are stored with
        // negative signs in the tensor. The XML values are the
        // magnitudes of the products of inertia.
        return math::Mat3d{
            Ixx, -Ixy, -Ixz,
            -Ixy,  Iyy, -Iyz,
            -Ixz, -Iyz,  Izz
        };
    }

    // Parse sections
    static IntegratorConfig parse_integrator(const pugi::xml_node& node) {
        IntegratorConfig cfg;
        if (!node) return cfg;

        cfg.type = attr_string(node, "type", cfg.type);
        cfg.dt = attr_double(node, "dt", cfg.dt);

        SIM_DEBUG("Config: integrator type={}, dt={}", cfg.type, cfg.dt);
        return cfg;
    }

    static RecorderConfig parse_recorder(const pugi::xml_node& node) {
        RecorderConfig cfg;
        if (!node) return cfg;

        cfg.filepath = attr_string(node, "filepath", cfg.filepath);
        cfg.interval = attr_double(node, "interval", cfg.interval);

        SIM_DEBUG("Config: recorder filepath={}, interval={}s", cfg.filepath, cfg.interval);
        return cfg;
    }

    static StopConfig parse_stop(const pugi::xml_node& node) {
        StopConfig cfg;
        if (!node) return cfg;

        cfg.max_time = child_double(node, "max_time", cfg.max_time);
        cfg.min_altitude = child_double(node, "min_altitude", cfg.min_altitude);
        cfg.poca_range_gate = child_double(node, "poca_range_gate", cfg.poca_range_gate);

        SIM_DEBUG("Config: stop max_time={}s, min_alt={}m, poca_gate={}m",
              cfg.max_time, cfg.min_altitude, cfg.poca_range_gate);
        return cfg;
    }

    static InitialConditions parse_initial_conditions(const pugi::xml_node& node) {
        InitialConditions ic;
        if (!node) return ic;

        auto pos_node = node.child("position");
        if (pos_node) {
            ic.position = read_vec3(pos_node, "north", "east", "down", ic.position);
        }

        ic.speed = child_double(node, "speed", ic.speed);
        ic.pitch_deg = child_double(node, "pitch_deg", ic.pitch_deg);
        ic.yaw_deg = child_double(node, "yaw_deg", ic.yaw_deg);
        ic.roll_deg = child_double(node, "roll_deg", ic.roll_deg);

        SIM_DEBUG("Config: IC speed={} m/s, pitch={} deg, yaw={} deg",
                ic.speed, ic.pitch_deg, ic.yaw_deg);
        return ic;
    }

    static AtmosphereConfig parse_atmosphere(const pugi::xml_node& node) {
        AtmosphereConfig cfg;
        if (!node) return cfg;

        cfg.type = attr_string(node, "type", cfg.type);
        return cfg;
    }

    static GravityConfig parse_gravity(const pugi::xml_node& node) {
        GravityConfig cfg;
        if (!node) return cfg;

        cfg.type = attr_string(node, "type", cfg.type);
        cfg.g = attr_double(node, "g", cfg.g);
        return cfg;
    }

    static PropulsionConfig parse_propulsion(const pugi::xml_node& node) {
        PropulsionConfig cfg;
        if (!node) return cfg;

        cfg.type = attr_string(node, "type", cfg.type);
        cfg.thrust = child_double(node, "thrust", cfg.thrust);
        cfg.burn_time = child_double(node, "burn_time", cfg.burn_time);
        cfg.total_mass = child_double(node, "total_mass", cfg.total_mass);
        cfg.prop_mass = child_double(node, "prop_mass", cfg.prop_mass);

        cfg.inertia = read_inertia(node, cfg.inertia);

        auto cg_node = node.child("cg");
        if (cg_node) {
            cfg.cg_body = read_vec3(cg_node, "x", "y", "z", cfg.cg_body);
        }

        SIM_DEBUG("Config: propulsion type={}, thrust={} N, burn={}s",
                cfg.type, cfg.thrust, cfg.burn_time);
        return cfg;
    }

    static AerodynamicsConfig parse_aerodynamics(const pugi::xml_node& node) {
        AerodynamicsConfig cfg;
        if (!node) return cfg;

        cfg.type = attr_string(node, "type", cfg.type);

        // Static coefficients
        cfg.CA = child_double(node, "CA", cfg.CA);
        cfg.CN_alpha = child_double(node, "CN_alpha", cfg.CN_alpha);
        cfg.CY_beta = child_double(node, "CY_beta", cfg.CY_beta);

        // Moment coefficients
        cfg.Cm_alpha = child_double(node, "Cm_alpha", cfg.Cm_alpha);
        cfg.Cn_beta = child_double(node, "Cn_beta", cfg.Cn_beta);
        cfg.Cl_delta = child_double(node, "Cl_delta", cfg.Cl_delta);
        cfg.Cm_delta = child_double(node, "Cm_delta", cfg.Cm_delta);
        cfg.Cn_delta = child_double(node, "Cn_delta", cfg.Cn_delta);

        // Damping
        cfg.Cmq = child_double(node, "Cmq", cfg.Cmq);
        cfg.Cnr = child_double(node, "Cnr", cfg.Cnr);
        cfg.Clp = child_double(node, "Clp", cfg.Clp);

        SIM_DEBUG("Config: aero type={}, CA={}, CN_alpha={}, Cm_alpha={}",
                cfg.type, cfg.CA, cfg.CN_alpha, cfg.Cm_alpha);
        return cfg;
    }

    static SeekerConfig parse_seeker(const pugi::xml_node& node) {
        SeekerConfig cfg;
        cfg.type = attr_string(node, "type", cfg.type);
        cfg.min_range = attr_double(node, "min_range", cfg.min_range);
        SIM_DEBUG("Config: seeker type={}, min_range={} m", cfg.type, cfg.min_range);
        return cfg;
    }

    static GuidanceConfig parse_guidance(const pugi::xml_node& node) {
        GuidanceConfig cfg;
        cfg.type = attr_string(node, "type", cfg.type);
        cfg.nav_ratio = attr_double(node, "nav_ratio", cfg.nav_ratio);
        SIM_DEBUG("Config: guidance type={}, N={}", cfg.type, cfg.nav_ratio);
        return cfg;
    }

    static AutopilotConfig parse_autopilot(const pugi::xml_node& node) {
        AutopilotConfig cfg;
        cfg.type = attr_string(node, "type", cfg.type);
        cfg.Kp = child_double(node, "Kp", cfg.Kp);
        cfg.Kd = child_double(node, "Kd", cfg.Kd);
        cfg.Kd_roll = child_double(node, "Kd_roll", cfg.Kd_roll);
        cfg.max_deflection = child_double(node, "max_deflection", cfg.max_deflection);
        SIM_DEBUG("Config: autopilot type={}, Kp={}, Kd={}", cfg.type, cfg.Kp, cfg.Kd);
        return cfg;
    }

    static ActuatorConfig parse_actuator(const pugi::xml_node& node) {
        ActuatorConfig cfg;
        cfg.type = attr_string(node, "type", cfg.type);
        cfg.time_constant = attr_double(node, "time_constant", cfg.time_constant);
        cfg.max_deflection = attr_double(node, "max_deflection", cfg.max_deflection);
        cfg.max_rate = attr_double(node, "max_rate", cfg.max_rate);
        SIM_DEBUG("Config: actuator type={}, τ={}s", cfg.type, cfg.time_constant);
        return cfg;
    }

    static VehicleConfig parse_vehicle(const pugi::xml_node& node) {
        VehicleConfig cfg;
        if (!node) return cfg;

        cfg.ref_area = attr_double(node, "ref_area", cfg.ref_area);
        cfg.ref_length = attr_double(node, "ref_length", cfg.ref_length);

        cfg.atmosphere = parse_atmosphere(node.child("atmosphere"));
        cfg.gravity = parse_gravity(node.child("gravity"));
        cfg.propulsion = parse_propulsion(node.child("propulsion"));
        cfg.aerodynamics = parse_aerodynamics(node.child("aerodynamics"));

        // GNC models are optional - only parse if present in XML
        if (auto n = node.child("seeker"))    cfg.seeker = parse_seeker(n);
        if (auto n = node.child("guidance"))  cfg.guidance = parse_guidance(n);
        if (auto n = node.child("autopilot")) cfg.autopilot = parse_autopilot(n);
        if (auto n = node.child("actuator"))  cfg.actuator = parse_actuator(n);

        SIM_DEBUG("Config: vehicle S_ref={} m^2, d_ref={} m, GNC={}",
              cfg.ref_area, cfg.ref_length,
              (cfg.seeker.has_value() ? "enabled" : "disabled"));
        return cfg;
    }

    static TargetConfig parse_target(const pugi::xml_node& node) {
        TargetConfig cfg;
        if (!node) return cfg;

        cfg.type = attr_string(node, "type", cfg.type);

        auto pos_node = node.child("position");
        if (pos_node) {
            cfg.position = read_vec3(pos_node, "north", "east", "down", cfg.position);
        }

        auto vel_node = node.child("velocity");
        if (vel_node) {
            cfg.velocity = read_vec3(vel_node, "north", "east", "down", cfg.velocity);
        }

        cfg.maneuver_g = child_double(node, "maneuver_g", cfg.maneuver_g);
        cfg.maneuver_start = child_double(node, "maneuver_start", cfg.maneuver_start);

        SIM_DEBUG("Config: target type={}, pos=[{},{},{}]",
                cfg.type, cfg.position.x(), cfg.position.y(), cfg.position.z());
        return cfg;
    }

    // Public API
    SimConfig ConfigParser::load(const std::string& filepath) {
        pugi::xml_document doc;
        pugi::xml_parse_result result = doc.load_file(filepath.c_str());

        if (!result) {
            std::string msg = "Failed to parse XML: " + std::string(result.description())
                            + " at offset " + std::to_string(result.offset);
            SIM_ERROR("{}", msg);
            throw std::runtime_error(msg);
        }

        auto root = doc.child("simulation");
        if (!root) {
            throw std::runtime_error("XML missing <simulation> root element");
        }

        SIM_INFO("Loading configuration from {}", filepath);

        SimConfig cfg;
        cfg.integrator = parse_integrator(root.child("integrator"));
        cfg.recorder = parse_recorder(root.child("recorder"));
        cfg.stop = parse_stop(root.child("stop_conditions"));
        cfg.initial_conditions = parse_initial_conditions(root.child("initial_conditions"));
        cfg.vehicle = parse_vehicle(root.child("vehicle"));

        // Parse all target elements
        for (auto target_node : root.children("target")) {
            cfg.targets.push_back(parse_target(target_node));
        }

        SIM_INFO("Configuration loaded: {} target(s) defined", cfg.targets.size());
        return cfg;
    }

    void ConfigParser::save_template(const std::string& filepath) {
        pugi::xml_document doc;

        // XML declaration
        auto decl = doc.prepend_child(pugi::node_declaration);
        decl.append_attribute("version") = "1.0";
        decl.append_attribute("encoding") = "UTF-8";

        auto root = doc.append_child("simulation");

        // Integrator
        auto integ = root.append_child("integrator");
        integ.append_attribute("type") = "rk4";
        integ.append_attribute("dt") = 0.001;

        // Recorder
        auto rec = root.append_child("recorder");
        rec.append_attribute("filepath") = "sim_output.h5";
        rec.append_attribute("interval") = 0.01;

        // Initial conditions
        auto ic = root.append_child("initial_conditions");
        auto pos = ic.append_child("position");
        pos.append_attribute("north") = 0.0;
        pos.append_attribute("east") = 0.0;
        pos.append_attribute("down") = 0.0;
        ic.append_child("speed").text().set(30.0);
        ic.append_child("pitch_deg").text().set(45.0);
        ic.append_child("yaw_deg").text().set(0.0);
        ic.append_child("roll_deg").text().set(0.0);

        // Vehicle
        auto veh = root.append_child("vehicle");
        veh.append_attribute("ref_area") = 0.05;
        veh.append_attribute("ref_length") = 0.2;

        // Atmosphere
        auto atm = veh.append_child("atmosphere");
        atm.append_attribute("type") = "us_standard_1976";

        // Gravity
        auto grav = veh.append_child("gravity");
        grav.append_attribute("type") = "constant";
        grav.append_attribute("g") = 9.80665;

        // Propulsion
        auto prop = veh.append_child("propulsion");
        prop.append_attribute("type") = "solid_rocket";
        prop.append_child("thrust").text().set(5000.0);
        prop.append_child("burn_time").text().set(3.0);
        prop.append_child("total_mass").text().set(100.0);
        prop.append_child("prop_mass").text().set(30.0);

        auto inertia = prop.append_child("inertia");
        inertia.append_attribute("Ixx") = 0.5;
        inertia.append_attribute("Iyy") = 10.0;
        inertia.append_attribute("Izz") = 10.0;
        inertia.append_attribute("Ixy") = 0.0;
        inertia.append_attribute("Ixz") = 0.0;
        inertia.append_attribute("Iyz") = 0.0;

        auto cg = prop.append_child("cg");
        cg.append_attribute("x") = 1.0;
        cg.append_attribute("y") = 0.0;
        cg.append_attribute("z") = 0.0;

        // Aerodynamics
        auto aero = veh.append_child("aerodynamics");
        aero.append_attribute("type") = "simple";
        aero.append_child("CA").text().set(0.3);
        aero.append_child("CN_alpha").text().set(10.0);
        aero.append_child("CY_beta").text().set(-10.0);
        aero.append_child("Cm_alpha").text().set(-3.0);
        aero.append_child("Cn_beta").text().set(3.0);
        aero.append_child("Cl_delta").text().set(-0.5);
        aero.append_child("Cmq").text().set(-20.0);
        aero.append_child("Cnr").text().set(-20.0);
        aero.append_child("Clp").text().set(-5.0);

        // Stop conditions
        auto stop = root.append_child("stop_conditions");
        stop.append_child("max_time").text().set(300.0);
        stop.append_child("min_altitude").text().set(0.0);
        stop.append_child("poca_range_gate").text().set(200.0);
        
        // Example target (commented guidance for future use)
        auto tgt = root.append_child(pugi::node_comment);
        tgt.set_value(" Target definitions (uncomment and modify as needed)\n"
                    "  <target type=\"stationary\">\n"
                    "    <position north=\"5000\" east=\"0\" down=\"-500\" />\n"
                    "  </target>\n"
                    "  <target type=\"constant_velocity\">\n"
                    "    <position north=\"10000\" east=\"1000\" down=\"-3000\" />\n"
                    "    <velocity north=\"-200\" east=\"0\" down=\"0\" />\n"
                    "  </target>\n"
                    "  ");

        doc.save_file(filepath.c_str(), "  ");
        SIM_INFO("Template configuration written to {}", filepath);
    }



} // namespace sim::core