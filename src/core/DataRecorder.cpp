#include "core/DataRecorder.hpp"
#include "core/Logger.hpp"

#include <H5Cpp.h>
#include <sstream>
#include <iomanip>
#include <stdexcept>

namespace sim::core {

// Pimpl holding HDF5 handles

struct DataRecorder::Impl {
    std::unique_ptr<H5::H5File> file;
    std::unique_ptr<H5::Group> run_group;
    bool run_active{false};
    int current_run_id{-1};
};

// Helper: format run group name

static std::string run_group_name(int run_id) {
    std::ostringstream ss;
    ss << "run_" << std::setw(4) << std::setfill('0') << run_id;
    return ss.str();
}

// Helper: write a 1D double vector as an HDF5 dataset

static void write_dataset_1d(H5::Group& group, const std::string& name,
                             const std::vector<double>& data)
{
    if (data.empty()) return;

    hsize_t dims[1] = {data.size()};
    H5::DataSpace dataspace(1, dims);
    auto dataset = group.createDataSet(name, H5::PredType::NATIVE_DOUBLE, dataspace);
    dataset.write(data.data(), H5::PredType::NATIVE_DOUBLE);
}

// Helper: write an Nx3 array as an HDF5 dataset

static void write_dataset_nx3(H5::Group& group, const std::string& name,
                              const std::vector<std::array<double, 3>>& data)
{
    if (data.empty()) return;

    hsize_t dims[2] = {data.size(), 3};
    H5::DataSpace dataspace(2, dims);
    auto dataset = group.createDataSet(name, H5::PredType::NATIVE_DOUBLE, dataspace);
    // std::array<double,3> is contiguous, and vector of them is contiguous
    dataset.write(data.data(), H5::PredType::NATIVE_DOUBLE);
}

// Construction / destruction

DataRecorder::DataRecorder(const Config& cfg)
    : cfg_(cfg)
    , impl_(std::make_unique<Impl>())
{
    try {
        impl_->file = std::make_unique<H5::H5File>(
            cfg_.filepath, H5F_ACC_TRUNC);
        SIM_INFO("DataRecorder: created {}", cfg_.filepath);
    } catch (const H5::FileIException& e) {
        SIM_ERROR("DataRecorder: failed to create {} — {}", cfg_.filepath, e.getCDetailMsg());
        throw;
    }
}

DataRecorder::~DataRecorder() {
    if (impl_ && impl_->run_active) {
        try {
            end_run();
        } catch (...) {
            // Don't throw from destructors
        }
    }
}

DataRecorder::DataRecorder(DataRecorder&&) noexcept = default;
DataRecorder& DataRecorder::operator=(DataRecorder&&) noexcept = default;

// Run lifecycle

void DataRecorder::begin_run(int run_id) {
    if (impl_->run_active) {
        SIM_WARN("DataRecorder: ending previous run {} before starting new one",
                 impl_->current_run_id);
        end_run();
    }

    clear_buffers();
    last_record_time_ = -1e30;

    std::string name = run_group_name(run_id);
    try {
        impl_->run_group = std::make_unique<H5::Group>(
            impl_->file->createGroup(name));
        impl_->run_active = true;
        impl_->current_run_id = run_id;
        SIM_INFO("DataRecorder: started {}", name);
    } catch (const H5::GroupIException& e) {
        SIM_ERROR("DataRecorder: failed to create group {} — {}", name, e.getCDetailMsg());
        throw;
    }
}

void DataRecorder::end_run() {
    if (!impl_->run_active) {
        SIM_WARN("DataRecorder: end_run called with no active run");
        return;
    }

    flush_buffers();

    // Write record count as attribute
    write_attribute("num_records", static_cast<int>(buf_time_.size()));

    impl_->run_group.reset();
    impl_->run_active = false;

    SIM_INFO("DataRecorder: finalized run_{:04d} ({} records)",
             impl_->current_run_id, buf_time_.size());
}

// Recording

void DataRecorder::record(double t, const State& state,
                          double altitude, double alpha, double beta,
                          double mach, double speed, double qbar)
{
    if (!impl_->run_active) return;

    // Check if enough time has elapsed since last record
    if (t - last_record_time_ < cfg_.record_interval - 1e-10) return;

    buffer_state(t, state, altitude, alpha, beta, mach, speed, qbar);
    last_record_time_ = t;
}

void DataRecorder::record_now(double t, const State& state,
                              double altitude, double alpha, double beta,
                              double mach, double speed, double qbar)
{
    if (!impl_->run_active) return;

    buffer_state(t, state, altitude, alpha, beta, mach, speed, qbar);
    last_record_time_ = t;
}

// Metadata

void DataRecorder::write_attribute(const std::string& name, double value) {
    if (!impl_->run_active) return;
    H5::DataSpace scalar(H5S_SCALAR);
    auto attr = impl_->run_group->createAttribute(
        name, H5::PredType::NATIVE_DOUBLE, scalar);
    attr.write(H5::PredType::NATIVE_DOUBLE, &value);
}

void DataRecorder::write_attribute(const std::string& name, int value) {
    if (!impl_->run_active) return;
    H5::DataSpace scalar(H5S_SCALAR);
    auto attr = impl_->run_group->createAttribute(
        name, H5::PredType::NATIVE_INT, scalar);
    attr.write(H5::PredType::NATIVE_INT, &value);
}

void DataRecorder::write_attribute(const std::string& name, const std::string& value) {
    if (!impl_->run_active) return;
    H5::StrType str_type(H5::PredType::C_S1, value.size() + 1);
    H5::DataSpace scalar(H5S_SCALAR);
    auto attr = impl_->run_group->createAttribute(name, str_type, scalar);
    attr.write(str_type, value);
}

// Private helpers

void DataRecorder::buffer_state(double t, const State& state,
                                double altitude, double alpha, double beta,
                                double mach, double speed, double qbar)
{
    buf_time_.push_back(t);
    buf_position_.push_back({state.position.x(), state.position.y(), state.position.z()});
    buf_velocity_body_.push_back({state.velocity_body.x(), state.velocity_body.y(), state.velocity_body.z()});
    buf_euler_.push_back({state.euler.phi, state.euler.theta, state.euler.psi});
    buf_omega_body_.push_back({state.omega_body.x(), state.omega_body.y(), state.omega_body.z()});
    buf_mass_.push_back(state.mass);
    buf_alpha_.push_back(alpha);
    buf_beta_.push_back(beta);
    buf_speed_.push_back(speed);
    buf_mach_.push_back(mach);
    buf_altitude_.push_back(altitude);
    buf_qbar_.push_back(qbar);
}

void DataRecorder::flush_buffers() {
    if (!impl_->run_group || buf_time_.empty()) return;

    auto& g = *impl_->run_group;

    write_dataset_1d(g, "time", buf_time_);
    write_dataset_nx3(g, "position", buf_position_);
    write_dataset_nx3(g, "velocity_body", buf_velocity_body_);
    write_dataset_nx3(g, "euler", buf_euler_);
    write_dataset_nx3(g, "omega_body", buf_omega_body_);
    write_dataset_1d(g, "mass", buf_mass_);
    write_dataset_1d(g, "alpha", buf_alpha_);
    write_dataset_1d(g, "beta", buf_beta_);
    write_dataset_1d(g, "speed", buf_speed_);
    write_dataset_1d(g, "mach", buf_mach_);
    write_dataset_1d(g, "altitude", buf_altitude_);
    write_dataset_1d(g, "qbar", buf_qbar_);

    SIM_DEBUG("DataRecorder: flushed {} records to disk", buf_time_.size());
}

void DataRecorder::clear_buffers() {
    buf_time_.clear();
    buf_position_.clear();
    buf_velocity_body_.clear();
    buf_euler_.clear();
    buf_omega_body_.clear();
    buf_mass_.clear();
    buf_alpha_.clear();
    buf_beta_.clear();
    buf_speed_.clear();
    buf_mach_.clear();
    buf_altitude_.clear();
    buf_qbar_.clear();
}

} // namespace sim::core