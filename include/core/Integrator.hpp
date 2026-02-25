#pragma once

#include <functional>
#include <concepts>
#include <memory>
#include <string>
#include <stdexcept>

namespace sim::core
{
    // ============================================================
    // Concept: Integrable
    // ============================================================
    // This concept constrains the template types S (state) and D (derivative)
    // to ensure they support the arithmetic required by the RK4 algorithm.
    //
    // S = state type (State)
    // D = derivative type (StateDerivative)
    //
    // The RK integrator assumes:
    //   - You can add a derivative to a state: S + D -> S
    //   - You can scale a derivative by a scalar: double * D -> D
    //   - You can add derivatives together: D + D -> D
    //
    // If these are not valid, the code will fail at compile time.
    template<typename S, typename D>
    concept Integrable = requires(S s, D d, double dt) {
        { s + d }  -> std::convertible_to<S>;
        { dt * d } -> std::convertible_to<D>;
        { d + d }  -> std::convertible_to<D>;
    };

    /// Abstract integrator interface (swappable via smart pointer)
    template <typename S, typename D>
        requires Integrable<S,D>
    class IntegratorBase {
    public:
        // std::function: a callable that returns D that takes (double, const S&) as parameters
        using DerivativeFunc = std::function<D(double, const S&)>;
        virtual ~IntegratorBase() = default;
        virtual S step(const DerivativeFunc& f, double t, const S& state, double dt) = 0;
    };

    /// 4th-order Runge-Kutta
    template <typename S, typename D>
        requires Integrable<S,D>
    class RungeKutta4 : public IntegratorBase<S, D> {
    public:
        using DerivativeFunc = typename IntegratorBase<S, D>::DerivativeFunc;

        S step(const DerivativeFunc& f, double t, const S& state, double dt) override {
            const D k1 = f(t,            state);
            const D k2 = f(t + dt*0.5,   state + (dt*0.5) * k1);
            const D k3 = f(t + dt*0.5,   state + (dt*0.5) * k2);
            const D k4 = f(t + dt,       state + dt * k3);
            const D weighted = (1.0/6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
            return state + dt * weighted;
        }
    };

    /// Factory: returns a smart pointer to the requested integrator
    template<typename S, typename D>
        requires Integrable<S, D>
    [[nodiscard]] std::unique_ptr<IntegratorBase<S, D>> make_integrator(const std::string& method = "rk4")
    {
        if (method == "rk4") return std::make_unique<RungeKutta4<S,D>>();
        throw std::invalid_argument("Unknown integrator method: " + method);
    }
}