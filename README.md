This project is a **polymorphic, dependency-injected component architecture** for a 6-DOF flight simulation.

**Abstract interfaces define contracts** — each subsystem (atmosphere, aerodynamics, propulsion, guidance, etc.) is defined as an abstract base class with pure virtual methods. This decouples the simulation loop from any specific implementation.

**Concrete models are injected via smart pointers** — `std::unique_ptr` provides clear ownership semantics. The Vehicle doesn't know or care what's behind the pointer, only that it satisfies the interface. Models can be swapped at the call site without touching the simulation internals.

**The Builder pattern handles construction** — assembles a complex object (Vehicle with 10 subsystem models) step-by-step with a fluent interface, using move semantics to transfer ownership.

**Templates with concepts enforce type safety at compile time** — the math library and integrator use C++20 concepts to constrain template parameters, catching type errors before runtime.

**The integrator is generic over state representation** — RK4 works with any types that satisfy the `Integrable` concept, meaning you could change the state formulation (say, quaternion-based instead of Euler) without modifying the integrator.

In short, this project is a modular simulation framework where subsystem models are defined by abstract interfaces, owned by smart pointers, and injected at configuration time. Therefore, you can swap physics models, test components in isolation, and extend the system without modifying existing code.