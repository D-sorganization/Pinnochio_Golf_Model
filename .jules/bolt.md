## 2025-12-13 - [Cached Property Bottleneck in Physics Loop]
**Learning:** In hot loops like physics simulation steps, accessing calculated properties on dataclasses (e.g., `segment.inertia_about_proximal_joint` which performs math) repeatedly causes significant overhead. Even simple dot-access accumulation adds up.
**Action:** Cache invariant physical properties into local instance attributes or simpler variables during initialization to avoid re-computation and deep attribute lookups in the loop.

## 2025-12-14 - [Expression Re-compilation in UI Loop]
**Learning:** The `DoublePendulumDynamics` UI implementation (`pendulum_pyqt_app.py`) re-compiles user expressions via `compile_forcing_functions` on every simulation step when in forward mode. This involves `ast.parse` and `compile`, which are extremely expensive operations to perform at 100Hz+.
**Action:** In future UI optimizations, cache the compiled `ExpressionFunction` objects and only re-compile when the user input string actually changes.
