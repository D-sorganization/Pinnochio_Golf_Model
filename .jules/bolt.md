## 2025-12-13 - [Cached Property Bottleneck in Physics Loop]
**Learning:** In hot loops like physics simulation steps, accessing calculated properties on dataclasses (e.g., `segment.inertia_about_proximal_joint` which performs math) repeatedly causes significant overhead. Even simple dot-access accumulation adds up.
**Action:** Cache invariant physical properties into local instance attributes or simpler variables during initialization to avoid re-computation and deep attribute lookups in the loop.
