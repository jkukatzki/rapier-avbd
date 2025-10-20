# AVBD Constraint Catalog & Implementation Cues

## 1. Houdini Reference Highlights
- The Houdini AVBD prototype enumerates constraint energies already supported in VBD—mass-spring, St. Venant–Kirchhoff, neo-Hookean, spring, joint, and motor—implying the augmented solver must accommodate each energy class without losing VBD compatibility.【F:llm_digests/avbd_houdini_impl_git_repo_digest.txt†L179-L198】
- Constraint stiffness is stored on dual elements (prims) and updated adaptively; the implementation merges dual solves once all incident vertices have been processed to avoid redundant passes, while keeping the SPD approximation available for all constraints except unstable neo-Hookean cases.【F:llm_digests/avbd_houdini_impl_git_repo_digest.txt†L193-L203】
- The authors confirm any VBD energy can be ported so long as the Hessian is decomposed into an SPD component plus a diagonal lumped correction, reinforcing that our Rust implementation needs explicit handling for non-SPD terms during block solves.【F:llm_digests/avbd_houdini_impl_git_repo_digest.txt†L279-L282】
- 3D rigid support relies on solving a 6×6 system per vertex: translational and angular Hessian blocks are inverted via LDLᵀ, with the angular increment interpreted as a rotation vector before being applied to a quaternion orientation update.【F:llm_digests/avbd_houdini_impl_git_repo_digest.txt†L395-L417】
- Houdini’s comparison with Vellum highlights that point-based coloring drives parallel work distribution; we must preserve deterministic color buckets even as we introduce new constraint builders.【F:llm_digests/avbd_houdini_impl_git_repo_digest.txt†L171-L176】

## 2. AVBD 2D Demo Building Blocks
### 2.1 Force Hierarchy and Persistent Data
- Every constraint derives from a `Force` base storing shared buffers for Jacobians, Hessians, constraint values, bounds, stiffness scalars, fracture limits, penalty ramps, and warm-start multipliers (`lambda`), mirroring the persistent caches scoped in Step 2.【F:llm_digests/avbd_2d_demo_digest.txt†L2397-L2444】
- Global compile-time constants define maximum constraint rows, penalty ranges, collision margins, and friction stick thresholds—useful cues for sizing pooled workspaces in Rust.【F:llm_digests/avbd_2d_demo_digest.txt†L2385-L2439】

### 2.2 Constraint-Specific Patterns
- **Joints** expose three rows capturing positional anchors plus torque arm data, with initialization routines filling cached rest configurations for stabilization and fracture logic.【F:llm_digests/avbd_2d_demo_digest.txt†L2454-L2469】
- **Springs** compute constraint values and derivatives from world-space anchor offsets, populating mixed linear-angular Hessian terms that mirror the 3×3→6×6 expansion discussed for 3D.【F:llm_digests/avbd_2d_demo_digest.txt†L2472-L2667】
- **Motors** add one-row constraints that drive angular speed within torque limits, reusing the same warm-start and stiffness storage as other forces.【F:llm_digests/avbd_2d_demo_digest.txt†L2502-L2514】
- **Contact manifolds** provide up to two frictional rows, embedding feature pairs for warm-start matching, cached Jacobians for normal/tangent directions, and stick flags to toggle static friction treatment.【F:llm_digests/avbd_2d_demo_digest.txt†L2517-L2556】

### 2.3 Solver Control Parameters
- The solver core tracks timestep, gravity, iteration count, and the α/β/γ parameters, matching the augmented energy schedule documented in prior steps and demonstrating where Rapier’s integration parameters must thread through the AVBD backend.【F:llm_digests/avbd_2d_demo_digest.txt†L2563-L2585】

## 3. Rapier Integration Considerations
- Rapier already divides dynamics into contact and joint constraint modules (`dynamics::solver::contact_constraint` and `dynamics::solver::joint_constraint`), which provides natural entry points for AVBD-specific builders per constraint type cataloged above.【F:llm_digests/rapier_digest.txt†L96-L122】
- The graph-coloring insights from Houdini emphasize operating on colored vertices; Rapier’s existing island solver and parallel constraint infrastructure must stage AVBD blocks per color bucket while respecting the adaptive dual updates noted earlier.【F:llm_digests/avbd_houdini_impl_git_repo_digest.txt†L171-L203】【F:llm_digests/rapier_digest.txt†L96-L112】
- Implementing 6×6 solves for rigid bodies demands constraint work buffers that can house coupled linear-angular Hessians and dual scalars; the 2D demo’s Force arrays and Houdini’s quaternion update guidance illustrate the data layout we need to replicate in pooled arenas.【F:llm_digests/avbd_houdini_impl_git_repo_digest.txt†L395-L417】【F:llm_digests/avbd_2d_demo_digest.txt†L2397-L2667】

These notes complete the Step 3 requirement to assemble constraint energy patterns and implementation cues, setting the stage for 3D-ready AVBD constraint builders within Rapier.
