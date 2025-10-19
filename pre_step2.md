# Step 2 Planning — Per-Body Solve Equations & Warm Starting

## Objectives
- Translate the per-body AVBD update (right-hand side, Hessian, pose increment) into implementation-ready notes aligned with Rapier’s rigid-body state layout.
- Capture warm-start parameter roles (α, β, γ, k_start, k_max) and recommended defaults to steer solver configuration discussions.
- Outline workspace requirements for caching per-constraint stiffness/dual scalars and per-body effective mass data to inform later pipeline integration.

## Task Checklist
- [x] Review roadmap references (AVBD paper digest, GPT-5 discussion digest) for per-body solve equations and parameter schedules.
- [x] Draft documentation covering body state variables, force/Hessian assembly, solve/update steps, and quaternion integration specifics.
- [x] Document warm-start scaling rules and recommended default values, tying them to stability rationale.
- [x] Enumerate data that must persist across frames (e.g., λ, k caches) and per-iteration scratch buffers for future implementation planning.

## Progress Log
- Initialized Step 2 plan with goals centered on per-body solves, warm-start behavior, and workspace implications.
- Reviewed AVBD paper and GPT-5 discussion digests for per-body solve structure and warm-start schedule guidance.
- Authored `docs/avbd_per_body_solve.md` covering per-body assembly, solve, and warm-start scheduling with roadmap citations.
- Confirmed checklist completion; ready to summarize Step 2 outcomes in `post_step2.md`.
