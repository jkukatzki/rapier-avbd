# AVBD Mathematical Foundation Brief

This brief distills the Augmented Vertex Block Descent (AVBD) formulation so that upcoming modeling and implementation tasks can reference a shared mathematical baseline. It follows the roadmap guidance to focus on the augmented Lagrangian dual step, stiffness escalation strategy, and stability claims that distinguish AVBD from classical Vertex Block Descent (VBD) and Projected Gauss-Seidel (PGS) solvers. 【F:llm_digests/avbd_paper_pdf_digest.txt†L11-L104】

## 1. Baseline VBD Problem Statement
AVBD inherits VBD’s objective of minimizing the implicit Euler variational energy per time step:

\[
\min_{x_{t+\Delta t}} \; \frac{1}{2\Delta t^2}\lVert x - y \rVert_M^2 + E(x),
\]

where \(y\) are inertial predictions assembled from the previous step’s positions and velocities, \(M\) is the block-diagonal mass matrix, and \(E(x)\) accumulates potential energies from all forces. 【F:llm_digests/avbd_paper_pdf_digest.txt†L205-L334】

Local Gauss–Seidel updates operate on one body/vertex at a time by solving the 6×6 (rigid-body) linear system

\[
H_i \Delta x_i = f_i,
\]

with force and Hessian contributions assembled from all incident forces/constraints. 【F:llm_digests/avbd_paper_pdf_digest.txt†L248-L327】 Direct inversion or LDLᵀ factorization applies because the system is small per vertex. 【F:llm_digests/avbd_paper_pdf_digest.txt†L316-L333】

## 2. Augmented Lagrangian Extension for Hard Constraints
Hard constraints introduce augmented energies per iteration \(n\):

\[
E_j^{(n)}(x) = \tfrac{1}{2} k_j^{(n)} C_j(x)^2 + \lambda_j^{(n)} C_j(x),
\]

leading to constraint forces

\[
f_{ij}^{(n)} = -\bigl(k_j^{(n)} C_j(x) + \lambda_j^{(n)}\bigr) \frac{\partial C_j}{\partial x_i}.
\]

Stiffness and dual variables initialize with \(k_j^{(0)} = k_{\mathrm{start}}\) and \(\lambda_j^{(0)} = 0\), then update after each primal sweep via

\[
\lambda_j^{(n+1)} = k_j^{(n)} C_j(x) + \lambda_j^{(n)}, \qquad k_j^{(n+1)} = k_j^{(n)} + \beta \lvert C_j(x) \rvert.
\]

The progressive stiffness ramp grows the dual support force without requiring infinite stiffness, preserving stability while enforcing the hard constraint. 【F:llm_digests/avbd_paper_pdf_digest.txt†L401-L519】

## 3. Inequality Bounds and Friction
AVBD clamps the augmented multiplier \(\lambda_j^+ = k_j^{(n)} C_j(x) + \lambda_j^{(n)}\) to bounds \([\lambda_j^{\min}, \lambda_j^{\max}]\) so forces respect unilateral contact and joint limits. 【F:llm_digests/avbd_paper_pdf_digest.txt†L524-L679】 The friction model enforces \(\lVert \boldsymbol{\lambda}_{tb}^+ \rVert \le \mu \lambda_n^+\) and switches between static/dynamic coefficients based on the previous frame’s tangential impulse. 【F:llm_digests/avbd_paper_pdf_digest.txt†L682-L754】

## 4. Stiffness Escalation for Finite-Ratio Forces
To avoid biasing early iterations toward the stiffest forces, finite-stiffness energies reuse the ramp from Equation (12) but clamp to each force’s physical stiffness \(k_j^*\):

\[
k_j^{(n+1)} = \min\bigl(k_j^*,\; k_j^{(n)} + \beta \lvert C_j(x) \rvert\bigr).
\]

This staged escalation allows softer forces to propagate their corrections before the full stiffness ratio is restored, dramatically improving convergence for coupled systems. 【F:llm_digests/avbd_paper_pdf_digest.txt†L757-L867】

## 5. Stable Hessian Approximation
The augmented formulation yields Hessians

\[
H_{ij} = k_j^{(n)} \frac{\partial C_j}{\partial x_i}\frac{\partial C_j}{\partial x_i}^T + G_{ij},
\]

where the second term may be indefinite. AVBD diagonally approximates \(G_{ij}\) using column norms to guarantee a symmetric positive definite matrix, enabling robust LDLᵀ solves and consistent progress each iteration. 【F:llm_digests/avbd_paper_pdf_digest.txt†L869-L915】

## 6. Error Moderation and Warm Starting
To prevent explosive corrections when hard constraints start a frame with error, AVBD regularizes constraint evaluations with

\[
C_j(x) = C_j^*(x) - \alpha C_j^*(x_t),
\]

so only a fraction \(1-\alpha\) of the prior error is corrected per step. 【F:llm_digests/avbd_paper_pdf_digest.txt†L917-L973】 Warm starting scales previous frame multipliers and stiffnesses before reuse:

\[
k_j^{(0)} = \max(\gamma k_j^t, k_{\mathrm{start}}), \qquad \lambda_j^{(0)} = \alpha \gamma \lambda_j^t,
\]

with recommended \(\alpha = 0.95\) and \(\gamma = 0.99\) to retain convergence speed without injecting energy. 【F:llm_digests/avbd_paper_pdf_digest.txt†L974-L1024】

## 7. Iteration Structure and Parallelism Hooks
Algorithm 1 highlights the per-color processing loop: collision detection and vertex coloring precede repeated sweeps that (1) assemble local right-hand sides, (2) solve each vertex block in parallel within a color, and (3) update dual variables and stiffness scalars. The workflow naturally supports deterministic color-bucket parallelism planned later in the roadmap. 【F:llm_digests/avbd_paper_pdf_digest.txt†L1030-L1099】

## 8. Key Takeaways for Implementation
- **Hybrid primal–dual updates:** Augmented Lagrangian terms allow hard constraints without infinite stiffness, while keeping the primal VBD solve structure. 【F:llm_digests/avbd_paper_pdf_digest.txt†L401-L519】
- **Adaptive stiffness schedules:** Progressive ramps for both infinite and finite stiffness forces resolve high-ratio convergence issues central to rigid-body stacking. 【F:llm_digests/avbd_paper_pdf_digest.txt†L757-L867】
- **Stability safeguards:** SPD Hessian approximations, error damping, and scaled warm starts preserve VBD’s unconditional stability claims under AVBD’s harder constraints. 【F:llm_digests/avbd_paper_pdf_digest.txt†L869-L1024】
- **Parallel-ready iteration loop:** The color-by-vertex structure remains intact, providing clear hooks for staged workspace and deterministic bucket execution in later phases. 【F:llm_digests/avbd_paper_pdf_digest.txt†L1030-L1099】

These notes should serve as the reference baseline for modeling body-level equations, solver workspace staging, and feature gating work in upcoming roadmap steps.
