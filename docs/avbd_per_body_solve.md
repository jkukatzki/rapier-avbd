# AVBD Per-Body Solve & Warm-Start Notes

This note packages the Step 2 research outcomes: the rigid-body update equations used by Augmented Vertex Block Descent (AVBD), the warm-start schedule parameters (α, β, γ, k_start, k_max), and the data that must persist across frames to support an implementation inside Rapier. It extends the mathematical foundation captured in `avbd_math_brief.md` with implementation-ready guidance.

## 1. Body State & Notation
- Each rigid body/vertex stores the configuration \(x_i = [p_i, q_i]\) with translation \(p_i\) and unit quaternion \(q_i\).【F:llm_digests/gpt5_discussion_digest.txt†L32-L67】
- Rapier already provides block-diagonal mass \(M_i\) (3×3 linear, 3×3 rotational) that feeds the AVBD local solve.【F:llm_digests/gpt5_discussion_digest.txt†L47-L67】【F:llm_digests/avbd_paper_pdf_digest.txt†L205-L314】

## 2. Right-Hand Side Assembly
- The inertial prediction \(y_i\) is formed from the previous pose, velocity, and external acceleration before each sweep.【F:llm_digests/gpt5_discussion_digest.txt†L60-L66】
- Each iteration assembles a force vector
  \[
  f_i = -\frac{1}{\Delta t^2} M_i (x_i - y_i) + \sum_{j \in F_i} f_{ij},
  \]
  with \(f_{ij} = -\partial E_j / \partial x_i\) from every incident constraint/force element.【F:llm_digests/avbd_paper_pdf_digest.txt†L275-L288】
- Inequality constraints retain clamping from the augmented energy (normal limits, friction cone) before contributing to \(f_{ij}\).【F:llm_digests/avbd_paper_pdf_digest.txt†L524-L754】

## 3. Hessian Assembly & Conditioning
- The 6×6 system matrix per body is
  \[
  H_i = \frac{1}{\Delta t^2} M_i + \sum_{j \in F_i} H_{ij}, \quad H_{ij} = \frac{\partial^2 E_j}{\partial x_i^2}.
  \]
  【F:llm_digests/avbd_paper_pdf_digest.txt†L289-L304】
- To guarantee a symmetric positive definite solve, replace the non-SPD term \(G_{ij}\) with the diagonal norm approximation \(\tilde{G}_{ij}\), enabling robust LDLᵀ factorization.【F:llm_digests/avbd_paper_pdf_digest.txt†L884-L910】

## 4. Local Solve & Pose Update
- Solve \(H_i \Delta x_i = f_i\) via LDLᵀ (or direct inverse for debugging) because the block size is fixed and small.【F:llm_digests/avbd_paper_pdf_digest.txt†L275-L320】
- Update the body translation and orientation with
  \(p_i \leftarrow p_i + \Delta p_i\) and
  \(q_i \leftarrow \text{normalize}(q_i + 0.5\,(0, \Delta \omega_i) \otimes q_i)\), matching Rapier’s quaternion integration.【F:llm_digests/gpt5_discussion_digest.txt†L63-L67】
- Maintain deterministic ordering within each color bucket; outer roadmap work will later parallelize colors without changing local math.【F:llm_digests/avbd_paper_pdf_digest.txt†L1030-L1039】

## 5. Warm-Start Schedule & Parameters
- Dual and stiffness updates per sweep follow
  \(\lambda_j^{(n+1)} = \lambda_j^{(n)} + k_j^{(n)} C_j(x)\) and
  \(k_j^{(n+1)} = \min(\beta\, k_j^{(n)}, k_{\max})\).【F:llm_digests/gpt5_discussion_digest.txt†L70-L82】
- Frame-to-frame warm start scales prior values before initialization:
  \[
  k_j^{(0)} = \max(\gamma k_j^t, k_{\mathrm{start}}), \quad \lambda_j^{(0)} = \alpha \gamma \lambda_j^t.
  \]
  Recommended defaults are \(\alpha = 0.95\), \(\beta = 10\), \(\gamma = 0.99\).【F:llm_digests/gpt5_discussion_digest.txt†L22-L82】【F:llm_digests/avbd_paper_pdf_digest.txt†L974-L1018】
- Scaling \(\lambda\) by \(\alpha\) prevents residual error correction energy from re-entering the system and destabilizing stacks.【F:llm_digests/avbd_paper_pdf_digest.txt†L1005-L1020】

## 6. Persistent Caches & Workspace Requirements
- Per constraint, persist \(k_j\), \(\lambda_j\), friction history, and geometric data to support both warm-start and inequality clamping in subsequent frames.【F:llm_digests/gpt5_discussion_digest.txt†L38-L82】【F:llm_digests/avbd_paper_pdf_digest.txt†L974-L1039】
- Per body, cache mass properties (already available) plus temporary accumulators for \(f_i\), \(H_i\), and the solved \(\Delta x_i\); these can live in per-color work buffers for parallel sweeps.【F:llm_digests/gpt5_discussion_digest.txt†L57-L118】【F:llm_digests/avbd_paper_pdf_digest.txt†L205-L320】
- Across frames, store warm-start scalars alongside Rapier’s existing contact manifold caches to minimize allocation churn and enable deterministic reuse.【F:llm_digests/gpt5_discussion_digest.txt†L39-L107】【F:llm_digests/avbd_paper_pdf_digest.txt†L1030-L1039】

These notes will seed the implementation sketch in upcoming steps, inform feature-flag wiring, and guide discussions about memory layout and pooling for the AVBD solver backend.
