# Step 3 Planning — Constraint Pattern Catalog

## Objectives
- Capture constraint energy variants and solver cues from the Houdini AVBD reference to inform 3D-ready formulations.
- Extract reusable constraint structures from the AVBD 2D demo and map them onto Rapier concepts (contacts, joints, motors).
- Outline documentation deliverables that translate the findings into implementation guidance for future solver work.

## Task Checklist
- [x] Review the roadmap-referenced Houdini repository digest passages for the catalog of energies, stiffness handling, and 6-DOF notes.
- [x] Mine the AVBD 2D demo digest for the Force hierarchy, constraint-specific data requirements, and solver parameter usage.
- [x] Draft a documentation artifact summarizing constraint types, required state, and 3D adaptation notes aligned with Rapier modules.

## Progress Log
- Initialized Step 3 plan targeting constraint catalogs drawn from Houdini and the 2D demo to steer upcoming solver implementation work.
- Parsed the Houdini reference digest to list energy variants, adaptive stiffness behavior, and rigid 6×6 solve expectations for 3D AVBD.
- Reviewed the AVBD 2D demo digest to document the Force hierarchy, constraint data layout, and solver parameters needed for warm-started blocks.
- Authored `docs/avbd_constraint_catalog.md` capturing cross-referenced constraint patterns and Rapier integration cues for future implementation steps.
