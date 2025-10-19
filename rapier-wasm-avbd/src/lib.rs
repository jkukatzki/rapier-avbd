use wasm_bindgen::prelude::*;
use rapier3d::prelude::*;
use std::time::Instant;

#[wasm_bindgen]
pub fn init_panic_hook() {
    console_error_panic_hook::set_once();
}

#[wasm_bindgen]
pub fn version() -> String {
    env!("CARGO_PKG_VERSION").to_string()
}

#[wasm_bindgen]
pub fn is_avbd_available() -> bool {
    true // Always true since solver_avbd is enabled in Cargo.toml
}

#[wasm_bindgen]
pub struct StepMetrics {
    solver_ms: f32,
    total_ms: f32,
    iterations: u32,
}

#[wasm_bindgen]
impl StepMetrics {
    pub fn solver_ms(&self) -> f32 {
        self.solver_ms
    }

    pub fn total_ms(&self) -> f32 {
        self.total_ms
    }

    pub fn iterations(&self) -> u32 {
        self.iterations
    }
}

#[wasm_bindgen]
pub struct RapierWorld {
    physics_pipeline: PhysicsPipeline,
    integration_parameters: IntegrationParameters,
    islands: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    gravity: Vector<Real>,
    base_gravity: Vector<Real>,
    gravity_scale: f32,
}

#[wasm_bindgen]
impl RapierWorld {
    #[wasm_bindgen(constructor)]
    pub fn new(gravity_x: f32, gravity_y: f32, gravity_z: f32, use_avbd: bool) -> RapierWorld {
        let mut integration_parameters = IntegrationParameters::default();
        
        if use_avbd {
            integration_parameters.solver_backend = SolverBackend::Avbd;
        }
        
        let base_gravity = Vector::new(gravity_x, gravity_y, gravity_z);
        RapierWorld {
            physics_pipeline: PhysicsPipeline::new(),
            integration_parameters,
            islands: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            gravity: base_gravity,
            base_gravity,
            gravity_scale: 1.0,
        }
    }

    pub fn step(&mut self) {
        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.islands,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            &(),
            &(),
        );
    }

    pub fn step_with_metrics(&mut self) -> StepMetrics {
        let start = Instant::now();
        self.step();
        let total_elapsed = start.elapsed().as_secs_f64() * 1_000.0;

        let mut solver_ms = 0.0f64;
        let mut iterations = 0u32;

        if self.integration_parameters.solver_backend == SolverBackend::Avbd {
            for report in self.physics_pipeline.take_avbd_reports() {
                solver_ms += report.solve_time.as_secs_f64() * 1_000.0;
                iterations = iterations.max(report.iteration_count as u32);
            }
        }

        StepMetrics {
            solver_ms: solver_ms as f32,
            total_ms: total_elapsed as f32,
            iterations,
        }
    }
    
    pub fn num_bodies(&self) -> usize {
        self.rigid_body_set.len()
    }
    
    pub fn num_colliders(&self) -> usize {
        self.collider_set.len()
    }
    
    pub fn get_solver_backend(&self) -> String {
        format!("{:?}", self.integration_parameters.solver_backend)
    }

    pub fn set_gravity_scale(&mut self, scale: f32) {
        self.gravity_scale = scale;
        let scaled = self.base_gravity * (scale as Real);
        self.gravity = scaled;
    }

    pub fn set_avbd_params(
        &mut self,
        iterations: u32,
        alpha: f32,
        beta: f32,
        gamma: f32,
        stiffness_min: f32,
        stiffness_max: f32,
        regularization: f32,
    ) {
        #[cfg(feature = "solver_avbd")]
        {
            self.integration_parameters.avbd_params = AvbdSolverParams {
                iterations: iterations as usize,
                alpha: alpha as Real,
                beta: beta as Real,
                gamma: gamma as Real,
                warmstart: true,
                allow_parallelism: false,
                stiffness_min: stiffness_min as Real,
                stiffness_max: stiffness_max as Real,
            };
            // Regularization maps to warmstart coefficient for now by clamping the stiffness range.
            let regularized = regularization.max(0.0) as Real;
            self.integration_parameters.avbd_params.stiffness_min =
                self.integration_parameters.avbd_params.stiffness_min.max(regularized);
        }
    }

    pub fn clear_dynamic_bodies(&mut self) {
        let handles: Vec<_> = self
            .rigid_body_set
            .iter()
            .filter_map(|(handle, body)| {
                if body.body_type() == RigidBodyType::Dynamic {
                    Some(handle)
                } else {
                    None
                }
            })
            .collect();

        for handle in handles {
            self.rigid_body_set.remove(
                handle,
                &mut self.islands,
                &mut self.collider_set,
                &mut self.impulse_joint_set,
                &mut self.multibody_joint_set,
                true,
            );
        }
    }
    
    /// Create a dynamic rigid body and return its handle
    pub fn create_dynamic_body(&mut self, x: f32, y: f32, z: f32) -> u32 {
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(Vector::new(x, y, z))
            .build();
        let handle = self.rigid_body_set.insert(rigid_body);
        handle.into_raw_parts().0
    }
    
    /// Create a fixed (static) rigid body and return its handle
    pub fn create_fixed_body(&mut self, x: f32, y: f32, z: f32) -> u32 {
        let rigid_body = RigidBodyBuilder::fixed()
            .translation(Vector::new(x, y, z))
            .build();
        let handle = self.rigid_body_set.insert(rigid_body);
        handle.into_raw_parts().0
    }
    
    /// Create a ball collider attached to a rigid body
    pub fn create_ball_collider(&mut self, body_handle: u32, radius: f32) -> u32 {
        let handle = RigidBodyHandle::from_raw_parts(body_handle, 0);
        let collider = ColliderBuilder::ball(radius).build();
        let collider_handle = self.collider_set.insert_with_parent(
            collider,
            handle,
            &mut self.rigid_body_set,
        );
        collider_handle.into_raw_parts().0
    }
    
    /// Create a cuboid collider attached to a rigid body
    pub fn create_cuboid_collider(&mut self, body_handle: u32, hx: f32, hy: f32, hz: f32) -> u32 {
        let handle = RigidBodyHandle::from_raw_parts(body_handle, 0);
        let collider = ColliderBuilder::cuboid(hx, hy, hz).build();
        let collider_handle = self.collider_set.insert_with_parent(
            collider,
            handle,
            &mut self.rigid_body_set,
        );
        collider_handle.into_raw_parts().0
    }
    
    /// Get the translation (position) of a rigid body
    pub fn get_body_translation(&self, body_handle: u32) -> Vec<f32> {
        let handle = RigidBodyHandle::from_raw_parts(body_handle, 0);
        if let Some(body) = self.rigid_body_set.get(handle) {
            let translation = body.translation();
            vec![translation.x, translation.y, translation.z]
        } else {
            vec![0.0, 0.0, 0.0]
        }
    }

    pub fn remove_body(&mut self, body_handle: u32) {
        let handle = RigidBodyHandle::from_raw_parts(body_handle, 0);
        if self.rigid_body_set.contains(handle) {
            self.rigid_body_set.remove(
                handle,
                &mut self.islands,
                &mut self.collider_set,
                &mut self.impulse_joint_set,
                &mut self.multibody_joint_set,
                true,
            );
        }
    }
}
