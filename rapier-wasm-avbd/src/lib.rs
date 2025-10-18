use rapier3d::{counters::Counters, prelude::*};
use wasm_bindgen::prelude::*;

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
    // The build script always enables the solver_avbd feature for rapier3d.
    true
}

#[wasm_bindgen]
pub struct StepMetrics {
    total_ms: f64,
    collision_ms: f64,
    island_ms: f64,
    solver_ms: f64,
    solver_assembly_ms: f64,
    solver_resolution_ms: f64,
    solver_writeback_ms: f64,
}

impl StepMetrics {
    fn from_counters(counters: &Counters) -> Self {
        Self {
            total_ms: counters.step_time_ms(),
            collision_ms: counters.collision_detection_time_ms(),
            island_ms: counters.island_construction_time_ms(),
            solver_ms: counters.solver_time_ms(),
            solver_assembly_ms: counters.solver.velocity_assembly_time.time_ms(),
            solver_resolution_ms: counters.solver.velocity_resolution_time.time_ms(),
            solver_writeback_ms: counters.solver.velocity_writeback_time.time_ms(),
        }
    }
}

#[wasm_bindgen]
impl StepMetrics {
    pub fn total_ms(&self) -> f64 {
        self.total_ms
    }

    pub fn collision_ms(&self) -> f64 {
        self.collision_ms
    }

    pub fn island_ms(&self) -> f64 {
        self.island_ms
    }

    pub fn solver_ms(&self) -> f64 {
        self.solver_ms
    }

    pub fn solver_assembly_ms(&self) -> f64 {
        self.solver_assembly_ms
    }

    pub fn solver_resolution_ms(&self) -> f64 {
        self.solver_resolution_ms
    }

    pub fn solver_writeback_ms(&self) -> f64 {
        self.solver_writeback_ms
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
}

#[wasm_bindgen]
impl RapierWorld {
    #[wasm_bindgen(constructor)]
    pub fn new(gravity_x: f32, gravity_y: f32, gravity_z: f32, use_avbd: bool) -> RapierWorld {
        let mut integration_parameters = IntegrationParameters::default();

        if use_avbd {
            integration_parameters.solver_backend = SolverBackend::Avbd;
        }

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
            gravity: Vector::new(gravity_x, gravity_y, gravity_z),
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
        self.step();
        StepMetrics::from_counters(&self.physics_pipeline.counters)
    }

    pub fn reset(&mut self) {
        self.islands = IslandManager::new();
        self.broad_phase = DefaultBroadPhase::new();
        self.narrow_phase = NarrowPhase::new();
        self.impulse_joint_set = ImpulseJointSet::new();
        self.multibody_joint_set = MultibodyJointSet::new();
        self.ccd_solver = CCDSolver::new();
        self.rigid_body_set = RigidBodySet::new();
        self.collider_set = ColliderSet::new();
        self.physics_pipeline.counters.reset();
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

    pub fn solver_iterations(&self) -> u32 {
        self.integration_parameters.num_solver_iterations as u32
    }

    pub fn set_solver_iterations(&mut self, iterations: u32) {
        self.integration_parameters.num_solver_iterations = iterations.max(1) as usize;
    }

    pub fn warmstart_coefficient(&self) -> f32 {
        self.integration_parameters.warmstart_coefficient as f32
    }

    pub fn set_warmstart_coefficient(&mut self, value: f32) {
        let clamped = value.clamp(0.0, 1.0);
        self.integration_parameters.warmstart_coefficient = clamped as Real;
    }

    pub fn set_gravity_scale(&mut self, scale: f32) {
        let gravity_scale = scale as Real;

        for handle in self.rigid_body_set.iter_mut() {
            handle.1.set_gravity_scale(gravity_scale, false);
        }
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
        let params = &mut self.integration_parameters.avbd_solver_params;
        params.iterations = iterations.max(1) as usize;
        params.alpha = alpha.clamp(0.0, 1.0) as Real;
        params.beta = beta.max(1.0) as Real;
        params.gamma = gamma.clamp(0.0, 1.0) as Real;

        let min_val = stiffness_min.max(0.0) as Real;
        let max_val = stiffness_max.max(min_val + Real::EPSILON) as Real;
        params.stiffness_min = min_val;
        params.stiffness_max = max_val;
        params.regularization = regularization.max(0.0) as Real;
    }

    pub fn set_body_gravity_scale(&mut self, body_handle: u32, scale: f32) {
        let handle = RigidBodyHandle::from_raw_parts(body_handle, 0);
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            if body.body_type() == RigidBodyType::Dynamic {
                body.set_gravity_scale(scale as Real, true);
            }
        }
    }

    pub fn get_avbd_params(&self) -> Vec<f32> {
        let params = &self.integration_parameters.avbd_solver_params;
        vec![
            params.iterations as f32,
            params.alpha as f32,
            params.beta as f32,
            params.gamma as f32,
            params.stiffness_min as f32,
            params.stiffness_max as f32,
            params.regularization as f32,
        ]
    }

    pub fn create_dynamic_body(&mut self, x: f32, y: f32, z: f32) -> u32 {
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(Vector::new(x, y, z))
            .build();
        let handle = self.rigid_body_set.insert(rigid_body);
        handle.into_raw_parts().0
    }

    pub fn create_fixed_body(&mut self, x: f32, y: f32, z: f32) -> u32 {
        let rigid_body = RigidBodyBuilder::fixed()
            .translation(Vector::new(x, y, z))
            .build();
        let handle = self.rigid_body_set.insert(rigid_body);
        handle.into_raw_parts().0
    }

    pub fn create_kinematic_body(&mut self, x: f32, y: f32, z: f32) -> u32 {
        let rigid_body = RigidBodyBuilder::kinematic_position_based()
            .translation(Vector::new(x, y, z))
            .build();
        let handle = self.rigid_body_set.insert(rigid_body);
        handle.into_raw_parts().0
    }

    pub fn remove_body(&mut self, body_handle: u32) {
        let handle = RigidBodyHandle::from_raw_parts(body_handle, 0);
        // Call remove with references to all required fields
        // This works because we're only borrowing &mut self once at the WASM boundary
        let _ = self.rigid_body_set.remove(
            handle,
            &mut self.islands,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            true,
        );
    }

    pub fn set_body_translation(&mut self, body_handle: u32, x: f32, y: f32, z: f32) {
        let handle = RigidBodyHandle::from_raw_parts(body_handle, 0);
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.set_translation(Vector::new(x, y, z), true);
        }
    }

    pub fn create_ball_collider(&mut self, body_handle: u32, radius: f32) -> u32 {
        let handle = RigidBodyHandle::from_raw_parts(body_handle, 0);
        let collider = ColliderBuilder::ball(radius).build();
        let collider_handle =
            self.collider_set
                .insert_with_parent(collider, handle, &mut self.rigid_body_set);
        collider_handle.into_raw_parts().0
    }

    pub fn create_cuboid_collider(
        &mut self,
        body_handle: u32,
        hx: f32,
        hy: f32,
        hz: f32,
    ) -> u32 {
        let handle = RigidBodyHandle::from_raw_parts(body_handle, 0);
        let collider = ColliderBuilder::cuboid(hx, hy, hz).build();
        let collider_handle =
            self.collider_set
                .insert_with_parent(collider, handle, &mut self.rigid_body_set);
        collider_handle.into_raw_parts().0
    }

    pub fn get_body_translation(&self, body_handle: u32) -> Vec<f32> {
        let handle = RigidBodyHandle::from_raw_parts(body_handle, 0);
        if let Some(body) = self.rigid_body_set.get(handle) {
            let translation = body.translation();
            vec![translation.x, translation.y, translation.z]
        } else {
            vec![0.0, 0.0, 0.0]
        }
    }

pub fn get_body_rotation(&self, body_handle: u32) -> Vec<f32> {
    let handle = RigidBodyHandle::from_raw_parts(body_handle, 0);

    if let Some(body) = self.rigid_body_set.get(handle) {
        let quat = body.rotation().quaternion();
        vec![quat.i, quat.j, quat.k, quat.w]
    } else {
        vec![0.0, 0.0, 0.0, 1.0]
    }
}
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn avbd_params_are_applied_and_clamped() {
        let mut world = RapierWorld::new(0.0, -9.81, 0.0, true);

        // Intentionally pass a zero iteration count to verify clamping to at least 1.
        world.set_avbd_params(0, 0.42, 7.5, 0.85, 3.0, 30.0, 1.0e-5);

        let params = world.get_avbd_params();
        eprintln!("avbd params after set_avbd_params: {:?}", params);

        assert_eq!(params.len(), 7, "expected full parameter vector");
        assert!(
            (params[0] - 1.0).abs() < 1.0e-6,
            "iterations should clamp to >= 1, got {}",
            params[0]
        );
        assert!((params[1] - 0.42).abs() < 1.0e-6, "alpha not applied correctly");
        assert!((params[2] - 7.5).abs() < 1.0e-6, "beta not applied correctly");
        assert!((params[3] - 0.85).abs() < 1.0e-6, "gamma not applied correctly");
        assert!((params[4] - 3.0).abs() < 1.0e-6, "stiffness min not applied correctly");
        assert!((params[5] - 30.0).abs() < 1.0e-6, "stiffness max not applied correctly");
        assert!(
            (params[6] - 1.0e-5).abs() < 1.0e-9,
            "regularization not applied correctly"
        );
    }
}
