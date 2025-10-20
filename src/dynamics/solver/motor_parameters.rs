use crate::utils::SimdRealCopy;

#[derive(Copy, Clone, PartialEq, Debug)]
pub struct MotorParameters<N: SimdRealCopy> {
    pub erp_inv_dt: N,
    pub cfm_coeff: N,
    pub cfm_gain: N,
    pub target_pos: N,
    pub target_vel: N,
    pub max_impulse: N,
}

impl<N: SimdRealCopy> Default for MotorParameters<N> {
    fn default() -> Self {
        Self {
            erp_inv_dt: N::zero(),
            cfm_coeff: N::zero(),
            cfm_gain: N::zero(),
            target_pos: N::zero(),
            target_vel: N::zero(),
            max_impulse: N::zero(),
        }
    }
}
