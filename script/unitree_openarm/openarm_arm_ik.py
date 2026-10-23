# IPOPT problem copied from unitreerobotics/xr_teleoperate
# teleop/robot_control/robot_arm_ik.py (G1_29_ArmIK):
#   50 * ||position||^2 + 1 * ||log3(R)||^2 + 0.02 * ||q||^2 + 0.1 * ||q - q_last||^2
#   joint limits, warm start, max_iter 30, then WeightedMovingFilter [0.4, 0.3, 0.2, 0.1].
# The model is OpenArm's 7 arm joints. The other arm and the fingers are locked.
# Feedforward torque is not returned; gravity stays on the existing CAN path.

import numpy as np
import casadi
import pinocchio as pin
from pinocchio import casadi as cpin

try:
    from weighted_moving_filter import WeightedMovingFilter
except ImportError:  # imported as a package
    from .weighted_moving_filter import WeightedMovingFilter


class OpenArmArmIK:
    def __init__(self, urdf_path, side):
        if side not in ("left", "right"):
            raise ValueError(f"side must be left or right, got {side}")
        self.side = side
        model = pin.buildModelFromUrdf(urdf_path)
        other = "right" if side == "left" else "left"
        lock_names = [
            f"openarm_{other}_joint{index}" for index in range(1, 8)
        ] + [
            f"openarm_{arm}_finger_joint{index}"
            for arm in ("left", "right")
            for index in (1, 2)
        ]
        lock_ids = []
        for name in lock_names:
            if model.existJointName(name):
                lock_ids.append(model.getJointId(name))
        reference = pin.neutral(model)
        self.model = pin.buildReducedModel(model, lock_ids, reference)
        self.data = self.model.createData()
        frame = f"openarm_{side}_hand_tcp"
        if not self.model.existFrame(frame):
            raise RuntimeError(f"URDF has no frame {frame}")
        self.frame_id = self.model.getFrameId(frame)
        if self.model.nq != 7:
            raise RuntimeError(f"{side} reduced model nq={self.model.nq}, expected 7")

        self.cmodel = cpin.Model(self.model)
        self.cdata = self.cmodel.createData()
        self.cq = casadi.SX.sym("q", self.model.nq, 1)
        self.cTf = casadi.SX.sym("tf", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)
        self.translational_error = casadi.Function(
            "translational_error",
            [self.cq, self.cTf],
            [self.cdata.oMf[self.frame_id].translation - self.cTf[:3, 3]],
        )
        self.rotational_error = casadi.Function(
            "rotational_error",
            [self.cq, self.cTf],
            [
                cpin.log3(
                    self.cdata.oMf[self.frame_id].rotation @ self.cTf[:3, :3].T
                )
            ],
        )

        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.model.nq)
        self.var_q_last = self.opti.parameter(self.model.nq)
        self.param_tf = self.opti.parameter(4, 4)
        translational_cost = casadi.sumsqr(
            self.translational_error(self.var_q, self.param_tf)
        )
        rotation_cost = casadi.sumsqr(self.rotational_error(self.var_q, self.param_tf))
        regularization_cost = casadi.sumsqr(self.var_q)
        smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)
        self.opti.subject_to(
            self.opti.bounded(
                self.model.lowerPositionLimit, self.var_q, self.model.upperPositionLimit
            )
        )
        self.opti.minimize(
            50 * translational_cost + rotation_cost + 0.02 * regularization_cost + 0.1 * smooth_cost
        )
        opts = {
            "expand": True,
            "detect_simple_bounds": True,
            "calc_lam_p": False,
            "print_time": False,
            "ipopt.sb": "yes",
            "ipopt.print_level": 0,
            "ipopt.max_iter": 30,
            "ipopt.tol": 1e-4,
            "ipopt.acceptable_tol": 5e-4,
            "ipopt.acceptable_iter": 5,
            "ipopt.warm_start_init_point": "yes",
            "ipopt.derivative_test": "none",
            "ipopt.jacobian_approximation": "exact",
        }
        self.opti.solver("ipopt", opts)
        self.init_data = np.zeros(self.model.nq)
        self.smooth_filter = WeightedMovingFilter(np.array([0.4, 0.3, 0.2, 0.1]), 7)

    def solve(self, wrist, current_q=None):
        if current_q is not None:
            self.init_data = np.asarray(current_q, dtype=float).reshape(7)
        self.opti.set_initial(self.var_q, self.init_data)
        self.opti.set_value(self.param_tf, np.asarray(wrist, dtype=float))
        self.opti.set_value(self.var_q_last, self.init_data)
        try:
            self.opti.solve()
            sol_q = np.array(self.opti.value(self.var_q), dtype=float).reshape(7)
        except Exception:
            sol_q = np.array(self.opti.debug.value(self.var_q), dtype=float).reshape(7)
            if not np.all(np.isfinite(sol_q)):
                return None
        self.smooth_filter.add_data(sol_q)
        sol_q = np.array(self.smooth_filter.filtered_data, dtype=float).reshape(7)
        self.init_data = sol_q
        return sol_q
