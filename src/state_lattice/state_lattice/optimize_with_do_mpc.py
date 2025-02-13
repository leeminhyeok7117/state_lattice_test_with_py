import do_mpc
import numpy as np
from casadi import *
from cubic_hermit_planner import hermite_with_constraints

class PathpOptimizer_MPC:
    
    def __init__(self):
        #차량의 동역학적 제약
        self.L = 1.04
        self.max_steer_angel = np.deg2rad(27)
        self.max_speed = 20 / 3.6
        self.max_minus_accel = -(20 / 3.6)
        self.max_plus_accel = 5 / 3.6
        self.max_curvature = np.tan(self.max_steer_angel) / self.L
                
    def setup_model(self):
        model = do_mpc.model.Model("continuous")
        
        #상태변수 정의: 위치(x,y), 속도(v), 헤딩값(heading)  
        self.x = model.set_variable(var_type='_x', var_name='x')
        self.y = model.set_variable(var_type='_x', var_name='y')
        self.yaw = model.set_variable(var_type='_x', var_name='yaw')
        self.v = model.set_variable(var_type='_x', var_name='v')
          
        #제어변수 정의: 가속도(a), 조향각(yaw)
        self.a = model.set_variable(var_type='_u', var_name='a')
        self.steer = model.set_variable(var_type='_u', var_name='steer')
        
        #ordinary diffrential Equation
        model.set_rhs('x', self.v * cos(self.yaw))
        model.set_rhs('y', self.v * sin(self.yaw))
        model.set_rhs('yaw', self.v * tan(self.steer) / self.L)
        model.set_rhs('v', self.a)

        model.setup()
        return model

    def configure_mpc(self, model, x_ref, y_ref, yaw_ref):
        mpc = do_mpc.controller.MPC(model)
        
        setup_mpc = {
            "n_horizon": 20,
            "t_step": 0.1,
            "state_discretization": "collocation",  
            "collocation_type": "radau",              
            "collocation_deg": 3,                     
            "store_full_solution": True,
            "nlpsol_opts": {
                "ipopt.print_level": 0,  
                "ipopt.sb": "yes",        
                "print_time": 0,         
            }
        }
        mpc.set_param(**setup_mpc)
        
        #비용함수를 계산함 마지막점만 비교해서 하려고했는데 안됨 이거 나중에 수정 ㄱㄱ
        mterm = (self.x - x_ref[-1])**2 + (self.y - y_ref[-1])**2 + (self.yaw - yaw_ref[-1])**2
        lterm = sum((self.x - x)**2 + (self.y - y)**2 + (self.yaw - yaw)**2 
            for x, y, yaw in zip(x_ref[:-1], y_ref[:-1], yaw_ref[:-1])
        )
        mpc.set_objective(lterm=lterm, mterm=mterm)
        mpc.set_rterm(a=1e-4, steer=1e-4)
        
        #최대, 최소 범위 제한
        mpc.bounds['lower', '_u', 'a'] = self.max_minus_accel     # 최대 감속
        mpc.bounds['upper', '_u', 'a'] = self.max_plus_accel     # 최대 가속
        
        mpc.bounds['lower', '_x', 'v'] = 0             # 최소 속도
        mpc.bounds['upper', '_x', 'v'] = self.max_speed     # 최대 속도

        mpc.bounds['lower', '_u', 'steer'] = -self.max_steer_angel  # 최소 조향각
        mpc.bounds['upper', '_u', 'steer'] = self.max_steer_angel  # 최대 조향각  
        
        self.max_curvature = tan(self.steer) / self.L
        
        curvature_expr = tan(self.steer) / self.L
        mpc.set_nl_cons('curvature_limit_upper', curvature_expr - self.max_curvature)
        mpc.set_nl_cons('curvature_limit_lower', -self.max_curvature - curvature_expr)
        
        mpc.setup()
        
        return mpc

    def MPC(self, state):
        x_ref, y_ref, yaw_ref, _ = hermite_with_constraints([0,0], [state[0],state[1]], 0, state[2])
        
        erp_model = self.setup_model()
        erp_controller = self.configure_mpc(erp_model, x_ref, y_ref, yaw_ref)
        
        initial_state = np.array([0.0, 0.0, 0.0, 0.0])  # 초기 위치 x=0, y=0, 속도 v=0, 조향각 yaw=0
        erp_controller.x0 = initial_state
        
        erp_controller.set_initial_guess()

        for _ in range(100):
            optimal_control = erp_controller.make_step(initial_state)
            print(f"Step {_}: x {initial_state[0].item():.2f}, y {initial_state[1].item():.2f}, yaw {initial_state[2].item():.2f}, a {optimal_control[0].item():.2f},steer {optimal_control[1].item():.2f}")
        
        
        return optimal_control

