from optimize_with_do_mpc import PathpOptimizer_MPC
import numpy as np

# Cubic Hermite Spline 경로 생성
goal_state = [10.0, 5.0, np.deg2rad(45)]  # 목표 상태 (x, y, yaw)

# Path Optimizer 객체 초기화
optimizer = PathpOptimizer_MPC()

# MPC 실행 및 최적화된 제어 입력 계산
optimal_control_input = optimizer.MPC(goal_state)

print("Final optimal control input:", optimal_control_input)


# import numpy as np
# import do_mpc
# import matplotlib.pyplot as plt

# def template_model():
#     model = do_mpc.model.Model("continuous")  # 연속 시간 모델
    
#     # 상태 변수 정의
#     x = model.set_variable(var_type="_x", var_name="x")  # 위치
#     v = model.set_variable(var_type="_x", var_name="v")  # 속도
    
#     # 제어 입력 정의
#     u = model.set_variable(var_type="_u", var_name="u")  # 가속도
    
#     # 시스템 방정식 정의
#     model.set_rhs("x", v)       # dx/dt = v
#     model.set_rhs("v", u)       # dv/dt = u
    
#     model.setup()
#     return model

# def template_mpc(model, target_position):
#     mpc = do_mpc.controller.MPC(model)

#     setup_mpc = {
#         "n_horizon": 20,
#         "t_step": 0.1,
#         "state_discretization": "collocation",  # 콜로케이션 사용
#         "collocation_type": "radau",              # 필수 매개변수
#         "collocation_deg": 3,                     # 필수 매개변수
#         "store_full_solution": True,
#         "nlpsol_opts": {
#             "ipopt.print_level": 0,  # 모든 출력 비활성화
#             "ipopt.sb": "yes",        # 라이선스 배너 숨김
#             "print_time": 0,          # 시간 출력 비활성화
#         }
#     }
#     mpc.set_param(**setup_mpc)

#     # 비용 함수 정의
#     mterm = (model.x["x"] - target_position)**2  # 종결 비용
#     lterm = (model.x["x"] - target_position)**2 + model.u["u"]**2  # 단계별 비용
    
#     mpc.set_objective(mterm=mterm, lterm=lterm)
    
#     # 제어 입력 변화에 대한 가중치
#     mpc.set_rterm(u=1e-2)
    
#     # 제약 조건 설정
#     mpc.bounds["lower", "_u", "u"] = -5.0  # 최소 가속도
#     mpc.bounds["upper", "_u", "u"] = 5.0   # 최대 가속도
    
#     mpc.setup()
#     return mpc

# def main():
#     target_position = 10.0  # 목표 위치
    
#     model = template_model()
#     mpc = template_mpc(model, target_position)
    
#     # 시뮬레이터 설정
#     simulator = do_mpc.simulator.Simulator(model)
#     simulator.set_param(t_step=0.1)
#     simulator.setup()
    
#     # 그래픽 설정
#     graphics = do_mpc.graphics.Graphics(mpc.data)
#     fig, ax = plt.subplots(2, sharex=True)
    
#     # 플롯 설정
#     graphics.add_line(var_type='_x', var_name='x', axis=ax[0], label='Position', color='green')
#     graphics.add_line(var_type='_x', var_name='v', axis=ax[0], label='Velocity', color='black')
#     graphics.add_line(var_type='_u', var_name='u', axis=ax[1], label='Control')
    
#     ax[0].set_ylabel('Position / Velocity')
#     ax[1].set_ylabel('Control Input')
#     ax[1].set_xlabel('Time [s]')
    
#     # 초기 상태 설정
#     x0 = np.array([0.0, 0.0])  # 초기 위치 x=0, 속도 v=0
#     mpc.x0 = x0
#     simulator.x0 = x0
#     mpc.set_initial_guess()

#     for _ in range(1000):  # 시뮬레이션 반복 실행
#         u0 = mpc.make_step(x0)
#         x0 = simulator.make_step(u0)  # 시뮬레이터로 상태 업데이트
        
#         # 그래픽 업데이트
#         graphics.plot_results()
#         graphics.plot_predictions()
#         graphics.reset_axes()
#         plt.pause(0.1)  # 실시간 업데이트
        
#         print(f"Step {_}: Position {x0[0].item():.2f}, Velocity {x0[1].item():.2f}, Control {u0.item():.2f}")
        
#         if abs(x0[0] - target_position) < 1e-2:  # 목표 도달 시 종료
#             break

#     plt.show()

# if __name__ == "__main__":
#     main()
