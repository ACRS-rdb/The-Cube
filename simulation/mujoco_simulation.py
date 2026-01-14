# 導入必要的庫
import numpy as np
from scipy.spatial.transform import Rotation  # 用於旋轉變換
from scipy.linalg import solve_continuous_are  # 用於求解連續代數黎卡提方程（未使用）
from pydrake.solvers import MathematicalProgram, Solve  # Drake 的數學規劃求解器
import mujoco  # MuJoCo 物理引擎
import mujoco.viewer  # MuJoCo 可視化工具
import time

# 載入 MuJoCo 模型與數據結構
m = mujoco.MjModel.from_xml_path('The_Cube_description/The_Cube.xml')  # 模型定義
d = mujoco.MjData(m)  # 模擬狀態數據

# 設定初始姿態（直立，無旋轉）
quat = Rotation.from_euler('xyz', [0, 0, 0], degrees=True).as_quat()  # 將歐拉角轉為四元數 [x,y,z,w]
d.joint('base_link').qpos = [quat[3], quat[0], quat[1], quat[2]]  # MuJoCo 使用 [w,x,y,z] 順序


# 前向動力學計算，更新所有派生量
mujoco.mj_forward(m, d)

# 準備線性化矩陣
# 狀態維度 = 2*nv（位置誤差 + 速度），控制維度 = nu
A = np.zeros((2 * m.nv, 2 * m.nv))  # 狀態轉移矩陣
B = np.zeros((2 * m.nv, m.nu))  # 控制輸入矩陣

# 在當前狀態點進行有限差分線性化
# 得到連續時間系統 dx/dt = A*x + B*u
mujoco.mjd_transitionFD(m, d, 1e-6, True, A, B, None, None)


# 目標姿態（直立）與反應輪角度（都為零）
q_target = np.array([quat[3], quat[0], quat[1], quat[2], 0, 0, 0])

# 使用歐拉方法離散化線性系統
# x[k+1] = A_d * x[k] + B_d * u[k]
A_d = np.eye(12) + A * m.opt.timestep  # 離散狀態轉移矩陣
B_d = B * m.opt.timestep  # 離散控制輸入矩陣

# 建立模型預測控制（MPC）優化問題
prog = MathematicalProgram()
N = 20  # 預測時域長度
x = prog.NewContinuousVariables(N + 1, 12, "x")  # 狀態軌跡變數（N+1 個時間步）
u = prog.NewContinuousVariables(N, 3, "u")  # 控制輸入軌跡（N 個時間步）

# 初始條件約束（每次迭代時會更新為當前誤差狀態）
initial_condition_constraint = prog.AddLinearEqualityConstraint(
    np.eye(12), 
    np.zeros(12), 
    x[0]
)
u_limit = 1.0  # 控制輸入限制（反應輪目標速度，單位：rad/s）

# 代價函數權重矩陣
# Q: 狀態誤差權重 [姿態誤差(3), 反應輪位置(3), 角速度(3), 反應輪速度(3)]
Q = np.diag([10000000000, 10000000000, 10000000000, 0, 0, 0, 7000, 7000, 7000, 6e-6, 6e-6, 6e-6])
R = np.eye(3) * 1e-20  # 控制輸入權重

x_target = np.zeros(12)  # 目標狀態（零誤差）

# 為預測時域內的每個時間步添加約束與代價
for k in range(N):
  # 動態約束：x[k+1] = A_d * x[k] + B_d * u[k]
  prog.AddLinearEqualityConstraint(A_d @ x[k] + B_d @ u[k] - x[k+1], np.zeros(12))
  
  # 控制輸入邊界約束
  prog.AddBoundingBoxConstraint(-u_limit, u_limit, u[k])
  
  # 狀態追蹤代價：(x[k] - x_target)^T * Q * (x[k] - x_target)
  prog.AddQuadraticErrorCost(Q, x_target, x[k])
  # 控制輸入代價：u[k]^T * R * u[k]
  prog.AddQuadraticCost(u[k].T @ R @ u[k])

# 終端代價（加重懲罰最後時刻的誤差）
Q_terminal = Q * 100.0
prog.AddQuadraticErrorCost(Q_terminal, x_target, x[N])

error_integral = np.zeros(3)  # 誤差積分項（未使用）

# 啟動 MuJoCo 互動式視窗
with mujoco.viewer.launch_passive(m, d) as viewer:
  while viewer.is_running():
    # === 計算當前狀態誤差 ===
    x_error = np.zeros(12)
    pos_err = np.zeros(m.nv)  # 位置/姿態誤差（nv=6：3自由度姿態 + 3反應輪角度）
    
    # 使用 MuJoCo 的姿態微分函數計算「速度形式」的姿態誤差
    # 注意：這實際上計算的是達到目標姿態所需的速度，而非真正的姿態誤差
    mujoco.mj_differentiatePos(m, pos_err, 1.0, q_target, d.qpos)    
    
    # 組合完整狀態誤差：[位置誤差, 速度]
    x_error = np.concatenate([pos_err, d.qvel])
    print(f"Position Error: {pos_err[:3]}, Velocity: {d.qvel[:3]}")
    
    # 更新 MPC 初始條件為當前誤差狀態
    initial_condition_constraint.evaluator().set_bounds(x_error, x_error)
    
    # === 求解 MPC 優化問題 ===
    result = Solve(prog)

    # 應用第一個時間步的最優控制（Model Predictive Control 策略）
    d.ctrl = result.GetSolution(u[0])
    print(f"Control Input: {d.ctrl}")

    # 執行一步物理模擬
    mujoco.mj_step(m, d)
    
    # 同步視窗顯示（更新渲染、處理用戶輸入）
    viewer.sync()

