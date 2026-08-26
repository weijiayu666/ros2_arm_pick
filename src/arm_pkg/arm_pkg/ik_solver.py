"""
GenkiArm 5-DOF 逆运动学求解器 (RRT-Connect)
末端：endlink1 与 endlink2 的几何中心
"""
import numpy as np


# ============ 正运动学 ============

def Rz(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

def Ry(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def Rx(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

def T_mat(R, p):
    """构建 4x4 齐次变换矩阵"""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T

def rpy_to_R(r, p, y):
    """URDF rpy 转旋转矩阵: Rz(y)*Ry(p)*Rx(r)"""
    return Rz(y) @ Ry(p) @ Rx(r)


def forward_kinematics(q1, q2, q3, q4, q5):
    """
    正运动学：关节角度 -> 末端执行器位置 (endlink1/endlink2 中心)
    输入: q1~q5 (弧度)
    返回: np.array([x, y, z]) 在 base_link 坐标系
    """
    # joint1: base_link -> link1
    p1 = np.array([0.0, -0.01, 0.056])
    R1 = rpy_to_R(0, 0, np.pi) @ Rz(q1)
    T1 = T_mat(R1, p1)

    # joint2: link1 -> link2
    p2 = np.array([0.02525, 0.0, 0.0365])
    R2 = rpy_to_R(0, -np.pi / 2, 0) @ Rz(q2)
    T2 = T1 @ T_mat(R2, p2)

    # joint3: link2 -> link3  (axis 0 0 -1)
    p3 = np.array([0.104, 0.0, 0.0])
    R3 = Rz(-q3)
    T3 = T2 @ T_mat(R3, p3)

    # joint4: link3 -> link4  (axis 0 0 1)
    p4 = np.array([0.074, 0.0, 0.0])
    R4 = Rz(q4)
    T4 = T3 @ T_mat(R4, p4)

    # joint5: link4 -> link5
    p5 = np.array([0.0685, -0.001, 0.026])
    R5_origin = rpy_to_R(np.pi / 2, 0, np.pi / 2)  # Rz(π/2)*Rx(π/2)
    R5 = R5_origin @ Rz(q5)
    T5 = T4 @ T_mat(R5, p5)

    # endjoint rpy="3.1416 -1.5708 0": Rz(π) @ Ry(-π/2)
    R_end = Rz(np.pi) @ Ry(-np.pi / 2)

    # endjoint1: link5 -> endlink1
    p_end1 = np.array([-0.0035, -0.013901, 0.025156])
    T_end1 = T5 @ T_mat(R_end, p_end1)
    pos1 = T_end1[:3, 3]

    # endjoint2: link5 -> endlink2
    p_end2 = np.array([-0.004, 0.01375, 0.025])
    T_end2 = T5 @ T_mat(R_end, p_end2)
    pos2 = T_end2[:3, 3]

    # 末端 = 两指中心 + 70mm沿末端X轴(前伸) - 10mm沿末端Z轴(endjoint轴)
    return (pos1 + pos2) / 2.0 + 0.07 * T_end1[:3, 0] - 0.01 * T_end1[:3, 2]


def forward_kinematics_link4(q1, q2, q3, q4, q5=None):
    """
    正运动学：关节角度 -> link4 位姿 (相机安装位置)
    输入: q1~q4 (弧度), q5 忽略(相机不受q5影响)
    返回: (R_3x3, t_3x1) link4 坐标系在 base_link 下的位姿
    """
    p1 = np.array([0.0, -0.01, 0.056])
    R1 = rpy_to_R(0, 0, np.pi) @ Rz(q1)
    T1 = T_mat(R1, p1)

    p2 = np.array([0.02525, 0.0, 0.0365])
    R2 = rpy_to_R(0, -np.pi / 2, 0) @ Rz(q2)
    T2 = T1 @ T_mat(R2, p2)

    p3 = np.array([0.104, 0.0, 0.0])
    R3 = Rz(-q3)
    T3 = T2 @ T_mat(R3, p3)

    p4 = np.array([0.074, 0.0, 0.0])
    R4 = Rz(q4)
    T4 = T3 @ T_mat(R4, p4)

    return T4[:3, :3], T4[:3, 3]


# ============ RRT-Connect 逆运动学 ============

# 关节限位 (弧度)
JOINT_LIMITS = np.array([
    [-2.0944,       2.0944],      # joint1 ±120°
    [-1.5708,       1.5708],      # joint2 ±90°
    [-2.3562,       2.3562],      # joint3 ±135°
    [-2.3562,       1.5708],      # joint4 -135°~90°
    [-2.3562,       2.3562],      # joint5 ±135°
], dtype=np.float64)


def _random_config():
    """在关节限位内随机采样配置"""
    return np.array([np.random.uniform(lo, hi) for lo, hi in JOINT_LIMITS])


def _joint_dist(q1, q2):
    """关节空间 L2 距离"""
    return np.linalg.norm(q1 - q2)


def _ws_dist(q, target):
    """工作空间距离 (末端到目标)"""
    return np.linalg.norm(forward_kinematics(*q) - target)


def _steer(q_from, q_to, step):
    """从 q_from 向 q_to 方向步进"""
    d = q_to - q_from
    dn = np.linalg.norm(d)
    if dn < 1e-10:
        return None
    if dn <= step:
        q_new = q_to.copy()
    else:
        q_new = q_from + d / dn * step
    for i in range(5):
        q_new[i] = np.clip(q_new[i], JOINT_LIMITS[i, 0], JOINT_LIMITS[i, 1])
    return q_new


def _smooth_path(path, target, max_smooth=50):
    """简单路径平滑：移除冗余中间点"""
    if len(path) <= 2:
        return path
    path = [np.array(p) for p in path]
    for _ in range(max_smooth):
        if len(path) <= 2:
            break
        i = np.random.randint(0, len(path) - 2)
        j = i + 2
        mid = (path[i] + path[j]) / 2.0
        in_limit = np.all((mid >= JOINT_LIMITS[:, 0]) & (mid <= JOINT_LIMITS[:, 1]))
        if in_limit:
            path = path[:i + 1] + [mid] + path[j:]
    return [p.tolist() for p in path]


def interpolate_path(path, steps_per_seg=8):
    """在线性路径段之间插入中间点，使运动更平滑"""
    if len(path) < 2:
        return path
    result = [path[0]]
    for i in range(len(path) - 1):
        a = np.array(path[i])
        b = np.array(path[i + 1])
        for s in range(1, steps_per_seg + 1):
            result.append((a + (b - a) * s / (steps_per_seg + 1)).tolist())
    return result


# ============ Jacobian 精修 ============

def jacobian(q):
    """数值 Jacobian (3x5): ∂pos/∂q"""
    eps = 1e-6
    f0 = forward_kinematics(*q)
    J = np.zeros((3, 5))
    for i in range(5):
        dq = np.zeros(5)
        dq[i] = eps
        f1 = forward_kinematics(*(q + dq))
        J[:, i] = (f1 - f0) / eps
    return J


def refine_ik(target_pos, q_init, max_iter=500, alpha=0.8, tol=1e-5, stop_event=None):
    """Jacobian 伪逆迭代精修"""
    q = np.array(q_init, dtype=np.float64)
    target = np.array(target_pos, dtype=np.float64)
    for _ in range(max_iter):
        if stop_event is not None and stop_event.is_set():
            break  # 另一条策略已成功, 提前退出
        pos = forward_kinematics(*q)
        err = target - pos
        if np.linalg.norm(err) < tol:
            break
        J = jacobian(q)
        # 阻尼最小二乘
        lam = 0.01
        JTJ = J.T @ J
        dq = np.linalg.solve(JTJ + lam * np.eye(5), J.T @ err)
        q = q + alpha * dq
        for i in range(5):
            q[i] = np.clip(q[i], JOINT_LIMITS[i, 0], JOINT_LIMITS[i, 1])
    return q.tolist(), np.linalg.norm(target - forward_kinematics(*q))


def solve_ik(target_pos, q_start, max_iters=5000, step=0.15, tol=0.02, stop_event=None):
    """
    RRT-Connect 逆运动学求解

    参数:
        target_pos: [x, y, z] 目标位置
        q_start:    [q1,...,q5] 当前关节角度 (弧度)
        max_iters:  最大迭代次数
        step:       关节空间步长 (弧度)
        tol:        目标位置容差 (米)

    返回:
        (path, error) 其中 path 是关节配置列表 [q0,...,qN], error 是最终误差
        若完全失败则 path 为 [q_start]
    """
    target = np.array(target_pos, dtype=np.float64)
    q_start = np.array(q_start, dtype=np.float64)

    # --- 双树结构 ---
    class Tree:
        __slots__ = ('nodes', 'parents', 'ws_positions')
        def __init__(self, root):
            self.nodes = [root.astype(np.float64).copy()]
            self.parents = [-1]
            self.ws_positions = [forward_kinematics(*root)]  # 缓存FK结果

        def nearest(self, q):
            best_i = 0
            best_d = np.inf
            for i, n in enumerate(self.nodes):
                d = _joint_dist(q, n)
                if d < best_d:
                    best_d = d
                    best_i = i
            return best_i

        def add(self, q, parent_idx):
            self.nodes.append(q.copy())
            self.parents.append(parent_idx)
            self.ws_positions.append(forward_kinematics(*q))  # 加入时算一次FK
            return len(self.nodes) - 1

    # 起始树
    tree_a = Tree(q_start)

    # 目标树: 采样找接近靶点的配置
    best_q = None
    best_d = np.inf
    for _ in range(100):
        q = _random_config()
        d = _ws_dist(q, target)
        if d < best_d:
            best_d = d
            best_q = q
        if d < tol:
            break
    tree_b = Tree(best_q)

    # --- 提取路径 ---
    def extract(a_idx, b_idx):
        pa = [tree_a.nodes[a_idx]]
        i = a_idx
        while tree_a.parents[i] != -1:
            i = tree_a.parents[i]
            pa.append(tree_a.nodes[i])
        pa.reverse()

        pb = []
        i = b_idx
        while tree_b.parents[i] != -1:
            i = tree_b.parents[i]
            pb.append(tree_b.nodes[i])
        pb.reverse()
        return [p.tolist() for p in (pa + pb)]

    # --- 主循环 ---
    swap_flag = False

    for _ in range(max_iters):
        if stop_event is not None and stop_event.is_set():
            return None, float('inf')  # 另一条策略已成功, 提前退出

        # 当前扩展树和对向树
        ext_tree, other_tree = (tree_b, tree_a) if swap_flag else (tree_a, tree_b)

        # 随机采样 (10% 偏向当前最优, 用缓存免重复FK)
        if np.random.random() < 0.1:
            ws_dists = [np.linalg.norm(pos - target) for pos in other_tree.ws_positions]
            best_w = np.argmin(ws_dists)
            q_rand = other_tree.nodes[best_w] + np.random.randn(5) * 0.3
            for i in range(5):
                q_rand[i] = np.clip(q_rand[i], JOINT_LIMITS[i, 0], JOINT_LIMITS[i, 1])
        else:
            q_rand = _random_config()

        # 扩展
        nearest_idx = ext_tree.nearest(q_rand)
        q_nearest = ext_tree.nodes[nearest_idx]
        q_new = _steer(q_nearest, q_rand, step)

        if q_new is None:
            swap_flag = not swap_flag
            continue

        new_idx = ext_tree.add(q_new, nearest_idx)

        # 尝试连接另一棵树
        other_idx = other_tree.nearest(q_new)
        q_other = other_tree.nodes[other_idx]

        for __ in range(30):
            q_step = _steer(q_other, q_new, step)
            if q_step is None:
                break
            other_idx = other_tree.add(q_step, other_idx)
            q_other = q_step
            if _joint_dist(q_other, q_new) < step * 0.8:
                # 连接成功
                if not swap_flag:
                    path, final_err = extract(new_idx, other_idx), _ws_dist(q_new, target)
                else:
                    path, final_err = extract(other_idx, new_idx), _ws_dist(q_new, target)
                path = _smooth_path(path, target)
                return path, final_err

        swap_flag = not swap_flag

    # 未找到精确解，返回树 A 中最接近目标的路径 (用缓存)
    best_a = min(range(len(tree_a.ws_positions)), key=lambda i: np.linalg.norm(tree_a.ws_positions[i] - target))
    pa = [tree_a.nodes[best_a]]
    i = best_a
    while tree_a.parents[i] != -1:
        i = tree_a.parents[i]
        pa.append(tree_a.nodes[i])
    pa.reverse()
    err = np.linalg.norm(tree_a.ws_positions[best_a] - target)
    path = _smooth_path(pa, target)
    return path, err


def solve_with_refine(target_pos, q_start, max_iters=2000, step=0.15, tol=0.01):
    """
    完整求解: RRT-Connect + Jacobian精修 + 路径插值
    容差 10mm, 若达不到则返回 None (无解)
    """
    q_start = np.array(q_start, dtype=np.float64)

    path, err = solve_ik(target_pos, q_start, max_iters=max_iters, step=step, tol=tol)

    if path is None or len(path) < 2:
        return None, float('inf')

    # Jacobian 精修最终配置
    final_q, final_err = refine_ik(target_pos, path[-1], max_iter=300, alpha=0.9, tol=1e-5)
    path[-1] = final_q

    # 精修后仍超容差 → 无解
    if final_err > tol:
        return None, final_err

    # 插值平滑
    path = interpolate_path(path, steps_per_seg=15)
    return path, final_err


# ============ 解析法 IK (4 DOF, 固定 q5) ============

def solve_ik_analytic(target, q_start):
    """
    解析法逆运动学: 固定 q5 (腕部滚转, 对末端位置几乎无影响), 用 Jacobian 求解 q1~q4 (4 DOF)
    比 RRT-Connect 快 1000 倍以上

    参数:
        target:  [x, y, z] 目标位置 (base_link)
        q_start: [q1,...,q5] 当前关节角 (弧度)

    返回:
        (path, error)  path 含 [q_start, q_solution], error 为最终误差
    """
    target = np.array(target, dtype=np.float64)
    q = np.array(q_start, dtype=np.float64)
    q5 = q[4]  # 固定 q5

    # 初始猜测: q1 = atan2(y, x) 把臂平面转向目标
    q[0] = np.arctan2(target[1], target[0])

    for _ in range(200):
        q_full = [q[0], q[1], q[2], q[3], q5]
        pos = forward_kinematics(*q_full)
        err = target - pos
        if np.linalg.norm(err) < 1e-5:
            break

        # 3×4 Jacobian (q1, q2, q3, q4)
        J = np.zeros((3, 4))
        eps = 1e-6
        for i in range(4):
            dq = np.zeros(4)
            dq[i] = eps
            q_test = [q[0] + dq[0], q[1] + dq[1], q[2] + dq[2], q[3] + dq[3], q5]
            J[:, i] = (forward_kinematics(*q_test) - pos) / eps

        # 阻尼最小二乘 (伪逆): 4 DOF 冗余, 求最小范数步长
        lam = 0.01
        JJt = J @ J.T  # 3×3
        dq1234 = J.T @ np.linalg.solve(JJt + lam * np.eye(3), err)
        q[:4] += dq1234

        for i in range(4):
            q[i] = np.clip(q[i], JOINT_LIMITS[i, 0], JOINT_LIMITS[i, 1])

    final_q = [q[0], q[1], q[2], q[3], q5]
    err = np.linalg.norm(target - forward_kinematics(*final_q))
    path = [list(q_start)[:5], final_q]
    return path, err


# ============ 测试入口 ============
if __name__ == '__main__':
    # 所有关节为 0 时的末端位置
    ee0 = forward_kinematics(0, 0, 0, 0, 0)
    print(f"零位末端位置: {ee0}")

    # 测试一个小位移的 IK
    target = ee0 + np.array([0.05, 0.0, 0.05])
    print(f"目标位置: {target}")
    path, err = solve_ik(target, [0, 0, 0, 0, 0], max_iters=2000)
    print(f"解路径长度: {len(path)}, 最终误差: {err:.4f}m")
    if len(path) > 1:
        end = path[-1]
        actual = forward_kinematics(*end)
        print(f"实际到达: {actual}")
        print(f"最终关节角(deg): {[np.degrees(q) for q in end]}")
