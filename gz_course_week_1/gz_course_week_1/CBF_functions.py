import numpy as np
import cvxpy as cp
def square_cbf(state, distance, center):
    xc, yc = center
    x_state, y_state = state[0] - xc,  state[1] - yc
    b_matrix = np.array([
        -x_state - distance,
        -y_state - distance,
        x_state - distance,
        y_state - distance
    ])
    grad_b_matrix = np.array([
        [ -1, 0],
        [ 0, -1],
        [ 1, 0],
        [ 0, 1]
    ])
    return b_matrix, grad_b_matrix
def rectangle_cbf(state, x_bounds, y_bounds):
    x_min, x_max = x_bounds
    y_min, y_max = y_bounds
    x_state, y_state = state[0], state[1]
    b_matrix = np.array([
        x_min - x_state,
        y_min - y_state,
        x_state - x_max,
        y_state - y_max,
    ])
    grad_b_matrix = np.array([
        [-1, 0],
        [0, -1],
        [1, 0],
        [0, 1]
    ])
    return b_matrix, grad_b_matrix
def rectangle_cbf_nonholonomic(pos, yaw, ell, x_bounds, y_bounds):
    x_min, x_max = x_bounds
    y_min, y_max = y_bounds
    cos_t, sin_t = np.cos(yaw), np.sin(yaw)
    p_l_x = pos[0] + ell * cos_t
    p_l_y = pos[1] + ell * sin_t
    b_matrix = np.array([
        x_min - p_l_x,
        y_min - p_l_y,
        p_l_x - x_max,
        p_l_y - y_max,
    ])
    grad_b_matrix = np.array([
        [-cos_t, ell * sin_t],
        [-sin_t, -ell * cos_t],
        [cos_t, -ell * sin_t],
        [sin_t, ell * cos_t],
    ])
    return b_matrix, grad_b_matrix
def smooth_square_cbf(state, distance, center, sharpness):
    xc, yc = center
    x, y = state[0] - xc, state[1] - yc
    k = sharpness
    exp_kx = np.exp(k * x)
    exp_neg_kx = np.exp(-k * x)
    exp_ky = np.exp(k * y)
    exp_neg_ky = np.exp(-k * y)
    log_sum_exp = exp_kx + exp_neg_kx + exp_ky + exp_neg_ky
    b_val = (1/k) * np.log(log_sum_exp) - distance
    grad_b_x = (exp_kx - exp_neg_kx) / log_sum_exp
    grad_b_y = (exp_ky - exp_neg_ky) / log_sum_exp
    grad_b = np.array([grad_b_x, grad_b_y])
    return b_val, grad_b
def square_hocbf(state, state_dot, k_b, distance, center):
    xc, yc = center
    x_state, y_state = state[0] - xc,  state[1] - yc
    x_dot_state, y_dot_state = state_dot[0], state_dot[1]
    b_pos_matrix = np.array([
        -x_state - distance,
        -y_state - distance,
        x_state - distance,
        y_state - distance
    ])
    b_vel_matrix = np.array([
        -x_dot_state + k_b * b_pos_matrix[0],
        -y_dot_state + k_b * b_pos_matrix[1],
         x_dot_state + k_b * b_pos_matrix[2],
         y_dot_state + k_b * b_pos_matrix[3]
    ])
    grad_b_matrix = np.array([
        [ -1, 0],
        [ 0, -1],
        [ 1, 0],
        [ 0, 1]
    ])
    return b_vel_matrix, grad_b_matrix
def CBFsolver(type, method, b_matrix, grad_b_matrix, small_gamma, f, u_nom, u, kcbf, delta_ub, dnn_term):
    if type == 'square_cbf':
        if method == 'QP':
            u_var = cp.Variable(u_nom.shape[0])
            u_nom_param = cp.Parameter(u_nom.shape[0])
            objective = cp.Minimize(cp.sum_squares(u_var - u_nom_param))
            B_grad_f = grad_b_matrix @ f
            A_qp = grad_b_matrix
            if small_gamma is None:
                perf_gamma = b_matrix
            else:
                perf_gamma = small_gamma
            b_qp = -B_grad_f - kcbf * perf_gamma
            constraints = [A_qp @ u_var <= b_qp]
            u_nom_param.value = u_nom
            prob = cp.Problem(objective, constraints)
            prob.solve(solver=cp.OSQP)
            u = u_var.value
            if u is None:
                u = u_nom
        if method == 'closed':
            B_grad_f = grad_b_matrix @ f
            A = grad_b_matrix
            if small_gamma is None:
                perf_gamma = b_matrix
            else:
                perf_gamma = small_gamma
            b = -B_grad_f - kcbf * perf_gamma
            closed_ans =  A @ u_nom - b
            u = u_nom.copy()
            if closed_ans[0] > 0:
                u[0] = -b[0]
            if closed_ans[1] > 0:
                u[1] = -b[1]
            if closed_ans[2] > 0:
                u[0] = b[2]
            if closed_ans[3] > 0:
                u[1] = b[3]
        return u
    if type == 'square_rcbf':
        if method == 'QP':
            u_var = cp.Variable(u_nom.shape[0])
            u_nom_param = cp.Parameter(u_nom.shape[0])
            objective = cp.Minimize(cp.sum_squares(u_var - u_nom_param))
            B_grad_f = grad_b_matrix @ f
            A_qp = grad_b_matrix
            if small_gamma is None:
                perf_gamma = b_matrix
            else:
                perf_gamma = small_gamma
            b_qp = -B_grad_f - kcbf * perf_gamma - delta_ub
            constraints = [A_qp @ u_var <= b_qp]
            u_nom_param.value = u_nom
            prob = cp.Problem(objective, constraints)
            prob.solve(solver=cp.OSQP)
            u = u_var.value
            if u is None:
                u = u_nom
        if method == 'closed':
            B_grad_f = grad_b_matrix @ f
            A = grad_b_matrix
            if small_gamma is None:
                perf_gamma = b_matrix
            else:
                perf_gamma = small_gamma
            b = -B_grad_f - kcbf * perf_gamma - delta_ub
            closed_ans =  A @ u_nom - b
            u = u_nom.copy()
            if closed_ans[0] > 0:
                u[0] = -b[0]
            if closed_ans[1] > 0:
                u[1] = -b[1]
            if closed_ans[2] > 0:
                u[0] = b[2]
            if closed_ans[3] > 0:
                u[1] = b[3]
        return u
    if type == 'square_adcbf':
        if method == 'QP':
            u_var = cp.Variable((u.shape[0],1))
            u_nom_param = cp.Parameter((u.shape[0],1))
            objective = cp.Minimize(cp.sum_squares(u_var - u_nom_param))
            B_grad_f = grad_b_matrix @ f
            A_qp = grad_b_matrix
            if small_gamma is None:
                perf_gamma = b_matrix
            else:
                perf_gamma = small_gamma
            if dnn_term is None:
                b_qp = -kcbf * perf_gamma
            else:
                b_qp = -kcbf * perf_gamma - dnn_term - B_grad_f
            constraints = [A_qp @ u_var <= b_qp]
            u_nom_param.value = u_nom
            prob = cp.Problem(objective, constraints)
            prob.solve(solver=cp.OSQP)
            u = u_var.value
            if u is None:
                u = u_nom
        if method == 'closed':
            B_grad_f = grad_b_matrix @ f
            A = grad_b_matrix
            if small_gamma is None:
                perf_gamma = b_matrix
            else:
                perf_gamma = small_gamma
            if dnn_term is None:
                print('You need to use DNN term')
                b = -kcbf * perf_gamma
            else:
                b = -kcbf * perf_gamma - dnn_term - B_grad_f
            closed_ans = (A @ u_nom) - b
            u = u_nom.copy()
            if closed_ans[0] > 0:
                u[0] = -b[0]
            if closed_ans[1] > 0:
                u[1] = -b[1]
            if closed_ans[2] > 0:
                u[0] = b[2]
            if closed_ans[3] > 0:
                u[1] = b[3]
        return u
def HOCBFsolver(type, method, high_state, b_matrix, grad_b_matrix, small_gamma, f, u_nom, u, kcbf, kb, delta_ub):
    if type == 'square_hocbf':
        if method == 'QP':
            u_var = cp.Variable(u.shape[0])
            u_nom_param = cp.Parameter(u.shape[0])
            objective = cp.Minimize(cp.sum_squares(u_var - u_nom_param))
            B_grad_f = grad_b_matrix @ f
            A_qp = grad_b_matrix
            if small_gamma is None:
                perf_gamma = b_matrix
            else:
                perf_gamma = small_gamma
            vel_term = kb * (grad_b_matrix @ high_state)
            b_qp = -B_grad_f - kcbf * perf_gamma -vel_term
            constraints = [A_qp @ u_var <= b_qp]
            u_nom_param.value = u_nom
            prob = cp.Problem(objective, constraints)
            prob.solve(solver=cp.OSQP)
            if prob.status == cp.OPTIMAL or prob.status == cp.OPTIMAL_INACCURATE:
                u = u_var.value
            else:
                print("QP failed or was infeasible. Applying zero control as a safe fallback.")
                u = np.zeros(u_nom.shape)
            return u
        if method == 'closed':
            B_grad_f = grad_b_matrix @ f
            A = grad_b_matrix
            if small_gamma is None:
                perf_gamma = b_matrix
            else:
                perf_gamma = small_gamma
            vel_term = kb * (grad_b_matrix @ high_state)
            b = -B_grad_f - kcbf * perf_gamma -vel_term
            closed_ans =  A @ u_nom - b
            u = u_nom.copy()
            if closed_ans[0] > 0:
                u[0] = -b[0]
            if closed_ans[1] > 0:
                u[1] = -b[1]
            if closed_ans[2] > 0:
                u[0] = b[2]
            if closed_ans[3] > 0:
                u[1] = b[3]
        return u
    if type == 'square_horcbf':
        if method == 'QP':
            u_var = cp.Variable(u.shape[0])
            u_nom_param = cp.Parameter(u.shape[0])
            objective = cp.Minimize(cp.sum_squares(u_var - u_nom_param))
            B_grad_f = grad_b_matrix @ f
            A_qp = grad_b_matrix
            if small_gamma is None:
                perf_gamma = b_matrix
            else:
                perf_gamma = small_gamma
            vel_term = kb * (grad_b_matrix @ high_state)
            b_qp = -B_grad_f - kcbf * perf_gamma -vel_term - 3*delta_ub
            constraints = [A_qp @ u_var <= b_qp]
            u_nom_param.value = u_nom
            prob = cp.Problem(objective, constraints)
            prob.solve(solver=cp.OSQP)
            if prob.status == cp.OPTIMAL or prob.status == cp.OPTIMAL_INACCURATE:
                u = u_var.value
            else:
                print("QP failed or was infeasible. Applying zero control as a safe fallback.")
                u = np.zeros(u_nom.shape)
            return u
        if method == 'closed':
            B_grad_f = grad_b_matrix @ f
            A = grad_b_matrix
            if small_gamma is None:
                perf_gamma = b_matrix
            else:
                perf_gamma = small_gamma
            vel_term = kb * (grad_b_matrix @ high_state)
            b = -B_grad_f - kcbf * perf_gamma -vel_term - 3*delta_ub
            closed_ans =  A @ u_nom - b
            u = u_nom.copy()
            if closed_ans[0] > 0:
                u[0] = -b[0]
            if closed_ans[1] > 0:
                u[1] = -b[1]
            if closed_ans[2] > 0:
                u[0] = b[2]
            if closed_ans[3] > 0:
                u[1] = b[3]
        return u
    if type == 'square_adhocbf':
        if method == 'QP':
            u_var = cp.Variable(u.shape[0])
            u_nom_param = cp.Parameter(u.shape[0])
            objective = cp.Minimize(cp.sum_squares(u_var - u_nom_param))
            B_grad_f = grad_b_matrix @ f
            A_qp = grad_b_matrix
            if small_gamma is None:
                perf_gamma = b_matrix
            else:
                perf_gamma = small_gamma
            vel_term = kb * (grad_b_matrix @ high_state)
            b_qp = -B_grad_f - kcbf * perf_gamma -vel_term
            constraints = [A_qp @ u_var <= b_qp]
            u_nom_param.value = u_nom
            prob = cp.Problem(objective, constraints)
            prob.solve(solver=cp.OSQP)
            if prob.status == cp.OPTIMAL or prob.status == cp.OPTIMAL_INACCURATE:
                u = u_var.value
            else:
                print("QP failed or was infeasible. Applying zero control as a safe fallback.")
                u = np.zeros(u_nom.shape)
            return u
        if method == 'closed':
            B_grad_f = grad_b_matrix @ f
            A = grad_b_matrix
            if small_gamma is None:
                perf_gamma = b_matrix
            else:
                perf_gamma = small_gamma
            vel_term = kb * (grad_b_matrix @ high_state)
            b = -B_grad_f - kcbf * perf_gamma -vel_term
            closed_ans =  A @ u_nom - b
            u = u_nom.copy()
            if closed_ans[0] > 0:
                u[0] = -b[0]
            if closed_ans[1] > 0:
                u[1] = -b[1]
            if closed_ans[2] > 0:
                u[0] = b[2]
            if closed_ans[3] > 0:
                u[1] = b[3]
        return u