import numpy as np
from scipy.linalg import block_diag
from scipy.optimize import minimize
import matplotlib.pyplot as plt

def getQ(n_seg, n_order, ts):
    Q = None
    
    for k in range(n_seg):
        T = ts[k]
        Q_k = np.array([
            [100800 * T**7, 50400 * T**6, 20160 * T**5, 5040 * T**4, 0, 0, 0, 0],
            [50400 * T**6, 25920 * T**5, 10800 * T**4, 2880 * T**3, 0, 0, 0, 0],
            [20160 * T**5, 10800 * T**4, 4800 * T**3, 1440 * T**2, 0, 0, 0, 0],
            [5040 * T**4, 2880 * T**3, 1440 * T**2, 576 * T**1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0]
        ])
        
        if Q is None:
            Q = Q_k
        else:
            Q = block_diag(Q, Q_k)
    
    return Q

def getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond):
    n_all_poly = n_seg * (n_order + 1)  # the number of all polynomial coefficients
    
    # p, v, a, j constraint in start
    Aeq_start = np.zeros((4, n_all_poly))
    beq_start = np.zeros(4)
    Aeq_start[:4, :n_order + 1] = np.array([
        [0, 0, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 2, 0, 0],
        [0, 0, 0, 0, 6, 0, 0, 0]
    ])
    beq_start = np.array(start_cond)  # p, v, a, j
    
    # p, v, a constraint in end
    Aeq_end = np.zeros((4, n_all_poly))
    beq_end = np.zeros(4)
    T = ts[-1]  # time of the last trajectory
    Aeq_end[:4, -n_order - 1:] = np.array([
        [T**7,     T**6,    T**5,    T**4,   T**3, T**2, T, 1],
        [7*T**6,   6*T**5,  5*T**4,  4*T**3, 3*T**2, 2*T, 1, 0],
        [42*T**5,  30*T**4, 20*T**3, 12*T**2,   6*T,   2, 0, 0],
        [210*T**4, 120*T**3, 60*T**2,   24*T,     6,   0, 0, 0]
    ])
    beq_end = np.array(end_cond)  # p, v, a, j
    
    # Position constraint in all middle waypoints
    Aeq_wp = np.zeros((n_seg - 1, n_all_poly))
    beq_wp = np.zeros(n_seg - 1)
    for midwp_index in range(n_seg - 1):
        index = midwp_index * (n_order + 1)
        T = ts[midwp_index]
        Aeq_wp[midwp_index, index:index + 8] = [T**7, T**6, T**5, T**4, T**3, T**2, T, 1]
    beq_wp = waypoints[1:n_seg]
    
    # Position continuity constraint between each 2 segments
    Aeq_con_p = np.zeros((n_seg - 1, n_all_poly))
    for con_p_index in range(n_seg - 1):
        index = con_p_index * (n_order + 1)
        T = ts[con_p_index]
        Aeq_con_p[con_p_index, index:index + 8] = [T**7, T**6, T**5, T**4, T**3, T**2, T, 1]
        Aeq_con_p[con_p_index, index + 8:index + 16] = [0, 0, 0, 0, 0, 0, 0, -1]
    
    # Velocity continuity constraint between each 2 segments
    Aeq_con_v = np.zeros((n_seg - 1, n_all_poly))
    for con_v_index in range(n_seg - 1):
        index = con_v_index * (n_order + 1)
        T = ts[con_v_index]
        Aeq_con_v[con_v_index, index:index + 8] = [7*T**6, 6*T**5, 5*T**4, 4*T**3, 3*T**2, 2*T, 1, 0]
        Aeq_con_v[con_v_index, index + 8:index + 16] = [0, 0, 0, 0, 0, 0, -1, 0]
    
    # Acceleration continuity constraint between each 2 segments
    Aeq_con_a = np.zeros((n_seg - 1, n_all_poly))
    for con_a_index in range(n_seg - 1):
        index = con_a_index * (n_order + 1)
        T = ts[con_a_index]
        Aeq_con_a[con_a_index, index:index + 8] = [42*T**5, 30*T**4, 20*T**3, 12*T**2, 6*T, 2, 0, 0]
        Aeq_con_a[con_a_index, index + 8:index + 16] = [0, 0, 0, 0, 0, -2, 0, 0]
    
    # Jerk continuity constraint between each 2 segments
    Aeq_con_j = np.zeros((n_seg - 1, n_all_poly))
    for con_j_index in range(n_seg - 1):
        index = con_j_index * (n_order + 1)
        T = ts[con_j_index]
        Aeq_con_j[con_j_index, index:index + 8] = [210*T**4, 120*T**3, 60*T**2, 24*T, 6, 0, 0, 0]
        Aeq_con_j[con_j_index, index + 8:index + 16] = [0, 0, 0, 0, -6, 0, 0, 0]
    
    # Combine all components to form Aeq and beq
    Aeq_con = np.vstack((Aeq_con_p, Aeq_con_v, Aeq_con_a, Aeq_con_j))
    beq_con = np.hstack((np.zeros(n_seg - 1), np.zeros(n_seg - 1), np.zeros(n_seg - 1), np.zeros(n_seg - 1)))
    Aeq = np.vstack((Aeq_start, Aeq_end, Aeq_wp, Aeq_con))
    beq = np.hstack((beq_start, beq_end, beq_wp, beq_con))
    
    return Aeq, beq

def MinimumSnapQPSolver(waypoints, ts, n_seg, n_order):
    start_cond = [waypoints[0], 0, 0, 0]  # p, v, a, j
    end_cond = [waypoints[-1], 0, 0, 0]   # p, v, a, j
    
    # STEP 1: compute Q of p'Qp
    Q = getQ(n_seg, n_order, ts)
    
    # STEP 2: compute Aeq and beq 
    Aeq, beq = getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond)
    
    f = np.zeros(Q.shape[0])
    
    # Define the objective function for minimization
    def objective(x):
        return 0.5 * np.dot(x.T, np.dot(Q, x)) + np.dot(f.T, x)
    
    # Define equality constraints
    constraints = {
        'type': 'eq',
        'fun': lambda x: np.dot(Aeq, x) - beq
    }
    
    # Solve the quadratic programming problem
    result = minimize(objective, np.zeros(Q.shape[1]), constraints=constraints, method='SLSQP')
    
    if result.success:
        poly_coef = result.x
    else:
        raise ValueError("Optimization failed")
    
    return poly_coef

def derivative_s(v, order):
    temp_v = np.zeros(len(v) - 1)
    
    for i in range(order):
        temp_v[i] = (order + 1 - i) * v[i]
    
    return temp_v

# main
# Define constants
n_order = 7  # order of polynomial
path = np.array([
    [5, 0],
    [10.5, 0],
    [10.5, 3],
    [5, 3],
    [1, 3],
    [1, 6],
    [6, 6],
    [12, 6],
])
n_seg = path.shape[0] - 1  # number of segments
n_poly_perseg = n_order + 1  # number of coefficients per segment

# Initialize time and distance arrays
ts = np.zeros(n_seg)  # time of each segment
dist = np.zeros(n_seg)  # distance of each segment
dist_sum = 0
T = 10  # total time
t_sum = 0

# Calculate distances between waypoints
for i in range(n_seg):
    dist[i] = np.sqrt((path[i+1, 0] - path[i, 0])**2 + (path[i+1, 1] - path[i, 1])**2)
    dist_sum += dist[i]

# Calculate time distribution proportionally to distances
for i in range(n_seg - 1):
    ts[i] = (dist[i] / dist_sum) * T
    t_sum += ts[i]

# Set the last segment time to ensure the total time sums to T
ts[n_seg - 1] = T - t_sum

poly_coef_x = MinimumSnapQPSolver(path[:, 0], ts, n_seg, n_order)
poly_coef_y = MinimumSnapQPSolver(path[:, 1], ts, n_seg, n_order)

# Display the trajectory
X_n, Y_n = [], []
Vx_n, Vy_n = [], []
Ax_n, Ay_n = [], []
Jx_n, Jy_n = [], []
t_values = []
k = 0
tstep = 0.01
for i in range(n_seg):
    Pxi = poly_coef_x[i*(n_order+1):(i+1)*(n_order+1)]
    Pyi = poly_coef_y[i*(n_order+1):(i+1)*(n_order+1)]
    Vxi = derivative_s(Pxi, 1)
    Vyi = derivative_s(Pyi, 1)
    Axi = derivative_s(Vxi, 1)
    Ayi = derivative_s(Vyi, 1)
    Jxi = derivative_s(Axi, 1)
    Jyi = derivative_s(Ayi, 1)
    
    t_range = np.arange(0, ts[i], tstep)
    for t in t_range:
        X_n.append(np.polyval(Pxi, t))
        Y_n.append(np.polyval(Pyi, t))
        Vx_n.append(np.polyval(Vxi, t))
        Vy_n.append(np.polyval(Vyi, t))
        Ax_n.append(np.polyval(Axi, t))
        Ay_n.append(np.polyval(Ayi, t))
        Jx_n.append(np.polyval(Jxi, t))
        Jy_n.append(np.polyval(Jyi, t))
        t_values.append(t + i * T / n_seg)
        k += 1

# Plot the path of minimum snap trajectory
plt.figure(1)
plt.title("Path of Minimum Snap Trajectory")
plt.plot(X_n, Y_n, 'g-', linewidth=2)
plt.xlim([-10, 30])
plt.ylim([-5, 10])
plt.scatter(path[:, 0], path[:, 1], c='r', marker='*')
plt.show()

t = np.array(t_values)

# Plot velocity, acceleration, and jerk
plt.figure(2)
plt.subplot(3, 2, 1)
plt.plot(t, Vx_n)
plt.title('Velocity of x')
plt.grid()

plt.subplot(3, 2, 2)
plt.plot(t, Vy_n)
plt.title('Velocity of y')
plt.grid()

plt.subplot(3, 2, 3)
plt.plot(t, Ax_n)
plt.title('Acceleration of x')
plt.grid()

plt.subplot(3, 2, 4)
plt.plot(t, Ay_n)
plt.title('Acceleration of y')
plt.grid()

plt.subplot(3, 2, 5)
plt.plot(t, Jx_n)
plt.title('Jerk of x')
plt.xlabel('Time (s)')
plt.grid()

plt.subplot(3, 2, 6)
plt.plot(t, Jy_n)
plt.title('Jerk of y')
plt.xlabel('Time (s)')
plt.grid()

plt.show()
