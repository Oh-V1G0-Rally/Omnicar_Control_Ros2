import itertools
import pickle
import numpy as np
import yaml
from scipy import interpolate
from sympy import closest_points

from functions.barrier_functions import canonical2D, ConcaveQuadraticBarrier, padding
from scipy.optimize import minimize

def pad_barrier_params(n, H, p, R, Omega, p_gamma, p_gamma_gamma, Omega_dot, gamma_dot):
    """
    Pads barrier function parameters to dimension n.
    Args:
        n: Target dimension for padding
        H, p, R, Omega, p_gamma, p_gamma_gamma, Omega_dot: Barrier parameters to pad
    Returns:
        Tuple of padded parameters
    """

    def pad_matrix(mat, n):
        if mat is None:
            return None
        padded = np.zeros((n, n)) if mat.ndim == 2 else np.zeros(n).T
        if mat.ndim == 2:
            padded[0:2, 0:2] = mat
        else:
            padded[0:2] = mat
        return padded

    return (pad_matrix(H, n),
            pad_matrix(p, n),
            pad_matrix(R, n),
            pad_matrix(Omega, n),
            pad_matrix(p_gamma, n),
            pad_matrix(p_gamma_gamma, n),
            pad_matrix(Omega_dot, n),
            gamma_dot)

class Path:
    def __init__(self,cfg):
        self.cfg = cfg
        self.gamma = 0.00
        self.dt = cfg["dt_control"]
        self.map_path = cfg["map_path"]
        self.track = cfg["track"]
        
        with open(self.map_path + self.track + "/" + self.track + "_centerline_tck.yaml" , 'rb') as f:
                tck_center = yaml.safe_load(f)
        self.tck_center = [ np.array(tck_center['t']),
                           [np.array(tck_center['x']), np.array(tck_center['y'])],
                            int(tck_center['k'])]

        # with open(self.map_path + self.track + "/" + self.track + "_right_boundary_tck.yaml" , 'rb') as f:
        #         tck_right = yaml.safe_load(f)
        # self.tck_right = [ np.array(tck_right['t']),
        #                    [np.array(tck_right['x']), np.array(tck_right['y'])],
        #                     int(tck_right['k'])]
        #
        # with open(self.map_path + self.track + "/" + self.track + "_left_boundary_tck.yaml" , 'rb') as f:
        #         tck_left = yaml.safe_load(f)
        # self.tck_left = [ np.array(tck_left['t']),
        #                    [np.array(tck_left['x']), np.array(tck_left['y'])],
        #                     int(tck_left['k'])]

        self.k = cfg["k_nlc"]
        self.epsilon = 0.15
        self.inv_epsilon = 1/0.15
        self.x = 0
        self.y = 0
        self.theta = 0
        self.dx_dt = 0
        self.dy_dt = 0
        self.dtheta_dt = 0
        self.dx_dgamma = 0
        self.dy_dgamma = 0
        self.dtheta_dgamma = 0
        self.dx2_dt = 0
        self.dy2_dt = 0
        self.dx2_dgamma = 0
        self.dy2_dgamma = 0
        self.dgamma_dt = 0
        self.get_high_order_terms(0)
        self.p_gamma_norm = self.get_p_gamma_upper_bound()

        

    def reset(self,gamma=0):
        self.gamma = gamma
        self.x = 0
        self.y = 0
        self.theta = 0
        self.dx_dt = 0
        self.dy_dt = 0
        self.dtheta_dt = 0
        self.dx_dgamma = 0
        self.dy_dgamma = 0
        self.dtheta_dgamma = 0
        self.dx2_dt = 0
        self.dy2_dt = 0
        self.dx2_dgamma = 0
        self.dy2_dgamma = 0
        self.dgamma_dt = 0
        self.get_high_order_terms(0)
        self.p_gamma_norm = self.get_p_gamma_upper_bound()
        self.get_path_parameters()

    def get_path_parameters(self):
        spline = interpolate.splev(self.gamma, self.tck_center)
        spline_derivative = interpolate.splev(self.gamma, self.tck_center, der=1)
        spline_2nd_derivative = interpolate.splev(self.gamma, self.tck_center, der=2)
        if self.tck_center[2] > 2:
            spline_3rd_derivative = interpolate.splev(self.gamma, self.tck_center, der=3)
        else:
            spline_3rd_derivative = np.zeros_like(spline_2nd_derivative)

        self.x = spline[0]
        self.y = spline[1]
        self.dx_dgamma = spline_derivative[0]
        self.dy_dgamma = spline_derivative[1]
        self.dx2_dgamma = spline_2nd_derivative[0]
        self.dy2_dgamma = spline_2nd_derivative[1]

        self.dx3_dgamma = spline_3rd_derivative[0]
        self.dy3_dgamma = spline_3rd_derivative[1]

        self.theta = np.arctan2(self.dy_dgamma, self.dx_dgamma)

    def update_with_vref(self,vref):
        self.dgamma_dt = self.vref_to_gamma_dot(vref)#vref / np.sqrt(self.dx_dgamma ** 2 + self.dy_dgamma ** 2)
        self.gamma += self.dgamma_dt * self.dt

    def vref_to_gamma_dot(self,vref):
        return vref / np.sqrt(self.dx_dgamma ** 2 + self.dy_dgamma ** 2)
    def gamma_dot_to_vref(self,gamma_dot):
        return gamma_dot * np.sqrt(self.dx_dgamma ** 2 + self.dy_dgamma ** 2)

    def update_with_gamma(self,gamma_dot):
        self.dgamma_dt = gamma_dot
        #print( gamma_dot * np.sqrt(self.dx_dgamma ** 2 + self.dy_dgamma ** 2) )
        self.gamma += self.dgamma_dt * self.dt
    def get_high_order_terms(self,vref=1,gamma_dot=None):

        self.get_path_parameters()
        if gamma_dot is None:
            self.dgamma_dt = vref / np.sqrt(self.dx_dgamma ** 2 + self.dy_dgamma ** 2)
        else:
            self.dgamma_dt = gamma_dot
            #vref = self.dgamma_dt * np.sqrt(self.dx_dgamma ** 2 + self.dy_dgamma ** 2)
        self.dx_dt = self.dx_dgamma*self.dgamma_dt
        self.dy_dt = self.dy_dgamma*self.dgamma_dt
        self.theta = np.arctan2(self.dy_dgamma,self.dx_dgamma)
        self.omega = (self.dx_dgamma*self.dy2_dgamma - self.dy_dgamma*self.dx2_dgamma)/(self.dx_dgamma**2 + self.dy_dgamma**2)*self.dgamma_dt
        num = self.dx_dgamma * self.dy2_dgamma - self.dy_dgamma * self.dx2_dgamma
        den = (self.dx_dgamma ** 2 + self.dy_dgamma ** 2)
        self.kappa = num / (den)

        # angular velocity
        self.omega = self.kappa  #  self.kappa * vref

        # curvature derivative dκ/dγ
        num_kp = ((self.dx_dgamma * self.dy3_dgamma - self.dy_dgamma * self.dx3_dgamma) * den
                  - 3 * (self.dx_dgamma * self.dy2_dgamma - self.dy_dgamma * self.dx2_dgamma)
                  * (self.dx_dgamma * self.dx2_dgamma + self.dy_dgamma * self.dy2_dgamma))
        den_kp = den ** 2.5
        dkappa_dgamma = num_kp / den_kp

        # angular acceleration (omega_dot)
        self.omega_dot = vref * self.dgamma_dt * dkappa_dgamma #+ kappa*vref_dot

        num_g = self.dx_dgamma * self.dx2_dgamma + self.dy_dgamma * self.dy2_dgamma
        den_g = (self.dx_dgamma ** 2 + self.dy_dgamma ** 2)

        self.dgamma_dt2 = - (vref ** 2) * num_g / (den_g ** 2)

        self.dx2_dt= (self.dgamma_dt**2)*self.dx2_dgamma + self.dgamma_dt2*self.dx_dgamma
        self.dy2_dt = (self.dgamma_dt**2)*self.dy2_dgamma + self.dgamma_dt2*self.dy_dgamma

    def get_spline_pose(self,gamma=0):
        spline = interpolate.splev(gamma, self.tck_center)
        spline_derivative = interpolate.splev(gamma, self.tck_center, der=1)

        spline_x = spline[0]
        spline_y = spline[1]

        x_derivative = spline_derivative[0]
        y_derivative = spline_derivative[1]
        theta = np.arctan2(y_derivative,x_derivative)
        
        return spline_x, spline_y, theta


    def get_barrier_function(self,r=1):
        """
        Creates barrier function parameters based on current path state.
        Returns tuple of barrier function parameters.
        """
        # Position and orientation parameters
        p = np.array([self.x, self.y]).T
        cbf_params = {
            'lambda_x': 1/r**2,
            'lambda_y': 1/r**2,
            'angle': np.rad2deg(self.theta)
        }

        # Create canonical 2D barrier matrix
        H = canonical2D([cbf_params['lambda_x'], cbf_params['lambda_y']],
                        cbf_params['angle'])

        # Velocity and rotation matrices
        p_gamma = np.array([self.dx_dgamma, self.dy_dgamma]).T
        p_gamma_gamma = np.array([self.dx2_dgamma, self.dy2_dgamma]).T ## THIS WILL BREAK PREVIOUS HOCBF

        c, s = np.cos(self.theta), np.sin(self.theta)
        R = np.array([[c, -s],
                      [s, c]])

        Omega = np.array([[0, -self.omega],
                          [self.omega, 0]])
        Omega_dot = np.array([[0, -self.omega_dot],
                              [self.omega_dot, 0]])
        return H, p, R, Omega, p_gamma, p_gamma_gamma, Omega_dot, self.dgamma_dt

    def update_barrier_function(self, cbf, n=None, radius=1):
        """
        Updates barrier function parameters with optional padding.
        Args:
            cbf: Control barrier function object
            n: Padding dimension (optional)
        Returns:
            Updated cbf object
        """
        barrier_params = self.get_barrier_function(r=radius)

        if n is not None:
            barrier_params = pad_barrier_params(n, *barrier_params)

        param_dict = {
            'hessian': barrier_params[0],
            'center': barrier_params[1],
            'R': barrier_params[2],
            'omega': barrier_params[3],
            'p_gamma': barrier_params[4],
            'p_gamma_gamma': barrier_params[5],
            'omega_dot': barrier_params[6],
            'gamma_dot': barrier_params[7]
        }

        cbf.set_params(**param_dict)
        return cbf

    def follow_path(self, x, y, theta):

        v = np.cos(theta) * self.dx_dt + np.sin(theta) * self.dy_dt - self.k * (
                    np.cos(theta) * (self.x - x) - np.sin(theta) * (self.y - y))
        w = self.inv_epsilon * (-np.sin(theta) * self.dx_dt + np.cos(theta) * self.dy_dt- self.k * (
                    np.sin(theta) * (self.x - x) + np.cos(theta) * (self.y - y)))
        return np.array([v,w])
    
    def get_p_gamma_upper_bound(self,start=0.01,stop=0.99):
        g = np.linspace(start,stop,10000)
        spline_derivative = interpolate.splev(g, self.tck_center, der=1)
        norm = np.linalg.norm(spline_derivative,2,axis=0)
        return np.max(norm)

    def get_p_gamma_norm(self,gamma=None):
        spline_derivative = self.get_p_gamma(gamma)
        norm = np.linalg.norm(spline_derivative,2,axis=0)
        return norm

    def get_p_gamma(self,gamma=None):
        if gamma is None:
            gamma = self.gamma
        return  interpolate.splev(gamma, self.tck_center, der=1)

    def get_delta_min(self,alpha=1,L=0.1,lambd = 1,v_d_ub = 0.01,eps0=0,lambd_issf=0,u1_bounds=(-4,4),u2_bounds=(-0.5,0.5)):
        lambd = lambd
        alpha = alpha
        v_min = 0
        p_gamma_norm = self.p_gamma_norm
        L = L
        v_d_bar = v_d_ub
        ISSF=True
        if eps0 ==0:
            ISSF=False

        # Box constraints
        E_min = (-(lambd*v_min - lambd*p_gamma_norm*v_d_bar) - np.sqrt((lambd*v_min - lambd*v_d_bar*p_gamma_norm)**2+alpha**2*lambd))/(-alpha*lambd)
        print(E_min)
        E_bounds = (E_min, 1)  #1.0005
        Delta_bounds = (0, 2*np.pi)
        u1_bounds = u1_bounds
        u2_bounds = u2_bounds



        def objective(E_Delta):
            E, Delta = E_Delta

            # Compute u_star that maximizes inner product
            u1_star = u1_bounds[1] if np.cos(Delta) >= 0 else u1_bounds[0]
            u2_star = u2_bounds[1] if np.sin(Delta) >= 0 else u2_bounds[0]

            # Term 1
            #term1 = (-lambd * E * v_min + (lambd / 2) * (1 - lambd * E ** 2)) / (lambd * E * p_gamma_norm)
            term1 = (alpha/2)*(1-E**2*lambd) / (lambd * E * p_gamma_norm)

            # Term 2 (maximized w.r.t u)
            term2 = (np.cos(Delta) * u1_star + L*np.sin(Delta) * u2_star) / ( p_gamma_norm)
            if ISSF:
                ISSF_term = -(E*lambd*max(L**2,1)/(eps0*np.exp(1/2*lambd_issf*(1-E**2*lambd))))/(p_gamma_norm)
                #ISSF_term = -(E * lambd* max(L ** 2, 1) / (eps0)) / ( p_gamma_norm)
            else:
                ISSF_term = 0

            return term1 + term2 + ISSF_term - v_d_bar

        # Initial guess
        x0 = np.array([1.0, 0.5])

        res = minimize(objective, x0, bounds=[E_bounds, Delta_bounds], method='L-BFGS-B')

        # Recover optimal u_star
        E_opt, Delta_opt = res.x
        u1_opt = u1_bounds[1]*np.cos(Delta_opt) #if np.cos(Delta_opt) >= 0 else u1_bounds[0]
        u2_opt = u2_bounds[1]*np.sin(Delta_opt) #if np.sin(Delta_opt) >= 0 else u2_bounds[0]

        print("Optimal solution (E, Delta, u1, u2):", (E_opt, Delta_opt, u1_opt, u2_opt))
        print("Minimum objective value:", res.fun)
        print("Slack: ",(alpha/2)*(1-E_opt**2*lambd) / (lambd * E_opt * p_gamma_norm) +  (np.cos(Delta_opt) * u1_opt + L*np.sin(Delta_opt) * u2_opt) / ( p_gamma_norm) -(E_opt*lambd*max(L**2,1)/(eps0*np.exp(1/2*lambd_issf*(1-E_opt**2*lambd))))/(p_gamma_norm))
        return res.fun

    def sample_path_ahead(self, N, dist):
        g_init = self.gamma
        p_list = []
        spline_derivative = interpolate.splev(g_init, self.tck_center, der=1)
        delta_gamma = (dist/2) / np.sqrt(spline_derivative[0] ** 2 + spline_derivative[1] ** 2)
        g = g_init
        for i in range(11):
            g_plus_1 = max(0, g - delta_gamma)
            p = interpolate.splev(g_plus_1, self.tck_center)
            p_list.append(p)
            g = g_plus_1
            spline_derivative = interpolate.splev(g, self.tck_center, der=1)
            delta_gamma = (dist/2) / np.sqrt(spline_derivative[0] ** 2 + spline_derivative[1] ** 2)
        g = self.gamma
        delta_gamma = 0
        p_list.reverse()
        for i in range(N-11):
            g_plus_1 = min(1000,g + delta_gamma)
            p = interpolate.splev(g_plus_1,self.tck_center)
            p_list.append(p)
            g = g_plus_1
            spline_derivative = interpolate.splev(g, self.tck_center, der=1)
            if i<10:
                delta_gamma = (dist/2) / np.sqrt(spline_derivative[0] ** 2 + spline_derivative[1] ** 2)
            else:
                delta_gamma = (dist) / np.sqrt(spline_derivative[0] ** 2 + spline_derivative[1] ** 2)
        return np.array(p_list)

    # def sample_path_ahead(self, N, dist):
    #     # 1. Build the relative spacing array (in true physical meters)
    #     s_back = np.arange(-11, 0) * (dist / 2.0)
    #     s_fwd_close = np.arange(0, 10) * (dist / 2.0)
    #
    #     remaining = N - 11 - 10
    #     s_fwd_far = s_fwd_close[-1] + np.arange(1, remaining + 1) * dist
    #
    #     # Combine all relative distances into one array
    #     s_rel = np.concatenate([s_back, s_fwd_close, s_fwd_far])
    #
    #     # 2. Convert current gamma into current physical distance
    #     s_current = self.gamma_to_s(self.gamma)
    #
    #     # 3. Add the spacing and clamp to the track's physical limits [0, max_s]
    #     s_targets = np.clip(s_current + s_rel, 0, self.max_s)
    #
    #     # 4. Map the physical distances back to the spline's gamma space
    #     gamma_targets = self.s_to_gamma(s_targets)
    #
    #     # 5. Evaluate the spline EXACTLY ONCE for all N points
    #     p_x, p_y = interpolate.splev(gamma_targets, self.tck_center)
    #
    #     return np.column_stack((p_x, p_y))

    def get_track_limits(self,side="right",gamma = None):
        if gamma is None:
            gamma = self.gamma
        track_center = interpolate.splev(gamma,self.tck_center)
        if side == "right":
            closest_point = interpolate.splev(gamma, self.tck_right)
        elif side == "left":
            closest_point = interpolate.splev(gamma, self.tck_left)

        return  closest_point, closest_point-track_center




