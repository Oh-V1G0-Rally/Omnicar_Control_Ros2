import time

import numpy as np
from scipy.special import expit

from optimization_programs.gurobi_qp import GurobiQuadraticProgram
from optimization_programs.fast_qp import QuadraticProgram



class CBFQuadraticQP():
    '''
    Class for the nominal QP controller.
    '''
    def __init__(self, plant, cbf,ub = np.inf, lb = -np.inf):
        # Dimensions and system model initialization
        self.plant = plant
        self.state_dim = self.plant.n
        self.control_dim = self.plant.m

        # QP parameters
        self.QP_dim = self.control_dim
        P = np.eye(self.QP_dim)

        q = np.zeros(self.QP_dim)
        self.QP = GurobiQuadraticProgram(P=P, q=q)
        self.QP_sol = np.zeros(self.QP_dim)

        self.cbf = None
        self.lb = lb*np.ones(self.control_dim)
        self.ub = ub*np.ones(self.control_dim)
        self.QP.set_bounds(self.lb, self.ub)
        self.set_cbf(cbf)

    def get_plant_info(self):
        x = self.plant.get_state()
        f = self.plant.f_fun(x)
        g = self.plant.g_fun(x)
        u_nom = self.plant.get_control()
        return x,f,g,u_nom

    def set_cbf(self,cbf):
        self.cbf = cbf
        x,f,g,u_nom = self.get_plant_info()
        A, b = self.cbf.get_cbf_constraint(x,f,g,u_nom)
        self.QP.set_inequality_constraints(A, b)

    def get_control(self):
        '''
        Computes the QP control.
        '''
        x,f,g,u_nom = self.get_plant_info()
        A, b = self.cbf.get_cbf_constraint(x,f,g,u_nom)
        # Solve QP
        self.QP.update_parameters(A=A, b=b)
        self.QP_sol = self.QP.get_solution()

        return self.QP_sol

    def get_smooth_control(self):
        '''
        Computes the QP control.
        '''
        x, f, g, u_nom = self.get_plant_info()
        h, nablah, Lfh, Lgh = self.cbf.get_barrier_terms(x,f,g)
        u = self.cbf.softplus_solution(x,f,g,u_nom)*Lgh.T
        return u

    def get_sofptlus_gradient(self, nabla_unom=np.array([[0], [0]])):
        x = self.plant.get_state()
        nabla_f = self.plant.jacobian_f(x)
        nabla_g = self.plant.jacobian_g(x)

        # Assuming this retrieves the filtered 'uf' under the hood
        x, f, g, u_nom = self.get_plant_info()

        h, nablah, Lfh, Lgh = self.cbf.get_barrier_terms(x, f, g)

        Lgh = Lgh.reshape(1, 2)
        u_nom = u_nom.reshape(2, 1)
        f = f.reshape(3, 1)

        alpha_h = self.cbf.kappa_fn1(h, self.cbf.beta1)
        vd_hgamma = self.cbf.time_derivative(x)
        nablah_2 = self.cbf.hessian(x)
        lambd = self.cbf.softplus_solution(x, f, g, u_nom)

        b = Lgh @ Lgh.T

        a = alpha_h + vd_hgamma + Lfh + (Lgh @ u_nom) - self.cbf.get_issf_term(x,f,g)

        dx_Lfh = f.T @ nablah_2 + nablah @ nabla_f

        term1_Lgh = g.T @ nablah_2
        term2_Lgh = np.einsum('i, ijk -> jk', nablah.flatten(), nabla_g)
        dx_Lgh = term1_Lgh + term2_Lgh

        # FIXED: Added the spatial derivative of the path velocity term
        # Reshaped to (1, 3) to guarantee clean row-vector addition in nabla_a
        nabla_vd_hgamma = (self.cbf.gamma_dot * (self.cbf.p_gamma.T @ self.cbf.H)).reshape(1, 3)

        nabla_b = 2 * (Lgh @ dx_Lgh)

        # FIXED: Added nabla_vd_hgamma to the full chain rule
        nabla_a = (self.cbf.beta1 * nablah) + dx_Lfh + (u_nom.T @ dx_Lgh) + nabla_vd_hgamma #+ (Lgh @ nabla_unom)
        if self.cbf.is_ISSF:
            nabla_a+=- nabla_b/self.cbf.eps0
        dx_lambda = self.get_dx_lambda( a, b, nabla_a, nabla_b)#(1 / (1 + np.exp(a / (b * self.cbf.sigma)))) * ((a * nabla_b - b * nabla_a) / b ** 2)

        return (Lgh.T @ dx_lambda) + (lambd * dx_Lgh)

    def get_k0_gradient_uf(self, uf):
        """
        Calculates the Jacobian of the safety control k0 with respect to the input filter uf.
        Returns an m x m matrix where m is the dimension of the control input.
        """
        # 1. Fetch standard barrier terms
        x, f, g, _= self.get_plant_info()
        h, nablah, Lfh, Lgh = self.cbf.get_barrier_terms(x, f, g)

        # 2. Ensure dimensions are correct for matrix math
        # Assuming your control space is 2D (like DiffDrive/Unicycle)
        Lgh = Lgh.reshape(1, 2)
        uf = uf.reshape(2, 1)

        # 3. Reconstruct 'b'
        b = Lgh @ Lgh.T  # Results in a 1x1 scalar

        # 4. Reconstruct 'a' using the filtered control (uf)
        alpha_h = self.cbf.kappa_fn1(h, self.cbf.beta1)
        vd_hgamma = self.cbf.time_derivative(x)

        a = Lfh + alpha_h + vd_hgamma + (Lgh @ uf) - self.cbf.get_issf_term(x,f,g) # Results in a 1x1 scalar

        # 5. Calculate partial derivative of lambda w.r.t 'a'
        dlambda_da, _ = self.get_lambda_derivatives(a,b)#-1.0 / (b * (1.0 + np.exp(a / (b * self.cbf.sigma))))

        # 6. Compute the final m x m Jacobian matrix
        # scalar * (m x 1) @ (1 x m) -> m x m matrix
        dk0_duf = dlambda_da * (Lgh.T @ Lgh)

        return dk0_duf

    def get_k0_derivative_gamma(self, uf,gamma_dot_dot=0):
        # 1. Fetch standard barrier terms
        x, f, g, _ = self.get_plant_info()
        h, nablah, Lfh, Lgh = self.cbf.get_barrier_terms(x, f, g)

        # 2. Ensure dimensions are correct for matrix math
        Lgh = Lgh.reshape(1, 2)
        uf = uf.reshape(2, 1)

        # 3. Base scalars
        b = Lgh @ Lgh.T  # 1x1 scalar
        alpha_h = self.cbf.kappa_fn1(h, self.cbf.beta1)
        vd_hgamma = self.cbf.time_derivative(x)

        # ISSF term added to 'a'
        a = Lfh + alpha_h + vd_hgamma + (Lgh @ uf) - self.cbf.get_issf_term(x, f, g)

        # 4. Get the current lambda value
        lambd = self.cbf.softplus_solution(x, f, g, uf)

        # 5. Gamma derivatives of the path
        h_gamma = self.cbf.time_derivative(x, gamma_dot=1)
        h_gamma_gamma = ((x - self.cbf.center).T @ self.cbf.H @ self.cbf.p_gamma_gamma) - (
                self.cbf.p_gamma.T @ self.cbf.H @ self.cbf.p_gamma)
        nablah_gamma = self.cbf.p_gamma.T @ self.cbf.H

        # 6. Lie derivative partials
        dLgh_dgamma = nablah_gamma @ g  # 1x2 vector
        dLfh_dgamma = nablah_gamma @ f  # 1x1 scalar

        # 7. Derivatives of a and b w.r.t gamma
        db_dgamma = 2 * (Lgh @ dLgh_dgamma.T)  # 1x1 scalar

        # FIXED: Added the derivative of the ISSF term (-db_dgamma / eps0)
        da_dgamma = (self.cbf.beta1 * h_gamma) + (h_gamma_gamma*self.cbf.gamma_dot + gamma_dot_dot*h_gamma) +  + dLfh_dgamma + (
                    dLgh_dgamma @ uf)
        if self.cbf.is_ISSF:
            da_dgamma = da_dgamma -  (db_dgamma / self.cbf.eps0)
        # 8. Chain Rule for Lambda
        dlambda_da, dlambda_db = self.get_lambda_derivatives(a, b)
        dlambda_dgamma = (dlambda_da * da_dgamma) + (dlambda_db * db_dgamma)  # 1x1 scalar

        # 9. Final Product Rule for k0
        dk0_dgamma = (dlambda_dgamma * Lgh.T) + (lambd * dLgh_dgamma.reshape(2, 1))

        # Brilliant addition of .reshape(2,) here to prevent NumPy broadcasting bugs in the Euler step!
        return dk0_dgamma.reshape(2, )

    def get_k0_derivative_vd(self, uf):
        # 1. Fetch standard barrier terms
        x, f, g, _ = self.get_plant_info()
        h, nablah, Lfh, Lgh = self.cbf.get_barrier_terms(x, f, g)

        Lgh = Lgh.reshape(1, 2)
        uf = uf.reshape(2, 1)

        # 2. Base scalars to calculate dlambda_da
        b = float(Lgh @ Lgh.T)
        alpha_h = self.cbf.kappa_fn1(h, self.cbf.beta1)
        vd_hgamma = self.cbf.time_derivative(x)
        a = float(Lfh + alpha_h + vd_hgamma + (Lgh @ uf) - self.cbf.get_issf_term(x, f, g))

        # 3. Partial of 'a' w.r.t v_d is exactly h_gamma (time_derivative with gamma_dot=1)
        h_gamma = self.cbf.time_derivative(x, gamma_dot=1)
        da_dvd = float(h_gamma)

        # 4. Chain Rule
        dlambda_da, _ = self.get_lambda_derivatives(a, b)
        dlambda_dvd = dlambda_da * da_dvd

        # 5. Final Assembly
        dk0_dvd = dlambda_dvd * Lgh.T

        return dk0_dvd.reshape(2, )
    def get_lambda_derivatives(self, a, b):
        """
        Computes the partial derivatives of the softplus lambda
        with respect to 'a' and 'b'.
        Args:
            a (float/ndarray): The 'a' term from the CBF formulation.
            b (float/ndarray): The 'b' term (Lgh @ Lgh.T).
        Returns:
            tuple: (dlambda_da, dlambda_db)
        """
        # exponent = a / (b * self.cbf.sigma)
        #
        # # Calculate the common denominator term once
        # denom = 1.0 + np.exp(exponent)
        #
        # # 1. Derivative w.r.t 'a'
        # dlambda_da = -1.0 / (b * denom)

        exponent = a / (b * self.cbf.sigma)

        # 1. Derivative w.r.t 'a'
        # 1 / (1 + exp(x)) is mathematically equivalent to expit(-x)
        dlambda_da = -expit(-exponent) / b



        # 2. Derivative w.r.t 'b'
        # Using the optimized relationship: dlambda_db = dlambda_da * (-a / b)
        dlambda_db = dlambda_da * (-a / b)
        return dlambda_da, dlambda_db

    def get_dx_lambda(self, a, b, nabla_a, nabla_b):
        # Calculate the exponent term
        exponent = a / (b * self.cbf.sigma)

        # Term 1: The safe sigmoid calculation
        # expit(-x) is perfectly equivalent to 1 / (1 + exp(x))
        sigmoid_term = expit(-exponent)

        # Term 2: The quotient rule term
        quotient_term = (a * nabla_b - b * nabla_a) / (b ** 2)

        # Combine
        dx_lambda = sigmoid_term * quotient_term

        return dx_lambda


    def get_orientation(self):
        state = self.plant.get_state()
        u_nom = self.plant.get_control()
        theta = np.arccos( np.dot(u_nom,self.cbf.gradient(state)) /(np.linalg.norm(u_nom) * np.linalg.norm(self.cbf.gradient(state))))
        print(theta)
        return theta

    def find_active_constraints(self):
        x = self.plant.get_state()
        u_nom = self.plant.get_control()
        g = self.plant.g_fun(x)
        f = self.plant.f_fun(x)
        n = x.shape[0]
        m = g.shape[1]
        f.reshape(n, 1)
        # Barrier function and gradient
        h = self.cbf(x)
        alpha_h = self.cbf.kappa_fn1(h, self.cbf.beta1)
        nablah = self.cbf.gradient(x).reshape(1, n)
        Lfh = (nablah @ f).reshape(1, )
        Lgh = (nablah @ g).reshape(m, )
        condition = Lgh @ u_nom + Lfh + alpha_h
        if Lgh @ u_nom + Lfh + alpha_h < 0:
            #print("Active")
            return True
        else:
            #print("Inactive")
            return False

    def get_slack(self):
        x,f,g,u_nom = self.get_plant_info()
        h, nablah, Lfh, Lgh = self.cbf.get_barrier_terms(x, f, g)
        u1 = self.ub[0] if Lgh[0]  >= 0 else self.lb[0]
        u2 = self.ub[1] if Lgh[1] >= 0 else self.lb[1]
        slack =  self.cbf.beta1*h + Lgh@np.array([u1,u2]) - self.cbf.get_issf_term(x,f,g)
        return slack



class HOCBFQuadraticQP(CBFQuadraticQP):
    def __init__(self, plant, cbf):
        super().__init__(plant, cbf)  # call parent init properly

    def get_control(self):
        '''
        Computes the QP control.
        '''
        x,f,g,u_nom = self.get_plant_info()
        jac = self.plant.jacobian_f(x)
        A, b = self.cbf.get_hocbf_constraint(x,f,g,jac,u_nom)

        lb = np.array([-10 - u_nom[0], -2- u_nom[1]])
        ub = np.array([10 - u_nom[0], 2- u_nom[1]])
        self.QP.set_bounds(lb, ub)
        self.QP.set_inequality_constraints(A, b)
        self.QP_sol = self.QP.get_solution()

        return self.QP_sol


class PathCBFQuadraticQP(CBFQuadraticQP):
    '''
    Class for the nominal QP controller.
    '''
    def __init__(self, plant, cbf, delta_lb=-np.inf,ub = np.inf, lb = -np.inf):
        # Dimensions and system model initialization
        super().__init__(plant, cbf)
        # QP parameters
        self.cbf.is_PCBF = True
        self.QP_dim = self.control_dim +1
        self.p = 1e7
        P = np.eye(self.QP_dim)
        P[self.QP_dim-1,self.QP_dim-1] = self.p
        q = np.zeros(self.QP_dim)
        self.QP = GurobiQuadraticProgram(P=P, q=q)
        self.QP_sol = np.zeros(self.QP_dim)

        self.delta_lb = min(0,delta_lb)
        self.set_cbf(cbf)
        if lb == -np.inf:
            self.lb = -np.inf*np.ones(self.control_dim)
        else:
            self.lb = np.hstack((lb, delta_lb))
        if ub == np.inf:
            self.ub = np.inf*np.ones(self.control_dim)
        else:
            self.ub = np.hstack((ub,np.inf))
        self.QP.set_bounds(self.lb, self.ub)


    def get_slack(self):
        x,f,g,u_nom = self.get_plant_info()
        h, nablah, Lfh, Lgh = self.cbf.get_barrier_terms(x, f, g)
        u1 = self.ub[0] if Lgh[0]  >= 0 else self.lb[0]
        u2 = self.ub[1] if Lgh[1] >= 0 else self.lb[1]
        slack =  self.cbf.beta1*h + Lgh@np.array([u1,u2]) - self.cbf.get_issf_term(x,f,g)
        return slack
