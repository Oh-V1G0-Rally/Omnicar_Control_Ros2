from scipy.special import expit

from functions.basic import Quadratic, ConcaveQuadratic
import numpy as np
def padding(n, H, p, R, Omega, p_dot):
    H_,p_,R_,Omega_,p_dot_ = None, None, None, None, None,
    if H is not None:
        H_ = np.zeros((n,n))
        H_[0:2,0:2] = H
    if p is not None:
        p_ = np.zeros(n).T
        p_[0:2] = p
    if R is not None:
        R_ = np.zeros((n, n))
        R_[0:2, 0:2] = R
    if Omega is not None:
        Omega_ = np.zeros((n, n))
        Omega_[0:2, 0:2] = Omega
    if p_dot is not None:
        p_dot_ = np.zeros(n).T
        p_dot_[0:2] = p_dot

    return H_, p_, R_, Omega_, p_dot_
def rot2D(theta):
    '''
    Standard 2D rotation matrix (theta is given in degrees)
    '''
    theta = np.deg2rad(theta)
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((c,-s),(s,c)))
    return R

def canonical2D(eigen, theta):
    '''
    Returns the (2x2) symmetric matrix with eigenvalues eigen and eigenvector angle theta.
    '''
    Diag = np.diag(eigen)
    R = rot2D(theta)
    H = R @ Diag @ R.T
    return H


def kappa_linear(h, beta):
    return beta * h

def kappa_squared(h, beta):
    return beta * np.sign(h) * h**2


class BarrierFunction:

    def __init__(self, **kwargs):
        # Common initialization for all barriers
        self.kappa_fn1 = kwargs.get("kappa_fn1", kappa_linear)
        self.kappa_fn2 = kwargs.get("kappa_fn2", kappa_linear)
        self.beta1 = kwargs.get("beta1", 1)
        self.beta2 = kwargs.get("beta2", 1)
        self.is_PCBF = kwargs.get("PCBF", False)
        self.is_ISSF = kwargs.get("ISSF", False)
        self.delta = kwargs.get("delta", 0)
        self.lambd = kwargs.get("lambd", 0)
        self.eps0 =  kwargs.get("eps0", 1)
        self.sigma = kwargs.get("sigma", 0.1)
    def get_barrier_terms(self, x,f,g):
        n = x.shape[0]
        m = g.shape[1]
        f.reshape(n, 1)
        h = self(x)
        nablah = self.gradient(x).reshape(1, n)
        Lfh = (nablah @ f).reshape(1, )
        Lgh = (nablah @ g).reshape(m, )

        return h, nablah, Lfh, Lgh
    def get_cbf_constraint(self, x, f, g, u_nom=0):
        h, nablah, Lfh, Lgh = self.get_barrier_terms(x,f,g)
        alpha_h = self.kappa_fn1(h, self.beta1)
        dh_dt = self.time_derivative(x)
        if self.is_PCBF:
            path_component = self.time_derivative(x, gamma_dot=1)
            a_cbf = -np.hstack((Lgh, path_component))
        else:
            a_cbf = -np.hstack([Lgh])
        b_cbf = alpha_h + Lfh + dh_dt + Lgh @ u_nom - self.get_issf_term(x,f,g)

        return a_cbf, b_cbf

    def cbf_closed_form(self, x, f, g, u_nom=0):
        h, nablah, Lfh, Lgh = self.get_barrier_terms(x,f,g)
        alpha_h = self.kappa_fn1(h, self.beta1)
        vd_hgamma = self.time_derivative(x)
        if Lgh @ u_nom + Lfh + alpha_h > 0:
            u_corr = 0
        else:
            u_corr = -Lgh.T * (Lgh @ u_nom + (Lfh + alpha_h + vd_hgamma)) / (Lgh.T @ Lgh)
        return u_corr

    # def softplus_solution(self, x, f, g, u_nom=0):
    #     h, nablah, Lfh, Lgh = self.get_barrier_terms(x,f,g)
    #     alpha_h = self.kappa_fn1(h, self.beta1)
    #     vd_hgamma = self.time_derivative(x)
    #     b = Lgh.T @ Lgh + 1e-6
    #     a = alpha_h + vd_hgamma + Lfh + Lgh @ u_nom - self.get_issf_term(x,f,g)
    #     if b <= 1e-6:
    #         lambd = 0
    #     else:
    #         #lambd =  self.sigma * np.log(1 + np.exp(-a/(self.sigma * b))) #(-a + np.sqrt(a**2+sigma*b**2))/(2*b)
    #         lambd = self.sigma * -np.log(expit(a / (self.sigma * b)))
    #     return lambd

    def softplus_solution(self, x, f, g, u_nom=0):
        h, nablah, Lfh, Lgh = self.get_barrier_terms(x, f, g)
        alpha_h = self.kappa_fn1(h, self.beta1)
        vd_hgamma = self.time_derivative(x)
        b = Lgh.T @ Lgh
        a = alpha_h + vd_hgamma + Lfh + Lgh @ u_nom - self.get_issf_term(x, f, g)

        if b <= 0:
            lambd = 0
        else:
            # 1. Isolate the exponent term
            z = -a / (self.sigma * b)

            # 2. Use the stable expit identity for softplus: max(0, z) - log(expit(|z|))
            softplus_val = np.maximum(0.0, z) - np.log(expit(np.abs(z)))

            lambd = self.sigma * softplus_val

        return lambd

    def pcbf_closed_form_solution(self, x, f, g, u_nom=0, p=1e9):
        if not self.is_PCBF:
            return 0
        h, nablah, Lfh, Lgh = self.get_barrier_terms(x,f,g)

        alpha_h = self.kappa_fn1(h, self.beta1)
        vd_hgamma = self.time_derivative(x)
        h_gamma = self.time_derivative(x, gamma_dot=1)

        F = Lfh + vd_hgamma + alpha_h
        if F > 0:
            return np.zeros((g.shape[1] + 1,))
        u = (-F * Lgh.T) / (Lgh.T @ Lgh + h_gamma ** 2 * p ** (-1))
        u_gamma = (-F * h_gamma) / ((Lgh.T @ Lgh) * p + h_gamma ** 2)  # [Lgh@u/(Lgh.T@Lgh)*h_gamma/self.p]

        return np.concatenate((u, u_gamma))

    def get_hocbf_constraint(self, x, f, g, jac, u_nom=0):
        n = x.shape[0]
        m = g.shape[1]
        f.reshape(n, 1)

        # Barrier function and gradient
        h = self(x)
        nablah = self.gradient(x).reshape(1, n)
        nablhah_2 = self.hessian(x)

        Lfh = (nablah @ f).reshape(1, )
        Lf2h = (f.T @ nablhah_2 @ f + nablah @ jac @ f).reshape(1, )
        LfLg = (f.T @ nablhah_2 @ g + nablah @ jac @ g).reshape(m, )
        dh_dt = self.time_derivative(x)
        dh2_dt = self.time_2nd_derivative(x)
        h_dot = Lfh + dh_dt
        # h_gamma = dh_dt

        if self.kappa_fn1 == kappa_squared:
            d_alpha1_dt = 2.0 * self.beta1 * np.abs(h) * h_dot
        elif self.kappa_fn1 == kappa_linear:
            d_alpha1_dt = self.beta1 * h_dot

        alpha1_h = self.kappa_fn1(h, self.beta1)
        psi1 = h_dot + alpha1_h
        alpha2_psi1 = self.kappa_fn2(psi1, self.beta2)

        a_cbf = -np.hstack([LfLg])
        b_cbf = Lf2h + dh2_dt + alpha2_psi1 + d_alpha1_dt + LfLg @ u_nom
        return a_cbf, b_cbf

    def get_issf_term(self,x,f,g):
        if not self.is_ISSF:
            return 0
        h, nablah, Lfh, Lgh = self.get_barrier_terms(x,f,g)
        norm = Lgh.T @ Lgh

        epsilon = self.eps0*np.exp(self.lambd*h)

        return norm/epsilon



class ConcaveQuadraticBarrier(ConcaveQuadratic,BarrierFunction):
    '''
    Class for Quadratic barrier functions. For positive definite Hessians, the unsafe set is described by the interior of an ellipsoid.
    The symmetric Hessian is parametrized by Hh(pi) = sum^n_i Li pi_i, where {Li} is the canonical basis of the space of (n,n) symmetric matrices.
    '''
    def __init__(self, *args, **kwargs):

        #kwargs["height"] = 1
        super().__init__(*args, **kwargs)
        BarrierFunction.__init__(self, **kwargs)

    @classmethod
    def geometry(cls, semiaxes: tuple, R: np.ndarray, center: list | np.ndarray, **kwargs):
        ''' Create Quadratic from geometric parameters: semiaxes lengths, rotation matrix R and center '''

        eigs = [1 / (ax ** 2) for ax in semiaxes]
        H = R @ np.diag(eigs) @ R.T
        return cls(hessian=H, center=center, height=-0.5, **kwargs)

    @classmethod
    def geometry2D(cls, semiaxes: tuple, angle: float, center: list | np.ndarray, **kwargs):
        ''' Create QuadraticBarrier from 2D geometric parameters: semiaxes lengths, angle and center '''

        R = rot2D(angle)
        return QuadraticBarrier.geometry(semiaxes, R, center, **kwargs)

class QuadraticBarrier(Quadratic,BarrierFunction):
    '''
    Class for Quadratic barrier functions. For positive definite Hessians, the unsafe set is described by the interior of an ellipsoid.
    The symmetric Hessian is parametrized by Hh(pi) = sum^n_i Li pi_i, where {Li} is the canonical basis of the space of (n,n) symmetric matrices.
    '''
    def __init__(self,  *args, **kwargs):

        kwargs["height"] = -1
        super().__init__(*args, **kwargs)
        BarrierFunction.__init__(self, **kwargs)


    @classmethod
    def geometry(cls, semiaxes: tuple, R: np.ndarray, center: list | np.ndarray, **kwargs):
        ''' Create Quadratic from geometric parameters: semiaxes lengths, rotation matrix R and center '''

        eigs = [ 1/(ax**2) for ax in semiaxes ]
        H = R @ np.diag(eigs) @ R.T
        return cls(hessian=H, center=center, height=-0.5, **kwargs)

    @classmethod
    def geometry2D(cls, semiaxes: tuple, angle: float, center: list | np.ndarray, **kwargs):
        ''' Create QuadraticBarrier from 2D geometric parameters: semiaxes lengths, angle and center '''

        R = rot2D(angle)
        return QuadraticBarrier.geometry(semiaxes, R, center, **kwargs)

class  QuadraticLyapunov(Quadratic):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.kappa_fn = kwargs.get("kappa_fn", kappa_linear)
        self.alpha = kwargs.get("alpha", 1)

    def get_clf_constraint(self,x,f,g,u_nom=0):
        '''
        Sets the i-th barrier constraint.
        '''
        n = x.shape[0]
        m = g.shape[1]
        f.reshape(n, 1)
        # Barrier function and gradient
        V = self(x)
        nablaV = self.gradient(x).reshape(1,n)

        LfV = (nablaV@f).reshape(1,)
        LgV = (nablaV@g).reshape(m,)
        alpha_V = self.kappa_fn(V, self.alpha)

        a_clf = np.hstack((LgV))
        b_clf = -alpha_V - LfV - LgV@u_nom
        return a_clf, b_clf
