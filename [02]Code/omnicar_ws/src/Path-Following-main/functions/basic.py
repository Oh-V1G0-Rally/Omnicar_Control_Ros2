import logging
import itertools
import numpy as np
import contourpy as ctp
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

from abc import ABC, abstractmethod
from dataclasses import dataclass



def commutation_matrix(n):
    '''
    Generate commutation matrix K relating the vectorization of a matrix n x n matrix A with the vectorization of its transpose A', as
    vec(A') = K vec(A).
    '''
    # determine permutation applied by K
    w = np.arange(n * n).reshape((n, n), order="F").T.ravel(order="F")
    # apply this permutation to the rows (i.e. to each column) of identity matrix and return result
    return np.eye(n * n)[w, :]


def vec(A):
    '''
    Vectorize matrix in a column-major form (Fortran-style).
    '''
    return A.flatten('F')


def mat(vec):
    '''
    De-vectorize a square matrix which was previously vectorized in a column-major form.
    '''
    n = np.sqrt(len(vec))
    if (not n.is_integer()):
        raise Exception('Input vector does not represent a vectorized square matrix.')
    n = int(n)
    return vec.reshape(n, n).T


@dataclass
class LeadingShape:
    ''' Data class for a leading shape (to be used as an approximation tool) '''
    shape: np.ndarray
    bound: str = ''
    approximate: bool = False


class Function(ABC):
    '''
    Implementation of abstract class for scalar functions of any input dimension.

    '''

    def __init__(self, **kwargs):

        # Initialize basic parameters
        self._dim = 2
        self._output_dim = 1
        self.color = mcolors.BASE_COLORS["k"]
        self.linestyle = "solid"
        self.alpha = 1.0
        self.limits = (-1, 1, -1, 1)
        self.spacing = 0.1

        self.set_params(**kwargs)

        if self._output_dim == 1:
            self.generate_contour()

    def _validate(self, point):
        ''' Validates input data '''

        if self._dim != 1:
            if not isinstance(point, (list, tuple, np.ndarray)):
                raise Exception("Input data point is not a numeric array.")
        else:
            if isinstance(point, (np.int64, np.float64, float, int)):
                point = np.array([point])

        if isinstance(point, (list, tuple)):
            point = np.array(point)
        return point

    @abstractmethod
    def _function(self, point: np.ndarray) -> np.ndarray:
        '''
        Abstract implementation of function value.
        Must receive point as input and return the corresponding function value.
        Overwrite on children classes.
        '''
        pass

    @abstractmethod
    def _gradient(self, point: np.ndarray) -> np.ndarray:
        '''
        Abstract implementation of gradient vector.
        Must receive point as input and return the corresponding gradient value.
        Overwrite on children classes.
        '''
        pass

    @abstractmethod
    def _jacobian(self, point: np.ndarray) -> np.ndarray:
        '''
        Abstract implementation of the Jacobian matrix.
        Must receive point as input and return the corresponding gradient value.
        Overwrite on children classes.
        '''
        pass

    @abstractmethod
    def _hessian(self, point: np.ndarray) -> np.ndarray:
        '''
        Abstract implementation of hessian computation. Must receive point as input and return the corresponding hessian value.
        Overwrite on children classes.
        '''
        pass

    def generate_contour(self):
        '''
        Create contour generator object for the given function.
        Parameters: limits (2x2 array) - min/max limits for x,y coords
                    spacing - grid spacing for contour generation
        '''
       # if self._dim != 2:
       #     logging.warning("Contour plot can only be used for 2D functions.")
        #    self.contour = None
        #    #return

        x_min, x_max, y_min, y_max = self.limits
        x = np.arange(x_min, x_max, self.spacing)
        y = np.arange(y_min, y_max, self.spacing)
        xg, yg = np.meshgrid(x, y)

        fvalues = np.zeros(xg.shape)
        for i, j in itertools.product(range(xg.shape[0]), range(xg.shape[1])):
            pt = np.hstack((np.array([xg[i, j], yg[i, j]]), np.zeros(self._dim-2)))
            fvalues[i, j] = self(pt)

        self.contour = ctp.contour_generator(x=xg, y=yg, z=fvalues)

    def __call__(self, point):
        return self._function(self._validate(point))

    def function(self, point):
        return self._function(self._validate(point))

    def gradient(self, point):
        return self._gradient(self._validate(point))

    def jacobian(self, point):
        return self._jacobian(self._validate(point))

    def hessian(self, point):
        return self._hessian(self._validate(point))

    def set_params(self, **params):
        ''' Sets function basic parameters (mostly plotting) '''

        for key in params.keys():
            key = key.lower()
            if key == "dim":
                self._dim = params["dim"]
                continue
            if key == "color":
                self.color = params["color"]
                continue
            if key == "linestyle":
                self.linestyle = params["linestyle"]
                continue
            if key == "limits":
                self.limits = params["limits"]
                continue
            if key == "spacing":
                self.spacing = params["spacing"]
                continue

    def get_levels(self, levels=[0.0]) -> list:
        ''' Generates function level sets from the contour generator object '''
        if not self.contour: return []

        level_contours = []
        for lvl in levels:
            line = self.contour.lines(lvl)
            level_contours.append(line)
        return level_contours

    def plot_levels(self, ax=plt, levels=[0.0], **kwargs):
        ''' Plots function level sets at the input axis ax. Additional args may be passed for color and linestyle '''

        color = self.color
        linestyle = self.linestyle
        alpha = self.alpha
        for key in kwargs.keys():
            key = key.lower()
            if key == "color":
                color = kwargs["color"]
                continue
            if key == "linestyle":
                linestyle = kwargs["linestyle"]
                continue
            if key == "alpha":
                alpha = kwargs["alpha"]
                continue

        collections = []
        for level in self.get_levels(levels):
            for segment in level:
                line2D = ax.plot(segment[:, 0], segment[:, 1], color=color, linestyle=linestyle, alpha=alpha)
                collections.append(line2D[0])
        return collections


class Quadratic(Function):
    '''
    Class for quadratic function representing x'Ax + b'x + c = 0.5 (x - p)'H(x-p) + height = 0.5 x'Hx - 0.5 p'(H + H')x + 0.5 p'Hp + height
    '''

    def to_factored(A, b, c):
        H = 2 * A
        x0 = - np.linalg.inv(H) @ b
        min = c - 0.5 * x0.T @ H @ x0
        return H, x0, min

    def from_factored(H, x0, min):
        A = 0.5 * H
        b = - H @ x0
        c = 0.5 * x0.T @ H @ x0 + min
        return A, b, c

    def __init__(self, **kwargs):
        ''' Initialize it with hessian, center and height '''
        super().__init__(**kwargs)
        # Determine dimension from hessian or center if provided
        if "hessian" in kwargs:
            self.H = np.array(kwargs["hessian"])
            self._dim = self.H.shape[0]
        elif "center" in kwargs:
            self.center = np.array(kwargs["center"])
            self._dim = len(self.center)
        else:
            self._dim = getattr(self, '_dim', 2)  # Default to 2 if not set

        # Define default values based on dimension
        defaults = {
            "hessian": np.zeros((self._dim, self._dim)),
            "center": np.zeros(self._dim),
            "height": 0.5,
            "R": np.zeros((self._dim, self._dim)),
            "omega": np.zeros((self._dim, self._dim)),
            "p_gamma": np.zeros(self._dim),
            "p_gamma_gamma": np.zeros(self._dim),
            "omega_dot": np.zeros((self._dim, self._dim)),
            "gamma_dot": 1,
        }

        # Map parameter names to attribute names
        param_mapping = {
            "hessian": "H",
            "center": "center",
            "height": "height",
            "R": "R",
            "omega": "Omega",
            "p_gamma": "p_gamma",
            "p_gamma_gamma": "p_gamma_gamma",
            "omega_dot": "Omega_dot",
            "gamma_dot": "gamma_dot",
        }

        # Set attributes with provided values or defaults
        for param_name, attr_name in param_mapping.items():
            value = kwargs.get(param_name, defaults[param_name])
            setattr(self, attr_name, np.array(value))


    def set_params(self, **kwargs):
        '''
        Sets the quadratic function parameters.
        '''
        super().set_params(**kwargs)
        for key, value in kwargs.items():
            if key == "hessian":
                self.H = np.array(value)
                self._dim = self.H.shape[0]
            elif key == "center":
                self.center = np.array(value)
                self._dim = len(self.center)
            elif key == "height":
                self.height = value
            elif key == "R":
                self.R = value
            elif key == "omega":
                self.Omega = value
            elif key == "p_gamma":
                self.p_gamma = value
            elif key == "p_gamma_gamma":
                self.p_gamma_gamma = value
            elif key == "omega_dot":
                self.Omega_dot = value
            elif key == "gamma_dot":
                self.gamma_dot = value

        self.A, self.b, self.c = Quadratic.from_factored(self.H, self.center, self.height)



    def _function(self, pt):
        '''
        General quadratic function.
        '''
        return np.array(pt) @ self.A @ np.array(pt) + self.b @ np.array(pt) + self.c

    def _gradient(self, pt):
        '''
        Gradient of general quadratic function.
        '''
        return (self.A + self.A.T) @ np.array(pt) + self.b

    def _jacobian(self, pt):
        return self._gradient(pt)

    def _hessian(self, pt):
        '''
        Hessian of general quadratic function.
        '''
        return (self.A + self.A.T)

    def get_values(self, x):
        '''
        Returns function, gradient and Hessian values at an input point x
        '''
        fun = self(x)
        nabla_fun = self.gradient(x)
        Hfun = self.hessian(x)
        return fun, nabla_fun, Hfun

    def eig(self):
        eigs, Q = np.linalg.eig(self.H)
        if np.allclose(eigs.imag, 0, atol=1e-12):
            eigs = eigs.real
        else:
            print("Warning: Significant imaginary parts detected!")
        return eigs, Q

    def eigvals(self):
        eigs, Q = self.eig()
        return eigs

    def __str__(self):
        return f"Quadratic 0.5 (x - p)'H(x-p) with H = \n{self.H}\n and center = {self.center} "

    def time_derivative(self,x, gamma_dot = None):
        if gamma_dot is None:
            gamma_dot = self.gamma_dot
        eigs = self.eigvals()
        Q = np.diag(eigs)
        X = (x - self.center)
        H_dot = 0.5*(self.R@self.Omega@Q@self.R.T + self.R @Q@self.Omega.T@self.R.T)

        dh_dt = gamma_dot*(-self.p_gamma.T@self.H@X + (X.T@H_dot@X))
        return dh_dt

    def time_2nd_derivative(self,x, gamma_dot = None):
        if gamma_dot is None:
            gamma_dot = self.gamma_dot
        eigs = self.eigvals()
        Q = np.diag(eigs)
        X = x-self.center
        H_dot = gamma_dot*(self.R @ self.Omega @ Q @ self.R.T)
        H_dot_dot = self.R@(gamma_dot**2*self.Omega@self.Omega@Q +self.Omega_dot@Q  + gamma_dot**2*self.Omega@Q@self.Omega.T)@self.R.T
        phi1_dot =  self.p_gamma_gamma.T@self.H@X + gamma_dot*self.p_gamma.T@H_dot@X - gamma_dot**2*self.p_gamma.T@self.H@self.p_gamma
        phi2_dot = X.T@H_dot_dot@X - 2*gamma_dot*self.p_gamma.T@H_dot@X
        dh2_dt2=-phi1_dot+phi2_dot
        return dh2_dt2

class ConcaveQuadratic(Quadratic):
    '''
    Class for concave quadratic functions:
        c - 0.5 (x - p)'H(x - p),   with H positive semidefinite
    Inherits from Quadratic but flips signs to ensure concavity.
    '''

    def set_params(self, **kwargs):
        '''
        Sets the concave quadratic parameters.
        '''
        super().set_params(**kwargs)
        self.A, self.b, self.c = Quadratic.from_factored(-self.H, self.center, self.height)


    def __str__(self):
        return f"ConcaveQuadratic c - 0.5 (x - p)'H(x-p) with H = \n{self.H}\n and center = {self.center} "

    def update(self,H,p,height):
        self.H = H
        self.center = p
        self.height = height
        self.A, self.b, self.c = Quadratic.from_factored(-self.H, self.center, self.height)

    def time_derivative(self,x, gamma_dot = None):
        return -super().time_derivative(x,gamma_dot)

    def time_2nd_derivative(self,x, gamma_dot = None):
        return -super().time_2nd_derivative(x,gamma_dot)
