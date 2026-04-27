import numpy as np

from functions.barrier_functions import kappa_linear, QuadraticLyapunov
from optimization_programs.gurobi_miqp import GurobiMIQP
from optimization_programs.gurobi_qp import GurobiQuadraticProgram
from optimization_programs.fast_qp import QuadraticProgram


class CLFQuadraticQP():
    '''
    Class for the nominal QP controller.
    '''
    def __init__(self, plant, clf,relaxed= False):
        # Dimensions and system model initialization
        self.plant = plant
        self.state_dim = self.plant.n
        self.control_dim = self.plant.m
        # QP parameters
        self.QP_dim = self.control_dim + 1* relaxed
        P = np.eye(self.QP_dim)
        q = np.zeros(self.QP_dim)
        self.QP = QuadraticProgram(P=P, q=q)
        self.QP_sol = np.zeros(self.QP_dim)

        self.set_clf(clf)

    def set_clf(self,clf):
        self.clf = clf
        x = self.plant.get_state()
        f = self.plant.f_fun(x)
        g = self.plant.g_fun(x)
        u_nom = self.plant.get_control()
        A, b = self.clf.get_clf_constraint(x,f,g,u_nom)
        self.QP.set_inequality_constraints(A, b)

    def get_control(self):
        '''
        Computes the QP control.
        '''
        x = self.plant.get_state()
        f = self.plant.f_fun(x)
        g = self.plant.g_fun(x)
        u_nom = self.plant.get_control()
        A, b = self.clf.get_clf_constraint(x,f,g,u_nom)
        #print(b)
        # Solve QP
        self.QP.update_parameters(A=A, b=b)
        self.QP_sol = self.QP.get_solution()
        if self.QP_sol is None:
            control = np.zeros(self.control_dim)
        else:
            control = self.QP_sol[0:self.control_dim,]
        return control


class CLFCBFQuadraticQP():
    '''
    Class for the nominal QP controller.
    '''
    def __init__(self, plant, clf, cbf):
        # Dimensions and system model initialization
        self.plant = plant

        self.state_dim = self.plant.n
        self.control_dim = self.plant.m

        self.QP_dim = self.control_dim + 1
        P = np.eye(self.QP_dim)
        P[self.QP_dim-1,self.QP_dim-1] = 1e4

        q = np.zeros(self.QP_dim)
        self.QP = GurobiQuadraticProgram(P=P, q=q)
        self.QP_sol = np.zeros(self.QP_dim)


        self.clfs = clf
        self.cbfs = cbf
        self.set_constraints()

    def get_constraints(self):
        x = self.plant.get_state()
        f = self.plant.f_fun(x)
        g = self.plant.g_fun(x)
        u_nom = self.plant.get_control()
        A , b = [] , []
        for cbf in self.cbfs:
            A_cbf, b_cbf = cbf.get_cbf_constraint(x,f,g,u_nom)
            A.append(np.hstack((A_cbf,0)))
            b.append(b_cbf)
        for clf in self.clfs:
            A_clf, b_clf = clf.get_clf_constraint(x,f,g,u_nom)
            A.append(np.hstack((A_clf,-1)))
            b.append(b_clf)
        b = np.array(b).reshape(len(b),)
        return np.array(A),b

    def set_constraints(self):
        A, b = self.get_constraints()
        self.QP.set_inequality_constraints(A, b)


    def get_control(self):
        '''
        Computes the QP control.
        '''
        A, b = self.get_constraints()
        # Solve QP
        self.QP.update_parameters(A=A, b=b)
        self.QP_sol = self.QP.get_solution()
        if self.QP_sol is None:
            control = np.zeros(self.control_dim)
        else:
            control = self.QP_sol[0:self.control_dim,]


        return control

    def get_active_constraints(self):
        active_indices = None
        if 'ineq' in self.QP.duals:
            # Find which are active (non-zero duals)
            active_indices = np.where(np.abs(self.QP.duals['ineq']) > 1e-5)[0]
            if 1 in active_indices:
                print("CLF Active ", end='')
            if 0 in active_indices:
                print("CBF Active ", end= '')
            print("")
        return active_indices


class NewCLFCBFQuadraticQP():
    def __init__(self, plant, clf, cbf, n_clfs = 5):
        # Dimensions and system model initialization
        self.plant = plant

        self.state_dim = self.plant.n
        self.control_dim = self.plant.m

        self.QP_dim = self.control_dim
        P = np.eye(self.QP_dim)
        self.path = None


        self.n_clfs = n_clfs
        self.main_clf = clf
        self.clfs = None
        self.build_clfs(dummy=True)
        self.cbfs = cbf

        q = np.zeros(self.QP_dim)
        self.QP = GurobiMIQP(P=P, q=q,n_clfs=n_clfs+1)
        self.QP_sol = np.zeros(self.QP_dim)
        self.found_deadlock = False


    def get_constraints(self):
        x = self.plant.get_state()
        f = self.plant.f_fun(x)
        g = self.plant.g_fun(x)
        u_nom = self.plant.get_control()
        A , b = [] , []
        for cbf in self.cbfs:
            A_cbf, b_cbf = cbf.get_cbf_constraint(x,f,g,u_nom)
            A.append(A_cbf)
            b.append(b_cbf)
        for clf in self.clfs:
            A_clf, b_clf = clf.get_clf_constraint(x,f,g)
            A.append(A_clf)
            b.append(b_clf)
        b = np.array(b).reshape(len(b),)
        return np.array(A),b

    def set_constraints(self):
        A, b = self.get_constraints()
        self.QP.set_inequality_constraints(A, b)

    def get_control(self):
        '''
        Computes the QP control.
        '''
        A, b = self.get_constraints()
        # Solve QP
        self.QP.update_parameters(A=A, b=b)
        self.QP_sol,  self.active_clfs = self.QP.get_solution()
        if self.active_clfs[0] >0 and self.found_deadlock == False:
            self.found_deadlock = True
            self.build_clfs(start=self.plant.get_state())
            self.set_constraints()
            self.QP_sol, self.active_clfs = self.QP.get_solution()

        #print(self.QP_sol)
        if self.QP_sol is None:
            control = np.zeros(self.control_dim)
        else:
            control = self.QP_sol[0:self.control_dim,]

        return control


    def find_path(self, start, goal):
        current_pos = np.array(start, dtype=float)
        goal = np.array(goal, dtype=float)
        path_len = self.n_clfs
        eta = 0.001
        path = [current_pos]

        # 1. Initialize tangent smart (towards goal)
        cbf_grad = normalize_vector(self.cbfs[0].gradient(current_pos))
        raw_tangent = get_tangent_vector(cbf_grad)
        goal_dir = normalize_vector(goal - current_pos)

        if np.dot(raw_tangent, goal_dir) < 0:
            last_tangent = -raw_tangent
        else:
            last_tangent = raw_tangent

        max_iter = 20000
        i = 0
        while np.linalg.norm(current_pos - goal) > 5e-2 and i < max_iter:
            grad_to_goal = normalize_vector(goal - current_pos)
            cbf_grad = normalize_vector(self.cbfs[0].gradient(current_pos))

            if np.dot(cbf_grad, grad_to_goal)  >= 0:
                step = grad_to_goal
                t_proj = get_tangent_vector(cbf_grad)
                if np.dot(t_proj, step) > 0:
                    last_tangent = t_proj
                else:
                    last_tangent = -t_proj
            else:
                tangent = get_tangent_vector(cbf_grad)
                if np.dot(last_tangent, tangent) < 0:
                    tangent = -tangent
                step = tangent
                last_tangent = tangent

            current_pos = current_pos + eta * step
            path.append(current_pos)
            i += 1

        path = np.array(path)
        if len(path) > path_len:
            indices = np.linspace(0, len(path) - 1, path_len, dtype=int)
            return np.flip(path[indices],axis=0)
        else:
            return np.flip(path,axis=0)

    def build_clfs(self,start = None, dummy = False):
        if dummy:
            H = self.main_clf.H*0
            alpha = 0
            xd = self.main_clf.center * 0
            self.path = [xd for _ in range(self.n_clfs)]
        else:
            xd = self.main_clf.center
            H = self.main_clf.H
            alpha = self.main_clf.alpha
            self.path = self.find_path(start, xd)
        self.clfs = [self.main_clf]
        for p in self.path:
            self.clfs.append(QuadraticLyapunov(hessian=H, center=p, height=0,
                                         limits=(-4, 4, -4, 4), spacing=0.05, alpha=alpha))

class CLFTrackingQP():
    '''
    Class for the nominal QP controller.
    '''

    def __init__(self, n, m,lambd=2):
        # Dimensions and system model initialization
        self.state_dim = n
        self.control_dim = m
        # QP parameters
        self.QP_dim = self.control_dim
        P = np.eye(self.QP_dim)
        q = np.zeros(self.QP_dim)
        self.QP = QuadraticProgram(P=P, q=q)
        self.QP_sol = np.zeros(self.QP_dim)
        self.QP.set_inequality_constraints(np.zeros((2,)), np.zeros((1,)))
        self.lambd = lambd

    def get_V(self, q_dot, gk0):
        return 0.5*(q_dot - gk0)@(q_dot - gk0).T
    def get_control(self,q_dot,xi,k0,k0_dot,g,grad_g,G,V):
        '''
        Computes the QP control.
        '''
        e = (q_dot - g@k0)
        g_dot = np.einsum('ijk,k->ij', grad_g, q_dot)
        A = e.T@g@G
        b = e.T@(g_dot@(xi - k0) -g@k0_dot )
        b = -b - self.lambd*V
        self.QP.update_parameters(A=A, b=np.array([b]))
        self.QP_sol = self.QP.get_solution()
        if self.QP_sol is None:
            control = np.zeros(self.control_dim)
        else:
            control = self.QP_sol[0:self.control_dim,]
        #print(control)
        return control


def get_tangent_vector(vector):
    # Avoid division by zero
    n = normalize_vector(vector)
    # Create an arbitrary vector that is NOT parallel to n.
    # We choose the unit vector of the smallest component of n
    # to ensure they are as different as possible (stability).
    min_idx = np.argmin(np.abs(n))
    arbitrary_v = np.zeros_like(n)
    arbitrary_v[min_idx] = 1.0

    # Gram-Schmidt process: Subtract the projection onto n
    # tangent = v - (v . n) * n
    tangent = arbitrary_v - np.dot(arbitrary_v, n) * n

    # Normalize result
    return tangent / np.linalg.norm(tangent)

def normalize_vector(vector):
    norm = np.linalg.norm(vector)
    if norm < 1e-10:
        return np.zeros_like(vector)

    # Normalize the vectorient
    n = vector / norm
    return n


