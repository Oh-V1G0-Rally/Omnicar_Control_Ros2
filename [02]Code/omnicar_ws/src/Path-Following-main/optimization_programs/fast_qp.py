import time

import numpy as np
from qpsolvers import solve_qp
from qpsolvers.solvers import available_solvers

class QuadraticProgram():
    def __init__(self, **kwargs):
        for key in kwargs:
            if key == 'P':
                P = kwargs[key]
            elif key == 'q':
                q = kwargs[key]
            elif key == 'A':
                A = kwargs[key]
            elif key == 'b':
                b = kwargs[key]
            elif key == 'Aeq':
                Aeq = kwargs[key]
            elif key == 'beq':
                beq = kwargs[key]
        if 'P' in locals() and 'q' in locals():
            self.set_cost(P,q)
        if 'A' in locals() and 'b' in locals():
            self.set_constraints(A,b)
        if 'Aeq' in locals() and 'beq' in locals():
            self.set_eq_constraints(Aeq,beq)
        else:
            self.initialize()
        self.last_solution = None
        self.message = None

    def initialize(self):
        '''
        Initializes QP with no constraints.
        '''
        self.A = None
        self.b = None
        self.Aeq = None
        self.beq = None
        self.lb = None
        self.ub = None

    def set_cost(self, P, q):
        '''
        Set cost of the type x'Px + q'x 
        '''
        if ( P.ndim != 2 or q.ndim != 1 ):
            raise Exception('P must be a 2-dim array and q must be a 1-dim array.')
        if P.shape[0] != P.shape[1]:
            raise Exception('P must be a square matrix.')
        if P.shape[0] != len(q):
            raise Exception('A and b must have the same number of lines.')
        self.P = P
        self.q = q
        self.dimension = len(q)

    def set_inequality_constraints(self, A, b):
        '''
        Set constraints of the type A x <= b
        '''
        if b.ndim != 1:
            raise Exception('b must be a 1-dim array.')
        if A.ndim == 2 and A.shape[0] != len(b):
            raise Exception('A and b must have the same number of lines.')
        self.A = A
        self.b = b
        self.dimension = A.shape[0]
        if A.ndim == 2:
            self.num_constraints = A.shape[1]
        else:
            self.num_constraints = 1

    def set_equality_constraints(self, Aeq, beq):
        '''
        Set constraints of the type A x == b
        '''
        if beq.ndim != 1:
            raise Exception('beq must be a 1-dim array.')
        if Aeq.ndim == 2 and Aeq.shape[0] != len(beq):
            raise Exception('Aeq and beq must have the same number of lines.')
        self.Aeq = Aeq
        self.beq = beq
        # self.dimension = Aeq.shape[0]
        if Aeq.ndim == 2:
            self.num_eq_constraints = Aeq.shape[1]
        else:
            self.num_eq_constraints = 1

    def set_bounds(self, lb, ub):
        '''
        Set lower and upper bounds on the decision variable x.
        '''

        self.lb = lb
        self.ub = ub

    def update_parameters(self,A=None,Aeq=None, P=None, q=None, b=None, beq=None, lb=None, ub=None):
        """Update parametric values without rebuilding the problem"""
        if A is not None:
            self.A = A
        if Aeq is not None:
            self.Aeq = Aeq
        if P is not None:
            self.P = P
        if q is not None:
            self.q = q
        if b is not None and self.b is not None:
            self.b = b
        if beq is not None and self.beq is not None:
            self.beq = beq
        if lb is not None and self.lb is not None:
            self.lb = lb
        if ub is not None and self.ub is not None:
            self.ub = ub

    def get_solution(self):
        '''
        Returns the solution of the configured QP.
        '''
        self.solve_QP()
        return self.last_solution

    def solve_QP(self):
        '''
        Method for solving the configured QP using quadprog.
        '''
        try:
            if "quadprog" in available_solvers:
                self.last_solution = solve_qp(P=self.P, q=self.q, G=self.A, h=self.b, A=self.Aeq, b=self.beq,lb=self.lb,ub=self.ub, solver="quadprog",verbose=False)
            else:
                self.last_solution = solve_qp(P=self.P, q=self.q, G=self.A, h=self.b, A=self.Aeq, b=self.beq,lb=self.lb,ub=self.ub, solver="daqp",verbose=False)
        except Exception as error:
            self.message = error
            print(error)