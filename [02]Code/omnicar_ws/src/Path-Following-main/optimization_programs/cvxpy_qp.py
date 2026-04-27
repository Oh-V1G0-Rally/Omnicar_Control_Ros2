import cvxpy as cp
import numpy as np
import time
class CVXPYQuadraticProgram():
    def __init__(self, **kwargs):
        self.initialize()
        for key, value in kwargs.items():
            if key == 'P':
                self.P = value
            elif key == 'q':
                self.q = value
            elif key == 'A':
                self.A = cp.Parameter(value.shape, value=value)
            elif key == 'b':
                self.b = cp.Parameter(value.shape, value=value)
            elif key == 'Aeq':
                self.Aeq = cp.Parameter(value.shape, value=value)
            elif key == 'beq':
                self.beq = cp.Parameter(value.shape, value=value)
            elif key == 'lb':
                self.lb = cp.Parameter(value.shape, value=value)
            elif key == 'ub':
                self.ub = cp.Parameter(value.shape, value=value)

        if hasattr(self, 'q'):
            self.dimension = len(self.q)

        self.x = cp.Variable(self.dimension)

        self.constraints = []
        self.problem = None
        self.last_solution = None
        self.message = None

    def initialize(self):
        self.P = None
        self.q = None
        self.A = None
        self.b = None
        self.Aeq = None
        self.beq = None
        self.lb = None
        self.ub = None
        self.dimension = None

    def set_cost(self, P, q):
        if P.ndim != 2 or q.ndim != 1:
            raise ValueError("P must be 2D and q must be 1D.")
        if P.shape[0] != P.shape[1]:
            raise ValueError("P must be square.")
        if P.shape[0] != len(q):
            raise ValueError("P and q dimension mismatch.")

        self.P = P
        self.q =q
        self.dimension = len(q)

    def set_inequality_constraints(self, A, b):
        """A x <= b"""
        self.A = cp.Parameter(A.shape, value=A)
        self.b = cp.Parameter(b.shape, value= b)
        # Update constraints
        self._rebuild_constraints()

    def set_equality_constraints(self, Aeq, beq):
        """Aeq x == beq"""
        self.Aeq = cp.Parameter(Aeq.shape, value=Aeq)
        self.beq = cp.Parameter(beq.shape, value= beq)
        self._rebuild_constraints()

    def set_bounds(self, lb, ub):
        self.lb = cp.Parameter(lb.shape, value=lb)
        self.ub = cp.Parameter(ub.shape, value=ub)
        self._rebuild_constraints()

    def _rebuild_constraints(self):
        """Rebuild constraints list using current parameters"""
        constraints = []

        if self.A is not None and self.b is not None:
            constraints.append(self.A @ self.x <= self.b)

        if self.Aeq is not None and self.beq is not None:
            constraints.append(self.Aeq @ self.x == self.beq)

        if self.lb is not None:
            constraints.append(self.x >= self.lb)

        if self.ub is not None:
            constraints.append(self.x <= self.ub)

        self.constraints = constraints
        self.problem = cp.Problem(cp.Minimize(0.5 * cp.quad_form(self.x, self.P)),
                                  self.constraints)

    def update_parameters(self,A=None,Aeq=None, P=None, q=None, b=None, beq=None, lb=None, ub=None):
        """Update parametric values without rebuilding the problem"""
        if A is not None:
            self.A.value = A
        if Aeq is not None:
            self.Aeq.value = Aeq
        if P is not None:
            self.P = P
        if q is not None:
            self.q.value = q
        if b is not None and self.b is not None:
            self.b.value = b
        if beq is not None and self.beq is not None:
            self.beq.value = beq
        if lb is not None and self.lb is not None:
            self.lb.value = lb
        if ub is not None and self.ub is not None:
            self.ub.value = ub

    def get_solution(self):

        self.solve()

        return self.last_solution

    def solve(self):
        """Solve the parametric QP"""
        if self.problem is None:
            raise RuntimeError("Problem not initialized. Set constraints first.")

        try:
            if self.problem.is_mixed_integer() == False:
                start = time.time()
                self.problem.solve(solver=cp.DAQP)
                end = time.time()
                print(f"Solve time (wall): {end - start:.6f} seconds")
            else:
                self.problem.solve(solver=cp.GUROBI)
            if self.x.value is None:
                raise RuntimeError("Solver failed to find a solution.")
            #print(self.problem.solver_stats)
            self.last_solution = self.x.value
            self.message = self.problem.status
        except Exception as e:
            self.last_solution = None
            self.message = str(e)
            print("QP Solve error:", e)

        return self.last_solution