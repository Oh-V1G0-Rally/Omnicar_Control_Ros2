import gurobipy as gp
from gurobipy import GRB
import numpy as np
import time


class GurobiQuadraticProgram:
    def __init__(self, **kwargs):
        self.initialize()
        for key, value in kwargs.items():
            setattr(self, key, value)

        if hasattr(self, 'q') and self.q is not None:
            self.dimension = len(self.q)
        elif hasattr(self, 'P') and self.P is not None:
            self.dimension = self.P.shape[0]

        env = gp.Env(empty=True)
        env.setParam("OutputFlag", 0)
        env.start()
        self.model = gp.Model("QP", env=env)

        if self.dimension:
            self.x = self.model.addMVar(shape=self.dimension, lb=-GRB.INFINITY, ub=GRB.INFINITY, name="x")

        if self.P is not None:
            self.set_cost(self.P, self.q)

        self._rebuild_constraints()

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

        self.duals = {}  # Store duals here

        self.model = None
        self.x = None
        self.constr_ineq = None
        self.constr_eq = None
        self.last_solution = None
        self.message = None

    def set_cost(self, P, q=None):
        if P.ndim != 2: raise ValueError("P must be 2D.")
        self.P = P
        self.q = q if q is not None else np.zeros(P.shape[0])
        self.dimension = len(self.q)

        if self.x is None or self.x.shape[0] != self.dimension:
            if self.x is not None: self.model.remove(self.x)
            self.x = self.model.addMVar(self.dimension, lb=-GRB.INFINITY, ub=GRB.INFINITY)

        obj = 0.5 * (self.x @ self.P @ self.x) + (self.q @ self.x)
        self.model.setObjective(obj, GRB.MINIMIZE)

    def set_inequality_constraints(self, A, b):
        self.A = A
        self.b = b
        if self.constr_ineq is not None: self.model.remove(self.constr_ineq)
        self.constr_ineq = self.model.addConstr(self.A @ self.x <= self.b, name="ineq")

    def set_equality_constraints(self, Aeq, beq):
        self.Aeq = Aeq
        self.beq = beq
        if self.constr_eq is not None: self.model.remove(self.constr_eq)
        self.constr_eq = self.model.addConstr(self.Aeq @ self.x == self.beq, name="eq")

    def set_bounds(self, lb, ub):
        self.lb = lb
        self.ub = ub
        if self.lb is not None: self.x.setAttr("LB", self.lb)
        if self.ub is not None: self.x.setAttr("UB", self.ub)

    def _rebuild_constraints(self):
        if self.A is not None and self.b is not None:
            self.set_inequality_constraints(self.A, self.b)
        if self.Aeq is not None and self.beq is not None:
            self.set_equality_constraints(self.Aeq, self.beq)
        if self.lb is not None or self.ub is not None:
            self.set_bounds(self.lb, self.ub)

    def update_parameters(self, A=None, Aeq=None, P=None, q=None, b=None, beq=None, lb=None, ub=None):
        if b is not None:
            self.b = b
            if self.constr_ineq: self.constr_ineq.setAttr("RHS", b)
        if beq is not None:
            self.beq = beq
            if self.constr_eq: self.constr_eq.setAttr("RHS", beq)
        if lb is not None:
            self.lb = lb
            self.x.setAttr("LB", lb)
        if ub is not None:
            self.ub = ub
            self.x.setAttr("UB", ub)
        if P is not None or q is not None:
            self.set_cost(P if P is not None else self.P, q if q is not None else self.q)
        if A is not None:
            self.set_inequality_constraints(A, self.b)
        if Aeq is not None:
            self.set_equality_constraints(Aeq, self.beq)

    def get_solution(self):
        self.solve()
        return self.last_solution

    def solve(self):
        if self.model is None: raise RuntimeError("Model not initialized.")
        try:
            start = time.time()
            self.model.optimize()
            end = time.time()
            #print(start-end)
            status = self.model.Status

            if status == GRB.OPTIMAL:
                self.last_solution = self.x.X
                self.message = "OPTIMAL"

                # --- EXTRACT DUALS ---
                self.duals = {}
                # Inequality Duals (Pi)
                if self.constr_ineq is not None:
                    self.duals['ineq'] = self.constr_ineq.Pi
                    # Equality Duals (Pi)
                if self.constr_eq is not None:
                    self.duals['eq'] = self.constr_eq.Pi
                # Bound Duals (Reduced Cost)
                # RC > 0 implies Active at Lower Bound
                # RC < 0 implies Active at Upper Bound
                self.duals['bounds'] = self.x.RC
            else:
                self.last_solution = None
                self.message = f"Infeasible/Error: {status}"

        except gp.GurobiError as e:
            self.last_solution = None
            self.message = str(e)
            print("Gurobi Error:", e)

        return self.last_solution

