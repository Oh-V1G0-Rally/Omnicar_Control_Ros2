import gurobipy as gp
from gurobipy import GRB
import numpy as np
import time


class GurobiMIQP:
    def __init__(self, **kwargs):
        self.initialize()
        for key, value in kwargs.items():
            setattr(self, key, value)

        if hasattr(self, 'q') and self.q is not None:
            self.dimension = len(self.q)
        elif hasattr(self, 'P') and self.P is not None:
            self.dimension = self.P.shape[0]
        self.M = 1e2
        self.M_cost = 1e3
        env = gp.Env(empty=True)
        env.setParam("FeasibilityTol",1e-9)
        env.setParam("OutputFlag", 0)
        env.setParam("IntFeasTol", 1e-9)
        env.setParam("NumericFocus", 3)

        env.start()
        self.model = gp.Model("MIQP", env=env)

        # 1. Initialize Continuous Variables (x)
        if self.dimension:
            self.x = self.model.addMVar(
                shape=self.dimension,
                lb=-GRB.INFINITY,
                ub=GRB.INFINITY,
                name="x"
            )


        self.z = self.model.addMVar(
            shape=self.n_clfs,
            vtype=GRB.BINARY,
            name="z"
        )

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

        self.model = None
        self.x = None
        self.z = None  # Placeholder for z
        self.constr_ineq = None
        self.constr_eq = None
        self.last_solution = None
        self.message = None
        self.binary_vars = None
        self.solve_time = None

    def set_cost(self, P, q=None):
        """
        Sets the quadratic cost for x.
        Note: Currently this does not include a cost term for z,
        but z is part of the model and can be added to the objective if needed.
        """
        if P.ndim != 2: raise ValueError("P must be 2D.")
        self.P = P
        self.q = q if q is not None else np.zeros(P.shape[0])
        self.dimension = len(self.q)

        if self.x is None or self.x.shape[0] != self.dimension:
            if self.x is not None: self.model.remove(self.x)
            self.x = self.model.addMVar(self.dimension, lb=-GRB.INFINITY, ub=GRB.INFINITY)


        obj = 0.5 * (self.x @ self.P @ self.x) + (self.q @ self.x) + (self.z.sum() * self.M)
        self.model.setObjective(obj, GRB.MINIMIZE)

    def _create_single_ineq_constraint(self, i):
        """
        Helper: Creates the specific constraint for row i based on your Big-M logic.
        Assumes self.A, self.b, self.z, and self.M are set.
        """
        expr_lhs = self.A[i] @ self.x

        if i == 0:
            return self.model.addConstr(
                expr_lhs <= self.b[i],
                name=f"ineq{i}"
            )

        elif i == 1:
            return self.model.addConstr(
                expr_lhs <= self.b[i] + self.z[i - 1] * self.M,
                name=f"ineq{i}"
            )

        elif i == self.A.shape[0] - 1:
            return self.model.addConstr(
                expr_lhs <= self.b[i] + (1 - self.z[i - 1]) * self.M,
                name=f"ineq{i}"
            )

        else:
            return self.model.addConstr(
                expr_lhs <= self.b[i] + self.z[i - 1] * self.M + (1 - self.z[i - 2]) * self.M,
                name=f"ineq{i}"
            )

    def set_inequality_constraints(self, A, b):
        self.A = A
        self.b = b

        # 1. Clean up old constraints (batch remove)
        if self.constr_ineq is not None:
            self.model.remove(self.constr_ineq)

        self.constr_ineq = []

        # 2. Rebuild using the helper
        for i in range(A.shape[0]):
            c = self._create_single_ineq_constraint(i)
            self.constr_ineq.append(c)



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

        if beq is not None:
            self.beq = beq
            if self.constr_eq:
                self.constr_eq.setAttr("RHS", beq)

        if Aeq is not None:
            self.set_equality_constraints(Aeq, self.beq)

        # Update state first
        if b is not None: self.b = b

        # Trigger Rebuild if necessary
        # Case 1: User provided A (Always rebuild)
        if A is not None:
            self.set_inequality_constraints(A, self.b)

        # Case 2: User provided b ONLY (Must rebuild to preserve Big-M offsets)
        elif b is not None and self.A is not None:
            self.set_inequality_constraints(self.A, self.b)

        # --- 3. Handle Bounds & Objective ---
        if lb is not None:
            self.lb = lb
            self.x.setAttr("LB", lb)
        if ub is not None:
            self.ub = ub
            self.x.setAttr("UB", ub)

        if P is not None or q is not None:
            self.set_cost(
                P if P is not None else self.P,
                q if q is not None else self.q
            )

    def get_solution(self):
        self.solve()
        return self.last_solution , self.binary_vars

    def solve(self):
        if self.model is None: raise RuntimeError("Model not initialized.")
        try:
            start = time.time()
            self.model.optimize()
            end = time.time()
            self.solve_time = end - start

            status = self.model.Status

            if status == GRB.OPTIMAL:
                self.last_solution = self.x.X
                self.binary_vars = self.z.X
                #print(self.x.X,self.z.X)
                self.message = "OPTIMAL"


            else:
                self.last_solution = None
                self.message = f"Infeasible/Error: {status}"

        except gp.GurobiError as e:
            self.last_solution = None
            self.message = str(e)
            print("Gurobi Error:", e)

        return self.last_solution