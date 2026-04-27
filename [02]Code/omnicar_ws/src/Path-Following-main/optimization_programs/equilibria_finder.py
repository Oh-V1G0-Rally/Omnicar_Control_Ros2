import casadi as ca
import numpy as np
from scipy.optimize import root
import matplotlib.pyplot as plt


# --- Helper Functions ---
def to_dm(arr):
    return ca.DM(arr)


def grad(eli, x):
    return (to_dm(eli.A) + to_dm(eli.A).T) @ x + to_dm(eli.b)


def val(eli, x):
    return x.T @ to_dm(eli.A) @ x + to_dm(eli.b).T @ x + eli.c


class EquilibriaFinder:
    def __init__(self, dim=2, clf=None, cbf=None, plant=None):
        self.opti = ca.Opti()
        self.x = self.opti.variable(dim)
        self.dim = dim
        self.clf = clf
        self.cbf = cbf
        self.plant = plant
        self.eqn = None

    def build_equations(self):
        x = self.x

        # Dynamics
        if hasattr(self.plant, 'get_f_sym') and hasattr(self.plant, 'get_g_sym'):
            f = self.plant.get_f_sym(x)
            g = self.plant.get_g_sym(x)
        else:
            f = ca.DM.zeros(self.dim)
            g = ca.DM.eye(self.dim)

        # Gradients
        nabla_h = grad(self.cbf, x)
        nabla_V = grad(self.clf, x)
        u_nom = -nabla_V

        # QP Closed Form
        Lf_h = ca.mtimes(nabla_h.T, f)
        Lg_h = ca.mtimes(nabla_h.T, g)

        numerator = -Lf_h - ca.mtimes(Lg_h, u_nom)
        denominator = ca.mtimes(Lg_h, Lg_h.T) + 1e-8
        lam = numerator / denominator

        u_safe = u_nom + lam * Lg_h.T
        dx_cl = f + ca.mtimes(g, u_safe)

        # --- General N-D Equations ---
        # 1. Boundary Condition
        eq1 = val(self.cbf, x)

        # 2. Tangency Condition (Projected Velocity = 0)
        n = nabla_h
        n_dot_n = ca.mtimes(n.T, n) + 1e-8
        v_dot_n = ca.mtimes(dx_cl.T, n)

        tangent_velocity_vec = dx_cl - (v_dot_n / n_dot_n) * n

        self.eqn = ca.vertcat(eq1, tangent_velocity_vec)

    def solve(self, n=100, center=None, span=10.0, num_samples=20000, plot=False):
        if self.eqn is None: self.build_equations()

        # 1. Handle Center
        if center is None:
            center_arr = np.zeros(self.dim)
        else:
            center_arr = np.array(center)
            # Pad or trim to match dim
            if center_arr.shape[0] != self.dim:
                new_c = np.zeros(self.dim)
                limit = min(self.dim, center_arr.shape[0])
                new_c[:limit] = center_arr[:limit]
                center_arr = new_c

        # 2. VISUALIZATION
        if plot:
            if self.dim == 2:
                self._plot_nullclines_2d(center_arr, span, 100)
            elif self.dim == 3:
                # Lower resolution for 3D to keep it fast
                self._plot_nullclines_3d(center_arr, span, 200)
            else:
                print(f"Plotting not supported for dim={self.dim}")

        # 3. MONTE CARLO SEARCH
        lower = center_arr - span
        upper = center_arr + span
        guesses = np.random.uniform(lower, upper, size=(num_samples, self.dim))

        eqn_norm = ca.norm_2(self.eqn)
        fn_error = ca.Function('fn_err', [self.x], [eqn_norm])

        candidates = []
        for i in range(num_samples):
            try:
                err = float(fn_error(guesses[i]))
                if not np.isnan(err):
                    candidates.append((err, guesses[i]))
            except:
                pass

        candidates.sort(key=lambda x: x[0])

        distinct_guesses = []
        for err, pos in candidates:
            if len(distinct_guesses) >= n: break
            if any(np.linalg.norm(pos - g) < 0.1 for g in distinct_guesses): continue
            distinct_guesses.append(pos)

        # 4. SOLVER
        fn_resid = ca.Function('fn_res', [self.x], [self.eqn])

        def scipy_func(x_vec):
            return np.array(fn_resid(x_vec)).flatten()

        solutions = []
        for guess in distinct_guesses:
            sol = root(scipy_func, guess, method='lm', tol=1e-9)
            if sol.success:
                root_val = sol.x
                if np.linalg.norm(scipy_func(root_val)) < 1e-6:
                    if not any(np.linalg.norm(root_val - s) < 1e-3 for s in solutions):
                        solutions.append(root_val)
        return solutions

    def _plot_nullclines_2d(self, center, span, res):
        # (Your existing 2D code, renamed for clarity)
        x_min, x_max = center[0] - span, center[0] + span
        y_min, y_max = center[1] - span, center[1] + span
        x = self.x

        # Reconstruct symbolics for plotting
        if hasattr(self.plant, 'get_f_sym'):
            f, g = self.plant.get_f_sym(x), self.plant.get_g_sym(x)
        else:
            f, g = ca.DM.zeros(self.dim), ca.DM.eye(self.dim)

        nabla_h = grad(self.cbf, x)
        u_nom = -grad(self.clf, x)

        Lf_h = ca.mtimes(nabla_h.T, f)
        Lg_h = ca.mtimes(nabla_h.T, g)
        lam = (-Lf_h - ca.mtimes(Lg_h, u_nom)) / (ca.mtimes(Lg_h, Lg_h.T) + 1e-8)
        dx_cl = f + ca.mtimes(g, u_nom + lam * Lg_h.T)

        cond_blue = val(self.cbf, x)
        cond_red = dx_cl[0] * nabla_h[1] - dx_cl[1] * nabla_h[0]  # 2D Cross product

        fn_plot = ca.Function('fn_plot', [x], [cond_blue, cond_red])

        X, Y = np.meshgrid(np.linspace(x_min, x_max, res), np.linspace(y_min, y_max, res))
        Z1, Z2 = np.zeros_like(X), np.zeros_like(X)

        for i in range(res):
            for j in range(res):
                r = fn_plot([X[i, j], Y[i, j]])
                Z1[i, j], Z2[i, j] = float(r[0]), float(r[1])

        plt.figure(figsize=(6, 6))
        plt.contour(X, Y, Z1, levels=[0], colors='blue')
        plt.contour(X, Y, Z2, levels=[0], colors='red')
        plt.title("2D Nullclines");
        plt.grid(True);
        plt.show()

    def _plot_nullclines_3d(self, center, span, res):
        """
        Plots interactive 3D Nullclines using Plotly.
        - Blue Surface: The Safe Set Boundary (h(x)=0)
        - Red Dots: Points where velocity aligns with the normal (Stuck set)
        """
        try:
            import plotly.graph_objects as go
        except ImportError:
            print("Error: Please install plotly (pip install plotly)")
            return

        print("Generating Interactive 3D Plot... (this may take a moment)")

        # 1. Setup Grid
        x_lin = np.linspace(center[0] - span, center[0] + span, res)
        y_lin = np.linspace(center[1] - span, center[1] + span, res)
        z_lin = np.linspace(center[2] - span, center[2] + span, res)
        X, Y, Z = np.meshgrid(x_lin, y_lin, z_lin, indexing='ij')

        # 2. Symbolic Evaluation
        x = self.x
        if hasattr(self.plant, 'get_f_sym'):
            f, g = self.plant.get_f_sym(x), self.plant.get_g_sym(x)
        else:
            f, g = ca.DM.zeros(3), ca.DM.eye(3)

        nabla_h = grad(self.cbf, x)
        u_nom = -grad(self.clf, x)

        Lf_h = ca.mtimes(nabla_h.T, f)
        Lg_h = ca.mtimes(nabla_h.T, g)
        lam = (-Lf_h - ca.mtimes(Lg_h, u_nom)) / (ca.mtimes(Lg_h, Lg_h.T) + 1e-8)
        dx_cl = f + ca.mtimes(g, u_nom + lam * Lg_h.T)

        cond_blue = val(self.cbf, x)  # Boundary Condition
        c = ca.cross(dx_cl, nabla_h)
        cond_red = ca.mtimes(c.T, c)  # Tangency Condition

        # Batched Evaluation
        fn_eval = ca.Function('fn_3d', [x], [cond_blue, cond_red])

        # Flatten grid for fast evaluation
        pts = np.vstack([X.ravel(), Y.ravel(), Z.ravel()]).T

        # Evaluate chunks to save memory if res is huge (optional, but good practice)
        # For typical usage, map is fine:
        res_map = fn_eval.map(pts.shape[0])
        results = res_map(pts.T)

        vals_h = np.array(results[0]).ravel()
        vals_align = np.array(results[1]).ravel()

        # 3. Filter "Red" Points (Tangency)
        # We only want to plot red dots where the condition is nearly zero
        mask_red = vals_align < 0.05
        red_pts = pts[mask_red]

        # Subsample red points if there are too many (prevents browser lag)
        if len(red_pts) > 3000:
            idx = np.random.choice(len(red_pts), 3000, replace=False)
            red_pts = red_pts[idx]

        # 4. Create Plotly Figure
        fig = go.Figure()

        # A. Blue Surface (h(x) = 0)
        # Plotly's Isosurface handles the marching cubes logic internally
        fig.add_trace(go.Isosurface(
            x=X.flatten(),
            y=Y.flatten(),
            z=Z.flatten(),
            value=vals_h,
            isomin=-0.1,  # Small range around 0
            isomax=0.1,
            surface_count=1,  # Draw only one shell
            opacity=0.3,
            colorscale='Blues',
            showscale=False,
            caps=dict(x_show=False, y_show=False, z_show=False),  # Hide the box edges
            name='Boundary h(x)=0'
        ))

        # B. Red Points (Tangency)
        fig.add_trace(go.Scatter3d(
            x=red_pts[:, 0],
            y=red_pts[:, 1],
            z=red_pts[:, 2],
            mode='markers',
            marker=dict(
                size=3,
                color='red',
                opacity=0.8
            ),
            name='Stuck Set (v || n)'
        ))

        # 5. Layout Formatting
        fig.update_layout(
            title="Interactive 3D Nullclines",
            scene=dict(
                xaxis_title='X',
                yaxis_title='Y',
                zaxis_title='Z',
                aspectmode='cube'  # Ensures the plot isn't stretched
            ),
            width=800,
            height=800
        )

        fig.show()

    def filter_non_repulsive_equilibria(self, roots):
        nr_solutions = []
        for x_star in roots:
            gV, gh = grad(self.clf, x_star), grad(self.cbf, x_star)
            dot_prod = np.dot(np.array(gV).T, np.array(gh))
            if dot_prod > 0:
                nr_solutions.append(x_star)
        return nr_solutions


    def find_splitting_pair(self, p1, p2, p3, origin=np.array([0, 0])):
        """
        Identifies which two points form a line such that the third point
        and the specified origin are on opposite sides of that line.
        """
        combinations = [
            (p1, p2, p3),
            (p2, p3, p1),
            (p1, p3, p2)
        ]

        for point_a, point_b, remaining_point in combinations:
            x1, y1 = point_a
            x2, y2 = point_b
            x3, y3 = remaining_point

            # Unpack the custom origin coordinates
            ox, oy = origin

            # Calculate coefficients for the line Ax + By + C = 0
            A = y1 - y2
            B = x2 - x1
            C = (x1 * y2) - (x2 * y1)

            # Evaluate the line equation for the Custom Origin
            # (Previously this was just C, which assumed ox=0, oy=0)
            val_origin = (A * ox) + (B * oy) + C

            # Evaluate the line equation for the Remaining Point
            val_point = (A * x3) + (B * y3) + C

            # Check if they have opposite signs
            if val_origin * val_point < 0:
                return (remaining_point,point_a, point_b)

        return None