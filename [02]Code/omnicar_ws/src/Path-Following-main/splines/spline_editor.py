import tkinter as tk
from tkinter import messagebox
import yaml
import math
import argparse
import numpy as np
from scipy import interpolate


class AdvancedPathEditor:
    def __init__(self, root, max_x=400, max_y=300, grid_size=1):
        self.root = root
        self.root.title("Advanced 2D Path Editor")

        self.width = 800
        self.height = 600
        self.max_x = max_x
        self.max_y = max_y

        # State variables
        self.points = []  # Stores {"x": float, "y": float}
        # Set initial zoom so the canvas shows [-max_x, max_x] x [-max_y, max_y]
        self.zoom = min(self.width / (2 * max_x), self.height / (2 * max_y))
        self.grid_size = grid_size  # Logical distance between grid lines
        self.dragging_index = None  # Keeps track of which point is being dragged
        self.selected_index = None   # Last point interacted with (for deletion)
        self.point_radius = 6  # Pixel radius for clicking/drawing

        # Setup Canvas
        self.canvas = tk.Canvas(root, width=self.width, height=self.height, bg="white")
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Event Bindings
        self.canvas.bind("<Button-1>", self.on_press)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)

        # Cross-platform scroll wheel bindings for Zoom
        self.canvas.bind("<MouseWheel>", self.on_zoom)  # Windows / macOS
        self.canvas.bind("<Button-4>", self.on_zoom)  # Linux scroll up
        self.canvas.bind("<Button-5>", self.on_zoom)  # Linux scroll down

        # Key bindings for deletion (bind to root so focus doesn't matter)
        self.root.bind("<BackSpace>", self.on_delete_point)
        self.root.bind("<Delete>", self.on_delete_point)

        # UI Controls
        self.btn_frame = tk.Frame(root)
        self.btn_frame.pack(pady=10)

        tk.Button(self.btn_frame, text="Clear", command=self.clear_canvas).pack(side=tk.LEFT, padx=10)
        tk.Button(self.btn_frame, text="Export to YAML", command=self.export_yaml).pack(side=tk.LEFT, padx=10)

        # Spline degree selector
        tk.Label(self.btn_frame, text="Spline degree (k):").pack(side=tk.LEFT, padx=(20, 4))
        self.spline_k = tk.IntVar(value=3)
        self.k_spinbox = tk.Spinbox(self.btn_frame, from_=1, to=9, width=3,
                                    textvariable=self.spline_k, command=self.redraw)
        self.k_spinbox.pack(side=tk.LEFT)

        # Initial Draw
        self.redraw()

    # --- Coordinate Transformations ---
    def logical_to_screen(self, lx, ly):
        """Converts math coordinates to canvas pixels."""
        sx = (self.width / 2) + (lx * self.zoom)
        sy = (self.height / 2) - (ly * self.zoom)
        return sx, sy

    def screen_to_logical(self, sx, sy):
        """Converts canvas pixels to math coordinates."""
        lx = (sx - (self.width / 2)) / self.zoom
        ly = ((self.height / 2) - sy) / self.zoom
        return lx, ly

    # --- Drawing Logic ---
    def redraw(self):
        self.canvas.delete("all")
        self.draw_grid()
        self.draw_path()

    def draw_grid(self):
        # Find the logical bounds of the current screen
        min_lx, max_ly = self.screen_to_logical(0, 0)
        max_lx, min_ly = self.screen_to_logical(self.width, self.height)

        # Calculate where to start drawing lines based on grid size
        start_x = int(min_lx // self.grid_size) * self.grid_size
        start_y = int(min_ly // self.grid_size) * self.grid_size

        # Fixed edge positions for labels (like matplotlib's axis borders)
        edge_margin = 2
        x_label_y = self.height - edge_margin  # bottom edge: x-axis labels
        y_label_x = edge_margin                # left edge: y-axis labels

        # Draw Vertical Grid Lines + x-axis labels at bottom edge
        x = start_x
        while x <= max_lx:
            sx, _ = self.logical_to_screen(x, 0)
            color = "gray" if x == 0 else "lightgray"
            width = 2 if x == 0 else 1
            self.canvas.create_line(sx, 0, sx, self.height, fill=color, width=width)
            # Tick and label at bottom edge
            self.canvas.create_line(sx, self.height - 6, sx, self.height, fill="gray", width=1)
            label = f"{x:.2f}".rstrip("0").rstrip(".")
            self.canvas.create_text(sx, x_label_y, text=label, fill="dimgray",
                                    font=("TkDefaultFont", 8), anchor=tk.S)
            x += self.grid_size

        # Draw Horizontal Grid Lines + y-axis labels at left edge
        y = start_y
        while y <= max_ly:
            _, sy = self.logical_to_screen(0, y)
            color = "gray" if y == 0 else "lightgray"
            width = 2 if y == 0 else 1
            self.canvas.create_line(0, sy, self.width, sy, fill=color, width=width)
            # Tick and label at left edge
            self.canvas.create_line(0, sy, 6, sy, fill="gray", width=1)
            label = f"{y:.2f}".rstrip("0").rstrip(".")
            self.canvas.create_text(y_label_x, sy, text=label, fill="dimgray",
                                    font=("TkDefaultFont", 8), anchor=tk.W)
            y += self.grid_size

    def draw_path(self):
        # Draw control points and coordinate labels
        for p in self.points:
            sx, sy = self.logical_to_screen(p["x"], p["y"])
            self.canvas.create_oval(sx - self.point_radius, sy - self.point_radius,
                                    sx + self.point_radius, sy + self.point_radius, fill="red")
            label = f"({p['x']:.2f}, {p['y']:.2f})"
            self.canvas.create_text(sx + 10, sy - 10, text=label, fill="black", anchor=tk.SW)

        # Draw B-spline if enough points exist
        tck = self._compute_spline()
        if tck is not None:
            gammas = np.linspace(0, 1, 2000, endpoint=True)
            path = interpolate.splev(gammas, tck)
            screen_pts = [self.logical_to_screen(px, py) for px, py in zip(path[0], path[1])]
            flat = [coord for pt in screen_pts for coord in pt]
            self.canvas.create_line(flat, fill="blue", width=2, smooth=False)
        else:
            # Fallback: straight line segments when too few points for the chosen k
            for i in range(len(self.points) - 1):
                sx1, sy1 = self.logical_to_screen(self.points[i]["x"], self.points[i]["y"])
                sx2, sy2 = self.logical_to_screen(self.points[i + 1]["x"], self.points[i + 1]["y"])
                self.canvas.create_line(sx1, sy1, sx2, sy2, fill="blue", width=2, dash=(4, 4))

    def _compute_spline(self):
        """Build and return a TCK spline from the current control points, or None if not enough points."""
        k = self.spline_k.get()
        if len(self.points) < k + 1:
            return None
        x = np.array([p["x"] for p in self.points])
        y = np.array([p["y"] for p in self.points])
        l = len(x)
        t = np.linspace(0, 1, l - k + 1, endpoint=True)
        t = np.append(np.zeros(k), t)
        t = np.append(t, np.ones(k))
        return [t, [x, y], k]

    # --- Interaction Events ---
    def on_press(self, event):
        lx, ly = self.screen_to_logical(event.x, event.y)

        # Check if we clicked on an existing point to drag it
        for i, p in enumerate(self.points):
            sx, sy = self.logical_to_screen(p["x"], p["y"])
            # Calculate distance using Pythagorean theorem
            distance = math.hypot(event.x - sx, event.y - sy)
            if distance <= self.point_radius * 2:  # Give a slightly larger hit box
                self.dragging_index = i
                self.selected_index = i
                return

        # If we didn't click a point, create a new one
        # Clamp to the configured logical bounds
        lx = max(-self.max_x, min(self.max_x, lx))
        ly = max(-self.max_y, min(self.max_y, ly))
        self.points.append({"x": lx, "y": ly})
        self.selected_index = len(self.points) - 1
        self.redraw()

    def on_drag(self, event):
        if self.dragging_index is not None:
            lx, ly = self.screen_to_logical(event.x, event.y)
            lx = max(-self.max_x, min(self.max_x, lx))
            ly = max(-self.max_y, min(self.max_y, ly))
            self.points[self.dragging_index] = {"x": lx, "y": ly}
            self.redraw()

    def on_release(self, event):
        self.dragging_index = None

    def on_delete_point(self, event=None):
        if not self.points:
            return
        idx = self.selected_index if self.selected_index is not None else len(self.points) - 1
        idx = max(0, min(idx, len(self.points) - 1))
        self.points.pop(idx)
        if self.points:
            self.selected_index = max(0, idx - 1)
        else:
            self.selected_index = None
        self.redraw()

    def on_zoom(self, event):
        # Determine scroll direction (Windows/Mac uses event.delta, Linux uses event.num)
        if event.num == 4 or event.delta > 0:
            self.zoom *= 1.1  # Zoom in
        elif event.num == 5 or event.delta < 0:
            self.zoom /= 1.1  # Zoom out

        self.redraw()

    # --- Utilities ---
    def clear_canvas(self):
        self.points.clear()
        self.redraw()

    def export_yaml(self):
        if not self.points:
            messagebox.showwarning("Empty Path", "No points to export!")
            return

        tck = self._compute_spline()
        if tck is None:
            k = self.spline_k.get()
            messagebox.showwarning(
                "Not enough points",
                f"Need at least {k + 1} points for a degree-{k} spline."
            )
            return

        t, (x_arr, y_arr), k = tck
        tck_data = {
            't': t.tolist(),
            'x': x_arr.tolist(),
            'y': y_arr.tolist(),
            'k': int(k),
        }

        with open("advanced_path.yaml", "w") as file:
            yaml.dump(tck_data, file, default_flow_style=False, sort_keys=False)

        messagebox.showinfo("Success", "Path exported to advanced_path.yaml successfully!")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Advanced 2D Path Editor")
    parser.add_argument("--max-x", type=float, default=10, help="Maximum logical x coordinate (default: 10)")
    parser.add_argument("--max-y", type=float, default=10, help="Maximum logical y coordinate (default: 10)")
    parser.add_argument("--grid-size", type=float, default=1, help="Logical distance between grid lines (default: 50)")
    args = parser.parse_args()

    root = tk.Tk()
    app = AdvancedPathEditor(root, max_x=args.max_x, max_y=args.max_y, grid_size=args.grid_size)
    root.mainloop()