import numpy as np
import typing as T
from ipywidgets import IntSlider, VBox, HBox, HTML
from IPython.display import display
import pandas as pd
import html

_LAST_OCCUPANCY_NAV_WIDGET = None

def snap_to_grid(state: np.ndarray, resolution: float) -> np.ndarray:
    """ Snap continuous coordinates to a finite-resolution grid

    Args:
        state (np.ndarray): a size-2 numpy array specifying the (x, y) coordinates
        resolution (float): resolution of the grid

    Returns:
        np.ndarray: state-vector snapped onto the specified grid
    """
    return resolution * np.round(state / resolution)


class StochOccupancyGrid2D(object):
    """ A stochastic occupancy grid derived from ROS2 map data

    The probability of grid cell being occupied is computed by the joint probability of
    any neighboring cell being occupied within some fixed window. For some examples of size-3
    occupancy windows,

    0.1 0.1 0.1
    0.1 0.1 0.1  ->  1 - (1 - 0.1)**9 ~= 0.61
    0.1 0.1 0.1

    0.0 0.1 0.0
    0.0 0.1 0.0  ->  1 - (1 - 0)**6 * (1 - 0.1)**3 ~= 0.27
    0.0 0.1 0.0

    The final occupancy probability is then converted to binary occupancy using a threshold
    """

    def __init__(self,
        resolution: float,
        size_xy: np.ndarray,
        origin_xy: np.ndarray,
        window_size: int,
        probs: T.Sequence[float],
        thresh: float = 0.5
    ) -> None:
        """
        Args:
            resolution (float): resolution of the map
            size_xy (np.ndarray): size-2 integer array representing map size
            origin_xy (np.ndarray): size-2 float array representing map origin coordinates
            window_size (int): window size for computing occupancy probabilities
            probs (T.Sequence[float]): map data
            thresh (float): threshold for final binarization of occupancy probabilites
        """
        self.resolution = resolution
        self.size_xy = size_xy
        self.origin_xy = origin_xy
        self.probs = np.reshape(np.asarray(probs), (size_xy[1], size_xy[0]))
        self.window_size = window_size
        self.thresh = thresh

    def state2grid(self, state_xy: np.ndarray) -> np.ndarray:
        """ convert real state coordinates to integer grid indices

        Args:
            state_xy (np.ndarray): real state coordinates (x, y)

        Returns:
            np.ndarray: quantized 2D grid indices (kx, ky)
        """
        state_snapped_xy = snap_to_grid(state_xy, self.resolution)
        grid_xy = ((state_snapped_xy - self.origin_xy) / self.resolution).astype(int)

        return grid_xy

    def grid2state(self, grid_xy: np.ndarray) -> np.ndarray:
        """ convert integer grid indices to real state coordinates

        Args:
            grid_xy (np.ndarray): integer grid indices (kx, ky)

        Returns:
            np.ndarray: real state coordinates (x, y)
        """
        return (grid_xy * self.resolution + self.origin_xy).astype(float)

    def is_free(self, state_xy: np.ndarray) -> bool:
        """ Check whether a state is free or occupied

        Args:
            state_xy (np.ndarray): size-2 state vectory of (x, y) coordinate

        Returns:
            bool: True if free, False if occupied
        """
        # combine the probabilities of each cell by assuming independence of each estimation
        grid_xy = self.state2grid(state_xy)

        half_size = int(round((self.window_size-1)/2))
        grid_xy_lower = np.maximum(0, grid_xy - half_size)
        grid_xy_upper = np.minimum(self.size_xy, grid_xy + half_size + 1)

        prob_window = self.probs[grid_xy_lower[1]:grid_xy_upper[1],
                                 grid_xy_lower[0]:grid_xy_upper[0]]
        p_total = np.prod(1. - np.maximum(prob_window / 100., 0.))

        return (1. - p_total) < self.thresh


def show_occupancy_navigator(occupancy_gt: np.ndarray,
                             init_center: T.Optional[T.Tuple[int,int]] = None,
                             window_size: T.Union[int, T.Tuple[int, int]] = (10, 10)) -> None:

    global _LAST_OCCUPANCY_NAV_WIDGET

    H, W = occupancy_gt.shape
    if init_center is None: 
        init_center = (W // 2, H // 2)

    if isinstance(window_size, int):
        window_h = int(window_size)
        window_w = int(window_size)
    else:
        if len(window_size) != 2:
            raise ValueError('window_size must be an int or a tuple of (height, width)')
        window_h = int(window_size[0])
        window_w = int(window_size[1])

    if window_h <= 0 or window_w <= 0:
        raise ValueError('window_size values must be positive')

    table_view = HTML()

    def _plot(cx: int, cy: int) -> None:
        half_h_low = window_h // 2
        half_h_high = window_h - half_h_low
        half_w_low = window_w // 2
        half_w_high = window_w - half_w_low

        x0 = max(0, cx - half_w_low)
        x1 = min(W, cx + half_w_high)
        y0 = max(0, cy - half_h_low)
        y1 = min(H, cy + half_h_high)

        window = occupancy_gt[y0:y1, x0:x1]
        display_df = pd.DataFrame(window).round(2)
        table_html = display_df.to_html(index=True, header=True, border=0)
        table_view.value = (
            "<div style='font-family:monospace; font-size:18px; margin:6px 0 10px 0;'>"
            + html.escape(f'center=({cx},{cy}) size={y1-y0}x{x1-x0}')
            + "</div>"
            + "<style>"
            + ".occ-grid table{border-collapse:collapse;}"
            + ".occ-grid th,.occ-grid td{border:1px solid #444;padding:6px 10px;text-align:center;font-size:18px;line-height:1.2;}"
            + ".occ-grid th{background:#f2f2f2;font-weight:700;}"
            + "</style>"
            + "<div class='occ-grid'>"
            + table_html
            + "</div>"
        )

    cx_slider = IntSlider(min=0, max=W-1, step=1, value=int(init_center[0]), description='cx')
    cy_slider = IntSlider(min=0, max=H-1, step=1, value=int(init_center[1]), description='cy')

    controls = HBox([cx_slider, cy_slider])
    nav_widget = VBox([controls, table_view])

    def _on_slider_change(change: T.Dict[str, T.Any]) -> None:
        if change.get('name') == 'value':
            _plot(int(cx_slider.value), int(cy_slider.value))

    cx_slider.observe(_on_slider_change, names='value')
    cy_slider.observe(_on_slider_change, names='value')

    # Keep only one navigator instance visible across repeated calls.
    if _LAST_OCCUPANCY_NAV_WIDGET is not None:
        try:
            _LAST_OCCUPANCY_NAV_WIDGET.close()
        except Exception:
            pass

    _LAST_OCCUPANCY_NAV_WIDGET = nav_widget
    display(nav_widget)
    _plot(int(cx_slider.value), int(cy_slider.value))
    