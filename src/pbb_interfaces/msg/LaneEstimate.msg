pbb_interfaces/FrameState state
# Ground-plane polynomials y(x), x forward and y LEFT, metres.
# Ascending powers: c0 + c1*x + c2*x*x; invalid boundaries have empty arrays.
bool left_valid
bool right_valid
float64[] left_coefficients
float64[] right_coefficients
# Farthest currently observed paint/model support, metres. Zero = unknown.
float64 left_extent_m
float64 right_extent_m
# Observed double-yellow median, same x-forward/y-left coordinates.
float64[] centerline_coefficients
float64 centerline_extent_m
bool right_double_yellow
