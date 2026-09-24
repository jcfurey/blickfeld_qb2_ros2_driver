# Angles in radians; rates in Hz. Unlisted vendor settings are preserved.
float64 horizontal_field_of_view
float64 vertical_field_of_view
uint32 scanlines_up
uint32 scanlines_down
float64 angle_spacing
uint8 UP_DOWN=1
uint8 UP=2
uint8 SEPARATE=3
uint8 frame_mode
bool target_frame_rate_enabled
float64 target_frame_rate
bool foveation_enabled
float64 foveation_fraction
float64 foveation_density_factor
# Read-only observations, ignored by setters.
float64 actual_frame_rate
bool uniform
uint8 pulse_mode
