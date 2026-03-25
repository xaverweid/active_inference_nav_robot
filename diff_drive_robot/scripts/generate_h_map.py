import os

# --- Configuration ---
RESOLUTION = 0.05
ORIGIN_X = -7.5
ORIGIN_Y = -5.5
WIDTH_M = 15.0   
HEIGHT_M = 12.0  

# Convert to pixels
PX_WIDTH = int(WIDTH_M / RESOLUTION)
PX_HEIGHT = int(HEIGHT_M / RESOLUTION)

# Create a blank map (254 = Free space)
grid = [[254 for _ in range(PX_WIDTH)] for _ in range(PX_HEIGHT)]

# Extracted from h_map_world.sdf: (Center X, Center Y, Width X, Width Y)
# Note: A 3x0.5x3 box with yaw=0 has WidthX=3.0, WidthY=0.5
#       A 3x0.5x3 box with yaw=-90 has WidthX=0.5, WidthY=3.0
boxes = [
    # Left West Wall
    (-7.25, -3.40, 0.5, 3.0),
    (-7.25, -0.40, 0.5, 3.0),
    (-7.25,  2.60, 0.5, 3.0),
    (-7.25,  3.60, 0.5, 3.0),
    # Left North & South Caps
    (-5.75,  5.35, 3.0, 0.5),
    (-5.75, -5.15, 3.0, 0.5),
    # Left East Wall
    (-4.25,  2.85, 0.5, 3.0),
    (-4.25,  3.60, 0.5, 3.0),
    (-4.25, -3.40, 0.5, 3.0),
    (-4.25, -2.65, 0.5, 3.0),
    # Bridge (Center crossbar)
    (-3.00,  1.60, 3.0, 0.5),
    (-1.00,  1.60, 3.0, 0.5),
    (-3.00, -1.40, 3.0, 0.5),
    (-1.00, -1.40, 3.0, 0.5),
    # Right West Wall
    ( 0.25,  2.85, 0.5, 3.0),
    ( 0.25,  3.60, 0.5, 3.0),
    ( 0.25, -3.40, 0.5, 3.0),
    ( 0.25, -2.65, 0.5, 3.0),
    # Right North & South Caps
    ( 1.75,  5.35, 3.0, 0.5),
    ( 1.75, -5.15, 3.0, 0.5),
    # Right East Wall
    ( 3.25, -3.40, 0.5, 3.0),
    ( 3.25, -0.40, 0.5, 3.0),
    ( 3.25,  2.60, 0.5, 3.0),
    # "The Asymmetric Pocket" Top Right
    ( 3.50,  5.35, 3.0, 0.5),
    ( 3.25,  4.60, 0.5, 3.0),
    ( 5.00,  3.85, 3.0, 0.5),
    # Information Anchors / Objects
    ( 2.50,  4.00, 0.6, 0.6), # LitterBin
    (-6.25,  4.50, 0.4, 0.4)  # Fire Hydrant
]

def draw_box(cx, cy, wx, wy):
    min_x = cx - (wx / 2.0)
    max_x = cx + (wx / 2.0)
    min_y = cy - (wy / 2.0)
    max_y = cy + (wy / 2.0)

    # Convert to pixels
    px_min = max(0, int((min_x - ORIGIN_X) / RESOLUTION))
    px_max = min(PX_WIDTH - 1, int((max_x - ORIGIN_X) / RESOLUTION))
    
    # Y is inverted in images (0 is top, max is bottom)
    py_max = PX_HEIGHT - max(0, int((min_y - ORIGIN_Y) / RESOLUTION)) 
    py_min = PX_HEIGHT - min(PX_HEIGHT - 1, int((max_y - ORIGIN_Y) / RESOLUTION)) 

    # Draw black pixels (0 = Occupied)
    for y in range(py_min, py_max):
        for x in range(px_min, px_max):
            grid[y][x] = 0

# Apply all boxes
for box in boxes:
    draw_box(*box)

# Write to Binary PGM format (P5)
output_file = 'h_map_trial1.pgm'
with open(output_file, 'wb') as f:
    # Header
    header = f"P5\n{PX_WIDTH} {PX_HEIGHT}\n255\n"
    f.write(header.encode('ascii'))
    
    # Pixel Data
    for row in grid:
        f.write(bytearray(row))

print(f"✓ Successfully generated {output_file}")
print(f"  Width: {PX_WIDTH}px, Height: {PX_HEIGHT}px")