import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

# Function to draw labeled boxes (blocks) with arrows
def draw_box(ax, text, xy, boxcolor="lightblue", fontsize=10, size=(0.2, 0.05), arrow_to=None):
    box = FancyBboxPatch((xy[0] - size[0] / 2, xy[1] - size[1] / 2), size[0], size[1], 
                         boxstyle="round,pad=0.3", color=boxcolor, ec="black", lw=1.5)
    ax.add_patch(box)
    ax.text(xy[0], xy[1], text, ha="center", va="center", fontsize=fontsize, zorder=2)
    
    # Draw arrow to the next block if specified
    if arrow_to:
        ax.annotate("", xy=arrow_to, xytext=xy, arrowprops=dict(arrowstyle="->", lw=1.5))

# Create figure and axis
fig, ax = plt.subplots(figsize=(8, 6))
ax.set_xlim(0, 1)
ax.set_ylim(0, 1)
ax.set_xticks([])
ax.set_yticks([])

# Label and draw sections
# Input Section
ax.text(0.2, 0.95, "Input Branches", ha="center", fontsize=12, fontweight='bold')
draw_box(ax, "Constant Features\n(6 inputs)", (0.2, 0.9), boxcolor="lightgreen", size=(0.25, 0.1))
draw_box(ax, "Sequential Data\n(3 inputs)", (0.8, 0.9), boxcolor="lightgreen", size=(0.25, 0.1))

# Constant branch layers
ax.text(0.2, 0.75, "Dense Layers", ha="center", fontsize=12, fontweight='bold')
draw_box(ax, "Dense Layer (64)", (0.2, 0.7), arrow_to=(0.2, 0.75))
draw_box(ax, "Dense Layer (32)", (0.2, 0.6), arrow_to=(0.2, 0.65))

# Sequential branch LSTM layers
ax.text(0.8, 0.8, "LSTM Layers", ha="center", fontsize=12, fontweight='bold')
draw_box(ax, "LSTM Layer (256)", (0.8, 0.75), arrow_to=(0.8, 0.8))
draw_box(ax, "LSTM Layer (512)", (0.8, 0.65), arrow_to=(0.8, 0.7))
draw_box(ax, "LSTM Layer (256)", (0.8, 0.55), arrow_to=(0.8, 0.6))

# Concatenation
ax.text(0.5, 0.5, "Concatenation", ha="center", fontsize=12, fontweight='bold')
draw_box(ax, "Merge (Concatenation)", (0.5, 0.45), boxcolor="yellow", size=(0.4, 0.1), arrow_to=(0.2, 0.55))
draw_box(ax, "", (0.5, 0.45), arrow_to=(0.8, 0.55))  # Arrow from LSTM

# Dense layers after concatenation
ax.text(0.5, 0.38, "Final Dense Layer", ha="center", fontsize=12, fontweight='bold')
draw_box(ax, "Dense Layer (50)", (0.5, 0.35), arrow_to=(0.5, 0.4))

# Output Section
ax.text(0.5, 0.25, "Outputs", ha="center", fontsize=12, fontweight='bold')
draw_box(ax, "Foot Position\n(Output 1)", (0.3, 0.2), boxcolor="lightcoral", size=(0.3, 0.1), arrow_to=(0.5, 0.3))
draw_box(ax, "Center of Mass Velocity\n(Output 2)", (0.7, 0.2), boxcolor="lightcoral", size=(0.3, 0.1), arrow_to=(0.5, 0.3))

# Set the title and display the plot
ax.set_title("LSTM Network Architecture: Detailed Block Diagram", fontsize=14, fontweight='bold')
plt.show()
