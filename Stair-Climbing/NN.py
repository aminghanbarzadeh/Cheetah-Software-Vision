import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

def draw_two_branch_model_diagram():
    # Create figure and axis
    fig, ax = plt.subplots(figsize=(12, 8))

    # Define block positions
    positions = {
        "Input Constant Features (6D)": (1, 6),
        "Dense Layer 1 (64 units, ReLU)": (3, 6),
        "Dense Layer 2 (32 units, ReLU)": (5, 6),
        "Input Time Sequence (2, 3D)": (1, 3),
        "LSTM Layer 1 (256 units)": (3, 3),
        "LSTM Layer 2 (512 units)": (5, 3),
        "LSTM Layer 3 (256 units)": (7, 3),
        "Concatenation Layer": (9, 5),
        "Dense Layer (50 units, ReLU, L2 Regularization)": (11, 5),
        "Dropout Layer (0.02)": (13, 5),
        "Output (2D: z-position and time sequence)": (15, 5)
    }

    # Draw rectangles and labels for each block
    for label, (x, y) in positions.items():
        ax.add_patch(Rectangle((x, y), 2, 1, edgecolor='black', facecolor='lightblue'))
        ax.text(x + 1, y + 0.5, label, horizontalalignment='center', verticalalignment='center', fontsize=10)

    # Drawing arrows
    arrow_params = dict(facecolor='black', shrink=0.05, width=0.5, headwidth=10)
    
    # Arrows for the constant input branch
    ax.annotate("", xy=(3, 6.5), xytext=(2, 6.5), arrowprops=arrow_params)  # Input Constant Features to Dense 64
    ax.annotate("", xy=(5, 6.5), xytext=(4, 6.5), arrowprops=arrow_params)  # Dense 64 to Dense 32
    ax.annotate("", xy=(7, 6.5), xytext=(6, 6.5), arrowprops=arrow_params)  # Dense 32 to Concatenation

    # Arrows for the time-sequence input branch
    ax.annotate("", xy=(3, 3.5), xytext=(2, 3.5), arrowprops=arrow_params)  # Input Time Sequence to LSTM 256
    ax.annotate("", xy=(5, 3.5), xytext=(4, 3.5), arrowprops=arrow_params)  # LSTM 256 to LSTM 512
    ax.annotate("", xy=(7, 3.5), xytext=(6, 3.5), arrowprops=arrow_params)  # LSTM 512 to LSTM 256
    ax.annotate("", xy=(9, 3.5), xytext=(8, 3.5), arrowprops=arrow_params)  # LSTM 256 to Concatenation

    # Arrows after concatenation
    ax.annotate("", xy=(11, 5.5), xytext=(10, 5.5), arrowprops=arrow_params)  # Concatenation to Dense 50
    ax.annotate("", xy=(13, 5.5), xytext=(12, 5.5), arrowprops=arrow_params)  # Dense 50 to Dropout
    ax.annotate("", xy=(15, 5.5), xytext=(14, 5.5), arrowprops=arrow_params)  # Dropout to Output

    # Set limits and remove axes
    ax.set_xlim(0, 17)
    ax.set_ylim(2, 7)
    ax.axis('off')

    # Title
    plt.title("Detailed Block Diagram of Two-Branch Neural Network", fontsize=14)
    
    # Display the diagram
    plt.show()

# Call the function to draw the diagram
draw_two_branch_model_diagram()
