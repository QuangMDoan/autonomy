#!/usr/bin/env python3

# add import and helper functions here 
import numpy as np

if __name__ == "__main__":
    # code goes here 
    # Set seed and generate data
    np.random.seed(42)
    x = np.random.normal(size=(4, 10))  # shape: (4, 10)
    print("Input matrix x:\n", np.round(x, decimals=3))

    # Compute L2 squared distance matrix
    # Step 1: Compute the dot product matrix
    dot_product = x @ x.T  # shape: (4, 4)

    # Step 2: Compute the squared norms
    squared_norms = np.sum(x**2, axis=1, keepdims=True)  # shape: (4, 1)

    # Step 3: Apply the formula
    dists_squared = squared_norms + squared_norms.T - 2 * dot_product  # shape: (4, 4)

    # Optional: To ensure numerical stability (remove small negatives)
    dists_squared = np.maximum(dists_squared, 0)

    # Print the result
    print("L2 squared distance matrix:\n", dists_squared)
