import numpy as np
import csv
from scipy.optimize import least_squares

def load_data(file_path):
    """ Load magnetometer data from a CSV file using the built-in csv module. """
    data = []
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header
        for row in reader:
            data.append([float(row[0]), float(row[1]), float(row[2])])
    return np.array(data)

def ellipsoid_fit(data):
    """ Fit an ellipsoid to the data using least squares. """
    # Build the design matrix D
    D = np.column_stack((
        data[:, 0]**2,  # x^2
        data[:, 1]**2,  # y^2
        data[:, 2]**2,  # z^2
        2 * data[:, 0] * data[:, 1],  # 2xy
        2 * data[:, 0] * data[:, 2],  # 2xz
        2 * data[:, 1] * data[:, 2],  # 2yz
        2 * data[:, 0],  # 2x
        2 * data[:, 1],  # 2y
        2 * data[:, 2],  # 2z
        np.ones(data.shape[0])  # constant term
    ))
    
    # Solve the least-squares problem D * params = 0
    u, s, vh = np.linalg.svd(D, full_matrices=False)
    params = vh[-1, :]  # The last row of V contains the solution
    return params / params[-1]  # Normalize the parameters

def extract_calibration_params(params):
    """ Extract hard-iron offsets and soft-iron correction matrix from ellipsoid parameters. """
    A = np.array([[params[0], params[3], params[4]],
                  [params[3], params[1], params[5]],
                  [params[4], params[5], params[2]]])
    
    b = np.array([params[6], params[7], params[8]])
    
    # Calculate the hard-iron offsets (center of the ellipsoid)
    offset = -0.5 * np.linalg.inv(A).dot(b)
    
    # Calculate the soft-iron correction matrix
    eigvals, eigvecs = np.linalg.eig(A)
    soft_iron_matrix = eigvecs.dot(np.diag(np.sqrt(1 / eigvals)).dot(eigvecs.T))
    
    return offset, soft_iron_matrix

def normalize_soft_iron_matrix(A):
    """ Normalize the soft-iron correction matrix using the maximum eigenvalue. """
    eigvals, eigvecs = np.linalg.eig(A)
    max_eigval = np.max(eigvals)
    A_normalized = A / max_eigval
    return A_normalized

def main(file_path):
    """ Main function to fit ellipsoid and compute calibration parameters. """
    data = load_data(file_path)
    params = ellipsoid_fit(data)
    offset, soft_iron_matrix = extract_calibration_params(params)
    soft_normalized = normalize_soft_iron_matrix(soft_iron_matrix)

    print("Hard-Iron Offsets (B):")
    print(offset)
    print("\nSoft-Iron Correction Matrix (A):")
    print(soft_iron_matrix)
    print("\nSoft-Iron Correction (A), normalized:")
    print(soft_normalized)

# Run the script with your data file
if __name__ == "__main__":
    file_path = "magnetometer_data.csv"  # Change this to the path of your data file
    main(file_path)
