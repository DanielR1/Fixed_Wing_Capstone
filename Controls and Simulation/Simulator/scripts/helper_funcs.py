import numpy as np
import quaternion_helpers as qhelp

def compute_alpha_beta(state):
    """
    Computes angle of attack (alpha) and sideslip (beta) from an attitude quaternion
    and global velocity vector.
    
    Parameters:
    q        : array-like, attitude quaternion
    v_global : array-like, [Vx, Vy, Vz] in the global frame
    qhelp    : module/object containing the quat_to_R helper function
    
    Returns:
    alpha_deg, beta_deg : floats, angles in degrees
    """
    # Ensure velocity is a numpy array
    v_global = state[3:6]
    v_global = np.array(v_global)

    q = state[6:10]
    
    # Get the rotation matrix from your helper function
    R = qhelp.quat_to_R(q)
    
    v_body = R.T @ v_global  
    
    # Extract body velocity components
    u, v, w = v_body[0], v_body[1], v_body[2]
    
    # Calculate alpha (Angle of Attack)
    # Using atan2 to safely handle u = 0
    alpha_rad = np.arctan2(w, u)
    
    # Calculate beta (Sideslip Angle)
    # Using atan2 with the V_xz magnitude to avoid domain errors from rounding
    beta_rad = np.arctan2(v, np.sqrt(u**2 + w**2))
    
    # Convert to degrees for standard aerospace use
    alpha_deg = np.degrees(alpha_rad)
    beta_deg = np.degrees(beta_rad)
    
    return alpha_deg, beta_deg

# === Example Usage ===
# v_glob = [15.0, 2.0, 1.0] # 15 m/s forward, 2 m/s right, 1 m/s down
# q_att = [1.0, 0.0, 0.0, 0.0] # Assuming no rotation for this example
# alpha, beta = compute_alpha_beta(q_att, v_glob, qhelp)
# print(f"Alpha: {alpha:.2f} deg, Beta: {beta:.2f} deg")