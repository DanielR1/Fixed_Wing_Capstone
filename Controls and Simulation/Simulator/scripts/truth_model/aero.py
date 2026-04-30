import numpy as np
import config
import os

# Load XFLR5 aero data at module level
script_dir = os.path.dirname(os.path.abspath(__file__))
aero_data_path = os.path.join(script_dir, 'AERO_XFLR5.tsv')
aero_data = np.loadtxt(aero_data_path, delimiter='\t', skiprows=1)
alpha_table = aero_data[:, 0]  # First column is alpha
CL_table = aero_data[:, 2]     # Third column is CL
CD_table = aero_data[:, 5]     # Sixth column is CD (total)
CM_table = aero_data[:, 8]     # Ninth column is Cm

#Computes Aero Forces and Moments from current state and control deflections using database lookup
#for now - assume aero only affects longitudinally. i.e. no side force or roll/yaw moments from aero.
def getAeroForcesMoments(state, ctrl_in):
    alpha = state[13]
    vx = state[3]
    vy = state[4]
    vz = state[5]
    V = np.sqrt(vx**2+vy**2+vz**2)
    rho = config.AIR_DENSITY
    S = config.WING_AREA
    c_mac = config.MEAN_AERO_CHORD

    (CL_xflr, CD_xflr, CM_xflr) = extrapolate_CL_CD_CM(alpha)
    CDMax_fp = config.CD_MAX_FLAT
    CD0_fp = config.CD_0_FLAT
    #flat plate - used in sim for high AOA
    CL_fp = 0.5*CDMax_fp*np.sin(2*alpha)
    CD_fp = CDMax_fp*(np.sin(alpha))**2+CD0_fp
    CM_fp = 0
    #if a nice AOA - use only xflr data
    if (alpha >= -5) and (alpha<= 18):
        CL = CL_xflr
        CD = CD_xflr
        CM = CM_xflr
    elif (alpha < -5) and (alpha >= -10):
        w = (alpha+5)/(-5+10) #weight. zero means fully use xflr, 1 means fully use flat plate
        CL = CL_xflr*(1-w)+CL_fp
        CD = CD_xflr*(1-w)+CD_fp
        CM = CM_xflr*(1-w)+CM_fp
    elif (alpha >18) and (alpha <= 24.6):
        w = (alpha-18)/(24.6-18) #weight. zero means fully use xflr, 1 means fully use flat plate
        CL = CL_xflr*(1-w)+CL_fp
        CD = CD_xflr*(1-w)+CD_fp
        CM = CM_xflr*(1-w)+CM_fp
    else:
        CL = CL_fp
        CD = CD_fp
        CM = CM_fp
    
    Lift = 0.5*rho*V**2*S*CL
    Drag = 0.5*rho*V**2*S*CD
    Pitch_moment = 0.5*rho*V**2*S*c_mac*CM

    # Convert lift and drag to body frame forces
    # Lift is perpendicular to velocity, Drag is parallel to velocity
    # In body frame: FX (axial), FZ (normal)
    # alpha is angle between body x-axis and velocity vector
    FX = -Drag * np.cos(alpha) - Lift * np.sin(alpha)  # Axial force (along body x)
    FZ = -Drag * np.sin(alpha) + Lift * np.cos(alpha)  # Normal force (along body z)
    FY = 0  # No side force (longitudinal only)
    MY = Pitch_moment  # Pitching moment about body y-axis
    
    #Calulcating extra moments from control input

    (T1, T2, delta_1, delta_2) = ctrl_in
    cx = config.MOMENT_COEFF_X
    cy = config.MOMENT_COEFF_Y
    A2 = np.array([
            [cx*T1, -cx*T2],
            [-cy*T1, -cy*T2]
        ])
    # Return forces [FX, FY, FZ] and moments [MX, MY, MZ]
    return np.array([FX, FY, FZ, 0, MY, 0])


def extrapolate_CL_CD_CM(alpha):
    """
    Linearly interpolate CL, CD, CM from XFLR5 data based on alpha (angle of attack)
    
    Args:
        alpha: Angle of attack in degrees
    
    Returns:
        (CL, CD, CM): Tuple of lift coefficient, drag coefficient, and pitching moment coefficient
    """
    # Check if alpha is outside the table range
    if alpha < alpha_table[0] or alpha > alpha_table[-1]:
        return (0, 0, 0)
    
    # Use numpy's interpolation function for linear interpolation
    CL = np.interp(alpha, alpha_table, CL_table)
    CD = np.interp(alpha, alpha_table, CD_table)
    CM = np.interp(alpha, alpha_table, CM_table)
    
    return (CL, CD, CM)