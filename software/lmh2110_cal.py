from scipy.interpolate import interp1d
from lmh2110_cal_table import *

def v_to_dbm(v, freq):
    freq /= 1e6
    cal_dbms = sorted(cal_table.keys())
    x = []
    for p in cal_dbms:
        px, py = cal_table[p]
        x.append(interp1d(px, py, fill_value='extrapolate')(freq))
    return interp1d(x, cal_dbms, fill_value='extrapolate')(v)
