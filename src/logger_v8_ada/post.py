import matplotlib.pyplot as plt
import matplotlib.dates as mdates

import os
from datetime import datetime
#from dateutil import tz
from CoolProp.HumidAirProp import HAPropsSI
from CoolProp.CoolProp import PropsSI
import scipy as sp
import glob 
from scipy import optimize
from scipy import interpolate

colors = ["C1", "C2", "C3"]
C2K = 273.15
p_ref = 101325

def read_rotronic(path, first):#, last):
    with open(path, 'r', errors='replace') as ifile:
        lines = ifile.readlines()
    
    date = []
    T = []
    RH = []
    for line in lines[33:]:
#        print(line)
        parts = line.split()
#        print(parts)
        this_date = datetime.strptime(parts[0]+" "+ parts[1], '%Y-%m-%d %H:%M:%S')
#        print(this_date)
        if this_date < first:
            continue
#        elif this_date > last:
#            break
        
        
        date.append(this_date)
        RH.append(float(parts[2]))
        T.append(float(parts[3]))
#        CO2.append(float(parts[3]))
    
    return date, sp.array(T), sp.array(RH)

def read(path_start, first):#, last):
    
    # Get file paths
    log_paths = glob.glob(os.path.join(path_start,"*.LOG"))
    log_paths.sort()

    # init
#    date, heater, T_RTC, T, RH = [], [], [], [], []
    
    date, T_RTC, T_sht31, T_si7021, RH_sht31, RH_si7021, heater_sht31, heater_si7021, voltage = [], [], [], [], [], [],[],[],[]
    
    
    for log_path in log_paths:
        
        # Skip folders etc. non files
        if not os.path.isfile(log_path):
            continue
        
        # Read one day/file and add
        #datei, heateri, T_RTCi, Ti, RHi = read_one(log_path, first)        
        datei, T_RTCi, T_sht31i, T_si7021i, RH_sht31i, RH_si7021i, heater_sht31i, heater_si7021i, voltagei = read_one(log_path, first)        
        date += datei
        T_RTC += T_RTCi
        T_sht31 += T_sht31i
        T_si7021 += T_si7021i
        RH_sht31 += RH_sht31i
        RH_si7021 += RH_si7021i

        heater_sht31 += heater_sht31i
        heater_si7021 += heater_si7021i
        voltage += voltagei 

    return date, sp.array(T_RTC), sp.array(T_sht31), sp.array(T_si7021), sp.array(RH_sht31), sp.array(RH_si7021), sp.array(heater_sht31), sp.array(heater_si7021), sp.array(voltage) 




def read_one(path, first):
    with open(path, 'r') as ifile:
        lines = ifile.readlines()
    
    date = []
    T_RTC = []
    T_sht31 = []
    T_si7021 = []
    RH_sht31 = []
    RH_si7021 = []
    heater_sht31 = []
    heater_si7021 = []
    voltage = []
    
    for line in lines[5:]:
        parts = line.split(",")
        this_date = datetime.strptime(parts[0], '%Y/%m/%d %H:%M:%S')
        if this_date < first:
            continue
        
        date.append(this_date)
        T_RTC.append(float(parts[1]))
        T_sht31.append(float(parts[2]))
        T_si7021.append(float(parts[3]))
        RH_sht31.append(float(parts[4]))
        RH_si7021.append(float(parts[5]))
        heater_sht31.append(bool(parts[6]))
        heater_si7021.append(bool(parts[7]))
        voltage.append(float(parts[8]))
    
    
    
    n = len(date)
    assert(len(T_RTC) == n) #and len(RH) == n)
    
    return date, T_RTC, T_sht31, T_si7021, RH_sht31, RH_si7021, heater_sht31, heater_si7021, voltage


def get_ws(Ts, RHs):
#    assert(len(Ts) == len(RHs))
    ws = HAPropsSI('W','P',p_ref,'T',Ts+C2K,'R',RHs/100) * PropsSI('D','P',p_ref,'T',Ts+C2K,"air")
    return ws


#def my_get_ws(Ts, RHs):
#    assert(len(Ts) == len(RHs))
#    p_sat = 1000*0.61078*sp.exp(17.27*Ts / (Ts+237.3) )
#    ws = RHs*p_sat/(287.05*(Ts+C2K))
#    return ws

def plot(date, T_RTC, T_sht31, T_si7021, RH_sht31, RH_si7021, 
         heater_sht31, heater_si7021, voltage , w_sht31, w_si7021,path_start,
         date_ref=None,T_ref=None, RH_ref=None, w_ref=None):
    fig, axes = plt.subplots(4,figsize=(15,4*4))
    
    # sht31 RH 2%, 0-100% 0.2C 0..85C
    # si7021 RH 3% 0-80%  0.4C -10..85C
    
    sht31_T_er = 0.2
    si7021_T_er = 0.4
    sht31_RH_er = 2
    si7021_RH_er = 3
    
    ref_T_er = 0.3
    ref_RH_er = 3
    ref_name = "Rotronic CL11"
    
    alpha=0.3
    
    ######################################################################
    # Temperature
    ######################################################################
    ax = axes[0]
 

    ax.plot(date, T_sht31, "-", label="SHT31")
#    ax.fill_between(date, T_sht31-sht31_T_er, T_sht31+sht31_T_er, alpha=alpha)
    ax.plot(date, T_si7021, "-", label="SI7021")
#    ax.fill_between(date, T_si7021-si7021_T_er, T_si7021+si7021_T_er, alpha=alpha)

    if not(T_ref is None):
        ax.plot(date_ref, T_ref, "--k", label=ref_name)
#        ax.fill_between(date_ref, T_ref-ref_T_er, T_ref+ref_T_er, alpha=alpha)

  
#    ax.plot(date, T_RTC, ":", label="RTC")

    ax.set_title("Temperature")
    ax.set_ylabel('$T$ $(\mathrm{C})$')

    ######################################################################
    # RH
    ######################################################################
    ax = axes[1]
   

    ax.plot(date, RH_sht31, "-", label="SHT31")
#    ax.fill_between(date, RH_sht31-sht31_RH_er, RH_sht31+sht31_RH_er, alpha=alpha)
    ax.plot(date, RH_si7021, "-", label="SI7021")
#    ax.fill_between(date, RH_si7021-si7021_RH_er, RH_si7021+si7021_RH_er, alpha=alpha)

#    ax.plot(RH)

    if not(RH_ref is None):
        ax.plot(date_ref, RH_ref, "--k", label=ref_name)
#        ax.fill_between(date_ref, RH_ref-ref_RH_er, RH_ref+ref_RH_er, alpha=alpha)



    ax.set_title("Relative humidity")
    ax.set_ylabel('$RH$ ($-$)')
    
#    ax.set_ylim(None, 23)

   


    
#    ax.set_ylim(None, 23)


    ######################################################################
    # w
    ######################################################################



    ax = axes[2]
   
    ax.plot(date, w_sht31*1e3, "-", label="SHT31")
    ax.plot(date, w_si7021*1e3, "-", label="SI7021")
    
    
    if not(w_ref is None):
        ax.plot(date_ref, w_ref*1e3, "k--", label=ref_name)
    
    
#    if T_ref:
#        ax.plot(date, T_ref, "--", label="T ref")
#        ax.fill_between(date, T_ref-ref_T_er, T_ref+ref_T_er, alpha=alpha)
    
    
    
#    ax.plot(date, my_get_ws(T_si7021, RH_si7021), "-", label="my_SI7021")
    
    ax.set_title("Moisture content")
    ax.set_ylabel('$v$ ($\mathrm{g}$ $\mathrm{water/m^3}$)')
    
    
    

    # voltage
    ax = axes[3]
    ax.plot(date, voltage, "-k", label="battery voltage")
    ax.set_ylim(voltage[-10:].mean(), voltage[:100].mean())
    ax.set_title("Battery voltage")
    ax.set_ylabel('$U$ $(\mathrm{V})$')

#    def fit(x, a, b, c):
#        return a * sp.exp(-b * x) + c
#    def fit(x, a, b, c):
#        return a*x + b
#    def fit(x, a, b, c):
#        return a*x**2 + b*x + c
#    def fit(x, a, b, c):
#        return a*sp.log(b*x) + c
#   
#    
#    popt, pcov = optimize.curve_fit(fit, sp.arange(len(voltage)), voltage, p0=(0, 1,  4.3))
#    print("fit", popt)
#
#    ax.plot(date, fit(sp.arange(len(voltage)), *popt), 'g--',
#            label="log fit")
#            label="log fit" + str(tuple(popt)))
#             label='fit: a=%5.3f, b=%5.3f, c=%5.3f' % tuple(popt))



    # Prettyfy
    for ax in axes:
        ax.set_xlim(date[0], date[-1])
        
        ax.legend(frameon=False)
#        fig_date_format = '%H:%M'
#        fig_date_format = '%m/%d'
        
        ax.xaxis.set_major_locator(mdates.MonthLocator())
#        ax.xaxis.set_major_locator(mdates.DayLocator())
#        ax.xaxis.set_minor_locator(mdates.HourLocator())
        
        ax.xaxis.set_major_formatter(mdates.DateFormatter('%d/%m'))    
#        ax.xaxis.set_minor_formatter(mdates.DateFormatter('%H'))
        ax.grid(True)

    axes[-1].set_xlabel("date (d/m)")


#    fig.autofmt_xdate()
    fig.tight_layout()
    fig.savefig(os.path.join(path_start, "TRHw.pdf"))


#def offset(date, T_RTC, T_sht31, T_si7021, RH_sht31, RH_si7021, 
#             heater_sht31, heater_si7021, voltage , w_sht31, w_si7021, path_start,
#             date_ref, T_ref, RH_ref, w_ref):
#    
#    offset_T_sht31 = (T_sht31-T_ref).mean()
#    offset_T_si7021 = (T_si7021-T_ref).mean()
#    
#    offset_RH_sht31 = (RH_sht31-RH_ref).mean()
#    offset_RH_si7021 = (RH_si7021-RH_ref).mean()
#    
#    print("offset_T_sht31", offset_T_sht31)
#    print("offset_T_si7021", offset_T_si7021)
#    
#    print("offset_RH_sht31", offset_RH_sht31)
#    print("offset_RH_si7021", offset_RH_si7021)
    
    


#def run():
#    path_start = os.path.join("data", "201904_rotronic_vertailu")
#    first = datetime.strptime("2019/04/17", '%Y/%m/%d')
##    first = datetime.strptime("2019/04/24", '%Y/%m/%d')
##    last = datetime.strptime("2019/04/15", '%Y/%m/%d')
#    
#    ref_path = os.path.join("data", "201904_rotronic_vertailu",
#                            "ref","rotronicCL11_data.XLS")
#    
#    date_ref, T_ref, RH_ref= read_rotronic(ref_path, first)
#
#    # Read
#    print("reading...")
#    #date, heater, T_RTC, T, RH  = read(path_start, first)
#    date, T_RTC, T_sht31, T_si7021, RH_sht31, RH_si7021, heater_sht31, heater_si7021, voltage = read(path_start, first)
#    
#    # OFFSETS, "hand optimized"
##    T_sht31 -= 0.2
##    RH_sht31 -= 3.5
##    RH_si7021 -= 1.55
#    
#    
#    print("calculating w...")
#    w_sht31 = get_ws(T_sht31,RH_sht31)
#    w_si7021 = get_ws(T_si7021,RH_si7021)
#    w_ref = get_ws(T_ref,RH_ref)
#    print("plotting...")
#    # Plot
#    if len(date) > 0:
#        plot(date, T_RTC, T_sht31, T_si7021, RH_sht31, RH_si7021, 
#             heater_sht31, heater_si7021, voltage , w_sht31, w_si7021, path_start,
#             date_ref, T_ref, RH_ref, w_ref)
#    else:
#        print("No data")
#
#
#    # OFFSET
##    offset(date, T_RTC, T_sht31, T_si7021, RH_sht31, RH_si7021, 
##             heater_sht31, heater_si7021, voltage , w_sht31, w_si7021, path_start,
##             date_ref, T_ref, RH_ref, w_ref)



def run_long():
    path_start = os.path.join("data", "201907_rotronic_vertailu")
    
    
    logger_path = os.path.join(path_start,"logs")
#    logger_path = os.path.join(path_start,"logs_short")
    
    first = datetime.strptime("2019/04/01", '%Y/%m/%d')
#    first = datetime.strptime("2019/04/24", '%Y/%m/%d')
#    last = datetime.strptime("2019/04/15", '%Y/%m/%d')
    
    ref_path = os.path.join("data", "201907_rotronic_vertailu",
                            "ref","long.XLS")
    
    date_ref, T_ref, RH_ref= read_rotronic(ref_path, first)
    assert len(date_ref) > 0

    # Read
    print("reading...")
    #date, heater, T_RTC, T, RH  = read(path_start, first)
    date, T_RTC, T_sht31, T_si7021, RH_sht31, RH_si7021, heater_sht31, heater_si7021, voltage = read(logger_path, first)
    
    # OFFSETS, "hand optimized"
    
    
    ref_secs = [datei.timestamp() for datei in date_ref]
    secs = [datei.timestamp() for datei in date]
    # T_sht31_diff
    T_sht31_diff = (T_ref - interpolate.interp1d(secs, T_sht31)(ref_secs)).mean()
    T_sht31 += T_sht31_diff
    # T_si7021_diff
    T_si7021_diff = (T_ref - interpolate.interp1d(secs, T_si7021)(ref_secs)).mean()
    T_si7021 += T_si7021_diff
    # RH_sht31
    RH_sht31_diff = (RH_ref - interpolate.interp1d(secs, RH_sht31)(ref_secs)).mean()
    RH_sht31 += RH_sht31_diff
    #RH_si7021
    RH_si7021_diff = (RH_ref - interpolate.interp1d(secs, RH_si7021)(ref_secs)).mean()
    RH_si7021 += RH_si7021_diff
    
    print("T_sht31_diff", T_sht31_diff)
    print("T_si7021_diff", T_si7021_diff)
    print("RH_sht31_diff", RH_sht31_diff)
    print("RH_si7021_diff", RH_si7021_diff)
    
#    T_sht31 -= 0.2
##    T_si7021 -=
#    RH_sht31 -= 3.4
#    RH_si7021 -= 1.45
    
#    T_sht31 -= 0.1729412166666666
#    T_si7021 += 0.01724486825396826
#    RH_sht31 -= 2.604373087301587
#    RH_si7021 -= 1.474475598412698
    
    
    print("calculating w...")
    w_sht31 = get_ws(T_sht31,RH_sht31)
    w_si7021 = get_ws(T_si7021,RH_si7021)
    w_ref = get_ws(T_ref,RH_ref)
    print("plotting...")
    # Plot
    if len(date) > 0:
        plot(date, T_RTC, T_sht31, T_si7021, RH_sht31, RH_si7021, 
             heater_sht31, heater_si7021, voltage , w_sht31, w_si7021, path_start,
             date_ref, T_ref, RH_ref, w_ref)
    else:
        print("No data")


    # OFFSET
#    offset(date, T_RTC, T_sht31, T_si7021, RH_sht31, RH_si7021, 
#             heater_sht31, heater_si7021, voltage , w_sht31, w_si7021, path_start,
#             date_ref, T_ref, RH_ref, w_ref)



if __name__ == "__main__":
    
    run_long()
    
#    print(get_ws(sp.array([5.5,5.7,6.1]), sp.array([52,51,50]))*1000)
