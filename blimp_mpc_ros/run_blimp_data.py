from . BlimpSim import BlimpSim

from . BlimpPlotter import BlimpPlotter
from . BlimpLogger import BlimpLogger

import numpy as np
import time
import sys


def main(args=sys.argv):
    ## Parameters

    TITLE = "Plots"

    # Neither of these selections matter - these objects
    # just need to be created in order to load and plot
    # the simulation data from the file.

    Simulator = BlimpSim

    ## Plotting
    print("Plotting data from " + sys.argv[1])

    if len(sys.argv) < 2:
        print("Please run with data file name as first argument.")
        sys.exit(0)

    dT = 0.001  # will be overridden by data load anyways
    sim = Simulator(dT)
    
    plotter = BlimpPlotter()

    sim.load_data(sys.argv[1])
    plotter.init_plot(TITLE, True)

    plotter.update_plot(sim)
    plotter.block()
