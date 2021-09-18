#!/usr/bin/python3 -B

'''
PyDrake Simulation of a 3D cartpole system
with a real-to-sim controller to update
simulation with a custom LeafSystem that
takes data from the real world system
(a furuta pendulum) and passes that data as
the dynamically updated state data for the
simulator plant

NOTE: built from ./cartpole_sim.py and
      ../sim/drake/jupy.../simple_legs.py
'''

import numpy as np
import argparse
from copy import copy
import asyncio
import math
import moteus
import numpy as np
import time

from moteus_src.moteusMotor import (QueryResolutionABS, moteusMotor)
from ctrlrs.energy_shaping_lqr import *

from pydrake.all import (SignalLogger, wrap_to, VectorSystem, LeafSystem,
                         Linearize, BasicVector)
from pydrake.common import FindResourceOrThrow
from pydrake.common import temp_directory
from pydrake.geometry import (DrakeVisualizer, SceneGraph)
from pydrake.lcm import DrakeLcm
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.systems.planar_scenegraph_visualizer import (
    ConnectPlanarSceneGraphVisualizer, PlanarSceneGraphVisualizer)


'''
TODO:
[x] Write up a passive cartpole sys
[x] Change cart_pole.sdf in drake to reflect furuta pendulum's values
[ ] Write up and convert sim cartpole state outputs into furuta pendulum cmds
        [x, theta, xdot, thetadot]_cartpole => [motor_pos, motor_vel]_furuta
[x] Test if asyncio works with drake during moteus function calls
'''

# -----------------------------------------------------------------------------------
# Function to test taking in the cart_pole MbP system's state_vector and converts
# the cart's position and velocity [x, xdot] data to the irl Furuta's moteus
class FurutaPendulumRealTest(VectorSystem):

    def __init__(self, cart_pole, cart_pole_context):
        VectorSystem.__init__(self, 4, 1)
        # VectorInput for cartpole_state
        self.cart_pole = cart_pole
        self.cart_pole_context = cart_pole.CreateDefaultContext()

        self.furuta = moteusMotor()
        # NOTE: these values need to be transformed
        #       from cartpole to furuta
        # e.g. cartpole x & xdot is linear while
        #      furuta x & xdot is rotational
        self.cartpole_x = 0
        self.cartpole_theta = 0
        self.cartpole_xdot = 0
        self.cartpole_thetadot = 0

    # NOTE: always calls this after creating an instance of this class
    async def init_state_vector(self):
        self.furuta.update_state_vector()

    async def DoCalcVecOutput(self, context, cart_pole_state, unused, output):
        # NOTE: get sim state vector and after conversion, move moteus
        print(cart_pole_state)


# function to test out 's moteus member
async def furuta_readings_test():
    test = FurutaPendulumRealTest()
    await test.init_state_vector()
    while True:
        await test.furuta.update_state_vector()
        #print("enc pos: ", test.furuta.state_vector[1])
        #print("enc vel: ", test.furuta.state_vector[3])

        # NOTE: test to determine if the moteusMotor.calc_enc_vel() is working with moteus' calc velocity for the base vs. our estimator (given the moteus' position encoder reading)
        print("real vel: ", test.furuta.state_vector[2])
        print("esti vel: ", test.furuta.calc_enc_vel(test.furuta.state_vector[0]))




# MAIN LOOP FUNCTIONS BELOW ---------------------------------------------------------

def arg_parse():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--target_realtime_rate", type=float, default=1.0,
        help="Desired rate relative to real time.  See documentation for "
             "Simulator::set_target_realtime_rate() for details.")
    parser.add_argument(
        "--simulation_time", type=float, default=10.0,
        help="Desired duration of the simulation in seconds.")
    parser.add_argument(
        "--time_step", type=float, default=0.,
        help="If greater than zero, the plant is modeled as a system with "
             "discrete updates and period equal to this time_step. "
             "If 0, the plant is modeled as a continuous system.")
    args = parser.parse_args()
    return args



# Original EnergyShapingLQR Controller in a pure simulation space
# Kept here as a point of reference and for testing purposes
async def main_sim():
    args = arg_parse()

    sdf_path = FindResourceOrThrow(
        "drake/examples/multibody/cart_pole/cart_pole.sdf")

    builder = DiagramBuilder()
    cart_pole = builder.AddSystem(MultibodyPlant(time_step=args.time_step))
    scene_graph = builder.AddSystem(SceneGraph()) # visualization & collision checking tool
    cart_pole.RegisterAsSourceForSceneGraph(scene_graph)
    Parser(plant=cart_pole).AddModelFromFile(sdf_path)

    # LQR weights
    Q = np.eye(4)
    R = np.eye(1)
    # fixed (unstable) equilibrium point
    x_star = [0., np.pi, 0., 0.]

    # users must call Finalize() after making any additions to the multibody plant and
    # before using this class in the Systems framework, e.g. diagram = builder.Build()
    cart_pole.Finalize()
    assert cart_pole.geometry_source_is_registered()

    # wire up scene_graph and cart_pole geometry
    builder.Connect(
        scene_graph.get_query_output_port(),
        cart_pole.get_geometry_query_input_port())
    builder.Connect(
        cart_pole.get_geometry_poses_output_port(),
        scene_graph.get_source_pose_port(cart_pole.get_source_id()))

    # hookup //tools:drake_visualizer
    DrakeVisualizer.AddToBuilder(builder=builder,
                                 scene_graph=scene_graph)

    # cartpole actuation (u) input port
    input_i = cart_pole.get_actuation_input_port().get_index()
    # cartpole state (x) output port
    output_i = cart_pole.get_state_output_port().get_index()

    # set the ctrlr included cart_pole context
    cart_pole_context = cart_pole.CreateDefaultContext()
    # set the fixed point to linearize around in lqr
    cart_pole_context.get_mutable_continuous_state_vector().SetFromVector(x_star)

    cart_pole.get_actuation_input_port().FixValue(cart_pole_context, [0])

    '''
    Controller code setup & wiring up happens here
    '''

    controller = builder.AddSystem(SwingUpAndBalanceController(cart_pole, cart_pole_context,
                                                               input_i, output_i,
                                                               Q, R, x_star))
    # NOTE: need to use MultibodyPlant.get_state_output_port() for connecting to controllers!!
    builder.Connect(cart_pole.get_state_output_port(), controller.get_input_port(0))
    #builder.Connect(controller.get_output_port(0), saturation.get_input_port(0))
    builder.Connect(controller.get_output_port(0), cart_pole.get_actuation_input_port())

    # A discrete sink block which logs its input to memory (not thread safe).
    # This data is then retrievable (e.g. after a simulation) via a handful
    # of accessor methods
    logger = builder.AddSystem(SignalLogger(4))
    builder.Connect(cart_pole.get_state_output_port(), logger.get_input_port(0))
    '''
    ----------------------------------------------
    '''


    diagram = builder.Build() # done defining & hooking up the system
    diagram_context = diagram.CreateDefaultContext()

    # instantiate a simulator
    simulator = Simulator(diagram, diagram_context)
    simulator.set_publish_every_time_step(False) # speed up sim
    simulator.set_target_realtime_rate(args.target_realtime_rate)

    # sim context, reset initial time & state
    sim_context = simulator.get_mutable_context()
    sim_context.SetTime(0.)
    sim_context.SetContinuousState([0.5, 0.2, 0, 0.1])

    # run sim until simulator.AdvanceTo(n) seconds
    simulator.Initialize()
    simulator.AdvanceTo(args.simulation_time)


    exit(0)



if __name__ == "__main__":
    #asyncio.run(main_sim())
    asyncio.run(furuta_readings_test())
