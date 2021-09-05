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
[ ] Write up and convert real world furuta readings into cartpole equivalents
        [x, theta, xdot, thetadot]_cartpole = [motor_pos, aux_enc, motor_vel, aux_vel]_furuta
[ ] Write up a passive cartpole simulator
[ ] Test if asyncio works with drake during moteus function calls
'''

# drake dynamic system that takes the furuta pendulum's sensor
# readings and translates them into the respective values and
# variables for cartpole
# responsible for:
#     * sending control signal to real furuta pendulum's moteus
#     * converting furuta pend's moteus readings to cartpole val
#     * taking input of drake ctrlr and giving it as torque cmds to moteus
#     * sending output state of cartpole to simulator plant
class FurutaPendulumToCartpole(LeafSystem):

    def __init__(self):
        LeafSystem.__init__(self)
        # TODO: VectorInput for control signal (1)
        #       VectorOutput for cartpole state (4)

        self.furuta = moteusMotor()
        self.cartpole_x = 0
        self.cartpole_xdot = 0
        self.cartpole_theta = 0
        self.cartpole_thetadot = 0

    async def CalcVecOutput(self):
        # NOTE: get control signal output, move moteus
        self.furuta.actuate('''u''')


# function to test out FurutaPendulumToCartpole's moteus member
async def main_furuta_read():
    test = FurutaPendulumToCartpole()
    await test.furuta.set_state_vector()
    result = await test.furuta.get_aux_enc()
    while True:
        await test.furuta.set_state_vector()
        print(test.furuta.state_vector[1])



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



#async def main():
async def main():
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
    #asyncio.run(main())
    asyncio.run(main_furuta_read())
