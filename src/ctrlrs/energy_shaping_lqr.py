#!/usr/bin/python3 -B

'''
LQR controller and energy shaping controller
and a state machine with pydrake

meant to be foundation to a simplified
furuta pendulum simulator and controller

based off of cartpole_lqr.py && rx_the_simple_pendulum.py

state vector: [x, theta, xdot, thetadot]
'''

import numpy as np
import argparse
import matplotlib.pyplot as plt
from copy import copy

from pydrake.all import (Saturation, SignalLogger, wrap_to, VectorSystem, AbstractValue,
                         LeafSystem, System, LinearSystem, Linearize, BasicVector, FramePoseVector)
from pydrake.common.containers import namedview
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
from pydrake.systems.controllers import LinearQuadraticRegulator


# Energy Shaping Controller
'''
 simply the energy shaping controller, with option to plot the
 closed-loop phase portrait.
 NOTE: system is not actually stable @ upright, only attractive
'''
class EnergyShapingCtrlr(VectorSystem):

    def __init__(self, cart_pole, x_star):
        # VectorSystem: A base class that specializes LeafSystem
        # for use with only zero or one vector input ports, and
        # only zero or one vector output ports.
        VectorSystem.__init__(self, 4, 1) # input size 4, output size 1
        self.cart_pole = cart_pole
        self.cart_pole_context = cart_pole.CreateDefaultContext()
        self.SetCartpoleParams(x_star)

    def SetCartpoleParams(self, x_star):
        # setting params
        self.cart_pole_context.get_mutable_continuous_state_vector().SetFromVector(x_star)
        # upright state w/ cart back at origin
        self.cart_pole_context.SetContinuousState([0, np.pi, 0, 0])
        # energy at upright state w/ cart back at origin
        self.desired_energy = self.cart_pole.EvalPotentialEnergy(self.cart_pole_context)

    # actual controller u (xdotdot in this case)
    def DoCalcVectorOutput(self, context, cart_pole_state, unused, output):
        # set context for current cartpole state
        self.cart_pole_context.SetContinuousState(cart_pole_state)
        # get cartpole params
        params = self.cart_pole_context.get_numeric_parameter(0)
        k_E = 0.1 # feedback controller gain for energy
        k_p = 0.1 # proportional gain to regulate cart
        k_d = 0.1 # derivative gain to regulate cart
        x = cart_pole_state[0]
        theta = cart_pole_state[1]
        xdot = cart_pole_state[2]
        thetadot = cart_pole_state[3]
        total_energy = (self.cart_pole.EvalPotentialEnergy(self.cart_pole_context) +
                        self.cart_pole.EvalKineticEnergy(self.cart_pole_context))
        # This is the energy shaping ctrlr for cartpole, which is essentially the
        # simple pendulum energy ctrlr (with cos term due to collocated PFL) and
        # a PD controller to regulate the cart
        output[:] = (k_E * thetadot * np.cos(theta) * (total_energy - self.desired_energy) -
                     k_p * x -
                     k_d * xdot)
                     #+
                     #params.damping() * thetadot) # damping term
        output[:] = output[:] * 50 # NOTE: gain to increase signal magnitude


# LQR controller class (to better organize and consolidate the weight matrices, Q, R)
# NOTE: might be better for future organization to make this into a standalone function
#       but I just want to experiment here
class BalancingLQRCtrlr():

    def __init__(self, cart_pole, cart_pole_context, input_i, output_i, Q=np.eye(4), R=np.eye(1),
                 x_star=[0., np.pi, 0., 0.]):
        self.Q = Q
        self.R = R

        self.linearized_cart_pole = Linearize(cart_pole, cart_pole_context,
                                              input_port_index=input_i, output_port_index=output_i)

    # lqr controller matrices (K ctrlr matrix, S Riccati eq matrix (used in optimal cost-to-go fxn))
    def get_LQR_matrices(self):
        (K, S) = LinearQuadraticRegulator(self.linearized_cart_pole.A(),
                                          self.linearized_cart_pole.B(), self.Q, self.R)
        return (K, S)

    def get_lin_matrices(self):
        A = self.linearized_cart_pole.A()
        B = self.linearized_cart_pole.B()
        C = self.linearized_cart_pole.C()
        D = self.linearized_cart_pole.D()
        return (A, B, C, D)


# Combined Energy Shaping (SwingUp) and LQR (Balance) Controller
# with a simple state machine
# NOTE: This is an extension of the VectorSystem class and thus does not
#       mix with abstract-valued ports like geometrically defined systems
class SwingUpAndBalanceController(VectorSystem):

    def __init__(self, cart_pole, cart_pole_context, input_i, ouput_i, Q, R, x_star):
        VectorSystem.__init__(self, 4, 1)

        self.cart_pole = cart_pole
        self.cart_pole_context = cart_pole.CreateDefaultContext()
        self.SetCartPoleParams(x_star)
        (self.K, self.S) = BalancingLQRCtrlr(cart_pole, cart_pole_context,
                                             input_i, ouput_i, Q, R, x_star).get_LQR_matrices()

        self.energy_shaping = EnergyShapingCtrlr(cart_pole, x_star)
        self.energy_shaping_context = self.energy_shaping.CreateDefaultContext()
        self.x_star = x_star

    def SetCartPoleParams(self, x_star):
        # setting params
        self.cart_pole_context.get_mutable_continuous_state_vector().SetFromVector(x_star)
        # upright state w/ cart back at origin
        self.cart_pole_context.SetContinuousState([0, np.pi, 0, 0])
        # energy at upright state w/ cart back at origin
        self.desired_energy = self.cart_pole.EvalPotentialEnergy(self.cart_pole_context)

    def DoCalcVectorOutput(self, context, cart_pole_state, unused, output):
        # xbar = x - x_star, i.e. xbar is the difference b/w current state and fixed point
        xbar = copy(cart_pole_state)
        # wrap_to(value: float, low: float, high: float) -> float
        #     For variables that are meant to be periodic, (e.g. over a 2π interval), wraps
        #     value into the interval [low, high). Precisely, wrap_to returns:
        #     value + k*(high-low) for the unique integer value k that lands the output
        #     in the desired interval. low and high must be finite, and low < high.
        xbar[1] = wrap_to(xbar[1], 0, 2.0*np.pi) - np.pi # theta

        # If x'Sx <= 2, then use LQR ctrlr. Cost-to-go J_star = x^T * S * x
        print(xbar.dot(self.S.dot(xbar)))
        if (xbar.dot(self.S.dot(xbar)) < 20.):
            print("LQR")
            output[:] = -self.K.dot(xbar) # u = -Kx
        else:
            self.energy_shaping.get_input_port(0).FixValue(self.energy_shaping_context,
                                                           cart_pole_state)
            output[:] = self.energy_shaping.get_output_port(0).Eval(self.energy_shaping_context)
