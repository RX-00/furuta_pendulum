# furuta_pendulum

# TODOs
- get pydrake 2D lqr cartpole running by itself
- get pydrake 2D cartpole w/ energy shaping state machine + lqr
- start to transfer over to moteus conversion w/ physical furuta pendulum 

# NOTES
- 0 position for motor is with the pendulum shaft in front of the weight self
    - d rezero 0
- 0 encoder position for pendulum is upright, with +-0.5 being downward
    - in sys model it's np.pi is upright
