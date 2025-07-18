# iLQR Slung-Load Quadrotor Simulation

This project implements an **iterative Linear Quadratic Regulator (iLQR)** controller for a quadrotor carrying a slung load, using a Simulink model. The simulation optimizes control inputs to follow a desired trajectory (including multiple waypoints) while minimizing a quadratic cost function.

## Project Structure

<br>├── `run_main_simulation.m` — Main entry point. Sets up and runs the iLQR optimization and simulation.
<br>├── `iLQCsimulation.slx` — Simulink model of the quadrotor with slung load, used for forward simulation.
<br>├── utils/ — General utilities plotting, data handling with `VisualizeTestData.m` not used
<br>├── helper/
<br>│ ├── `gendiscreteAB.m` — Discretize linearized system models
<br>│ ├── `gencostapprox.m` — Set up cost function approximations
<br>│ ├── `genlinmdl.m` — Generate linearized models
<br>│ ├── `stage_cost.m` and `terminal_cost.m` — Define cost at each step and at the end
<br>│ └── `update_policy.m` — Backward pass policy update for iLQR
<br>├── generated_fcns/ — Functions for model derivatives and cost terms, mostly generated symbolically
<br>│ ├── Model linearizations: `Mdl_Alin.m`, `Mdl_Blin.m` 
<br>│ ├── Discretization: `discretize_A_lin.m`, `discretize_B_lin.m`
<br>│ └── Cost terms


## How to Run

1. **Set up your mission and parameters in JSON files in init_param/**  
   If you change the mission (e.g., waypoints, payload), run the appropriate `genX.m` script to regenerate parameters and reference trajectories.

2. **Run the main simulation**  
```matlab
run('run_main_simulation.m')
```
This will:
- Initialize parameters and models
- Generate linearized and discretized system models
- Approximate the cost function
- Run the iLQR optimization loop (up to 50 iterations or until convergence)
- Apply the optimized control to the Simulink model


## Notes
- SLX model requires R2025a to open
- The iLQR algorithm optimizes a sequence of control inputs to minimize a quadratic cost over a time horizon.
- The cost function and reference trajectory must be defined to guide the quadrotor through all desired waypoints.
- If you change the mission (e.g., waypoints), rerun the relevant `genX.m` script.


### Not yet working

- **Multiple waypoints**: Reference trajectory may contain multiple waypoints, but cost function currently does not support or penalize for multiple waypoints. The iLQR controller optimizes for what is encoded in the cost.

---

## References

C. De Crousaz, F. Farshidian, and J. Buchli, “Aggressive optimal control for agile flight with a slung load,” in IROS 2014 Workshop on Machine Learning in Planning and Control of Robot Motion, 2014.

## License

This project is for academic and research purposes. See LICENSE file for details.