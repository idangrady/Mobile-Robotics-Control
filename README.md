# Mobile-Robotics-Control
TUe System and Control


## Progress
### week 3
- [x] Create beta functionality using trig functions, for a smarted and more adaptive obstacle identification
- [ ] Connect to created MAP and nodes
- [ ] Add goal-oriented obstacle avoidance functionality
- [ ] Use odometry data to estimate robot position and connect it to the decision process
- [ ] Finish global path planning and connect it to local path planning


## Setting up and Running the MRC Simulator

Follow these instructions to set up and run the MRC simulator on your system:
(for a more detailed explanation, navigate to 'explanation' in week1)
1. **Installation**: Make sure the MRC simulator is installed on your system. If you haven't installed it yet, please refer to the simulator's documentation for installation instructions.

2. **Building the Project**:
   - Navigate to the `build` directory in your local repository.
   - Delete all existing files and folders by running the command: `rm -r *`
   - Generate the necessary build files by running the command: `cmake ..`
   - Compile the project using the `make` command.

3. **Launching the Simulator**:
   - Open the MRC simulator by running the command: `mrc-sim`.
   - Launch the sim-rviz visualization tool using the command: `sim-rviz`.

4. **Running the Simulator**:
   - Navigate to the `bin` directory in your repository.
   - Run the simulator by executing the command: `./hello`.



## Website:
https://cstwiki.wtb.tue.nl/wiki/Mobile_Robot_Control_2023
