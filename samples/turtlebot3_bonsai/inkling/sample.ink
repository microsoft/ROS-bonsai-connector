inkling "2.0"

using Math
using Goal

# Distance to stay away from obstacles in meters
const MinDistFromObstacle = 0.05

# Velocity limitations
const MaxLinearVelocity = 0.22 # (m/s)
const MaxAngularVelocity = 2.84 # (rad/s)

# Define a type that represents the per-iteration state
# returned by the simulator.
type SimState {
    nearest_scan_range: number,
    nearest_scan_radians: number,
    twist_linear_velocity_x: number,
    twist_angular_velocity_z: number,
    odometry_position_x: number,
    odometry_position_y: number,
    goal_pose_x: number,
    goal_pose_y: number,
}

# Define a type that represents the per-iteration action
# accepted by the simulator.
type SimAction {
    # increment in 
    input_angular_velocity_z: number<-1*MaxAngularVelocity .. 1*MaxAngularVelocity>,
    input_linear_velocity_x: number<0 .. 1*MaxLinearVelocity>,
}

# Per-episode configuration that can be sent to the simulator.
# All iterations within an episode will use the same configuration.
type SimConfig {
    sample_range: number,
    goal_pose_x: number,
    goal_pose_y: number,
}

simulator Simulator(action: SimAction, config: SimConfig): SimState {
}   

# Define a concept graph with a single concept
graph (input: SimState): SimAction {
    concept AvoidObstacles(input): SimAction {
        curriculum {
            # The source of training for this concept is a simulator
            # that takes an action as an input and outputs a state.
            source simulator (Action: SimAction, Config: SimConfig): SimState {
                # Automatically launch the simulator with this
                # registered package name.
                package "Turtlebot3-Warehouse"
            }

            training {
                EpisodeIterationLimit: 3000
            }

            # The objective of training is expressed as a goal with two
            # subgoals: don't let the pole fall over, and don't move
            # the cart off the track.
            goal (State: SimState) {
                avoid `Collision`:
                    State.nearest_scan_range
                    in Goal.RangeBelow(MinDistFromObstacle)
                reach `Distance from Goal`:
                    Math.Hypot(State.odometry_position_x - State.goal_pose_x, State.odometry_position_y - State.goal_pose_y)
                    in Goal.Range(MinDistFromObstacle, 4*MinDistFromObstacle)
            }

            lesson `Default start` {
                # Specify the configuration parameters that are initialized
                #   at the start of each episode
                scenario {
                    sample_range: 60,
                    goal_pose_x: number<-1.5 .. 5>,
                    goal_pose_y: number<-8 .. 8>,
                }
            }
        }
    }
}
