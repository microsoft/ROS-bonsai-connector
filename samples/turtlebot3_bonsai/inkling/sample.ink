inkling "2.0"

using Math
using Goal

# Velocity limitations
const MaxLinearVelocity = 0.22 # (m/s)
const MaxAngularVelocity = 2.84 # (rad/s)

# Define a type that represents the per-iteration state
# returned by the simulator.
type SimState {
    last_scan_range: number,
    nearest_scan_range: number,
    nearest_scan_radians: number,
    lidar_range_min: number
}

# Define a type that represents the per-iteration action
# accepted by the simulator.
type SimAction {
    # increment in
    input_angular_velocity_z: number<-1 * MaxAngularVelocity .. 1 * MaxAngularVelocity>,
    input_linear_velocity_x: number<0.1 .. 1 * MaxLinearVelocity>,
}

# Per-episode configuration that can be sent to the simulator.
# All iterations within an episode will use the same configuration.
type SimConfig {
    sample_range: number,
}

simulator Simulator(action: SimAction, config: SimConfig): SimState {
}
# The source of training for this concept is a simulator
# that takes an action as an input and outputs a state.
simulator turtlesim(Action: SimAction, Config: SimConfig): SimState {
    # Automatically launch the simulator with this
    # registered package name.
    package "turtlebot3_sim"
}

# Define a concept graph with a single concept
graph (input: SimState): SimAction {

    concept AvoidObstacles(input): SimAction {
        curriculum {

            source turtlesim

            training {
                # Limit the number of iterations per episode to 750. The default
                # is 1000, which makes it much tougher to succeed.
                EpisodeIterationLimit: 750
            }

            goal (State: SimState) {
                drive `Distance From Obstacle`:
                    State.nearest_scan_range
                    in Goal.RangeAbove(4*State.lidar_range_min)
            }

            lesson `Default start` {
                # Specify the configuration parameters that are initialized
                #   at the start of each episode
                scenario {
                    sample_range: 360,
                }
            }
        }
    }
}