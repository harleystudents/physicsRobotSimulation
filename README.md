# Simulated Arm Mechanism

## Project Overview
This project builds a reusable arm mechanism simulator on top of WPILib so it can plug directly into future robot codebases. The scope expanded during development to capture more realistic physics—treating braking as a force, integrating tightly with TalonFX-style controllers, and keeping the architecture open for additional mechanism types down the road.

## Goals
- Deliver a mechanism simulator that outputs position, velocity, and acceleration based on voltage or current inputs.
- Model torques, gravity, and braking with enough fidelity that closed-loop controllers behave like on real hardware.
- Keep the design modular so other simulated subsystems can share the same utilities.

## Development Process
- Planned a `utils` package that would host reusable mechanism simulators.
- Split the simulation into small, purpose-specific classes to keep each concept understandable.
- Made `ArmMechanism` the orchestration point, backed by helpers such as `HardLimits`, `MechanismState`, `DCMotor`, and `Torques`.
- Established a framework that accepts voltage/current inputs and produces position, velocity, and acceleration outputs, then layered the arm simulation on top.
- Implemented Euler integration to derive acceleration, velocity, and position from applied torques.
- Embedded PID and feedforward control inside the motor abstraction so the simulated TalonFX behaves like real hardware.

## Tools and Techniques
- **Chunking the project into modules** — kept tasks manageable and will absolutely be reused on future efforts.
- **Gemini** — assisted with repetitive edits such as reformatting; likely to remain part of the toolkit.
- **Asking questions proactively** — enabled quicker clarity when design or physics questions arose.
- **MathGPT** — clarified physics and mathematics needed for the simulation and should be valuable again.
- **ChatGPT** — provided general debugging support and hardware Q&A; useful for future development.

## Process Changes
- After confirming the physics model worked, unit handling and control loops were refined so that commanded motion behaved correctly (`4242c7a`, `e202252`).
- Gravity modeling and braking were revisited to better match reality, leading to explicit braking-force simulation and torque logging (`d0dcaef`, `afd0f42`).
- A visualization layer was introduced to surface torque data while debugging control behavior (`afd0f42`).
- Position control was rebuilt with more realistic constants and feedforward tuning before the final "it works" validation (`6834c3`, `3809380`, `e0c0667`).

## Outcomes
The simulator succeeds in modeling an articulated arm that can be driven by simulated TalonFX hardware. `ArmMechanism` computes motor, gravity, and braking torques, integrates them into a state update, and enforces mechanical limits, while `SimulatedDCMotor` mirrors on-robot PID, feedforward, and motion constraints to generate realistic voltage/current commands. The wrist subsystem wires these pieces into the robot project, feeding logs and visualization data so developers can command voltage or position targets just like on a physical robot. Overall, the project meets its learning and integration goals by delivering a realistic, extensible arm simulation ready for reuse in future codebases.
