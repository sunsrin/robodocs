# 201: The AdvantageKit Architecture

Our codebase uses AdvantageKit and the IO pattern.

## The IO Pattern (Hardware Abstraction Layer)
We separate logic from hardware so we can run the code without a physical robot.

Instead of a Subsystem directly importing a `CANSparkMax`, it imports an `IO` interface (e.g., `ModuleIO.java`). 
The IO interface has two implementations:
1. `ModuleIOSim`: Uses WPILib physics classes.
2. `ModuleIOTalonFX`: Talks to physical CAN devices.

AdvantageKit automatically logs all Inputs defined in the IO interface.
