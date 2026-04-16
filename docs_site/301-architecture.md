# 301: System Design

This is the high-level architecture diagram of the robot code.

## Architecture Diagram

```mermaid
graph TD
    A[RobotContainer<br/>Instantiates Subsystems, binds Commands] --> B[Subsystem e.g., Drive.java<br/>Contains all math, kinematics, and periodic control logic.<br/>Does NOT touch hardware directly.]
    B --> C[IO Interface e.g., ModuleIO.java<br/>Defines inputs and outputs.<br/>AdvantageKit logs the Inputs here.]
    C --> D[Sim Implementation<br/>e.g., ModuleIOSim.java<br/>Simulates physics using WPILib]
    C --> E[Real Implementation<br/>e.g., ModuleIOTalonFX.java<br/>Talks to physical CAN devices]
```

## Key Rules
- **No Hardware in Subsystems:** Subsystems only interact with their `IO` interface.
- **Inputs are Logged:** All sensor data is read into an `AutoLogged` object.
- **Simulation First:** Every subsystem must have a Simulation IO implementation.
