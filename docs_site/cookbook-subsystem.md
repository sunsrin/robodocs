# How to Add a New Subsystem

Follow these steps to add a new AdvantageKit-compatible subsystem.

1. **Define the `SubsystemIO` interface:** Create the `Inputs` object and update methods.
2. **Create the `SubsystemIOSim`:** Write the physics simulation.
3. **Create the `SubsystemIOReal`:** Write the actual SparkMax/TalonFX code.
4. **Create the `Subsystem` class:** Handle the logic and math.
5. **Instantiate in `RobotContainer`:** Pass the correct IO implementation based on `Constants.Mode`.
