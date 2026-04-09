// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.trashcompactor;

import org.littletonrobotics.junction.AutoLog;

public interface TrashCompactorIO {
  @AutoLog
  public static class TrashCompactorIOInputs {
    public boolean connected;
    public double positionRads;
    public double velocityRadsPerSec;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelsius;
  }

  public enum TrashCompactorIOOutputMode {
    BRAKE,
    COAST,
    RUN_VOLTS,
    RUN_AMPS,
    RUN_POSITION
  }

  public static class TrashCompactorIOOutputs {
    public TrashCompactorIOOutputMode mode = TrashCompactorIOOutputMode.BRAKE;
    public double appliedVolts = 0.0;
    public double appliedAmps = 0.0;

    public double position = 0.0;
    public double velocity = 0.0;

    public double kP = 0.0;
    public double kD = 0.0;

    public double feedforward = 0.0;
  }

  public default void updateInputs(TrashCompactorIOInputs inputs) {}

  public default void applyOutputs(TrashCompactorIOOutputs outputs) {}
}
