// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.util;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/** Utility class for logging energy usage. */
public class EnergyLogger {
  private static double totalCurrent = 0.0;
  private static double totalPower = 0.0;
  private static double totalEnergy = 0.0;

  private static Map<String, Double> subsytemCurrents = new HashMap<>();
  private static Map<String, Double> subsytemPowers = new HashMap<>();
  private static Map<String, Double> subsytemEnergies = new HashMap<>();

  private static BatteryIOInputsAutoLogged inputs = new BatteryIOInputsAutoLogged();

  public static void updateBatteryVoltage() {
    inputs.batteryVoltage = RobotController.getBatteryVoltage();
    inputs.rioCurrent = RobotController.getInputCurrent();
    Logger.processInputs("EnergyLogger", inputs);
  }

  public static void recordEnergyUsage(String key, double... amps) {
    double totalAmps = 0.0;
    for (int i = 0; i < amps.length; i++) {
      totalAmps += amps[i];
    }

    double power = totalAmps * inputs.batteryVoltage;
    double energy = power * Constants.loopPeriodSecs;

    totalCurrent += totalAmps;
    totalPower += power;
    totalEnergy += energy;

    subsytemCurrents.put(key, totalAmps);
    subsytemPowers.put(key, power);
    subsytemEnergies.put(key, subsytemEnergies.getOrDefault(key, 0.0) + energy);

    String[] keys = key.split("/");
    if (keys.length < 2) {
      return;
    }

    String subkey = "";
    for (int i = 0; i < keys.length - 1; i++) {
      subkey += keys[i];
      if (i < keys.length - 2) {
        subkey += "/";
      }
      subsytemCurrents.put(subkey, subsytemCurrents.getOrDefault(subkey, 0.0) + totalAmps);
      subsytemPowers.put(subkey, subsytemPowers.getOrDefault(subkey, 0.0) + power);
      subsytemEnergies.put(subkey, subsytemEnergies.getOrDefault(subkey, 0.0) + energy);
    }
  }

  public static void recordOutputs() {
    recordEnergyUsage(
        "Controls/MacMini",
        (RobotBase.isReal() ? MacPowerMonitor.getCurrentPowerWatts() : 0.0)
            / (inputs.batteryVoltage > 0.0 ? inputs.batteryVoltage : 12.0)
            / 0.9);
    recordEnergyUsage("Controls/roboRIO", inputs.rioCurrent);
    recordEnergyUsage("Controls/CANcoders", 0.05 * 4);
    recordEnergyUsage("Controls/Pigeon", 0.04);
    recordEnergyUsage("Controls/CANivore", 0.03);
    recordEnergyUsage("Controls/Radio", 0.5);

    Logger.recordOutput("EnergyLogger/Current", totalCurrent, "amps");
    Logger.recordOutput("EnergyLogger/Power", totalPower, "watts");
    Logger.recordOutput("EnergyLogger/Energy", joulesToWattHours(totalEnergy), "watt hours");

    for (var entry : subsytemCurrents.entrySet()) {
      Logger.recordOutput("EnergyLogger/Current/" + entry.getKey(), entry.getValue(), "amps");
      subsytemCurrents.put(entry.getKey(), 0.0);
    }
    for (var entry : subsytemPowers.entrySet()) {
      Logger.recordOutput("EnergyLogger/Power/" + entry.getKey(), entry.getValue(), "watts");
      subsytemPowers.put(entry.getKey(), 0.0);
    }
    for (var entry : subsytemEnergies.entrySet()) {
      Logger.recordOutput(
          "EnergyLogger/Energy/" + entry.getKey(),
          joulesToWattHours(entry.getValue()),
          "watt hours");
    }

    totalPower = 0.0;
    totalCurrent = 0.0;
  }

  private static double joulesToWattHours(double joules) {
    return joules / 3600.0;
  }

  @AutoLog
  public static class BatteryIOInputs {
    public double batteryVoltage = 12.0;
    public double rioCurrent = 0.0;
  }
}
