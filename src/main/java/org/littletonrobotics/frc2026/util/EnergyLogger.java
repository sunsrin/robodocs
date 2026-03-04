// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.util;

import edu.wpi.first.wpilibj.RobotController;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/** Utility class for logging code execution times. */
public class EnergyLogger {
  private static double totalCurrent = 0.0;
  private static double totalPower = 0.0;
  private static double totalEnergy = 0.0;

  private static Map<String, Double> subsytemCurrents = new HashMap<>();
  private static Map<String, Double> subsytemPowers = new HashMap<>();
  private static Map<String, Double> subsytemEnergies = new HashMap<>();

  private static BatteryIOInputsAutoLogged inputs = new BatteryIOInputsAutoLogged();

  public static void periodicBeforeScheduler() {
    inputs.batteryVoltage = RobotController.getBatteryVoltage();
    Logger.processInputs("Battery/BatteryVoltage", inputs);
  }

  public static void recordSubsytemEnergy(String key, double... amps) {
    double totalAmps = 0.0;
    for (int i = 0; i < amps.length; i++) {
      totalAmps += Math.abs(amps[i]);
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

    for (int i = keys.length - 2; i >= 0; i--) {
      String subkey = "";
      for (int j = 0; j <= i; j++) {
        subkey += keys[j];
        if (j < i) subkey += "/";
      }
      subsytemCurrents.put(subkey, subsytemCurrents.getOrDefault(subkey, 0.0) + totalAmps);
      subsytemPowers.put(subkey, subsytemPowers.getOrDefault(subkey, 0.0) + power);
      subsytemEnergies.put(subkey, subsytemEnergies.getOrDefault(subkey, 0.0) + energy);
    }
  }

  public static void recordOutputs() {
    Logger.recordOutput("EnergyUtil/Current", totalCurrent, "amps");
    Logger.recordOutput("EnergyUtil/Power", totalPower, "watts");
    Logger.recordOutput("EnergyUtil/Energy", joulesToWattHours(totalEnergy), "watt hours");

    for (var entry : subsytemCurrents.entrySet()) {
      Logger.recordOutput("EnergyUtil/Current/" + entry.getKey(), entry.getValue(), "amps");
      subsytemCurrents.put(entry.getKey(), 0.0);
    }
    for (var entry : subsytemPowers.entrySet()) {
      Logger.recordOutput("EnergyUtil/Power/" + entry.getKey(), entry.getValue(), "watts");
      subsytemPowers.put(entry.getKey(), 0.0);
    }
    for (var entry : subsytemEnergies.entrySet()) {
      Logger.recordOutput(
          "EnergyUtil/Energy/" + entry.getKey(), joulesToWattHours(entry.getValue()), "watt hours");
    }

    totalPower = 0.0;
    totalCurrent = 0.0;
  }

  private static double joulesToWattHours(double joules) {
    return joules / 3600.0;
  }

  @AutoLog
  public static class BatteryIOInputs {
    public double batteryVoltage = 0.0;
  }
}
