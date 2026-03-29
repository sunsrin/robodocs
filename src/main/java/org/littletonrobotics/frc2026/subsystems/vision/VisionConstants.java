// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import java.util.Optional;
import java.util.function.Function;
import lombok.Builder;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.frc2026.RobotState;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.frc2026.util.geometry.GeomUtil;

@ExtensionMethod({GeomUtil.class})
public class VisionConstants {
  public static final boolean alwaysRecord = false;

  public static final double ambiguityThreshold = 0.4;
  public static final double targetLogTimeSecs = 0.1;
  public static final double fieldBorderMargin = 0.5;
  public static final double xyStdDevCoefficient = 0.01;
  public static final double thetaStdDevCoefficient = 0.03;
  public static final double fuelDetectConfidenceThreshold = 0.0; // Enforced by Northstar
  public static final double robotDetectConfidenceThreshold = 0.0; // Enforced by Northstar

  private static LoggedTunableNumber[] cameraPitchFudgeDegrees =
      new LoggedTunableNumber[] {
        new LoggedTunableNumber("Vision/Camera0PitchFudgeDeg", 0.0),
        new LoggedTunableNumber("Vision/Camera1PitchFudgeDeg", 1.6),
        new LoggedTunableNumber("Vision/Camera2PitchFudgeDeg", 0.0),
        new LoggedTunableNumber("Vision/Camera3PitchFudgeDeg", 0.0)
      };

  private static int monoExposure = 1800;
  private static double monoGain = 15.0;
  private static double monoDenoise = 1.0;
  private static int colorExposure = 4500;
  private static double colorGain = 5.0;
  private static double cameraBalanceRed = 1.2;
  private static double cameraBalanceBlue = 1.2;

  public static CameraConfig[] cameras =
      switch (Constants.robot) {
        case DARWIN ->
            new CameraConfig[] {
              CameraConfig.builder()
                  .poseFunction(
                      (Double timestamp) -> {
                        return Optional.of(
                            new Pose3d(
                                Units.inchesToMeters(-10.343),
                                Units.inchesToMeters(-8.102),
                                Units.inchesToMeters(20.940),
                                new Rotation3d(
                                    0.0,
                                    Units.degreesToRadians(
                                        -22.5 + cameraPitchFudgeDegrees[0].get()),
                                    Units.degreesToRadians(175.0))));
                      })
                  .id("40552081")
                  .width(1800)
                  .height(1200)
                  .exposure(monoExposure)
                  .gain(monoGain)
                  .denoise(monoDenoise)
                  .stdDevFactor(1.0)
                  .fovRads(Units.degreesToRadians(75.0))
                  .build(),
              CameraConfig.builder()
                  .poseFunction(
                      (Double timestamp) -> {
                        if (RobotState.getInstance().isHopperExtended()) {
                          return Optional.of(
                              new Pose3d(
                                  Units.inchesToMeters(24.099),
                                  Units.inchesToMeters(8.947),
                                  Units.inchesToMeters(24.529),
                                  new Rotation3d(
                                      0.0,
                                      Units.degreesToRadians(
                                          20.070 + cameraPitchFudgeDegrees[1].get()),
                                      Units.degreesToRadians(-4.698))));
                        } else {
                          return Optional.empty();
                        }
                      })
                  .id("24736221")
                  .width(1280)
                  .height(960)
                  .exposure(colorExposure)
                  .gain(colorGain)
                  .stdDevFactor(1.0)
                  .fovRads(Units.degreesToRadians(90.0))
                  .cameraBalanceRed(cameraBalanceRed)
                  .cameraBalanceBlue(cameraBalanceBlue)
                  .build(),
              CameraConfig.builder()
                  .poseFunction(
                      (Double timestamp) -> {
                        return Optional.of(
                            new Pose3d(
                                Units.inchesToMeters(1.343),
                                Units.inchesToMeters(12.856),
                                Units.inchesToMeters(22.512),
                                new Rotation3d(
                                    0.0,
                                    Units.degreesToRadians(20.0 + cameraPitchFudgeDegrees[2].get()),
                                    Units.degreesToRadians(70.0))));
                      })
                  .id("24736167")
                  .width(1280)
                  .height(960)
                  .exposure(colorExposure)
                  .gain(colorGain)
                  .stdDevFactor(1.0)
                  .fovRads(Units.degreesToRadians(90.0))
                  .cameraBalanceRed(cameraBalanceRed)
                  .cameraBalanceBlue(cameraBalanceBlue)
                  .build(),
              CameraConfig.builder()
                  .poseFunction(
                      (Double timestamp) -> {
                        return Optional.of(
                            new Pose3d(
                                Units.inchesToMeters(-10.339),
                                Units.inchesToMeters(-10.169),
                                Units.inchesToMeters(23.261),
                                new Rotation3d(
                                    0.0,
                                    Units.degreesToRadians(20.0 + cameraPitchFudgeDegrees[3].get()),
                                    Units.degreesToRadians(-75.0))));
                      })
                  .id("24737133")
                  .width(1280)
                  .height(960)
                  .exposure(colorExposure)
                  .gain(colorGain)
                  .stdDevFactor(1.0)
                  .fovRads(Units.degreesToRadians(90.0))
                  .cameraBalanceRed(cameraBalanceRed)
                  .cameraBalanceBlue(cameraBalanceBlue)
                  .build()
            };
        case ALPHABOT ->
            new CameraConfig[] {
              CameraConfig.builder()
                  .poseFunction(
                      (Double timestamp) -> {
                        return Optional.of(
                            new Pose3d(
                                Units.inchesToMeters(-28.0 / 2.0 + 2.5),
                                Units.inchesToMeters(-28.0 / 2.0 + 2.75),
                                Units.inchesToMeters(18.75),
                                new Rotation3d(
                                    0.0,
                                    Units.degreesToRadians(-27.0),
                                    Units.degreesToRadians(-152.5))));
                      })
                  .id("40530395")
                  .width(1800)
                  .height(1200)
                  .exposure(monoExposure)
                  .gain(monoGain)
                  .denoise(monoDenoise)
                  .stdDevFactor(1.0)
                  .fovRads(Units.degreesToRadians(75.0))
                  .build(),
              CameraConfig.builder()
                  .poseFunction(
                      (Double timestamp) -> {
                        return Optional.of(
                            new Pose3d(
                                Units.inchesToMeters(-28.0 / 2.0 + 2.25),
                                Units.inchesToMeters(-28.0 / 2.0 + 2.75),
                                Units.inchesToMeters(18.0),
                                new Rotation3d(
                                    Units.degreesToRadians(11.0),
                                    Units.degreesToRadians(15.0),
                                    Units.degreesToRadians(-167.0))));
                      })
                  .id("24737133")
                  .width(1280)
                  .height(960)
                  .exposure(colorExposure)
                  .gain(colorGain)
                  .denoise(monoDenoise)
                  .stdDevFactor(1.0)
                  .fovRads(Units.degreesToRadians(90.0))
                  .cameraBalanceRed(cameraBalanceRed)
                  .cameraBalanceBlue(cameraBalanceBlue)
                  .build()
            };
        default -> new CameraConfig[] {};
      };

  @Builder
  public record CameraConfig(
      Function<Double, Optional<Pose3d>> poseFunction,
      String id,
      int width,
      int height,
      int autoExposure,
      int exposure,
      double gain,
      double denoise,
      double stdDevFactor,
      double fovRads,
      double cameraBalanceRed,
      double cameraBalanceBlue) {}

  private VisionConstants() {}
}
