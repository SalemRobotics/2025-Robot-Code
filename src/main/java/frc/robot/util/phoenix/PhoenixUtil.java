// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

public class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command, String message) {
    StatusCode error = StatusCode.StatusCodeNotInitialized;

    for (int i = 0; i < maxAttempts; i++) {
      error = command.get();
      if (error.isOK()) return;
    }

    DriverStation.reportError(message + error.getDescription(), false);
  }

  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var status = command.get();
      if (status.isOK()) break;
    }
  }

  /**
   * Gets the average timestamp of many Phoenix 6 Status Signals.
   *
   * @return The average timestamp (in FPGA time)
   */
  public static double averageTimestamp(BaseStatusSignal firstSignal, BaseStatusSignal... signals) {
    double timestamp = currentToFPGATime(firstSignal.getTimestamp().getTime());

    for (int i = 0; i < signals.length; i++) {
      timestamp += currentToFPGATime(signals[i].getTimestamp().getTime());
    }

    return timestamp / (1 + signals.length);
  }

  public static double currentToFPGATime(double currentTime) {
    return (Timer.getFPGATimestamp() - Utils.getCurrentTimeSeconds()) + currentTime;
  }
}
