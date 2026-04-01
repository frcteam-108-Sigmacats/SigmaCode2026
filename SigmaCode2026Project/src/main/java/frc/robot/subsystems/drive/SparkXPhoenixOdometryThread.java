// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version includes an overload for Spark signals, which checks for errors to ensure that
 * all measurements in the sample are valid. All queues (Phoenix, Spark, generic, timestamp) are
 * gated behind a single validity check to prevent queue length desynchronization.
 */
public class SparkXPhoenixOdometryThread {
  // signalsLock must always be acquired AFTER odometryLock to prevent deadlock.
  // Consistent lock ordering everywhere: odometryLock -> signalsLock.
  private final Lock signalsLock = new ReentrantLock();
  private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
  private final List<Queue<Double>> phoenixQueues = new ArrayList<>();

  private final boolean isCANFD = new CANBus("PhoenixBus").isNetworkFD();

  // Use this constant consistently everywhere instead of hardcoding 250.
  private final double ODOMETRY_FREQUENCY = isCANFD ? 250 : 100;

  private final List<SparkBase> sparks = new ArrayList<>();
  private final List<DoubleSupplier> sparkSignals = new ArrayList<>();
  private final List<DoubleSupplier> genericSignals = new ArrayList<>();
  private final List<Queue<Double>> sparkQueues = new ArrayList<>();
  private final List<Queue<Double>> genericQueues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  private static SparkXPhoenixOdometryThread instance = null;
  private Notifier notifier = new Notifier(this::run);

  public static SparkXPhoenixOdometryThread getInstance() {
    if (instance == null) {
      instance = new SparkXPhoenixOdometryThread();
    }
    return instance;
  }

  private SparkXPhoenixOdometryThread() {
    notifier.setName("OdometryThread");
  }

  public void start() {
    if (timestampQueues.size() > 0) {
      notifier.startPeriodic(1.0 / DriveConstants.odometryFrequency);
    }
  }

  /** Registers a Spark signal to be read from the thread. */
  public Queue<Double> registerSignal(SparkBase spark, DoubleSupplier signal) {
    // FIX #5: Queue capacity raised to 20 to match all other queues and prevent
    // desync under thread stalls.
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      sparks.add(spark);
      sparkSignals.add(signal);
      sparkQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /** Registers a generic signal to be read from the thread. */
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      genericSignals.add(signal);
      genericQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /** Registers a Phoenix status signal to be read from the thread. */
  public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
    // Queue capacity raised to 20 to match all other queues.
    ArrayBlockingQueue<Double> queue = new ArrayBlockingQueue<>(20);
    // Always acquire odometryLock before signalsLock.
    Drive.odometryLock.lock();
    signalsLock.lock();
    try {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 1];
      System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
      newSignals[phoenixSignals.length] = signal;
      phoenixSignals = newSignals;
      phoenixQueues.add(queue);
    } finally {
      signalsLock.unlock();
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /** Returns a new queue that returns timestamp values for each sample. */
  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  private void run() {

    // Block OUTSIDE the lock for both bus types
    try {
      if (isCANFD && phoenixSignals.length > 0) {
        BaseStatusSignal.waitForAll(2.0 / ODOMETRY_FREQUENCY, phoenixSignals);
      } else {
        Thread.sleep((long) (1000.0 / ODOMETRY_FREQUENCY));
      }
    } catch (InterruptedException e) {
      e.printStackTrace();
      return;
    }
    // Always acquire odometryLock before signalsLock.
    Drive.odometryLock.lock();
    signalsLock.lock();
    try {
      // Use ODOMETRY_FREQUENCY constant instead of hardcoded 250.
      // Non-CAN FD path is the fallback for both no-Phoenix and non-FD cases,
      // preventing an uncapped spin loop when no Phoenix signals are registered on CAN FD.
      if (isCANFD && phoenixSignals.length > 0) {
        BaseStatusSignal.waitForAll(2.0 / ODOMETRY_FREQUENCY, phoenixSignals);
      }

      // Capture timestamp after signals are ready.
      double timestamp = RobotController.getFPGATime() / 1e6;

      // Read and validate Spark signals BEFORE writing anything to any queue.
      // Previously, Phoenix queues were written unconditionally before the Spark validity
      // check, causing Phoenix and Spark/timestamp queues to grow at different rates
      // whenever a Spark error occurred. This desynchronized queue lengths and caused
      // Drive.periodic() to pair module positions with wrong timestamps.
      double[] sparkValues = new double[sparkSignals.size()];
      boolean isValid = true;
      for (int i = 0; i < sparkSignals.size(); i++) {
        sparkValues[i] = sparkSignals.get(i).getAsDouble();
        if (sparks.get(i).getLastError() != REVLibError.kOk) {
          isValid = false;
        }
      }

      // Gate ALL queues (Phoenix, Spark, generic, timestamp) behind one validity check
      // so all queues always have the same number of samples.
      if (isValid) {
        for (int i = 0; i < phoenixSignals.length; i++) {
          phoenixQueues.get(i).offer(phoenixSignals[i].getValueAsDouble());
        }
        for (int i = 0; i < sparkSignals.size(); i++) {
          sparkQueues.get(i).offer(sparkValues[i]);
        }
        for (int i = 0; i < genericSignals.size(); i++) {
          genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
        }
        for (int i = 0; i < timestampQueues.size(); i++) {
          timestampQueues.get(i).offer(timestamp);
        }
      }
    } finally {
      signalsLock.unlock();
      Drive.odometryLock.unlock();
    }
  }
}

// // Copyright 2021-2025 FRC 6328
// // http://github.com/Mechanical-Advantage
// //
// // This program is free software; you can redistribute it and/or
// // modify it under the terms of the GNU General Public License
// // version 3 as published by the Free Software Foundation or
// // available in the root directory of this project.
// //
// // This program is distributed in the hope that it will be useful,
// // but WITHOUT ANY WARRANTY; without even the implied warranty of
// // MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// // GNU General Public License for more details.

// package frc.robot.subsystems.drive;

// import com.ctre.phoenix6.BaseStatusSignal;
// import com.ctre.phoenix6.CANBus;
// import com.ctre.phoenix6.StatusSignal;
// import com.revrobotics.REVLibError;
// import com.revrobotics.spark.SparkBase;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.wpilibj.Notifier;
// import edu.wpi.first.wpilibj.RobotController;
// import java.util.ArrayList;
// import java.util.List;
// import java.util.Queue;
// import java.util.concurrent.ArrayBlockingQueue;
// import java.util.concurrent.locks.Lock;
// import java.util.concurrent.locks.ReentrantLock;
// import java.util.function.DoubleSupplier;

// /**
//  * Provides an interface for asynchronously reading high-frequency measurements to a set of
// queues.
//  *
//  * <p>This version includes an overload for Spark signals, which checks for errors to ensure that
//  * all measurements in the sample are valid.
//  */
// public class SparkXPhoenixOdometryThread {
//   private final Lock signalsLock = new ReentrantLock();
//   private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
//   private final List<Queue<Double>> phoenixQueues = new ArrayList<>();

//   private final boolean isCANFD = new CANBus("PhoenixBus").isNetworkFD();

//   private final double ODOMETRY_FREQUENCY = isCANFD ? 250 : 100;

//   private final List<SparkBase> sparks = new ArrayList<>();
//   private final List<DoubleSupplier> sparkSignals = new ArrayList<>();
//   private final List<DoubleSupplier> genericSignals = new ArrayList<>();
//   private final List<Queue<Double>> sparkQueues = new ArrayList<>();
//   private final List<Queue<Double>> genericQueues = new ArrayList<>();
//   private final List<Queue<Double>> timestampQueues = new ArrayList<>();

//   private static SparkXPhoenixOdometryThread instance = null;
//   private Notifier notifier = new Notifier(this::run);

//   public static SparkXPhoenixOdometryThread getInstance() {
//     if (instance == null) {
//       instance = new SparkXPhoenixOdometryThread();
//     }
//     return instance;
//   }

//   private SparkXPhoenixOdometryThread() {
//     notifier.setName("OdometryThread");
//   }

//   public void start() {
//     if (timestampQueues.size() > 0) {
//       notifier.startPeriodic(1.0 / DriveConstants.odometryFrequency);
//     }
//   }

//   /** Registers a Spark signal to be read from the thread. */
//   public Queue<Double> registerSignal(SparkBase spark, DoubleSupplier signal) {
//     Queue<Double> queue = new ArrayBlockingQueue<>(20);
//     Drive.odometryLock.lock();
//     try {
//       sparks.add(spark);
//       sparkSignals.add(signal);
//       sparkQueues.add(queue);
//     } finally {
//       Drive.odometryLock.unlock();
//     }
//     return queue;
//   }

//   /** Registers a generic signal to be read from the thread. */
//   public Queue<Double> registerSignal(DoubleSupplier signal) {
//     Queue<Double> queue = new ArrayBlockingQueue<>(20);
//     Drive.odometryLock.lock();
//     try {
//       genericSignals.add(signal);
//       genericQueues.add(queue);
//     } finally {
//       Drive.odometryLock.unlock();
//     }
//     return queue;
//   }

//   public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
//     ArrayBlockingQueue<Double> queue = new ArrayBlockingQueue<>(10);
//     signalsLock.lock();
//     Drive.odometryLock.lock();
//     try {
//       BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 1];
//       System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
//       newSignals[phoenixSignals.length] = signal;
//       phoenixSignals = newSignals;
//       phoenixQueues.add(queue);
//     } finally {
//       signalsLock.unlock();
//       Drive.odometryLock.unlock();
//     }
//     return queue;
//   }

//   /** Returns a new queue that returns timestamp values for each sample. */
//   public Queue<Double> makeTimestampQueue() {
//     Queue<Double> queue = new ArrayBlockingQueue<>(20);
//     Drive.odometryLock.lock();
//     try {
//       timestampQueues.add(queue);
//     } finally {
//       Drive.odometryLock.unlock();
//     }
//     return queue;
//   }

//   private void run() {
//     // Save new data to queues
//     Drive.odometryLock.lock();
//     signalsLock.lock();
//     try {
//       if (isCANFD && phoenixSignals.length > 0) {
//         BaseStatusSignal.waitForAll(2.0 / 250, phoenixSignals);
//       } else {
//         // "waitForAll" does not support blocking on multiple signals with a bus
//         // that is not CAN FD, regardless of Pro licensing. No reasoning for this
//         // behavior is provided by the documentation.
//         Thread.sleep((long) (1000.0 / ODOMETRY_FREQUENCY));
//         if (phoenixSignals.length > 0) BaseStatusSignal.refreshAll(phoenixSignals);
//       }
//       // Get sample timestamp
//       double timestamp = RobotController.getFPGATime() / 1e6;

//       for (int i = 0; i < phoenixSignals.length; i++) {
//         phoenixQueues.get(i).offer(phoenixSignals[i].getValueAsDouble());
//       }

//       // Read Spark values, mark invalid in case of error
//       double[] sparkValues = new double[sparkSignals.size()];
//       boolean isValid = true;
//       for (int i = 0; i < sparkSignals.size(); i++) {
//         sparkValues[i] = sparkSignals.get(i).getAsDouble();
//         if (sparks.get(i).getLastError() != REVLibError.kOk) {
//           isValid = false;
//         }
//       }

//       // If valid, add values to queues
//       if (isValid) {
//         for (int i = 0; i < sparkSignals.size(); i++) {
//           sparkQueues.get(i).offer(sparkValues[i]);
//         }
//         for (int i = 0; i < genericSignals.size(); i++) {
//           genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
//         }
//         for (int i = 0; i < timestampQueues.size(); i++) {
//           timestampQueues.get(i).offer(timestamp);
//         }
//       }
//     } catch (InterruptedException e) {
//       e.printStackTrace();
//     } finally {
//       signalsLock.unlock();
//       Drive.odometryLock.unlock();
//     }
//   }
// }
