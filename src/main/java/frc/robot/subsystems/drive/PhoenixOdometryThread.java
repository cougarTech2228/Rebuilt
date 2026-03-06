// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. When using
 * a CANivore, the thread uses the "waitForAll" blocking method to enable more consistent sampling.
 * This also allows Phoenix Pro users to benefit from lower latency between devices using CANivore
 * time synchronization.
 */
public class PhoenixOdometryThread extends Thread {
  private final Lock signalsLock =
      new ReentrantLock(); // Prevents conflicts when registering signals
  private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
  private final List<DoubleSupplier> genericSignals = new ArrayList<>();

  // [2228 CHANGE] Replaced Queue<Double> with allocation-free DoubleQueue primitives
  private final List<DoubleQueue> phoenixQueues = new ArrayList<>();
  private final List<DoubleQueue> genericQueues = new ArrayList<>();
  private final List<DoubleQueue> timestampQueues = new ArrayList<>();

  private static boolean isCANFD = frc.robot.RobotContainer.kCanivore.isNetworkFD();
  private static PhoenixOdometryThread instance = null;

  public static PhoenixOdometryThread getInstance() {
    if (instance == null) {
      instance = new PhoenixOdometryThread();
    }
    return instance;
  }

  private PhoenixOdometryThread() {
    setName("PhoenixOdometryThread");
    setDaemon(true);
  }

  @Override
  public void start() {
    if (timestampQueues.size() > 0) {
      super.start();
    }
  }

  /** Registers a Phoenix signal to be read from the thread. */
  public DoubleQueue registerSignal(StatusSignal<Angle> signal) {
    DoubleQueue queue = new DoubleQueue(20);
    signalsLock.lock();
    Drive.odometryLock.lock();
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

  /** Registers a generic signal to be read from the thread. */
  public DoubleQueue registerSignal(DoubleSupplier signal) {
    DoubleQueue queue = new DoubleQueue(20);
    signalsLock.lock();
    Drive.odometryLock.lock();
    try {
      genericSignals.add(signal);
      genericQueues.add(queue);
    } finally {
      signalsLock.unlock();
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /** Returns a new queue that returns timestamp values for each sample. */
  public DoubleQueue makeTimestampQueue() {
    DoubleQueue queue = new DoubleQueue(20);
    Drive.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  @Override
  public void run() {
    while (true) {
      // Wait for updates from all signals
      signalsLock.lock();
      try {
        if (isCANFD && phoenixSignals.length > 0) {
          BaseStatusSignal.waitForAll(2.0 / Drive.ODOMETRY_FREQUENCY, phoenixSignals);
        } else {
          // "waitForAll" does not support blocking on multiple signals with a bus
          // that is not CAN FD, regardless of Pro licensing. No reasoning for this
          // behavior is provided by the documentation.
          Thread.sleep((long) (1000.0 / Drive.ODOMETRY_FREQUENCY));
          if (phoenixSignals.length > 0) BaseStatusSignal.refreshAll(phoenixSignals);
        }
      } catch (InterruptedException e) {
        e.printStackTrace();
      } finally {
        signalsLock.unlock();
      }

      // Save new data to queues
      Drive.odometryLock.lock();
      try {
        double timestamp = RobotController.getFPGATime() / 1e6;

        // [2228 CHANGE] Removed loop over all signals. We only need to check the latency of the
        // first signal, which avoids allocating dozens of Timestamp objects every loop.
        if (phoenixSignals.length > 0) {
          timestamp -= phoenixSignals[0].getTimestamp().getLatency();
        }

        // Add new samples to queues
        for (int i = 0; i < phoenixSignals.length; i++) {
          phoenixQueues.get(i).offer(phoenixSignals[i].getValueAsDouble());
        }
        for (int i = 0; i < genericSignals.size(); i++) {
          genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
        }
        for (int i = 0; i < timestampQueues.size(); i++) {
          timestampQueues.get(i).offer(timestamp);
        }
      } finally {
        Drive.odometryLock.unlock();
      }
    }
  }

  /** [2228 Change] A lightweight, allocation-free circular buffer for primitive doubles.
   * This completely eliminates autoboxing overhead and redundant ArrayBlockingQueue locking.
   */
  public static class DoubleQueue {
    private final double[] array;
    private int head = 0;
    private int tail = 0;
    private int size = 0;

    public DoubleQueue(int capacity) {
      array = new double[capacity];
    }

    public void offer(double value) {
      if (size == array.length) return; // Drop if full
      array[tail] = value;
      tail = (tail + 1) % array.length;
      size++;
    }

    public double poll() {
      if (size == 0) return 0.0;
      double val = array[head];
      head = (head + 1) % array.length;
      size--;
      return val;
    }

    public int size() {
      return size;
    }

    public void clear() {
      head = 0;
      tail = 0;
      size = 0;
    }
  }
}