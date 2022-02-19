// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {

  UsbCamera camera;
  // Mat mat;
  // CvSource outputStream;
  // CvSink cvSink;

  
  /** Creates a new CameraSubsystem. */
  public CameraSubsystem() {
    camera = CameraServer.startAutomaticCapture(0);
    camera.setResolution(640, 480);

    // cvSink = CameraServer.getVideo();

    // outputStream = CameraServer.putVideo("Stream", 640, 480);

    // mat = new Mat();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if (cvSink.grabFrame(mat) == 0) {
    //   outputStream.notifyError(cvSink.getError());
    // } else {

    //   Imgproc.rectangle(mat, new Point(10, 10), new Point(50, 50), new Scalar(0, 255, 0), 2);

    //   outputStream.putFrame(mat);
    // }
  }
}
