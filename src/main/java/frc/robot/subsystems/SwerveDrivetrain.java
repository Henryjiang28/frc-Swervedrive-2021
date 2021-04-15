// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrivetrain extends SubsystemBase { 
  private static final double TRACKWIDTH = 0;
  private static final double WHEELBASE = 0;
  public static final double kMaxSpeed = Units.feetToMeters(13.6); // 13.6 feet per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    TalonFX frontLeftDriveMotor = new TalonFX(Constants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR);
    TalonFX frontLeftSteeringMotor = new TalonFX(Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR);
   

    TalonFX frontRightDriveMotor = new TalonFX(Constants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR);
    TalonFX frontRightSteeringMotor = new TalonFX(Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR);

    TalonFX backLeftDriveMotor = new TalonFX(Constants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR);
    TalonFX backLeftSteeringMotor = new TalonFX(Constants.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR);
    


    TalonFX backRightDriveMotor = new TalonFX(Constants.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR);
    TalonFX backRightSteeringMotor = new TalonFX(Constants.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR);

    AHRS gyro = new AHRS(I2C.Port.kMXP);
    // frontRightDriveMotor.setInverted(true);
    // backRightDriveMotor.setInverted(true);


  
  /**
   * TODO: These are example values and will need to be adjusted for your robot!
   * Modules are in the order of -
   * Front Left
   * Front Right
   * Back Left
   * Back Right
   * 
   * Positive x values represent moving toward the front of the robot whereas
   * positive y values represent moving toward the left of the robot
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#constructing-the-kinematics-object
   */
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics
    (
            new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0)
    );

   

  // TODO: Update these CAN device IDs to match  TalonFX + CANCoder device IDs
  // TODO: Update module offsets to match  CANCoder offsets

  private final  SwerveModuleMK3 frontLeftModule = new SwerveModuleMK3(frontLeftDriveMotor, frontLeftSteeringMotor, new CANCoder(0), Rotation2d.fromDegrees(0)); // Front Left
  private final  SwerveModuleMK3 frontRightModule = new SwerveModuleMK3(frontRightDriveMotor,frontRightSteeringMotor, new CANCoder(1),Rotation2d.fromDegrees(0)); // Front Right
  private final  SwerveModuleMK3 backLeftModule = new SwerveModuleMK3( backLeftDriveMotor,  backLeftSteeringMotor, new CANCoder(2), Rotation2d.fromDegrees(0));// Back Left
  private final  SwerveModuleMK3 backRightModule= new SwerveModuleMK3(backRightDriveMotor, backRightSteeringMotor, new CANCoder(3), Rotation2d.fromDegrees(0));  // Back Right

  public SwerveModuleMK3[] modules = new SwerveModuleMK3[] {frontLeftModule, frontRightModule ,backLeftModule ,backRightModule  };


  public SwerveDrivetrain() {
    gyro.reset(); 

  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] states =
      kinematics.toSwerveModuleStates(
        fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
          : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.normalizeWheelSpeeds(states, kMaxSpeed);
    for (int i = 0; i < states.length; i++) {
      SwerveModuleMK3 module = modules[i];
      SwerveModuleState state = states[i];
      module.setDesiredState(state);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Front Left Module Angle", frontLeftModule.getAngleOffset());
    SmartDashboard.putNumber("Front Right Module Angle", frontRightModule.getAngleOffset());
    SmartDashboard.putNumber("Back Left Module Angle", backLeftModule.getAngleOffset());
    SmartDashboard.putNumber("Back Right Module Angle", backRightModule.getAngleOffset());
    SmartDashboard.putNumber("gryo angle", gyro.getAngle()); 
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
