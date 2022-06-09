# FRC-Swervedrive-2021

The implementation of Swerve drive in the drivetrain subsystem folder.
Swerve drive control allows the robot to navigate, not by turning the wheels oppisite each other but by moving the wheels themselves. The wheels can drive forward and backward but the wheel assembly is turned by another motor and are tracked by encoders.The SwerveDriveKinematics class converts the ChassisSpeeds object to several SwerveModuleState objects, which contains velocities and angles for each swerve module of a swerve drive robot. This allows the robot to go in a certain direction and speed but with each modules at different states. 

Two Falcon500 motors are used on each side of drivetrain to allow turning and moving at the same time. Our team uses SwerveModuleMK3 package accompanied with TalonFx motor controllers and Cancoders. 
