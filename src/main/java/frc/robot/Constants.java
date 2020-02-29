/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public final class Constants {

    public static final double climb_speed = 1;
    public  static final double telescopic_speed = 1;
    public static final double shooting_speed = 1;
    public static final double intake_speed = 0.7;
    public static final double shooter_belt_speed = 1;
    public static final double storage_belt_max_speed = 0.8;
    
    
    //PID
    public static final double drive_kP = 0.03, drive_kI = 0.00003, drive_kD = 0.006;

    public static final double shooter_kP = 0, shooter_kI=0, shooter_kD = 0;

    
    public static class Ports{
        
        public static class Sensors {
            
            public static final int STORAGE_ULTRASONIC_PING = 0; 
            public static final int STORAGE_ULTRASONIC_ECHO = 1;
            public static final int SHOOTER_LIMIT_LOW = 7;
            public static final int SHOOTER_LIMIT_HIGH = 8;
            public static final int STORAGE_OPTIC = 9; 
            public static final int DRIVE_ENC_LEFT_A = 0;
            public static final int DRIVE_ENC_LEFT_B = 1;
            public static final int DRIVE_ENC_RIGHT_A = 2;
            public static final int DRIVE_ENC_RIGHT_B = 3;

        }

        public static class Motors{
            
            public static final int DRIVE_LEFT_FRONT = 1;
            public static final int DRIVE_LEFT_BACK = 2;
            public static final int DRIVE_RIGHT_FRONT = 3;
            public static final int DRIVE_RIGHT_BACK = 4;

            public static final int SHOOTER_ANGLE = 0;
            public static final int SHOOTER_BELT = 9;
            public static final int SHOOTER_SHOOT = 11;

            public static final int COLLECTOR_JOINT = 8;
            public static final int COLLECTOR_INTAKE = 9;
            public static final int STORAGE_BELT = 10;

            public static final int CLIMB_RIGHT = 2;
            public static final int CLIMB_LEFT = 3;
            public static final int CLIMB_TELESCOPIC = 1;
    
        }
    }
    
    public static class OI_Map {

        public static final int PILOT = 0;
        public static final int COPILOT = 1; 

        public static final int BUTTON_A = 1; 
        public static final int BUTTON_B = 2;
        public static final int BUTTON_X = 3;
        public static final int BUTTON_Y = 4;
        public static final int BUTTON_LEFT = 5;
        public static final int BUTTON_RIGHT = 6;
        
    }
    
    public static class Autonomous{
        
        // DISTANCE TO SHOT
        public static final double minDistToShot = 0, maxDistToShot = 0;
        
        
        // Constantes provindas do FRC-Characterization
        // Não foram definidas ainda.
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;
        public static final double kPDriveVel = 8.5;
        public static final double kTrackwidthMeter = 0.69;
        public static final double kMaxSpeedMetersPerSeconds = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

    }

}
