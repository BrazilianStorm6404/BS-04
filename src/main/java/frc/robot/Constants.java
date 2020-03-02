/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public final class Constants {

    public static final double SHOOTER_ANGLE_SPEED = 0.5;
    public static final double CLIMB_SPEED = 1;
    public static final double TELESCOPIC_SPEED_RAISE = 0.9;
    public static final double TELESCOPIC_SPEED_LOWER = 0.2;
    public static final double SHOOTING_SPEED = 1;
    public static final double INTAKE_SPEED = 1;
    public static final double SHOOTER_BELT_SPEED = 1;
    public static final double STORAGE_BELT_MAX_SPEED = 0.8;
    public static final double STORAGE_MIN_PIXY_AREA = 100;
    //PID
    public static final double DRIVE_kP = 0.03, DRIVE_kI = 0.00003, DRIVE_kD = 0.006;

    public static final double SHOOTER_kP = 0, SHOOTER_kI=0, SHOOTER_kD = 0;

    
    public static class Ports{
        
        public static class Sensors {
            
            // Portas S0 e S1 foram invertidas de 9 (S1) e 8 (S0) para 8 (S1) e 9 (S0)
            public static final int STORAGE_OPTIC_S1 = 8;
            public static final int STORAGE_OPTIC_S0 = 9; 
            public static final int SHOOTER_LIMIT_HIGH = 7;
            public static final int SHOOTER_LIMIT_LOW = 6;
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
        public static final double MIN_DIST_TO_SHOOT = 0, MAX_DIST_TO_SHOOT = 0;
        
        
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
