/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public final class Constants {

    //#region SPEEDS
    public static final double CLIMB_SPEED = 1;
    public static final double TELESCOPIC_SPEED = 1;
    public static final double SHOOTING_SPEED = 1;
    public static final double INTAKE_SPEED = 0.8;
    public static final double SHOOTER_BELT_SPEED = 1;
    public static final double STORAGE_BELT_SPEED = 0.8;
    //#endregion
    
    //#region PID
    public static final double drive_kP = 0.03, drive_kI = 0.00003, drive_kD = 0.006;

    public static final double shooter_kP = 0, shooter_kI=0, shooter_kD = 0;
    //#endregion
    
    //#region PORTS
    public static class Ports{
        
        public static class Sensors {
            
            public static final int DRIVE_ENC_LEFT_A = 0;
            public static final int DRIVE_ENC_LEFT_B = 1;
            public static final int DRIVE_ENC_RIGHT_A = 2;
            public static final int DRIVE_ENC_RIGHT_B = 3;
            public static final int SHOOTER_LIMIT_LOW = 6;
            public static final int SHOOTER_LIMIT_HIGH = 7;
            public static final int INTAKE_OPTIC = 8;
            public static final int STORAGE_OPTIC = 9; 
        }

        public static class Motors{
            
            //CAN
            public static final int DRIVE_LEFT_FRONT = 0;
            public static final int DRIVE_LEFT_BACK = 1;
            public static final int DRIVE_RIGHT_FRONT = 2;
            public static final int DRIVE_RIGHT_BACK = 3;

            public static final int INTAKE_ANGLE = 4;
            public static final int INTAKE_COLLECTOR = 5; 

            public static final int STORAGE_BELT = 6;

            public static final int SHOOTER_BELT = 7;
            public static final int SHOOTER_SHOOT = 8;

            //PWM
            public static final int SHOOTER_ANGLE = 0;
            
            public static final int CLIMB_TELESCOPIC = 1;
            public static final int CLIMB_BACK = 2;
            public static final int CLIMB_FRONT = 3;
    
        }
    }
    //#endregion
    
    //#region OI
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
    //#endregion
    
    //#region AUTONOMOUS
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
    //#endregion
}
