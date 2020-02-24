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
    
    
    //PID
    public static final double kP = 0.03, kI = 0.00003, kD = 0.006;
    
    public static class Ports{
        
        public static class Sensors {
            
            public static final int STORAGE_IR = 0; //VERIFICAR
            public static final int COLLECTOR_LIMIT = 0;
            public static final int STORAGE_ULTRASONIC_PING = 0; //***
            public static final int STORAGE_ULTRASONIC_ECHO = 0;
            public static final int SHOOTER_LIMIT_HIGH = 0;
            public static final int DRIVE_ENC_LEFT_A = 1;
            public static final int DRIVE_ENC_LEFT_B = 2;
            public static final int DRIVE_ENC_RIGTH_A = 3;
            public static final int DRIVE_ENC_RIGTH_B = 4;

        }

        public static class Motors{
            
            public static final int DRIVE_LEFT_FRONT = 10;
            public static final int DRIVE_LEFT_BACK = 20;
            public static final int DRIVE_RIGHT_FRONT = 30;
            public static final int DRIVE_RIGHT_BACK = 40;

            public static final int SHOOTER_ANGLE = 9; //VERIFICAR
            public static final int SHOOTER_BELT = 00000000000000; //***
            public static final int SHOOTER_SHOOT = 80;

            public static final int COLLECTOR_JOINT = 50;
            public static final int COLLECTOR_INTAKE = 60;
            public static final int STORAGE_BELT = 70;

            public static final int CLIMB_RIGHT = 2;
            public static final int CLIMB_LEFT = 3;
            public static final int CLIMB_TELESCOPIC = 1;
    
        }
    }
    
    public static class OI_Map {

        public static final int CONTROLLER_1 = 0;
        public static final int CONTROLLER_2 = 1; 
        public static final int BUTTON_A = 1; 
        public static final int BUTTON_B = 2;
        public static final int BUTTON_X = 3;
        public static final int BUTTON_Y = 4;
        public static final int X_AXIS = 4;
        public static final int Y_AXIS = 5;

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
