/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    //#region SENSORS
    /**
     * Enum for all sensors in the robot.
     */
    public static enum Sensors {
        
        POT_VERTICAL(0),
        LIMIT_IR_PRES(0),
        ULTRASONIC_BALL(0),
        LIMIT_OPTIC_LOW(0),
        LIMIT_OPTIC_HIGH(0),
        ENC_T1_WHEEL_A(1),
        ENC_T1_WHEEL_B(2),
        ENC_T2_WHEEL_A(0),
        ENC_T2_WHEEL_b(0);

        private int PortValue;

        Sensors(int PortValue) {
            this.PortValue = PortValue;
        }

        public int getPort() {
            return PortValue;
        }
    }
    //#endregion

    //#region MOTORS
    /**
     * Enum for all motors in the robot.
     */
    public static enum Motors {
        DRIVE_RIGHT_FRONT(-1,13), 
        DRIVE_RIGHT_BACK(-1,14),  
        DRIVE_LEFT_FRONT(-1,11),
        DRIVE_LEFT_BACK(-1,12),

        INTAKE(0,0), 
        SHOOTER(-1,18),  
        STORAGE(-1,17),
        HORIZONTAL_TURNER(0,0),
        VERTICAL_TURNER(0,0),

        TELESCOPIC(2,-1), 
        CLIMB_LEFT(7,-1),  
        CLIMB_RIGHT(8,-1);

        private int PortValuePWM;
        private int PortValueCAN;

        private static double climb_speed = 1;
        private static double telescopic_speed = 1;
        private static double shooting_speed = 0.7;
        private static double intake_speed = 1;

        Motors(int PortValuePWM,int PortValueCAN) {
            this.PortValuePWM = PortValuePWM;
            this.PortValueCAN = PortValueCAN;
        }

        public int getPortCAN() {
            return PortValueCAN;
        }

        public int getPortPWM() {
            return PortValuePWM;
        }
        
        public static double getClimbSpeed() {
            return climb_speed;
        }

        public static double getTelescopicSpeed() {
            return telescopic_speed;
        }

        public static double getShootingSpeed() {
            return shooting_speed;
        }

        public static double getIntakeSpeed() {
            return intake_speed;
        }

    }
    //#endregion

    //#region OI
    /**
     * Enum for the OI, including buttons and controllers.
     */
    public static enum OI_Map {
        CONTROLLER_1(0), 
        CONTROLLER_2(1), 
        BUTTON_A(1), 
        BUTTON_B(2), 
        BUTTON_X(3), 
        BUTTON_Y(4),
        X_AXIS(4),
        Y_AXIS(5);

        private int PortValue;

        OI_Map(int PortValue) {
            this.PortValue = PortValue;
        }

        public int getPort() {
            return PortValue;
        }

    }
    //#endregion
    
    //#region AUTONOMOUS
        //PID
        public static double kP = 0.03, kI = 0.00003, kD = 0.006;

        // DISTANCE TO SHOT
        public static double minDistToShot = 0, maxDistToShot = 0;
        // Constantes provindas do FRC-Characterization
        // Não foram definidas ainda.
        public static double ksVolts = 0.22;
        public static double kvVoltSecondsPerMeter = 1.98;
        public static double kaVoltSecondsSquaredPerMeter = 0.2;
        public static double kPDriveVel = 8.5;
        public static double kTrackwidthMeter = 0.69;
        public static double kMaxSpeedMetersPerSeconds = 3;
        public static double kMaxAccelerationMetersPerSecondSquared = 3;
        public static double kRamseteB = 2;
        public static double kRamseteZeta = 0.7;
    //#endregion

}
