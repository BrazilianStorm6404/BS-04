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
        _HORIZONTAL(0),
        _VERTICAL(1),
        OS_STORAGE_1(2),
        OS_STORAGE_2(3),
        OS_STORAGE_3(4),
        ENC_STORAGE_WHEEL(5);

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
        DRIVE_RIGHT_FRONT(1,1), 
        DRIVE_RIGHT_BACK(0,0),  
        DRIVE_LEFT_FRONT(3,3),
        DRIVE_LEFT_BACK(2,2),
        INTAKE(4,4), 
        SHOOTER(5,5),  
        STORAGE(6,6),
        STORAGE_WHEEL(7,7),
        HORIZONTAL_TURNER(8,8),
        VERTICAL_TURNER(9,9),
        TELESCOPIC(10,10), 
        CLIMB_LEFT(11,11),  
        CLIMB_RIGHT(12,12);

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
        public static double kP = 0, kI = 0, kD = 0;

        // DISTANCE TO SHOT
        public static double minDistToShot = 0,maxDistToShot = 0;
    //#endregion

}
