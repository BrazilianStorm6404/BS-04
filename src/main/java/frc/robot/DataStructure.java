// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/** Add your docs here. */
public class DataStructure {
    public boolean girar;
    public double valor, forward, rot;
    public DataStructure(boolean Girar, double Valor, double Forward, double Rotation) {
        girar = Girar;
        valor = Valor;
        forward = Forward;
        rot = Rotation;
    }
}
