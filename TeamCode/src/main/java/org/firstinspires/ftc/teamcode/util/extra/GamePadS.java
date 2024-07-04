//package org.firstinspires.ftc.teamcode.util;
//
//import com.arcrobotics.ftclib.command.Command;
//import com.arcrobotics.ftclib.command.RunCommand;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.qualcomm.robotcore.hardware.Gamepad;
//
//import java.util.function.DoubleSupplier;
//
//
//public class GamePadS {
//
//    public static enum Button {
//        A(GamepadKeys.Button.A),
//        B(GamepadKeys.Button.B),
//        X(GamepadKeys.Button.X),
//        Y(GamepadKeys.Button.Y),
//        DPAD_UP(GamepadKeys.Button.DPAD_UP),
//        DPAD_DOWN(GamepadKeys.Button.DPAD_DOWN),
//        DPAD_LEFT(GamepadKeys.Button.DPAD_LEFT),
//        DPAD_RIGHT(GamepadKeys.Button.DPAD_RIGHT),
//        RIGHT_STICK_BUTTON(GamepadKeys.Button.RIGHT_STICK_BUTTON),
//        LEFT_STICK_BUTTON(GamepadKeys.Button.LEFT_STICK_BUTTON),
//        START(GamepadKeys.Button.START),
//        BACK(GamepadKeys.Button.BACK);
//
//        final GamepadKeys.Button button;
//        Button(GamepadKeys.Button button) {
//            this.button = button;
//
//        }
//
//
//    }
//    GamepadEx gamepadEx;
//    public GamePadS(Gamepad gamepad) {
//        gamepadEx = new GamepadEx(gamepad);
//
//    }
//
//    public void whileActiveContinuous(Button button, Command command) {
//        gamepadEx.getGamepadButton(button.button).whileActiveContinuous(command);
//    }
//
//    public void whenHeld(Button button, Command command) {
//        gamepadEx.getGamepadButton(button.button).whenHeld(command);
//    }
//
//    public void whenReleased(Button button, Command command) {
//        gamepadEx.getGamepadButton(button.button).whenReleased(command);
//    }
//
//    public void cancelWhenActive(Button button, Command command) {
//        gamepadEx.getGamepadButton(button.button).cancelWhenActive(command);
//    }
//
//    public void toggleWhenActive(Button button, Command commandOne, Command commandTwo, boolean interruptible) {
//        gamepadEx.getGamepadButton(button.button).toggleWhenActive(commandOne, commandTwo, interruptible);
//    }
//
//    public void toggleWhenActive(Button button, Command commandOne, Command commandTwo) {
//        gamepadEx.getGamepadButton(button.button).toggleWhenActive(commandOne, commandTwo);
//    }
//
//    public void toggleWhenActive(Button button, Command command) {
//        gamepadEx.getGamepadButton(button.button).toggleWhenActive(command);
//    }
//
//    public DoubleSupplier getRightX() {
//        return () -> gamepadEx.getRightX();
//    }
//
//    public DoubleSupplier getRightY() {
//        return () -> gamepadEx.getRightY();
//    }
//
//    public DoubleSupplier getLeftX() {
//        return getLeftX();
//    }
//
//    public DoubleSupplier getLeftY() {
//        return getLeftY();
//    }
//    //TODO: implement other methods
//}
