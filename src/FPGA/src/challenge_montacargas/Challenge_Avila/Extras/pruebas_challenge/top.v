//=============================================================================
// File:        top.v
// Project:     TE3003B - Forklift Motor Control Challenge (Delivery 1)
// Target:      Tang Nano 20K
// Description: Top-level wrapper that exposes the physical pins used by the
//              FPGA to control a DC motor through an L298N H-bridge module.
//              Three logic outputs are driven: IN1 and IN2 (direction) and
//              ENA (PWM speed control).
//=============================================================================

module top (
    input  wire clk_27mhz,   // Tang Nano 20K on-board 27 MHz oscillator
    //input  wire rst_n,       // Active-low reset (S1 button)
    input  wire pb0,         // DIP switch position 0
    input  wire pb1,         // DIP switch position 1
    output wire l298_in1,    // To L298N IN1 (direction)
    output wire l298_in2,    // To L298N IN2 (direction)
    output wire l298_ena     // To L298N ENA (PWM)
);

    motor_pwm u_motor_pwm (
        .clk    (clk_27mhz),
        // .rst_n  (rst_n),
        .pb0    (pb0),
        .pb1    (pb1),
        .in1    (l298_in1),
        .in2    (l298_in2),
        .ena    (l298_ena)
    );

endmodule
