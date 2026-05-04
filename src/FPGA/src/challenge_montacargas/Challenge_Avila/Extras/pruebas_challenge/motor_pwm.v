//=============================================================================
// File:        motor_pwm.v
// Project:     TE3003B - Forklift Motor Control Challenge (Delivery 1)
// Target:      Tang Nano 20K (GW2AR-LV18QN88PC8/I7)
// Description: PWM + direction generator for a DC motor driven through an
//              L298N dual H-bridge module. The user selects one of three
//              states with two DIP-switch inputs (PB1, PB0):
//                - Stop    (motor de-energized, both DIP at 0 or both at 1)
//                - Forward (motor lifts the forklift carriage up)
//                - Reverse (motor lowers the forklift carriage down)
//
//              Direction (IN1, IN2) is driven combinationally directly from
//              the DIP switches, which guarantees fast and clean response.
//              The ENA pin carries a 50 Hz PWM signal whose duty cycle is set
//              high enough (90 %) to overcome the static friction of the
//              gearbox and reliably move the motor. The same count math from
//              the challenge specification (1.0 ms, 1.5 ms, 2.0 ms over a
//              20 ms period) is preserved as documentation, with the active
//              "ON" pulse equivalent to 18 ms (90 % of 20 ms = 486,000 counts
//              at 27 MHz).
//
// Clock:       27 MHz internal oscillator.
// Counts:      Period 20 ms  -> 0.020 s * 27e6 = 540,000 counts.
//              ON pulse 90 % -> 0.018 s * 27e6 = 486,000 counts.
//
// L298N truth table when ENA is high:
//   PB1 PB0 |  IN1 IN2 | Motor action
//    0   0  |   0   0  | Stop (coast)
//    0   1  |   1   0  | Forward (subir)
//    1   0  |   0   1  | Reverse (bajar)
//    1   1  |   1   1  | Stop (brake)
//
// Note: when PB1 == PB0 the motor is parked, so ENA is forced low to remove
// any residual current through the bridge.
//=============================================================================
module motor_pwm (
    input  wire clk,
    input  wire pb0,
    input  wire pb1,
    output wire in1,
    output wire in2,
    output reg  ena
);

    // ------------------------------------------------------------------
    // Direccion: el DIP switch va directo al puente H. PB1=PB0 mantiene
    // el motor parado (ambos a 0 = coast; ambos a 1 = brake).
    // ------------------------------------------------------------------
    assign in1 = pb0;
    assign in2 = pb1;

    // ------------------------------------------------------------------
    // PWM @ 50 Hz - periodo de 20 ms, duty 90%.
    //   Periodo: 20 ms -> 540,000 cuentas (a 27 MHz).
    //   Pulso ON: 18 ms -> 486,000 cuentas (90 % de duty).
    //   Se eligio 90 % para vencer la friccion del reductor; con duty
    //   menor el motor zumba pero no llega a girar.
    // ------------------------------------------------------------------
    localparam [19:0] PERIOD     = 20'd540_000;
    localparam [19:0] PULSE_HIGH = 20'd486_000;

    reg [19:0] counter = 20'd0;

    always @(posedge clk) begin
        if (counter >= PERIOD - 1)
            counter <= 20'd0;
        else
            counter <= counter + 1'b1;
    end

    // motor_active = 1 cuando uno y solo uno de los DIP esta en alto.
    // Asi se cancela el PWM cuando el motor esta detenido.
    wire motor_active = pb0 ^ pb1;

    always @(posedge clk) begin
        ena <= motor_active && (counter < PULSE_HIGH);
    end

endmodule