module led(
    input Clock,
    output IO_voltage
);

parameter MAX  = 999;   // Período: 27MHz / 1000 = 27 kHz
parameter DUTY = 500;   // 50% duty cycle

reg [9:0] counter = 0;  // 10 bits es suficiente para MAX=999

always @(posedge Clock) begin
    if (counter < MAX)
        counter <= counter + 1'b1;
    else
        counter <= 0;
end

assign IO_voltage = ~(counter < DUTY);

endmodule
