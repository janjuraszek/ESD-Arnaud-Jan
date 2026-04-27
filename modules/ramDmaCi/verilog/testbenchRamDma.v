`timescale 1ps/1ps

module ramDma_tb;

  reg        start, clock, reset;
  reg [31:0] valueA, valueB;
  reg [7:0]  ciN;
  wire       done;
  wire [31:0] result;

  ramDmaCi #(.customId(8'h00)) dut (
  
    .start(start), .clock(clock), .reset(reset),
    .valueA(valueA), .valueB(valueB),
    .ciN(ciN), .done(done), .result(result)
  );

  initial
    begin
      reset  = 1'b1;
      clock  = 1'b0;
      start  = 1'b0;
      ciN    = 8'h00;
      valueA = 32'b0;
      valueB = 32'b0;
      repeat (4) #5 clock = ~clock;
      reset = 1'b0;
      forever #5 clock = ~clock;
    end

  initial
    begin
      @(negedge reset);

      // write to address 5
      @(negedge clock);
      valueA = 32'h00000205; // bit9 = 1 write, address = 5
      valueB = 32'hABABABAB;
      start  = 1'b1;
      @(posedge done);
      @(negedge clock);
      start = 1'b0;

      // read from address 5
      @(negedge clock);
      valueA = 32'h00000005; // bit9 = 0 read, address = 5
      valueB = 32'b0;
      start  = 1'b1;
      @(posedge done);
      @(negedge clock);
      start = 1'b0;
      $display("Read result: %h (expected ABABABAB)", result);

      // write to address 0
      @(negedge clock);
      valueA = 32'h00000200;
      valueB = 32'h12345678;
      start  = 1'b1;
      @(posedge done);
      @(negedge clock);
      start = 1'b0;

      // read from address 0
      @(negedge clock);
      valueA = 32'h00000000;
      valueB = 32'b0;
      start  = 1'b1;
      @(posedge done);
      @(negedge clock);
      start = 1'b0;
      $display("Read result: %h (expected 12345678)", result);

      $finish;
    end

  initial
    begin
      $dumpfile("ramDmaCiSignals.vcd");
      $dumpvars(1, dut);
    end

endmodule