`timescale 1ps/1ps

module ramDma_tb;

  reg        start, clock, reset;
  reg [31:0] valueA, valueB;
  reg [7:0]  ciN;
  wire       done;
  wire [31:0] result;

  // bus signals
  reg         busGrant, busyIn, busErrorIn;
  reg         endTransactionIn, dataValidIn;
  reg [31:0]  addressDataIn;
  wire        requestBus;
  wire        beginTransactionOut, endTransactionOut;
  wire [31:0] addressDataOut;
  wire [3:0]  byteEnablesOut;
  wire [7:0]  burstSizeOut;
  wire        readNotWriteOut;

  ramDmaCi #(.customInstructionId(8'd0)) dut (
    .start(start), .clock(clock), .reset(reset),
    .valueA(valueA), .valueB(valueB),
    .ciN(ciN), .done(done), .result(result),
    .requestBus(requestBus), .busGrant(busGrant),
    .beginTransactionOut(beginTransactionOut),
    .addressDataOut(addressDataOut),
    .endTransactionOut(endTransactionOut),
    .byteEnablesOut(byteEnablesOut),
    .burstSizeOut(burstSizeOut),
    .readNotWriteOut(readNotWriteOut),
    .busyIn(busyIn), .busErrorIn(busErrorIn),
    .endTransactionIn(endTransactionIn),
    .dataValidIn(dataValidIn),
    .addressDataIn(addressDataIn));

  initial clock = 0;
  always #5 clock = ~clock;

  // CI write task
  task ciWrite;
    input [31:0] addr;
    input [31:0] data;
    begin
      @(negedge clock);
      valueA = addr; valueB = data; start = 1'b1;
      @(posedge done); @(negedge clock);
      start = 1'b0;
    end
  endtask

  // CI read task
  task ciRead;
    input [31:0] addr;
    begin
      @(negedge clock);
      valueA = addr; valueB = 32'b0; start = 1'b1;
      @(posedge done); @(negedge clock);
      start = 1'b0;
    end
  endtask

  initial begin
    // init
    reset=1; start=0; busGrant=0; busyIn=0;
    busErrorIn=0; endTransactionIn=0;
    dataValidIn=0; addressDataIn=0;
    ciN=8'd0; valueA=0; valueB=0;
    repeat(4) #5 clock=~clock;
    reset=0;

    // configure DMA: bus start = 0x00100000, mem start = 0, block = 4, burst = 3
    ciWrite(32'h00000600, 32'h00100000); // A[12:10]=001, A[9]=1: write bus addr
    ciWrite(32'h00000A00, 32'h00000000); // A[12:10]=010, A[9]=1: write mem addr
    ciWrite(32'h00000E00, 32'h00000004); // A[12:10]=011, A[9]=1: write block size=4
    ciWrite(32'h00001200, 32'h00000003); // A[12:10]=100, A[9]=1: write burst size=3

    // start DMA (write 1 to control)
    ciWrite(32'h00001600, 32'h00000001); // A[12:10]=101, A[9]=1: write control

    // wait for bus request, grant it
    @(posedge requestBus);
    @(negedge clock);
    busGrant = 1'b1;
    @(negedge clock);
    busGrant = 1'b0;

    // simulate slave sending 4 words
    repeat(4) begin
      @(negedge clock);
      dataValidIn  = 1'b1;
      addressDataIn = $random;
    end
    @(negedge clock);
    dataValidIn = 1'b0;

    // send endTransaction
    @(negedge clock);
    endTransactionIn = 1'b1;
    @(negedge clock);
    endTransactionIn = 1'b0;

    // poll status until idle
    ciRead(32'h00001400); // A[12:10]=101, A[9]=0: read status
    $display("Status: %b (expected 00=idle)", result[1:0]);

    // read back memory location 0 via CI
    ciRead(32'h00000000);
    $display("Mem[0] = %08x", result);

    $finish;
  end

  initial begin
    $dumpfile("ramDmaCi_dma.vcd");
    $dumpvars(1, dut);
  end

endmodule