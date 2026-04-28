module ramDmaCi #( parameter [7:0] customInstructionId = 8'h00 )
                ( input wire        start,
                                   clock,
                                   reset,
                  input wire [31:0] valueA,
                                   valueB,
                  input wire [7:0]  ciN,
                  output wire       done,
                  output wire [31:0] result,
                  // bus master interface
                  output wire        requestBus,
                  input wire         busGrant,
                  output reg         beginTransactionOut,
                  output reg  [31:0] addressDataOut,
                  output reg         endTransactionOut,
                  output reg  [3:0]  byteEnablesOut,
                  output reg  [7:0]  burstSizeOut,
                  output wire        readNotWriteOut,
                  // bus slave signals (in)
                  input wire         busyIn,
                                     busErrorIn,
                                     endTransactionIn,
                                     dataValidIn,
                  input wire [31:0]  addressDataIn );

  // -------------------------------------------------------
  // FSM states
  // -------------------------------------------------------
  localparam [2:0] IDLE      = 3'd0,
                   REQUEST   = 3'd1,
                   INIT      = 3'd2,
                   READ      = 3'd3,
                   END_TRANS = 3'd4,
                   ERROR     = 3'd5;

  // -------------------------------------------------------
  // CI interface logic
  // -------------------------------------------------------
  wire s_validCi     = (ciN == customInstructionId) && (start == 1'b1);
  wire s_doOperation = s_validCi;
  wire [2:0] s_ciSel = valueA[12:10];
  wire       s_ciWE  = valueA[9];

  // -------------------------------------------------------
  // DMA configuration registers
  // -------------------------------------------------------
  reg [31:0] s_busAddrReg;
  reg [8:0]  s_memAddrReg;
  reg [9:0]  s_blockSizeReg;
  reg [7:0]  s_burstSizeReg;
  reg [1:0]  s_statusReg;     // [0]=busy, [1]=busError
  reg [2:0]  s_dmaState;

  // -------------------------------------------------------
  // Registered bus-in signals (reduce critical path)
  // -------------------------------------------------------
  reg        s_endTransactionReg, s_dataValidReg, s_busErrorReg;
  reg [31:0] s_addressDataReg;

  always @(posedge clock)
    begin
      s_endTransactionReg <= endTransactionIn & ~reset;
      s_dataValidReg      <= dataValidIn      & ~reset;
      s_busErrorReg       <= busErrorIn       & ~reset;
      s_addressDataReg    <= addressDataIn;
    end

  // -------------------------------------------------------
  // DMA counters
  // -------------------------------------------------------
  reg [31:0] s_currentBusAddrReg;
  reg [8:0]  s_currentMemAddrReg;
  reg [9:0]  s_wordsLeftReg;
  reg [7:0]  s_burstCountReg;

  // current burst size: min(wordsLeft-1, burstSize)
  wire [7:0] s_thisBurst = (s_wordsLeftReg <= {2'b0, s_burstSizeReg} + 10'd1) ?
                            s_wordsLeftReg[7:0] - 8'd1 : s_burstSizeReg;

  // -------------------------------------------------------
  // FSM next-state logic
  // -------------------------------------------------------
  reg [2:0] s_dmaStateNext;

  always @*
    case (s_dmaState)
      IDLE      : s_dmaStateNext = (s_statusReg[0] == 1'b1) ? REQUEST : IDLE;
      REQUEST   : s_dmaStateNext = (busGrant == 1'b1) ? INIT : REQUEST;
      INIT      : s_dmaStateNext = READ;
      READ      : s_dmaStateNext = (s_busErrorReg == 1'b1) ? ERROR :
                                   (s_endTransactionReg == 1'b1 && s_wordsLeftReg == 10'd0) ? IDLE :
                                   (s_endTransactionReg == 1'b1) ? REQUEST :
                                   READ;
      ERROR     : s_dmaStateNext = (s_endTransactionReg == 1'b1) ? IDLE : ERROR;
      default   : s_dmaStateNext = IDLE;
    endcase

  // -------------------------------------------------------
  // FSM register + counters
  // -------------------------------------------------------
  always @(posedge clock)
    begin
      if (reset) begin
        s_dmaState          <= IDLE;
        s_currentBusAddrReg <= 32'd0;
        s_currentMemAddrReg <= 9'd0;
        s_wordsLeftReg      <= 10'd0;
        s_burstCountReg     <= 8'd0;
        s_statusReg         <= 2'd0;
      end else begin
        s_dmaState <= s_dmaStateNext;

        // start DMA: latch counters, set busy
        if (s_dmaState == IDLE && s_dmaStateNext == REQUEST) begin
          s_currentBusAddrReg <= s_busAddrReg;
          s_currentMemAddrReg <= s_memAddrReg;
          s_wordsLeftReg      <= s_blockSizeReg;
          s_statusReg[1]      <= 1'b0; // clear error
        end

        // load burst counter on INIT
        if (s_dmaState == INIT)
          s_burstCountReg <= s_thisBurst;

        // on each valid data word received: write to memory, advance pointers
        if (s_dmaState == READ && s_dataValidReg == 1'b1) begin
          s_currentBusAddrReg <= s_currentBusAddrReg + 32'd4;
          s_currentMemAddrReg <= s_currentMemAddrReg + 9'd1;
          s_wordsLeftReg      <= s_wordsLeftReg - 10'd1;
          s_burstCountReg     <= s_burstCountReg - 8'd1;
        end

        // bus error handling
        if (s_dmaState == READ && s_busErrorReg == 1'b1)
          s_statusReg[1] <= 1'b1;

        // status busy bit
        s_statusReg[0] <= (s_dmaStateNext != IDLE) ? 1'b1 : 1'b0;

        // CI writes to DMA config registers
        if (s_doOperation && s_ciWE) begin
          case (s_ciSel)
            3'b001: s_busAddrReg   <= valueB;
            3'b010: s_memAddrReg   <= valueB[8:0];
            3'b011: s_blockSizeReg <= valueB[9:0];
            3'b100: s_burstSizeReg <= valueB[7:0];
            3'b101: if (valueB[0] == 1'b1 && s_statusReg[0] == 1'b0)
                      s_statusReg[0] <= 1'b1; // trigger DMA
            default: ;
          endcase
        end
      end
    end

  // -------------------------------------------------------
  // Bus master output registers
  // -------------------------------------------------------
  assign requestBus    = (s_dmaState == REQUEST) ? 1'b1 : 1'b0;
  reg s_readNotWriteReg;
  always @(posedge clock)
    s_readNotWriteReg <= (s_dmaState == INIT) ? 1'b1 : 1'b0;
  assign readNotWriteOut = s_readNotWriteReg;

  always @(posedge clock)
    begin
      beginTransactionOut <= (s_dmaState == INIT) ? 1'b1 : 1'b0;
      addressDataOut      <= (s_dmaState == INIT) ? s_currentBusAddrReg : 32'd0;
      byteEnablesOut      <= (s_dmaState == INIT) ? 4'hF : 4'd0;
      burstSizeOut        <= (s_dmaState == INIT) ? s_thisBurst : 8'd0;
      endTransactionOut   <= (s_dmaState == ERROR && s_busErrorReg) ? 1'b1 : 1'b0;
    end

  // -------------------------------------------------------
  // Port B of SSRAM: DMA writes incoming data
  // -------------------------------------------------------
  wire s_dmaWrite = (s_dmaState == READ) && s_dataValidReg;

  // -------------------------------------------------------
  // Dual port SSRAM
  // -------------------------------------------------------
  wire [31:0] s_memDataOut;

  dualPortSSRAM #( .bitwidth(32), .nrOfEntries(512), .readAfterWrite(0) )
  mem ( .clockA(clock),         .clockB(~clock),
        .writeEnableA(s_doOperation && s_ciWE && s_ciSel == 3'b000),
        .writeEnableB(s_dmaWrite),
        .addressA(valueA[8:0]), .addressB(s_currentMemAddrReg),
        .dataInA(valueB),       .dataInB(s_addressDataReg),
        .dataOutA(s_memDataOut),.dataOutB());

  // -------------------------------------------------------
  // CI done & result
  // -------------------------------------------------------
  reg s_done, s_readPending;
  reg [31:0] s_result;
  reg [31:0] s_readMux;

  wire s_doneWrite = s_doOperation && s_ciWE;

  assign done   = s_doneWrite | s_done;
  assign result = s_result;

  // select what to return for CI reads
  always @*
    case (s_ciSel)
      3'b000: s_readMux = s_memDataOut;
      3'b001: s_readMux = s_busAddrReg;
      3'b010: s_readMux = {23'd0, s_memAddrReg};
      3'b011: s_readMux = {22'd0, s_blockSizeReg};
      3'b100: s_readMux = {24'd0, s_burstSizeReg};
      3'b101: s_readMux = {30'd0, s_statusReg};
      default: s_readMux = 32'd0;
    endcase

  always @(posedge clock)
    begin
      if (reset) begin
        s_done        <= 1'b0;
        s_result      <= 32'd0;
        s_readPending <= 1'b0;
      end else begin
        s_done <= 1'b0;

        if (s_readPending) begin
          s_result      <= s_readMux;
          s_done        <= 1'b1;
          s_readPending <= 1'b0;
        end else if (s_doOperation && !s_ciWE) begin
          s_readPending <= 1'b1;
        end
      end
    end
endmodule

module dualPortSSRAM #( parameter bitwidth = 32,
                        parameter nrOfEntries = 512,
                        parameter readAfterWrite = 0 )
                      ( input wire                          clockA, clockB,
                                                            writeEnableA, writeEnableB,
                        input wire [$clog2(nrOfEntries)-1:0] addressA, addressB,
                        input wire [bitwidth-1:0]            dataInA, dataInB,
                        output reg [bitwidth-1:0]            dataOutA, dataOutB );

  reg [bitwidth-1:0] memoryContent [nrOfEntries-1:0];

  always @(posedge clockA)
    begin 
      if (readAfterWrite != 0) dataOutA <= memoryContent[addressA];
      if (writeEnableA == 1'b1) memoryContent[addressA] <= dataInA;
      if (readAfterWrite == 0) dataOutA <= memoryContent[addressA];
    end

  always @(posedge clockB)
    begin
      if (readAfterWrite != 0) dataOutB <= memoryContent[addressB];
      if (writeEnableB == 1'b1) memoryContent[addressB] <= dataInB;
      if (readAfterWrite == 0) dataOutB <= memoryContent[addressB];
    end

endmodule         