module ramDmaCi #( parameter [7:0] customId = 8'h00 )
                ( input wire        start,
                                   clock,
                                   reset,
                  input wire [31:0] valueA,
                                   valueB,
                  input wire [7:0]  ciN,
                  output wire       done,
                  output wire [31:0] result );

    wire [31:0] memDataOut;
    wire s_validCi     = (ciN == customId) && (start == 1'b1);
    wire s_validAccess = (valueA[31:10] == 22'b0);
    wire s_doOperation = s_validCi && s_validAccess;


    dualPortSSRAM #( .bitwidth(32), .nrOfEntries(512), .readAfterWrite(0) )
    mem ( .clockA(clock), .clockB(~clock), 
          .writeEnableA(valueA[9] && s_doOperation), .writeEnableB(1'b0), 
          .addressA(valueA[8:0]), .addressB(9'b0),
          .dataInA(valueB), .dataInB(32'b0),
          .dataOutA(memDataOut), .dataOutB());

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
      if (readAfterWrite != 0) dataOutA = memoryContent[addressA];
      if (writeEnableA == 1'b1) memoryContent[addressA] = dataInA;
      if (readAfterWrite == 0) dataOutA = memoryContent[addressA];
    end

  always @(posedge clockB)
    begin
      if (readAfterWrite != 0) dataOutB = memoryContent[addressB];
      if (writeEnableB == 1'b1) memoryContent[addressB] = dataInB;
      if (readAfterWrite == 0) dataOutB = memoryContent[addressB];
    end

endmodule         