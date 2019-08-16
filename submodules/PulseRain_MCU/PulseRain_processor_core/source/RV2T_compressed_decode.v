`include "RV2T_common.vh"

`default_nettype none

module RV2T_compressed_decode
(
  input  wire [31:0] instr_i,
  output reg [31:2]  instr_o,
  output wire        is_compressed_o,
  output reg         illegal_instr_o
);

  always @(*) begin
    illegal_instr_o = 1'b0;
    instr_o         = '0;

    unique case (instr_i[1:0])
      // C0
      2'b00: begin
        unique case (instr_i[15:13])
          3'b000: begin
            // c.addi4spn -> addi rd', x2, imm
            instr_o = {2'b0, instr_i[10:7], instr_i[12:11], instr_i[5], instr_i[6], 2'b00, 5'h02, 3'b000, 2'b01, instr_i[4:2], `CMD_OP_IMM};
            if (instr_i[12:5] == 8'b0)  illegal_instr_o = 1'b1;
          end

          3'b001: begin
            // c.fld -> fld rd', imm(rs1')
            illegal_instr_o = 1'b1;
          end

          3'b010: begin
            // c.lw -> lw rd', imm(rs1')
            instr_o = {5'b0, instr_i[5], instr_i[12:10], instr_i[6], 2'b00, 2'b01, instr_i[9:7], 3'b010, 2'b01, instr_i[4:2], `CMD_LOAD};
          end

          3'b011: begin
            // c.flw -> flw rd', imm(rs1')
            illegal_instr_o = 1'b1;
          end

          3'b101: begin
            // c.fsd -> fsd rs2', imm(rs1')
            illegal_instr_o = 1'b1;
          end

          3'b110: begin
            // c.sw -> sw rs2', imm(rs1')
            instr_o = {5'b0, instr_i[5], instr_i[12], 2'b01, instr_i[4:2], 2'b01, instr_i[9:7], 3'b010, instr_i[11:10], instr_i[6], 2'b00, `CMD_STORE};
          end

          3'b111: begin
            // c.fsw -> fsw rs2', imm(rs1')
            illegal_instr_o = 1'b1;
          end
          default: begin
            illegal_instr_o = 1'b1;
          end
        endcase
      end


      // C1
      2'b01: begin
        unique case (instr_i[15:13])
          3'b000: begin
            // c.addi -> addi rd, rd, nzimm
            // c.nop
            instr_o = {{6 {instr_i[12]}}, instr_i[12], instr_i[6:2], instr_i[11:7], 3'b0, instr_i[11:7], `CMD_OP_IMM};
          end

          3'b001, 3'b101: begin
            // 001: c.jal -> jal x1, imm
            // 101: c.j   -> jal x0, imm
            instr_o = {instr_i[12], instr_i[8], instr_i[10:9], instr_i[6], instr_i[7], instr_i[2], instr_i[11], instr_i[5:3], {9 {instr_i[12]}}, 4'b0, ~instr_i[15], `CMD_JAL};
          end

          3'b010: begin
            // c.li -> addi rd, x0, nzimm
            instr_o = {{6 {instr_i[12]}}, instr_i[12], instr_i[6:2], 5'b0, 3'b0, instr_i[11:7], `CMD_OP_IMM};
            if (instr_i[11:7] == 5'b0)  illegal_instr_o = 1'b1;
          end

          3'b011: begin
            // c.lui -> lui rd, imm
            instr_o = {{15 {instr_i[12]}}, instr_i[6:2], instr_i[11:7], `CMD_LUI};

            if (instr_i[11:7] == 5'h02) begin
              // c.addi16sp -> addi x2, x2, nzimm
              instr_o = {{3 {instr_i[12]}}, instr_i[4:3], instr_i[5], instr_i[2], instr_i[6], 4'b0, 5'h02, 3'b000, 5'h02, `CMD_OP_IMM};
            end else if (instr_i[11:7] == 5'b0) begin
              illegal_instr_o = 1'b1;
            end

            if ({instr_i[12], instr_i[6:2]} == 6'b0) illegal_instr_o = 1'b1;
          end

          3'b100: begin
            unique case (instr_i[11:10])
              2'b00,
              2'b01: begin
                // 00: c.srli -> srli rd, rd, shamt
                // 01: c.srai -> srai rd, rd, shamt
                instr_o = {1'b0, instr_i[10], 5'b0, instr_i[6:2], 2'b01, instr_i[9:7], 3'b101, 2'b01, instr_i[9:7], `CMD_OP_IMM};
                if (instr_i[12] == 1'b1)  illegal_instr_o = 1'b1;
                if (instr_i[6:2] == 5'b0) illegal_instr_o = 1'b1;
              end

              2'b10: begin
                // c.andi -> andi rd, rd, imm
                instr_o = {{6 {instr_i[12]}}, instr_i[12], instr_i[6:2], 2'b01, instr_i[9:7], 3'b111, 2'b01, instr_i[9:7], `CMD_OP_IMM};
              end

              2'b11: begin
                unique case ({instr_i[12], instr_i[6:5]})
                  3'b000: begin
                    // c.sub -> sub rd', rd', rs2'
                    instr_o = {2'b01, 5'b0, 2'b01, instr_i[4:2], 2'b01, instr_i[9:7], 3'b000, 2'b01, instr_i[9:7], `CMD_OP};
                  end

                  3'b001: begin
                    // c.xor -> xor rd', rd', rs2'
                    instr_o = {7'b0, 2'b01, instr_i[4:2], 2'b01, instr_i[9:7], 3'b100, 2'b01, instr_i[9:7], `CMD_OP};
                  end

                  3'b010: begin
                    // c.or  -> or  rd', rd', rs2'
                    instr_o = {7'b0, 2'b01, instr_i[4:2], 2'b01, instr_i[9:7], 3'b110, 2'b01, instr_i[9:7], `CMD_OP};
                  end

                  3'b011: begin
                    // c.and -> and rd', rd', rs2'
                    instr_o = {7'b0, 2'b01, instr_i[4:2], 2'b01, instr_i[9:7], 3'b111, 2'b01, instr_i[9:7], `CMD_OP};
                  end

                  3'b100,
                  3'b101,
                  3'b110,
                  3'b111: begin
                    // 100: c.subw
                    // 101: c.addw
                    illegal_instr_o = 1'b1;
                  end
                endcase
              end
            endcase
          end

          3'b110, 3'b111: begin
            // 0: c.beqz -> beq rs1', x0, imm
            // 1: c.bnez -> bne rs1', x0, imm
            instr_o = {{4 {instr_i[12]}}, instr_i[6:5], instr_i[2], 5'b0, 2'b01, instr_i[9:7], 2'b00, instr_i[13], instr_i[11:10], instr_i[4:3], instr_i[12], `CMD_BRANCH};
          end
        endcase
      end

      // C2
      2'b10: begin
        unique case (instr_i[15:13])
          3'b000: begin
            // c.slli -> slli rd, rd, shamt
            instr_o = {7'b0, instr_i[6:2], instr_i[11:7], 3'b001, instr_i[11:7], `CMD_OP_IMM};
            if (instr_i[11:7] == 5'b0)  illegal_instr_o = 1'b1;
            if (instr_i[12] == 1'b1 || instr_i[6:2] == 5'b0)  illegal_instr_o = 1'b1;
          end

          3'b001: begin
            // c.fldsp -> fld rd, imm(x2)
            illegal_instr_o = 1'b1;
          end

          3'b010: begin
            // c.lwsp -> lw rd, imm(x2)
            instr_o = {4'b0, instr_i[3:2], instr_i[12], instr_i[6:4], 2'b00, 5'h02, 3'b010, instr_i[11:7], `CMD_LOAD};
            if (instr_i[11:7] == 5'b0)  illegal_instr_o = 1'b1;
          end

          3'b011: begin
            // c.flwsp -> flw rd, imm(x2)
            illegal_instr_o = 1'b1;
          end

          3'b100: begin
            if (instr_i[12] == 1'b0) begin
              // c.mv -> add rd/rs1, x0, rs2
              instr_o = {7'b0, instr_i[6:2], 5'b0, 3'b0, instr_i[11:7], `CMD_OP};

              if (instr_i[6:2] == 5'b0) begin
                // c.jr -> jalr x0, rd/rs1, 0
                instr_o = {12'b0, instr_i[11:7], 3'b0, 5'b0, `CMD_JALR};
              end
            end else begin
              // c.add -> add rd, rd, rs2
              instr_o = {7'b0, instr_i[6:2], instr_i[11:7], 3'b0, instr_i[11:7], `CMD_OP};

              if (instr_i[11:7] == 5'b0) begin
                // c.ebreak -> ebreak
                instr_o = {30'h04_00_1C};
                if (instr_i[6:2] != 5'b0)
                  illegal_instr_o = 1'b1;
              end else if (instr_i[6:2] == 5'b0) begin
                // c.jalr -> jalr x1, rs1, 0
                instr_o = {12'b0, instr_i[11:7], 3'b000, 5'b00001, `CMD_JALR};
              end
            end
          end

          3'b101: begin
            // c.fsdsp -> fsd rs2, imm(x2)
            illegal_instr_o = 1'b1;
          end

          3'b110: begin
            // c.swsp -> sw rs2, imm(x2)
            instr_o = {4'b0, instr_i[8:7], instr_i[12], instr_i[6:2], 5'h02, 3'b010, instr_i[11:9], 2'b00, `CMD_STORE};
          end

          3'b111: begin
            // c.fswsp -> fsw rs2, imm(x2)
            illegal_instr_o = 1'b1;
          end
        endcase
      end

      default: begin
        // 32 bit (or more) instruction
        instr_o = instr_i[31:2];
      end
    endcase
  end

  assign is_compressed_o = (instr_i[1:0] != 2'b11);

endmodule