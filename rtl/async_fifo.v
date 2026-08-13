`timescale 1ns/1ps

// Dual-clock asynchronous FIFO.
// Default depth is 32 entries. ADDR_WIDTH and FIFO_DEPTH must agree, and
// FIFO_DEPTH must be a power of two.
//
// This implementation uses only Verilog-2005 constructs.
module async_fifo #(
    parameter integer DATA_WIDTH = 8,
    parameter integer FIFO_DEPTH = 32,
    parameter integer ADDR_WIDTH = 5
) (
    input  wire                  wr_clk,
    input  wire                  wr_rst_n,
    input  wire                  wr_en,
    input  wire [DATA_WIDTH-1:0] wr_data,
    output reg                   full,

    input  wire                  rd_clk,
    input  wire                  rd_rst_n,
    input  wire                  rd_en,
    output reg  [DATA_WIDTH-1:0] rd_data,
    output reg                   empty
);

    localparam integer PTR_WIDTH = ADDR_WIDTH + 1;

    reg [DATA_WIDTH-1:0] mem [0:FIFO_DEPTH-1];

    reg [PTR_WIDTH-1:0] wr_bin;
    reg [PTR_WIDTH-1:0] wr_gray;
    reg [PTR_WIDTH-1:0] rd_bin;
    reg [PTR_WIDTH-1:0] rd_gray;

    (* ASYNC_REG = "TRUE" *) reg [PTR_WIDTH-1:0] rd_gray_sync1;
    (* ASYNC_REG = "TRUE" *) reg [PTR_WIDTH-1:0] rd_gray_sync2;
    (* ASYNC_REG = "TRUE" *) reg [PTR_WIDTH-1:0] wr_gray_sync1;
    (* ASYNC_REG = "TRUE" *) reg [PTR_WIDTH-1:0] wr_gray_sync2;

    wire write_accept;
    wire read_accept;
    wire [PTR_WIDTH-1:0] wr_bin_next;
    wire [PTR_WIDTH-1:0] rd_bin_next;
    wire [PTR_WIDTH-1:0] wr_gray_next;
    wire [PTR_WIDTH-1:0] rd_gray_next;
    wire [PTR_WIDTH-1:0] rd_gray_full_compare;
    wire full_next;
    wire empty_next;

    assign write_accept = wr_en && !full;
    assign read_accept  = rd_en && !empty;

    assign wr_bin_next  = wr_bin + write_accept;
    assign rd_bin_next  = rd_bin + read_accept;
    assign wr_gray_next = (wr_bin_next >> 1) ^ wr_bin_next;
    assign rd_gray_next = (rd_bin_next >> 1) ^ rd_bin_next;

    // A full FIFO is detected by comparing the next write Gray pointer with
    // the synchronized read Gray pointer after inverting its two MSBs.
    assign rd_gray_full_compare =
        {~rd_gray_sync2[PTR_WIDTH-1:PTR_WIDTH-2],
          rd_gray_sync2[PTR_WIDTH-3:0]};
    assign full_next  = (wr_gray_next == rd_gray_full_compare);
    assign empty_next = (rd_gray_next == wr_gray_sync2);

`ifndef SYNTHESIS
    initial begin
        if (DATA_WIDTH < 1) begin
            $display("ERROR: async_fifo DATA_WIDTH must be at least 1");
            $finish;
        end
        if ((FIFO_DEPTH < 2) || ((FIFO_DEPTH & (FIFO_DEPTH - 1)) != 0)) begin
            $display("ERROR: async_fifo FIFO_DEPTH must be a power of two >= 2");
            $finish;
        end
        if (FIFO_DEPTH != (1 << ADDR_WIDTH)) begin
            $display("ERROR: async_fifo FIFO_DEPTH must equal 2**ADDR_WIDTH");
            $finish;
        end
    end
`endif

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wr_bin  <= {PTR_WIDTH{1'b0}};
            wr_gray <= {PTR_WIDTH{1'b0}};
            full    <= 1'b0;
        end else begin
            if (write_accept)
                mem[wr_bin[ADDR_WIDTH-1:0]] <= wr_data;

            wr_bin  <= wr_bin_next;
            wr_gray <= wr_gray_next;
            full    <= full_next;
        end
    end

    // rd_data is registered. It changes on the read-clock edge that accepts
    // rd_en and holds its previous value when no read is accepted.
    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            rd_bin  <= {PTR_WIDTH{1'b0}};
            rd_gray <= {PTR_WIDTH{1'b0}};
            rd_data <= {DATA_WIDTH{1'b0}};
            empty   <= 1'b1;
        end else begin
            if (read_accept)
                rd_data <= mem[rd_bin[ADDR_WIDTH-1:0]];

            rd_bin  <= rd_bin_next;
            rd_gray <= rd_gray_next;
            empty   <= empty_next;
        end
    end

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            rd_gray_sync1 <= {PTR_WIDTH{1'b0}};
            rd_gray_sync2 <= {PTR_WIDTH{1'b0}};
        end else begin
            rd_gray_sync1 <= rd_gray;
            rd_gray_sync2 <= rd_gray_sync1;
        end
    end

    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            wr_gray_sync1 <= {PTR_WIDTH{1'b0}};
            wr_gray_sync2 <= {PTR_WIDTH{1'b0}};
        end else begin
            wr_gray_sync1 <= wr_gray;
            wr_gray_sync2 <= wr_gray_sync1;
        end
    end

endmodule
