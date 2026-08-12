`timescale 1ns/1ps

// Asynchronous FIFO with independent write and read clock domains.
// FIFO_DEPTH must be a power of two so that Gray-code pointers can be used
// safely for clock-domain crossing.
module async_fifo #(
    parameter integer DATA_WIDTH = 8,
    parameter integer FIFO_DEPTH = 32
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

    function integer clog2;
        input integer value;
        integer temp;
        begin
            temp = value - 1;
            for (clog2 = 0; temp > 0; clog2 = clog2 + 1)
                temp = temp >> 1;
        end
    endfunction

    localparam integer ADDR_WIDTH = clog2(FIFO_DEPTH);
    localparam integer PTR_WIDTH  = ADDR_WIDTH + 1;

    reg [DATA_WIDTH-1:0] mem [0:FIFO_DEPTH-1];

    reg [PTR_WIDTH-1:0] wr_bin;
    reg [PTR_WIDTH-1:0] wr_gray;
    reg [PTR_WIDTH-1:0] rd_bin;
    reg [PTR_WIDTH-1:0] rd_gray;

    (* ASYNC_REG = "TRUE" *) reg [PTR_WIDTH-1:0] rd_gray_sync1;
    (* ASYNC_REG = "TRUE" *) reg [PTR_WIDTH-1:0] rd_gray_sync2;
    (* ASYNC_REG = "TRUE" *) reg [PTR_WIDTH-1:0] wr_gray_sync1;
    (* ASYNC_REG = "TRUE" *) reg [PTR_WIDTH-1:0] wr_gray_sync2;

    function [PTR_WIDTH-1:0] gray_to_bin;
        input [PTR_WIDTH-1:0] gray;
        integer i;
        begin
            gray_to_bin[PTR_WIDTH-1] = gray[PTR_WIDTH-1];
            for (i = PTR_WIDTH-2; i >= 0; i = i - 1)
                gray_to_bin[i] = gray_to_bin[i+1] ^ gray[i];
        end
    endfunction

    wire write_accept = wr_en && !full;
    wire read_accept  = rd_en && !empty;

    wire [PTR_WIDTH-1:0] wr_bin_next = wr_bin + write_accept;
    wire [PTR_WIDTH-1:0] rd_bin_next = rd_bin + read_accept;
    wire [PTR_WIDTH-1:0] wr_gray_next =
        (wr_bin_next >> 1) ^ wr_bin_next;
    wire [PTR_WIDTH-1:0] rd_gray_next =
        (rd_bin_next >> 1) ^ rd_bin_next;

    wire [PTR_WIDTH-1:0] rd_bin_sync = gray_to_bin(rd_gray_sync2);

    // The extra pointer bit distinguishes equal addresses that are one full
    // FIFO span apart.
    wire full_next =
        (wr_bin_next[PTR_WIDTH-1] != rd_bin_sync[PTR_WIDTH-1]) &&
        (wr_bin_next[ADDR_WIDTH-1:0] == rd_bin_sync[ADDR_WIDTH-1:0]);
    wire empty_next = (rd_gray_next == wr_gray_sync2);

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
    end
`endif

    // Write-domain storage and pointer update.
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

    // Read-domain data and pointer update. rd_data changes only after an
    // accepted read and retains its previous value while the FIFO is empty.
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

    // Synchronize the Gray-coded read pointer into the write domain.
    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            rd_gray_sync1 <= {PTR_WIDTH{1'b0}};
            rd_gray_sync2 <= {PTR_WIDTH{1'b0}};
        end else begin
            rd_gray_sync1 <= rd_gray;
            rd_gray_sync2 <= rd_gray_sync1;
        end
    end

    // Synchronize the Gray-coded write pointer into the read domain.
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
