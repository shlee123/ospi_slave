`timescale 1ns/1ps

module tb_async_fifo;
    localparam integer DATA_WIDTH = 8;
    localparam integer FIFO_DEPTH = 32;

    reg wr_clk;
    reg wr_rst_n;
    reg wr_en;
    reg [DATA_WIDTH-1:0] wr_data;
    wire full;
    reg rd_clk;
    reg rd_rst_n;
    reg rd_en;
    wire [DATA_WIDTH-1:0] rd_data;
    wire empty;
    reg small_wr_en;
    reg [7:0] small_wr_data;
    wire small_full;
    reg small_rd_en;
    wire [7:0] small_rd_data;
    wire small_empty;
    integer i;
    integer errors;

    async_fifo #(
        .DATA_WIDTH(DATA_WIDTH),
        .FIFO_DEPTH(FIFO_DEPTH),
        .ADDR_WIDTH(5)
    ) dut (
        .wr_clk(wr_clk), .wr_rst_n(wr_rst_n), .wr_en(wr_en),
        .wr_data(wr_data), .full(full),
        .rd_clk(rd_clk), .rd_rst_n(rd_rst_n), .rd_en(rd_en),
        .rd_data(rd_data), .empty(empty)
    );

    // Minimum legal Gray-pointer FIFO depth, also used by the Request FIFO.
    async_fifo #(
        .DATA_WIDTH(8),
        .FIFO_DEPTH(2),
        .ADDR_WIDTH(1)
    ) dut_depth_two (
        .wr_clk(wr_clk), .wr_rst_n(wr_rst_n), .wr_en(small_wr_en),
        .wr_data(small_wr_data), .full(small_full),
        .rd_clk(rd_clk), .rd_rst_n(rd_rst_n), .rd_en(small_rd_en),
        .rd_data(small_rd_data), .empty(small_empty)
    );

    initial wr_clk = 1'b0;
    always #5 wr_clk = ~wr_clk;
    initial rd_clk = 1'b0;
    always #7 rd_clk = ~rd_clk;

    initial begin
        wr_rst_n = 1'b0;
        rd_rst_n = 1'b0;
        wr_en = 1'b0;
        rd_en = 1'b0;
        wr_data = 8'b0;
        small_wr_en = 1'b0;
        small_wr_data = 8'b0;
        small_rd_en = 1'b0;
        errors = 0;
        #30;
        wr_rst_n = 1'b1;
        rd_rst_n = 1'b1;

        for (i = 0; i < FIFO_DEPTH; i = i + 1) begin
            @(negedge wr_clk);
            wr_en = 1'b1;
            wr_data = i;
        end
        @(negedge wr_clk);
        wr_en = 1'b0;
        wait (full === 1'b1);
        wait (empty === 1'b0);

        for (i = 0; i < FIFO_DEPTH; i = i + 1) begin
            @(negedge rd_clk);
            rd_en = 1'b1;
            @(posedge rd_clk);
            #1;
            if (rd_data !== i[DATA_WIDTH-1:0]) begin
                $display("ERROR: FIFO index=%0d expected=%02x actual=%02x",
                         i, i[DATA_WIDTH-1:0], rd_data);
                errors = errors + 1;
            end
        end
        @(negedge rd_clk);
        rd_en = 1'b0;
        wait (empty === 1'b1);

        // Verify that the two-entry configuration reaches full and preserves
        // both words across the asynchronous clock boundary.
        @(negedge wr_clk);
        small_wr_en = 1'b1;
        small_wr_data = 8'ha5;
        @(negedge wr_clk);
        small_wr_data = 8'h5a;
        @(negedge wr_clk);
        small_wr_en = 1'b0;
        wait (small_full === 1'b1);
        wait (small_empty === 1'b0);

        @(negedge rd_clk);
        small_rd_en = 1'b1;
        @(posedge rd_clk);
        #1;
        if (small_rd_data !== 8'ha5) begin
            $display("ERROR: depth-two FIFO first word=%02x", small_rd_data);
            errors = errors + 1;
        end
        @(posedge rd_clk);
        #1;
        if (small_rd_data !== 8'h5a) begin
            $display("ERROR: depth-two FIFO second word=%02x", small_rd_data);
            errors = errors + 1;
        end
        @(negedge rd_clk);
        small_rd_en = 1'b0;
        wait (small_empty === 1'b1);

        if (errors == 0)
            $display("PASS: async_fifo full-depth test");
        else
            $display("FAIL: async_fifo errors=%0d", errors);
        #20 $finish;
    end
endmodule
