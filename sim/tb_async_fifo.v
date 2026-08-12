`timescale 1ns/1ps

module tb_async_fifo;

    localparam integer DATA_WIDTH = 8;
    localparam integer FIFO_DEPTH = 32;

    reg                  wr_clk;
    reg                  wr_rst_n;
    reg                  wr_en;
    reg [DATA_WIDTH-1:0] wr_data;
    wire                 full;

    reg                  rd_clk;
    reg                  rd_rst_n;
    reg                  rd_en;
    wire [DATA_WIDTH-1:0] rd_data;
    wire                  empty;

    integer i;
    integer errors;

    async_fifo #(
        .DATA_WIDTH(DATA_WIDTH),
        .FIFO_DEPTH(FIFO_DEPTH)
    ) dut (
        .wr_clk   (wr_clk),
        .wr_rst_n (wr_rst_n),
        .wr_en    (wr_en),
        .wr_data  (wr_data),
        .full     (full),
        .rd_clk   (rd_clk),
        .rd_rst_n (rd_rst_n),
        .rd_en    (rd_en),
        .rd_data  (rd_data),
        .empty    (empty)
    );

    initial wr_clk = 1'b0;
    always #5 wr_clk = ~wr_clk;

    initial rd_clk = 1'b0;
    always #7 rd_clk = ~rd_clk;

    initial begin
        wr_rst_n = 1'b0;
        rd_rst_n = 1'b0;
        wr_en    = 1'b0;
        rd_en    = 1'b0;
        wr_data  = {DATA_WIDTH{1'b0}};
        errors   = 0;

        #30;
        wr_rst_n = 1'b1;
        rd_rst_n = 1'b1;

        // Fill the FIFO with an incrementing pattern.
        for (i = 0; i < FIFO_DEPTH; i = i + 1) begin
            @(negedge wr_clk);
            wr_en   = 1'b1;
            wr_data = i[DATA_WIDTH-1:0];
        end
        @(negedge wr_clk);
        wr_en = 1'b0;

        wait (full === 1'b1);

        // Allow the synchronized write pointer to reach the read domain.
        wait (empty === 1'b0);

        // Drain and check the FIFO in order.
        for (i = 0; i < FIFO_DEPTH; i = i + 1) begin
            @(negedge rd_clk);
            rd_en = 1'b1;
            @(posedge rd_clk);
            #1;
            if (rd_data !== i[DATA_WIDTH-1:0]) begin
                $display("ERROR: index=%0d expected=%0h actual=%0h",
                         i, i[DATA_WIDTH-1:0], rd_data);
                errors = errors + 1;
            end
        end
        @(negedge rd_clk);
        rd_en = 1'b0;

        wait (empty === 1'b1);

        if (errors == 0)
            $display("PASS: async_fifo smoke test");
        else
            $display("FAIL: async_fifo smoke test, errors=%0d", errors);

        #20;
        $finish;
    end

endmodule
