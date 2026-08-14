`timescale 1ns/1ps

`ifndef TB_SRDY_OPEN_DRAIN
`define TB_SRDY_OPEN_DRAIN 0
`endif

module tb_ospi_slave;
    localparam integer AXI_ADDR_WIDTH = 32;
    localparam integer AXI_DATA_WIDTH = 32;
    localparam integer AXI_ID_WIDTH = 6;

    reg clk;
    reg rst_n;
    reg SCLK;
    reg CSN;
    tri [7:0] D;
    tri1 SRDY;
    reg [7:0] master_d_out;
    reg master_d_oe;
    assign D = master_d_oe ? master_d_out : 8'bz;

    wire [AXI_ID_WIDTH-1:0] AWID;
    wire [31:0] AWADDR;
    wire [7:0] AWLEN;
    wire [2:0] AWSIZE;
    wire [1:0] AWBURST;
    wire [2:0] AWPROT;
    wire AWVALID;
    wire AWREADY;
    wire [31:0] WDATA;
    wire [3:0] WSTRB;
    wire WLAST;
    wire WVALID;
    wire WREADY;
    wire [5:0] BID;
    wire [1:0] BRESP;
    reg BVALID;
    wire BREADY;
    wire [5:0] ARID;
    wire [31:0] ARADDR;
    wire [7:0] ARLEN;
    wire [2:0] ARSIZE;
    wire [1:0] ARBURST;
    wire [2:0] ARPROT;
    wire ARVALID;
    wire ARREADY;
    wire [5:0] RID;
    wire [31:0] RDATA;
    wire [1:0] RRESP;
    wire RLAST;
    wire RVALID;
    wire RREADY;

    reg [31:0] memory [0:255];
    reg write_active;
    reg [31:0] write_address;
    reg [7:0] write_beats_left;
    reg read_active;
    reg [31:0] read_address;
    reg [7:0] read_beats_left;
    integer aw_count;
    integer ar_count;
    integer errors;
    integer read_count;
    reg [7:0] read_bytes [0:7];

    assign AWREADY = !write_active && !BVALID;
    assign WREADY = write_active;
    assign BID = 6'b0;
    assign BRESP = 2'b00;
    assign ARREADY = !read_active;
    assign RID = 6'b0;
    assign RDATA = memory[read_address[9:2]];
    assign RRESP = 2'b00;
    assign RVALID = read_active;
    assign RLAST = read_active && (read_beats_left == 8'd1);

    ospi_slave_top #(
        .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
        .AXI_ID_WIDTH(AXI_ID_WIDTH),
        .SRDY_OPEN_DRAIN(`TB_SRDY_OPEN_DRAIN)
    ) dut (
        .clk(clk), .rst_n(rst_n), .SCLK(SCLK), .CSN(CSN), .D(D), .SRDY(SRDY),
        .M_AXI_AWID(AWID), .M_AXI_AWADDR(AWADDR), .M_AXI_AWLEN(AWLEN),
        .M_AXI_AWSIZE(AWSIZE), .M_AXI_AWBURST(AWBURST),
        .M_AXI_AWPROT(AWPROT), .M_AXI_AWVALID(AWVALID),
        .M_AXI_AWREADY(AWREADY), .M_AXI_WDATA(WDATA),
        .M_AXI_WSTRB(WSTRB), .M_AXI_WLAST(WLAST),
        .M_AXI_WVALID(WVALID), .M_AXI_WREADY(WREADY),
        .M_AXI_BID(BID), .M_AXI_BRESP(BRESP), .M_AXI_BVALID(BVALID),
        .M_AXI_BREADY(BREADY), .M_AXI_ARID(ARID), .M_AXI_ARADDR(ARADDR),
        .M_AXI_ARLEN(ARLEN), .M_AXI_ARSIZE(ARSIZE),
        .M_AXI_ARBURST(ARBURST), .M_AXI_ARPROT(ARPROT),
        .M_AXI_ARVALID(ARVALID), .M_AXI_ARREADY(ARREADY),
        .M_AXI_RID(RID), .M_AXI_RDATA(RDATA), .M_AXI_RRESP(RRESP),
        .M_AXI_RLAST(RLAST), .M_AXI_RVALID(RVALID), .M_AXI_RREADY(RREADY)
    );

    initial clk = 1'b0;
    always #3 clk = ~clk;

    // Minimal AXI memory slave.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            write_active <= 1'b0;
            write_address <= 32'b0;
            write_beats_left <= 8'b0;
            BVALID <= 1'b0;
            read_active <= 1'b0;
            read_address <= 32'b0;
            read_beats_left <= 8'b0;
            aw_count <= 0;
            ar_count <= 0;
        end else begin
            if (AWVALID && AWREADY) begin
                write_active <= 1'b1;
                write_address <= AWADDR;
                write_beats_left <= AWLEN + 1'b1;
                aw_count <= aw_count + 1;
                if ((AWPROT !== 3'b001) || (AWSIZE !== 3'd2) ||
                    (AWBURST !== 2'b01)) begin
                    $display("ERROR: invalid AXI Write attributes");
                    errors = errors + 1;
                end
            end
            if (WVALID && WREADY) begin
                if (WSTRB[0]) memory[write_address[9:2]][7:0] <= WDATA[7:0];
                if (WSTRB[1]) memory[write_address[9:2]][15:8] <= WDATA[15:8];
                if (WSTRB[2]) memory[write_address[9:2]][23:16] <= WDATA[23:16];
                if (WSTRB[3]) memory[write_address[9:2]][31:24] <= WDATA[31:24];
                write_address <= write_address + 4;
                write_beats_left <= write_beats_left - 1'b1;
                if (WLAST) begin
                    write_active <= 1'b0;
                    BVALID <= 1'b1;
                end
            end
            if (BVALID && BREADY)
                BVALID <= 1'b0;

            if (ARVALID && ARREADY) begin
                read_active <= 1'b1;
                read_address <= ARADDR;
                read_beats_left <= ARLEN + 1'b1;
                ar_count <= ar_count + 1;
                if ((ARPROT !== 3'b001) || (ARSIZE !== 3'd2) ||
                    (ARBURST !== 2'b01)) begin
                    $display("ERROR: invalid AXI Read attributes");
                    errors = errors + 1;
                end
            end
            if (RVALID && RREADY) begin
                read_address <= read_address + 4;
                read_beats_left <= read_beats_left - 1'b1;
                if (RLAST)
                    read_active <= 1'b0;
            end
        end
    end

    task ospi_write_byte;
        input [7:0] value;
        reg accepted;
        begin
            accepted = 1'b0;
            master_d_out = value;
            master_d_oe = 1'b1;
            while (!accepted) begin
                #5 SCLK = 1'b1;
                #1 accepted = (SRDY === 1'b1);
                #4 SCLK = 1'b0;
            end
        end
    endtask

    task ospi_write_a0_release;
        input [7:0] value;
        begin
            ospi_write_byte(value);
            master_d_oe = 1'b0;
        end
    endtask

    task ospi_read_tick;
        begin
            #5 SCLK = 1'b1;
            #1;
            if (SRDY === 1'b1) begin
                read_bytes[read_count] = D;
                read_count = read_count + 1;
            end
            #4 SCLK = 1'b0;
        end
    endtask

    initial begin
        rst_n = 1'b0;
        SCLK = 1'b0;
        CSN = 1'b1;
        master_d_out = 8'b0;
        master_d_oe = 1'b0;
        errors = 0;
        read_count = 0;
        memory[8'h40] = 32'b0;
        memory[8'h41] = 32'b0;
        #20 rst_n = 1'b1;
        #10;

        if (`TB_SRDY_OPEN_DRAIN != 0) begin
            if ((SRDY !== 1'b1) || (dut.core_srdy_oe !== 1'b0)) begin
                $display("ERROR: open-drain SRDY must release while unselected");
                errors = errors + 1;
            end
        end else begin
            if ((SRDY !== 1'b1) || (dut.core_srdy_oe !== 1'b1)) begin
                $display("ERROR: push-pull SRDY must drive high while unselected");
                errors = errors + 1;
            end
        end
        if ((D !== 8'hzz) || (dut.core_d_ie !== 1'b0) ||
            (dut.core_srdy_ie !== 1'b0)) begin
            $display("ERROR: unselected OSPI IO enable state mismatch");
            errors = errors + 1;
        end

        // OSPI Write: two 32-bit words at 0x100.
        CSN = 1'b0;
        #10;
        ospi_write_byte(8'h19);
        ospi_write_byte(8'h00);
        ospi_write_byte(8'h00);
        ospi_write_byte(8'h01);
        ospi_write_byte(8'h00);
        ospi_write_byte(8'hde);
        ospi_write_byte(8'had);
        ospi_write_byte(8'hbe);
        ospi_write_byte(8'hef);
        ospi_write_byte(8'h01);
        ospi_write_byte(8'h23);
        ospi_write_byte(8'h45);
        ospi_write_byte(8'h67);
        CSN = 1'b1;
        master_d_oe = 1'b0;
        wait ((memory[8'h40] == 32'hdeadbeef) &&
              (memory[8'h41] == 32'h01234567));

        if (`SINGLE_TRAN != 0) begin
            if (aw_count != 2) begin
                $display("ERROR: SINGLE_TRAN expected 2 AW, got %0d", aw_count);
                errors = errors + 1;
            end
        end else begin
            if (aw_count != 1) begin
                $display("ERROR: Burst expected 1 AW, got %0d", aw_count);
                errors = errors + 1;
            end
        end

        // OSPI Read from the same AXI memory.
        #20;
        CSN = 1'b0;
        #10;
        ospi_write_byte(8'h11);
        ospi_write_byte(8'h00);
        ospi_write_byte(8'h00);
        ospi_write_byte(8'h01);
        ospi_write_a0_release(8'h00);
        read_count = 0;
        while (read_count < 8)
            ospi_read_tick();
        CSN = 1'b1;

        if ({read_bytes[0], read_bytes[1], read_bytes[2], read_bytes[3]} !==
            32'hdeadbeef) begin
            $display("ERROR: first AXI Read word mismatch");
            errors = errors + 1;
        end
        if ({read_bytes[4], read_bytes[5], read_bytes[6], read_bytes[7]} !==
            32'h01234567) begin
            $display("ERROR: second AXI Read word mismatch");
            errors = errors + 1;
        end
        if (`SINGLE_TRAN != 0) begin
            if (ar_count != 2) begin
                $display("ERROR: SINGLE_TRAN expected 2 AR, got %0d", ar_count);
                errors = errors + 1;
            end
        end else begin
            if (ar_count != 1) begin
                $display("ERROR: Burst expected 1 AR, got %0d", ar_count);
                errors = errors + 1;
            end
        end

        if (errors == 0)
            $display("PASS: ospi_slave AXI backend test");
        else
            $display("FAIL: ospi_slave AXI backend errors=%0d", errors);
        #20 $finish;
    end
endmodule
