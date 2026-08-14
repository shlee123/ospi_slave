`timescale 1ns/1ps

// TB_TERMINATE_FAILURE rule: Icarus uses its non-standard exit-status task;
// VCS and other SystemVerilog simulators use the standard $fatal system task.
`ifdef __ICARUS__
`define TB_TERMINATE_FAILURE $finish_and_return(1)
`else
`define TB_TERMINATE_FAILURE $fatal(1, "tb_apb_ospi regression failed")
`endif

module tb_apb_ospi;
`ifdef FSDB
    initial begin
        $fsdbDumpfile(`FSDB_FILE);
        $fsdbDumpvars(0, tb_apb_ospi);
        $fsdbDumpMDA();
    end
`endif

    localparam integer AXI_DATA_WIDTH = 32;
    localparam integer AXI_ID_WIDTH = 6;
    localparam [31:0] TEST_ADDRESS = 32'h00000100;

    reg PCLK;
    reg reset_n;
    reg sys_clk;

    reg [31:0] PADDR;
    reg PSEL;
    reg PENABLE;
    reg PWRITE;
    reg [31:0] PWDATA;
    reg [3:0] PSTRB;
    reg [2:0] PPROT;
    wire [31:0] PRDATA;
    wire PREADY;
    wire PSLVERR;

    wire SCLK;
    wire CSN;
    tri [7:0] D;
    tri1 SRDY;

    reg [31:0] timeout_PADDR;
    reg timeout_PSEL;
    reg timeout_PENABLE;
    reg timeout_PWRITE;
    reg [31:0] timeout_PWDATA;
    reg [3:0] timeout_PSTRB;
    reg [2:0] timeout_PPROT;
    wire [31:0] timeout_PRDATA;
    wire timeout_PREADY;
    wire timeout_PSLVERR;
    wire timeout_SCLK;
    wire timeout_CSN;
    tri [7:0] timeout_D;

    wire [AXI_ID_WIDTH-1:0] AWID;
    wire [31:0] AWADDR;
    wire [7:0] AWLEN;
    wire [2:0] AWSIZE;
    wire [1:0] AWBURST;
    wire [2:0] AWPROT;
    wire AWVALID;
    wire AWREADY;
    wire [AXI_DATA_WIDTH-1:0] WDATA;
    wire [(AXI_DATA_WIDTH/8)-1:0] WSTRB;
    wire WLAST;
    wire WVALID;
    wire WREADY;
    wire [AXI_ID_WIDTH-1:0] BID;
    wire [1:0] BRESP;
    reg BVALID;
    wire BREADY;
    wire [AXI_ID_WIDTH-1:0] ARID;
    wire [31:0] ARADDR;
    wire [7:0] ARLEN;
    wire [2:0] ARSIZE;
    wire [1:0] ARBURST;
    wire [2:0] ARPROT;
    wire ARVALID;
    wire ARREADY;
    wire [AXI_ID_WIDTH-1:0] RID;
    reg [AXI_DATA_WIDTH-1:0] RDATA;
    wire [1:0] RRESP;
    wire RLAST;
    wire RVALID;
    wire RREADY;

    reg [7:0] memory [0:8191];
    reg write_active;
    reg [31:0] write_address;
    reg read_active;
    reg [31:0] read_address;
    integer aw_count;
    integer ar_count;
    integer errors;
    integer lane;
    integer saved_aw_count;
    reg apb_error;
    reg [31:0] apb_read_data;

    ospi_master #(
        .SCLK_HALF_PERIOD(2),
        .TIMEOUT_CYCLES(64)
    ) master (
        .PCLK(PCLK), .PRESETn(reset_n), .PADDR(PADDR), .PSEL(PSEL),
        .PENABLE(PENABLE), .PWRITE(PWRITE), .PWDATA(PWDATA),
        .PSTRB(PSTRB), .PPROT(PPROT), .PRDATA(PRDATA),
        .PREADY(PREADY), .PSLVERR(PSLVERR), .SCLK(SCLK), .CSN(CSN),
        .D(D), .SRDY(SRDY)
    );

    // Standalone instance used to verify TIMEOUT_CYCLES. Its SRDY is tied
    // low and it is intentionally not connected to an OSPI slave.
    ospi_master #(
        .SCLK_HALF_PERIOD(2),
        .TIMEOUT_CYCLES(4)
    ) timeout_master (
        .PCLK(PCLK), .PRESETn(reset_n), .PADDR(timeout_PADDR),
        .PSEL(timeout_PSEL), .PENABLE(timeout_PENABLE),
        .PWRITE(timeout_PWRITE), .PWDATA(timeout_PWDATA),
        .PSTRB(timeout_PSTRB), .PPROT(timeout_PPROT),
        .PRDATA(timeout_PRDATA), .PREADY(timeout_PREADY),
        .PSLVERR(timeout_PSLVERR), .SCLK(timeout_SCLK),
        .CSN(timeout_CSN), .D(timeout_D), .SRDY(1'b0)
    );

    ospi_slave_top #(
        .AXI_ADDR_WIDTH(32),
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
        .AXI_ID_WIDTH(AXI_ID_WIDTH),
        .AXI_PROT(3'b001)
    ) slave (
        .clk(sys_clk), .rst_n(reset_n), .SCLK(SCLK), .CSN(CSN),
        .D(D), .SRDY(SRDY), .M_AXI_AWID(AWID), .M_AXI_AWADDR(AWADDR),
        .M_AXI_AWLEN(AWLEN), .M_AXI_AWSIZE(AWSIZE),
        .M_AXI_AWBURST(AWBURST), .M_AXI_AWPROT(AWPROT),
        .M_AXI_AWVALID(AWVALID), .M_AXI_AWREADY(AWREADY),
        .M_AXI_WDATA(WDATA), .M_AXI_WSTRB(WSTRB),
        .M_AXI_WLAST(WLAST), .M_AXI_WVALID(WVALID),
        .M_AXI_WREADY(WREADY), .M_AXI_BID(BID), .M_AXI_BRESP(BRESP),
        .M_AXI_BVALID(BVALID), .M_AXI_BREADY(BREADY),
        .M_AXI_ARID(ARID), .M_AXI_ARADDR(ARADDR),
        .M_AXI_ARLEN(ARLEN), .M_AXI_ARSIZE(ARSIZE),
        .M_AXI_ARBURST(ARBURST), .M_AXI_ARPROT(ARPROT),
        .M_AXI_ARVALID(ARVALID), .M_AXI_ARREADY(ARREADY),
        .M_AXI_RID(RID), .M_AXI_RDATA(RDATA), .M_AXI_RRESP(RRESP),
        .M_AXI_RLAST(RLAST), .M_AXI_RVALID(RVALID),
        .M_AXI_RREADY(RREADY)
    );

    assign AWREADY = !write_active && !BVALID;
    assign WREADY = write_active;
    assign BID = {AXI_ID_WIDTH{1'b0}};
    assign BRESP = 2'b00;
    assign ARREADY = !read_active;
    assign RID = {AXI_ID_WIDTH{1'b0}};
    assign RRESP = 2'b00;
    assign RVALID = read_active;
    assign RLAST = read_active;

    always @* begin
        RDATA = 32'b0;
        for (lane = 0; lane < 4; lane = lane + 1)
            RDATA[(lane*8) +: 8] =
                memory[(read_address & 32'hfffffffc) + lane];
    end

    initial PCLK = 1'b0;
    always #5 PCLK = ~PCLK;
    initial sys_clk = 1'b0;
    always #3 sys_clk = ~sys_clk;

    // Minimal AXI memory target used to close the end-to-end path from APB,
    // through the OSPI master and slave, to the slave's AXI master backend.
    always @(posedge sys_clk or negedge reset_n) begin
        if (!reset_n) begin
            write_active <= 1'b0;
            write_address <= 32'b0;
            BVALID <= 1'b0;
            read_active <= 1'b0;
            read_address <= 32'b0;
            aw_count <= 0;
            ar_count <= 0;
        end else begin
            if (AWVALID && AWREADY) begin
                write_active <= 1'b1;
                write_address <= AWADDR;
                aw_count <= aw_count + 1;
                if ((AWLEN !== 8'b0) || (AWSIZE !== 3'd2) ||
                    (AWBURST !== 2'b01) || (AWPROT !== 3'b001)) begin
                    $display("ERROR: invalid AXI Write attributes");
                    errors = errors + 1;
                end
            end
            if (WVALID && WREADY) begin
                for (lane = 0; lane < 4; lane = lane + 1) begin
                    if (WSTRB[lane])
                        memory[(write_address & 32'hfffffffc) + lane] <=
                            WDATA[(lane*8) +: 8];
                end
                if (!WLAST) begin
                    $display("ERROR: APB word must map to one AXI Write beat");
                    errors = errors + 1;
                end
                write_active <= 1'b0;
                BVALID <= 1'b1;
            end
            if (BVALID && BREADY)
                BVALID <= 1'b0;

            if (ARVALID && ARREADY) begin
                read_active <= 1'b1;
                read_address <= ARADDR;
                ar_count <= ar_count + 1;
                if ((ARLEN !== 8'b0) || (ARSIZE !== 3'd2) ||
                    (ARBURST !== 2'b01) || (ARPROT !== 3'b001)) begin
                    $display("ERROR: invalid AXI Read attributes");
                    errors = errors + 1;
                end
            end
            if (RVALID && RREADY)
                read_active <= 1'b0;
        end
    end

    task apb_write;
        input [31:0] address;
        input [31:0] data;
        input [3:0] strobe;
        input [2:0] protection;
        output response_error;
        reg complete;
        begin
            @(negedge PCLK);
            PADDR = address;
            PWDATA = data;
            PSTRB = strobe;
            PPROT = protection;
            PWRITE = 1'b1;
            PSEL = 1'b1;
            PENABLE = 1'b0;
            @(negedge PCLK);
            PENABLE = 1'b1;
            complete = 1'b0;
            while (!complete) begin
                @(posedge PCLK);
                #1 complete = (PREADY === 1'b1);
            end
            response_error = PSLVERR;
            @(negedge PCLK);
            PSEL = 1'b0;
            PENABLE = 1'b0;
            PWRITE = 1'b0;
        end
    endtask

    task apb_read;
        input [31:0] address;
        input [2:0] protection;
        output [31:0] data;
        output response_error;
        reg complete;
        begin
            @(negedge PCLK);
            PADDR = address;
            PSTRB = 4'b0;
            PPROT = protection;
            PWRITE = 1'b0;
            PSEL = 1'b1;
            PENABLE = 1'b0;
            @(negedge PCLK);
            PENABLE = 1'b1;
            complete = 1'b0;
            while (!complete) begin
                @(posedge PCLK);
                #1 complete = (PREADY === 1'b1);
            end
            data = PRDATA;
            response_error = PSLVERR;
            @(negedge PCLK);
            PSEL = 1'b0;
            PENABLE = 1'b0;
        end
    endtask

    task apb_timeout_read;
        output response_error;
        reg complete;
        begin
            @(negedge PCLK);
            timeout_PADDR = TEST_ADDRESS;
            timeout_PSTRB = 4'b0;
            timeout_PPROT = 3'b011;
            timeout_PWRITE = 1'b0;
            timeout_PSEL = 1'b1;
            timeout_PENABLE = 1'b0;
            @(negedge PCLK);
            timeout_PENABLE = 1'b1;
            complete = 1'b0;
            while (!complete) begin
                @(posedge PCLK);
                #1 complete = (timeout_PREADY === 1'b1);
            end
            response_error = timeout_PSLVERR;
            @(negedge PCLK);
            timeout_PSEL = 1'b0;
            timeout_PENABLE = 1'b0;
        end
    endtask

    initial begin
        reset_n = 1'b0;
        PADDR = 32'b0;
        PSEL = 1'b0;
        PENABLE = 1'b0;
        PWRITE = 1'b0;
        PWDATA = 32'b0;
        PSTRB = 4'b0;
        PPROT = 3'b0;
        timeout_PADDR = 32'b0;
        timeout_PSEL = 1'b0;
        timeout_PENABLE = 1'b0;
        timeout_PWRITE = 1'b0;
        timeout_PWDATA = 32'b0;
        timeout_PSTRB = 4'b0;
        timeout_PPROT = 3'b0;
        errors = 0;
        memory[TEST_ADDRESS + 0] = 8'b0;
        memory[TEST_ADDRESS + 1] = 8'b0;
        memory[TEST_ADDRESS + 2] = 8'b0;
        memory[TEST_ADDRESS + 3] = 8'b0;

        #30 reset_n = 1'b1;
        #20;

        // PPROT is accepted by the APB interface but is not carried by the
        // current OSPI packet; the slave therefore keeps its configured PROT.
        apb_write(TEST_ADDRESS, 32'hdeadbeef, 4'b1111, 3'b101,
                   apb_error);
        if (apb_error) begin
            $display("ERROR: aligned full-word APB Write returned PSLVERR");
            errors = errors + 1;
        end
        wait ((memory[TEST_ADDRESS + 0] == 8'hef) &&
              (memory[TEST_ADDRESS + 1] == 8'hbe) &&
              (memory[TEST_ADDRESS + 2] == 8'had) &&
              (memory[TEST_ADDRESS + 3] == 8'hde));

        apb_read(TEST_ADDRESS, 3'b010, apb_read_data, apb_error);
        if (apb_error || (apb_read_data !== 32'hdeadbeef)) begin
            $display("ERROR: APB Read data=%08x error=%0d",
                     apb_read_data, apb_error);
            errors = errors + 1;
        end

        saved_aw_count = aw_count;
        apb_write(TEST_ADDRESS + 2, 32'h12345678, 4'b1111, 3'b000,
                   apb_error);
        if (!apb_error || (aw_count != saved_aw_count)) begin
            $display("ERROR: unaligned APB Write was not rejected locally");
            errors = errors + 1;
        end

        apb_write(TEST_ADDRESS, 32'h12345678, 4'b0011, 3'b111,
                   apb_error);
        if (!apb_error || (aw_count != saved_aw_count)) begin
            $display("ERROR: partial APB Write was not rejected locally");
            errors = errors + 1;
        end

        apb_timeout_read(apb_error);
        if (!apb_error || (timeout_CSN !== 1'b1) ||
            (timeout_SCLK !== 1'b0)) begin
            $display("ERROR: SRDY timeout did not return PSLVERR and idle OSPI");
            errors = errors + 1;
        end

        if ((aw_count != 1) || (ar_count != 1)) begin
            $display("ERROR: expected one forwarded Write and Read, AW=%0d AR=%0d",
                     aw_count, ar_count);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("PASS: APB-to-OSPI-to-AXI Read/Write and error handling");
        else begin
            $display("FAIL: tb_apb_ospi errors=%0d", errors);
            `TB_TERMINATE_FAILURE;
        end
        #20 $finish;
    end

endmodule
