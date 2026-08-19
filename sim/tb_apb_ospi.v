`timescale 1ns/1ps

// End-to-end regression for the APB4-to-OSPI behavioral master model.
//
// Coverage:
// - Program Mode register map at 0x0FFF_0000.
// - Program Mode 8x32-bit TX/RX FIFOs.
// - Cmd register write starts the OSPI transaction.
// - Direct Mode 0x1000_0000..0x8FFF_FFFF.
// - Direct Mode 4-byte APB Read/Write bypasses TX/RX FIFOs.
// - 4-bit active-low CSN mapping and per-window local address translation.
// - APB wait-state behavior for TX-full / RX-empty.
// - Direct OSPI timeout behavior.
// - Unaligned / partial-write error handling.
//
// TB_STOP_ON_ERROR:
//   1 (default from Makefile) = terminate at first regression error.
//   0                         = accumulate errors and finish regression.

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

`ifdef TB_STOP_ON_ERROR
    localparam integer STOP_ON_ERROR = `TB_STOP_ON_ERROR;
`else
    localparam integer STOP_ON_ERROR = 1;
`endif

    localparam integer AXI_DATA_WIDTH = 32;
    localparam integer AXI_ID_WIDTH   = 6;

    localparam [31:0] REG_CMD   = 32'h0FFF_0000;
    localparam [31:0] REG_ADDR  = 32'h0FFF_0004;
    localparam [31:0] REG_WDATA = 32'h0FFF_0008;
    localparam [31:0] REG_RDATA = 32'h0FFF_000C;
    localparam [31:0] REG_CTRL  = 32'h0FFF_0010;

    localparam [31:0] DIRECT0_BASE = 32'h1000_0000;
    localparam [31:0] DIRECT1_BASE = 32'h3000_0000;
    localparam [31:0] DIRECT2_BASE = 32'h5000_0000;
    localparam [31:0] DIRECT3_BASE = 32'h7000_0000;

    localparam [31:0] DIRECT_TEST_OFS = 32'h0000_0100;
    localparam [31:0] PROGRAM_WR_ADDR = 32'h0000_0200;
    localparam [31:0] PROGRAM_RD_ADDR = 32'h0000_0204;

    reg PCLK;
    reg reset_n;
    reg sys_clk;

    // Main APB master interface connected end-to-end to ospi_slave_top.
    reg  [31:0] PADDR;
    reg         PSEL;
    reg         PENABLE;
    reg         PWRITE;
    reg  [31:0] PWDATA;
    reg  [3:0]  PSTRB;
    reg  [2:0]  PPROT;
    wire [31:0] PRDATA;
    wire        PREADY;
    wire        PSLVERR;

    wire        SCLK;
    wire [3:0]  CSN;
    tri  [7:0]  D;
    tri1        SRDY;

    // Address-map-only master. SRDY is tied high so Direct Writes complete
    // without a real slave. The testbench samples command/address/data bytes.
    reg  [31:0] map_PADDR;
    reg         map_PSEL;
    reg         map_PENABLE;
    reg         map_PWRITE;
    reg  [31:0] map_PWDATA;
    reg  [3:0]  map_PSTRB;
    reg  [2:0]  map_PPROT;
    wire [31:0] map_PRDATA;
    wire        map_PREADY;
    wire        map_PSLVERR;
    wire        map_SCLK;
    wire [3:0]  map_CSN;
    tri  [7:0]  map_D;

    // Dedicated short-timeout master used only for TX-full / RX-empty APB
    // wait-state verification. Keeping this separate from map_master prevents
    // the functional CSN-mapping test from timing out before 9 OSPI bytes
    // can be serialized.
    reg  [31:0] fifo_PADDR;
    reg         fifo_PSEL;
    reg         fifo_PENABLE;
    reg         fifo_PWRITE;
    reg  [31:0] fifo_PWDATA;
    reg  [3:0]  fifo_PSTRB;
    reg  [2:0]  fifo_PPROT;
    wire [31:0] fifo_PRDATA;
    wire        fifo_PREADY;
    wire        fifo_PSLVERR;
    wire        fifo_SCLK;
    wire [3:0]  fifo_CSN;
    tri  [7:0]  fifo_D;

    // Timeout-only master. SRDY is forced low.
    reg  [31:0] timeout_PADDR;
    reg         timeout_PSEL;
    reg         timeout_PENABLE;
    reg         timeout_PWRITE;
    reg  [31:0] timeout_PWDATA;
    reg  [3:0]  timeout_PSTRB;
    reg  [2:0]  timeout_PPROT;
    wire [31:0] timeout_PRDATA;
    wire        timeout_PREADY;
    wire        timeout_PSLVERR;
    wire        timeout_SCLK;
    wire [3:0]  timeout_CSN;
    tri  [7:0]  timeout_D;

    // AXI backend for Slave0.
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
    reg  BVALID;
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
    reg  [AXI_DATA_WIDTH-1:0] RDATA;
    wire [1:0] RRESP;
    wire RLAST;
    wire RVALID;
    wire RREADY;

    reg [7:0] memory [0:8191];
    reg write_active;
    reg [31:0] write_address;
    reg read_active;
    reg [31:0] read_address;

    integer errors;
    integer lane;
    integer aw_count;
    integer ar_count;
    integer wait_guard;

    reg apb_error;
    reg [31:0] apb_read_data;

    integer map_byte_count;
    reg [7:0] map_bytes [0:8];
    reg [3:0] map_seen_csn;

    task report_error;
        input [8*200-1:0] message;
        begin
            $display("%0t ERROR: %0s", $time, message);
            errors = errors + 1;
            if (STOP_ON_ERROR != 0) begin
                `TB_TERMINATE_FAILURE;
            end
        end
    endtask

    task report_info;
        input [8*200-1:0] message;
        begin
            $display("%0t INFO: %0s", $time, message);
        end
    endtask

    // Main behavioral master.
    ospi_master #(
        .APB_TIMEOUT_CYCLES(1024),
        .OSPI_TIMEOUT_CYCLES(1024)
    ) master (
        .PCLK(PCLK),
        .PRESETn(reset_n),
        .PADDR(PADDR),
        .PSEL(PSEL),
        .PENABLE(PENABLE),
        .PWRITE(PWRITE),
        .PWDATA(PWDATA),
        .PSTRB(PSTRB),
        .PPROT(PPROT),
        .PRDATA(PRDATA),
        .PREADY(PREADY),
        .PSLVERR(PSLVERR),
        .SCLK(SCLK),
        .CSN(CSN),
        .D(D),
        .SRDY(SRDY)
    );

    // No real slave is needed here because only Direct Writes are used.
    // Keep a generous APB timeout: one Direct Write serializes 9 OSPI bytes
    // (Cmd + 4-byte address + 4-byte data), so this instance must not use
    // the intentionally-short FIFO timeout setting.
    ospi_master #(
        .APB_TIMEOUT_CYCLES(64),
        .OSPI_TIMEOUT_CYCLES(64)
    ) map_master (
        .PCLK(PCLK),
        .PRESETn(reset_n),
        .PADDR(map_PADDR),
        .PSEL(map_PSEL),
        .PENABLE(map_PENABLE),
        .PWRITE(map_PWRITE),
        .PWDATA(map_PWDATA),
        .PSTRB(map_PSTRB),
        .PPROT(map_PPROT),
        .PRDATA(map_PRDATA),
        .PREADY(map_PREADY),
        .PSLVERR(map_PSLVERR),
        .SCLK(map_SCLK),
        .CSN(map_CSN),
        .D(map_D),
        .SRDY(1'b1)
    );

    // Dedicated FIFO wait-state timeout checker.
    ospi_master #(
        .APB_TIMEOUT_CYCLES(8),
        .OSPI_TIMEOUT_CYCLES(64)
    ) fifo_timeout_master (
        .PCLK(PCLK),
        .PRESETn(reset_n),
        .PADDR(fifo_PADDR),
        .PSEL(fifo_PSEL),
        .PENABLE(fifo_PENABLE),
        .PWRITE(fifo_PWRITE),
        .PWDATA(fifo_PWDATA),
        .PSTRB(fifo_PSTRB),
        .PPROT(fifo_PPROT),
        .PRDATA(fifo_PRDATA),
        .PREADY(fifo_PREADY),
        .PSLVERR(fifo_PSLVERR),
        .SCLK(fifo_SCLK),
        .CSN(fifo_CSN),
        .D(fifo_D),
        .SRDY(1'b1)
    );

    ospi_master #(
        .APB_TIMEOUT_CYCLES(8),
        .OSPI_TIMEOUT_CYCLES(8)
    ) timeout_master (
        .PCLK(PCLK),
        .PRESETn(reset_n),
        .PADDR(timeout_PADDR),
        .PSEL(timeout_PSEL),
        .PENABLE(timeout_PENABLE),
        .PWRITE(timeout_PWRITE),
        .PWDATA(timeout_PWDATA),
        .PSTRB(timeout_PSTRB),
        .PPROT(timeout_PPROT),
        .PRDATA(timeout_PRDATA),
        .PREADY(timeout_PREADY),
        .PSLVERR(timeout_PSLVERR),
        .SCLK(timeout_SCLK),
        .CSN(timeout_CSN),
        .D(timeout_D),
        .SRDY(1'b0)
    );

    // Current RTL slave remains a single-CSN slave. The system-level master
    // connects Slave0 to CSN[0]. CSN[1:3] are verified separately above.
    ospi_slave_top #(
        .AXI_ADDR_WIDTH(32),
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
        .AXI_ID_WIDTH(AXI_ID_WIDTH),
        .AXI_PROT(3'b001)
    ) slave0 (
        .clk(sys_clk),
        .rst_n(reset_n),
        .SCLK(SCLK),
        .CSN(CSN[0]),
        .D(D),
        .SRDY(SRDY),

        .M_AXI_AWID(AWID),
        .M_AXI_AWADDR(AWADDR),
        .M_AXI_AWLEN(AWLEN),
        .M_AXI_AWSIZE(AWSIZE),
        .M_AXI_AWBURST(AWBURST),
        .M_AXI_AWPROT(AWPROT),
        .M_AXI_AWVALID(AWVALID),
        .M_AXI_AWREADY(AWREADY),

        .M_AXI_WDATA(WDATA),
        .M_AXI_WSTRB(WSTRB),
        .M_AXI_WLAST(WLAST),
        .M_AXI_WVALID(WVALID),
        .M_AXI_WREADY(WREADY),

        .M_AXI_BID(BID),
        .M_AXI_BRESP(BRESP),
        .M_AXI_BVALID(BVALID),
        .M_AXI_BREADY(BREADY),

        .M_AXI_ARID(ARID),
        .M_AXI_ARADDR(ARADDR),
        .M_AXI_ARLEN(ARLEN),
        .M_AXI_ARSIZE(ARSIZE),
        .M_AXI_ARBURST(ARBURST),
        .M_AXI_ARPROT(ARPROT),
        .M_AXI_ARVALID(ARVALID),
        .M_AXI_ARREADY(ARREADY),

        .M_AXI_RID(RID),
        .M_AXI_RDATA(RDATA),
        .M_AXI_RRESP(RRESP),
        .M_AXI_RLAST(RLAST),
        .M_AXI_RVALID(RVALID),
        .M_AXI_RREADY(RREADY)
    );

    assign AWREADY = !write_active && !BVALID;
    assign WREADY  = write_active;
    assign BID     = {AXI_ID_WIDTH{1'b0}};
    assign BRESP   = 2'b00;

    assign ARREADY = !read_active;
    assign RID     = {AXI_ID_WIDTH{1'b0}};
    assign RRESP   = 2'b00;
    assign RVALID  = read_active;
    assign RLAST   = read_active;

    always @* begin
        RDATA = 32'b0;
        for (lane = 0; lane < 4; lane = lane + 1)
            RDATA[(lane*8) +: 8] =
                memory[(read_address & 32'h0000_1ffc) + lane];
    end

    initial PCLK = 1'b0;
    always #5 PCLK = ~PCLK;

    initial sys_clk = 1'b0;
    always #3 sys_clk = ~sys_clk;

    // Minimal AXI memory target.
    always @(posedge sys_clk or negedge reset_n) begin
        if (!reset_n) begin
            write_active <= 1'b0;
            write_address <= 32'b0;
            read_active <= 1'b0;
            read_address <= 32'b0;
            BVALID <= 1'b0;
            aw_count <= 0;
            ar_count <= 0;
        end else begin
            if (AWVALID && AWREADY) begin
                write_active <= 1'b1;
                write_address <= AWADDR;
                aw_count <= aw_count + 1;
                if ((AWLEN !== 8'b0) ||
                    (AWSIZE !== 3'd2) ||
                    (AWBURST !== 2'b01) ||
                    (AWPROT !== 3'b001))
                    report_error("invalid AXI Write attributes");
            end

            if (WVALID && WREADY) begin
                for (lane = 0; lane < 4; lane = lane + 1) begin
                    if (WSTRB[lane])
                        memory[(write_address & 32'h0000_1ffc) + lane]
                            <= WDATA[(lane*8) +: 8];
                end

                if (!WLAST)
                    report_error("32-bit request must terminate with WLAST");

                write_active <= 1'b0;
                BVALID <= 1'b1;
            end

            if (BVALID && BREADY)
                BVALID <= 1'b0;

            if (ARVALID && ARREADY) begin
                read_active <= 1'b1;
                read_address <= ARADDR;
                ar_count <= ar_count + 1;

                if ((ARLEN !== 8'b0) ||
                    (ARSIZE !== 3'd2) ||
                    (ARBURST !== 2'b01) ||
                    (ARPROT !== 3'b001))
                    report_error("invalid AXI Read attributes");
            end

            if (RVALID && RREADY)
                read_active <= 1'b0;
        end
    end

    // Capture the Direct-Mode serialization generated by map_master.
    always @(posedge map_SCLK) begin
        if ((map_CSN != 4'hf) && (map_byte_count < 9)) begin
            map_bytes[map_byte_count] = map_D;
            map_seen_csn = map_CSN;
            map_byte_count = map_byte_count + 1;
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

    task map_apb_write;
        input [31:0] address;
        input [31:0] data;
        output response_error;
        reg complete;
        begin
            map_byte_count = 0;
            map_seen_csn = 4'hf;

            @(negedge PCLK);
            map_PADDR = address;
            map_PWDATA = data;
            map_PSTRB = 4'hf;
            map_PPROT = 3'b000;
            map_PWRITE = 1'b1;
            map_PSEL = 1'b1;
            map_PENABLE = 1'b0;

            @(negedge PCLK);
            map_PENABLE = 1'b1;

            complete = 1'b0;
            while (!complete) begin
                @(posedge PCLK);
                #1 complete = (map_PREADY === 1'b1);
            end

            response_error = map_PSLVERR;

            @(negedge PCLK);
            map_PSEL = 1'b0;
            map_PENABLE = 1'b0;
            map_PWRITE = 1'b0;
        end
    endtask

    task map_apb_read;
        input [31:0] address;
        output [31:0] data;
        output response_error;
        output integer wait_cycles;
        reg complete;
        begin
            @(negedge PCLK);
            map_PADDR = address;
            map_PSTRB = 4'b0;
            map_PPROT = 3'b000;
            map_PWRITE = 1'b0;
            map_PSEL = 1'b1;
            map_PENABLE = 1'b0;

            @(negedge PCLK);
            map_PENABLE = 1'b1;

            complete = 1'b0;
            wait_cycles = 0;
            while (!complete) begin
                @(posedge PCLK);
                wait_cycles = wait_cycles + 1;
                #1 complete = (map_PREADY === 1'b1);
            end

            data = map_PRDATA;
            response_error = map_PSLVERR;

            @(negedge PCLK);
            map_PSEL = 1'b0;
            map_PENABLE = 1'b0;
        end
    endtask

    task fifo_timeout_write;
        input [31:0] data;
        output response_error;
        output integer wait_cycles;
        reg complete;
        begin
            @(negedge PCLK);
            fifo_PADDR = REG_WDATA;
            fifo_PWDATA = data;
            fifo_PSTRB = 4'hf;
            fifo_PPROT = 3'b000;
            fifo_PWRITE = 1'b1;
            fifo_PSEL = 1'b1;
            fifo_PENABLE = 1'b0;

            @(negedge PCLK);
            fifo_PENABLE = 1'b1;

            complete = 1'b0;
            wait_cycles = 0;
            while (!complete) begin
                @(posedge PCLK);
                wait_cycles = wait_cycles + 1;
                #1 complete = (fifo_PREADY === 1'b1);
            end

            response_error = fifo_PSLVERR;

            @(negedge PCLK);
            fifo_PSEL = 1'b0;
            fifo_PENABLE = 1'b0;
            fifo_PWRITE = 1'b0;
        end
    endtask

    task fifo_timeout_read;
        output [31:0] data;
        output response_error;
        output integer wait_cycles;
        reg complete;
        begin
            @(negedge PCLK);
            fifo_PADDR = REG_RDATA;
            fifo_PSTRB = 4'b0;
            fifo_PPROT = 3'b000;
            fifo_PWRITE = 1'b0;
            fifo_PSEL = 1'b1;
            fifo_PENABLE = 1'b0;

            @(negedge PCLK);
            fifo_PENABLE = 1'b1;

            complete = 1'b0;
            wait_cycles = 0;
            while (!complete) begin
                @(posedge PCLK);
                wait_cycles = wait_cycles + 1;
                #1 complete = (fifo_PREADY === 1'b1);
            end

            data = fifo_PRDATA;
            response_error = fifo_PSLVERR;

            @(negedge PCLK);
            fifo_PSEL = 1'b0;
            fifo_PENABLE = 1'b0;
        end
    endtask

    task timeout_enable;
        output response_error;
        reg complete;
        begin
            @(negedge PCLK);
            timeout_PADDR = REG_CTRL;
            timeout_PWDATA = 32'h8000_0000;
            timeout_PSTRB = 4'hf;
            timeout_PPROT = 3'b000;
            timeout_PWRITE = 1'b1;
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
            timeout_PWRITE = 1'b0;
        end
    endtask

    task timeout_direct_read;
        output response_error;
        output integer wait_cycles;
        reg complete;
        begin
            @(negedge PCLK);
            timeout_PADDR = DIRECT0_BASE + DIRECT_TEST_OFS;
            timeout_PSTRB = 4'b0;
            timeout_PPROT = 3'b000;
            timeout_PWRITE = 1'b0;
            timeout_PSEL = 1'b1;
            timeout_PENABLE = 1'b0;

            @(negedge PCLK);
            timeout_PENABLE = 1'b1;

            complete = 1'b0;
            wait_cycles = 0;
            while (!complete) begin
                @(posedge PCLK);
                wait_cycles = wait_cycles + 1;
                #1 complete = (timeout_PREADY === 1'b1);
            end

            response_error = timeout_PSLVERR;

            @(negedge PCLK);
            timeout_PSEL = 1'b0;
            timeout_PENABLE = 1'b0;
        end
    endtask

    task check_map_transfer;
        input [31:0] apb_address;
        input [3:0] expected_csn;
        input [31:0] write_data;
        reg map_error;
        reg [31:0] expected_local;
        begin
            expected_local = apb_address;
            case (expected_csn)
                4'b1110: expected_local = apb_address - DIRECT0_BASE;
                4'b1101: expected_local = apb_address - DIRECT1_BASE;
                4'b1011: expected_local = apb_address - DIRECT2_BASE;
                4'b0111: expected_local = apb_address - DIRECT3_BASE;
                default: expected_local = 32'hxxxx_xxxx;
            endcase

            map_apb_write(apb_address, write_data, map_error);

            if (map_error)
                report_error("Direct map write unexpectedly returned PSLVERR");

            if (map_seen_csn !== expected_csn)
                report_error("Direct map selected incorrect CSN");

            if (map_byte_count != 9)
                report_error("Direct map did not serialize exactly 9 bytes");

            if (map_bytes[0] !== 8'h18)
                report_error("Direct map command byte is not 8'h18");

            if ((map_bytes[1] !== expected_local[31:24]) ||
                (map_bytes[2] !== expected_local[23:16]) ||
                (map_bytes[3] !== expected_local[15:8])  ||
                (map_bytes[4] !== expected_local[7:0]))
                report_error("Direct map local OSPI address translation mismatch");

            if ((map_bytes[5] !== write_data[31:24]) ||
                (map_bytes[6] !== write_data[23:16]) ||
                (map_bytes[7] !== write_data[15:8])  ||
                (map_bytes[8] !== write_data[7:0]))
                report_error("Direct map Write data serialization mismatch");
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

        map_PADDR = 32'b0;
        map_PSEL = 1'b0;
        map_PENABLE = 1'b0;
        map_PWRITE = 1'b0;
        map_PWDATA = 32'b0;
        map_PSTRB = 4'b0;
        map_PPROT = 3'b0;

        fifo_PADDR = 32'b0;
        fifo_PSEL = 1'b0;
        fifo_PENABLE = 1'b0;
        fifo_PWRITE = 1'b0;
        fifo_PWDATA = 32'b0;
        fifo_PSTRB = 4'b0;
        fifo_PPROT = 3'b0;

        timeout_PADDR = 32'b0;
        timeout_PSEL = 1'b0;
        timeout_PENABLE = 1'b0;
        timeout_PWRITE = 1'b0;
        timeout_PWDATA = 32'b0;
        timeout_PSTRB = 4'b0;
        timeout_PPROT = 3'b0;

        errors = 0;
        map_byte_count = 0;
        map_seen_csn = 4'hf;

        for (lane = 0; lane < 8192; lane = lane + 1)
            memory[lane] = 8'b0;

        #30 reset_n = 1'b1;
        #30;

        report_info("Enable main Program/Direct Mode master");
        apb_write(REG_CTRL, 32'h8000_0000, 4'hf, 3'b001, apb_error);
        if (apb_error)
            report_error("Ctrl enable write returned PSLVERR");

        // ------------------------------------------------------------
        // Direct Mode Slave0 end-to-end Write and Read.
        // ------------------------------------------------------------
        report_info("Direct Mode Slave0 end-to-end Write");
        apb_write(DIRECT0_BASE + DIRECT_TEST_OFS,
                  32'hDEAD_BEEF, 4'hf, 3'b101, apb_error);
        if (apb_error)
            report_error("Direct Mode Write returned PSLVERR");

        wait_guard = 0;
        while ((memory[DIRECT_TEST_OFS + 0] !== 8'hEF ||
                memory[DIRECT_TEST_OFS + 1] !== 8'hBE ||
                memory[DIRECT_TEST_OFS + 2] !== 8'hAD ||
                memory[DIRECT_TEST_OFS + 3] !== 8'hDE) &&
               (wait_guard < 2000)) begin
            @(posedge PCLK);
            wait_guard = wait_guard + 1;
        end

        if (wait_guard >= 2000)
            report_error("Direct Mode Write did not reach AXI memory");

        report_info("Direct Mode Slave0 end-to-end Read");
        apb_read(DIRECT0_BASE + DIRECT_TEST_OFS,
                 3'b010, apb_read_data, apb_error);

        if (apb_error)
            report_error("Direct Mode Read returned PSLVERR");
        if (apb_read_data !== 32'hDEAD_BEEF)
            report_error("Direct Mode Read data mismatch");

        // ------------------------------------------------------------
        // Program Mode: FIFO Write + Cmd-triggered transfer.
        // ------------------------------------------------------------
        report_info("Program Mode FIFO Write transaction");
        apb_write(REG_WDATA, 32'hA1B2_C3D4, 4'hf, 3'b000, apb_error);
        if (apb_error)
            report_error("Program Mode TX FIFO push returned PSLVERR");

        apb_write(REG_ADDR, PROGRAM_WR_ADDR, 4'hf, 3'b000, apb_error);
        if (apb_error)
            report_error("Program Mode Addr write returned PSLVERR");

        // 8'h18: CmdType=1, Write=1, length=4 bytes.
        apb_write(REG_CMD, 32'h0000_0018, 4'hf, 3'b000, apb_error);
        if (apb_error)
            report_error("Program Mode Cmd Write start returned PSLVERR");

        wait_guard = 0;
        while ((memory[PROGRAM_WR_ADDR + 0] !== 8'hD4 ||
                memory[PROGRAM_WR_ADDR + 1] !== 8'hC3 ||
                memory[PROGRAM_WR_ADDR + 2] !== 8'hB2 ||
                memory[PROGRAM_WR_ADDR + 3] !== 8'hA1) &&
               (wait_guard < 2000)) begin
            @(posedge PCLK);
            wait_guard = wait_guard + 1;
        end

        if (wait_guard >= 2000)
            report_error("Program Mode Write did not reach AXI memory");

        // ------------------------------------------------------------
        // Program Mode Read: Cmd starts transfer; Rdata waits for RX FIFO.
        // ------------------------------------------------------------
        memory[PROGRAM_RD_ADDR + 0] = 8'h88;
        memory[PROGRAM_RD_ADDR + 1] = 8'h77;
        memory[PROGRAM_RD_ADDR + 2] = 8'h66;
        memory[PROGRAM_RD_ADDR + 3] = 8'h55;

        report_info("Program Mode RX FIFO Read transaction");
        apb_write(REG_ADDR, PROGRAM_RD_ADDR, 4'hf, 3'b000, apb_error);
        if (apb_error)
            report_error("Program Mode Read Addr write returned PSLVERR");

        // 8'h10: CmdType=1, Read=0, length=4 bytes.
        apb_write(REG_CMD, 32'h0000_0010, 4'hf, 3'b000, apb_error);
        if (apb_error)
            report_error("Program Mode Read Cmd Write returned PSLVERR");

        apb_read(REG_RDATA, 3'b000, apb_read_data, apb_error);
        if (apb_error)
            report_error("Program Mode Rdata pop returned PSLVERR");
        if (apb_read_data !== 32'h5566_7788)
            report_error("Program Mode Read data mismatch");

        // ------------------------------------------------------------
        // Local APB error cases.
        // ------------------------------------------------------------
        report_info("Check Direct Mode unaligned and partial write errors");
        apb_write(DIRECT0_BASE + DIRECT_TEST_OFS + 2,
                  32'h1234_5678, 4'hf, 3'b000, apb_error);
        if (!apb_error)
            report_error("unaligned Direct Mode Write was not rejected");

        apb_write(DIRECT0_BASE + DIRECT_TEST_OFS,
                  32'h1234_5678, 4'b0011, 3'b000, apb_error);
        if (!apb_error)
            report_error("partial Direct Mode Write was not rejected");

        // ------------------------------------------------------------
        // CSN[3:0] address-window mapping and local-address translation.
        // ------------------------------------------------------------
        report_info("Enable standalone Direct Mode map checker");
        map_apb_write(REG_CTRL, 32'h8000_0000, apb_error);
        if (apb_error)
            report_error("map_master Ctrl enable returned PSLVERR");

        report_info("Check Direct Mode CSN0 mapping");
        check_map_transfer(DIRECT0_BASE + DIRECT_TEST_OFS,
                           4'b1110, 32'h0102_0304);

        report_info("Check Direct Mode CSN1 mapping");
        check_map_transfer(DIRECT1_BASE + DIRECT_TEST_OFS,
                           4'b1101, 32'h1112_1314);

        report_info("Check Direct Mode CSN2 mapping");
        check_map_transfer(DIRECT2_BASE + DIRECT_TEST_OFS,
                           4'b1011, 32'h2122_2324);

        report_info("Check Direct Mode CSN3 mapping");
        check_map_transfer(DIRECT3_BASE + DIRECT_TEST_OFS,
                           4'b0111, 32'h3132_3334);

        // ------------------------------------------------------------
        // TX FIFO full / RX FIFO empty APB wait-state and timeout behavior.
        // fifo_timeout_master intentionally uses APB_TIMEOUT_CYCLES=8.
        // ------------------------------------------------------------
        report_info("Check TX FIFO full APB wait then timeout");
        for (lane = 0; lane < 8; lane = lane + 1) begin
            fifo_timeout_write(32'hA500_0000 + lane, apb_error, wait_guard);
            if (apb_error)
                report_error("TX FIFO push before full unexpectedly failed");
        end

        fifo_timeout_write(32'hFFFF_FFFF, apb_error, wait_guard);
        if (!apb_error)
            report_error("TX FIFO full did not timeout with PSLVERR");
        if (wait_guard < 8)
            report_error("TX FIFO full did not hold PREADY low for timeout");

        report_info("Check RX FIFO empty APB wait then timeout");
        fifo_timeout_read(apb_read_data, apb_error, wait_guard);
        if (!apb_error)
            report_error("RX FIFO empty did not timeout with PSLVERR");
        if (wait_guard < 8)
            report_error("RX FIFO empty did not hold PREADY low for timeout");

        // ------------------------------------------------------------
        // Direct OSPI SRDY timeout.
        // ------------------------------------------------------------
        report_info("Check Direct Mode SRDY timeout");
        timeout_enable(apb_error);
        if (apb_error)
            report_error("timeout_master Ctrl enable returned PSLVERR");

        timeout_direct_read(apb_error, wait_guard);
        if (!apb_error)
            report_error("SRDY-low Direct Read did not return PSLVERR");

        // Allow worker to unwind after abort/timeout.
        repeat (4) @(posedge PCLK);
        if (timeout_CSN !== 4'hf)
            report_error("timeout master did not return CSN to idle 4'hF");
        if (timeout_SCLK !== 1'b0)
            report_error("timeout master did not return SCLK low");

        // ------------------------------------------------------------
        // Final result.
        // ------------------------------------------------------------
        if (errors == 0) begin
            $display("%0t INFO: tb_apb_ospi PASS", $time);
            $finish;
        end else begin
            $display("%0t ERROR: tb_apb_ospi FAIL errors=%0d", $time, errors);
            `TB_TERMINATE_FAILURE;
        end
    end

endmodule
