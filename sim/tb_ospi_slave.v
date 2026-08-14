`timescale 1ns/1ps

// TB_AXI_DATA_WIDTH options: 8, 16, 32, 64, 128, 256, 512, 1024.
`ifndef TB_AXI_DATA_WIDTH
`define TB_AXI_DATA_WIDTH 32
`endif

// TB_SRDY_OPEN_DRAIN options: 0 = push-pull, 1 = open-drain with pull-up.
`ifndef TB_SRDY_OPEN_DRAIN
`define TB_SRDY_OPEN_DRAIN 0
`endif

// TB_TEST_ADDRESS rule: 32-bit, 4-byte aligned OSPI effective address.
`ifndef TB_TEST_ADDRESS
`define TB_TEST_ADDRESS 32'h00000100
`endif

// TB_4K_BOUNDARY options: 0 = normal request, 1 = require two bursts because
// the 8-byte request starts at 0xFFC and crosses into the next 4KB region.
`ifndef TB_4K_BOUNDARY
`define TB_4K_BOUNDARY 0
`endif

// TB_LONG_BURST options: 0 = normal 8-byte test, 1 = 512-byte width-8 test
// that requires two 256-beat AXI bursts. Use only with SINGLE_TRAN=0.
`ifndef TB_LONG_BURST
`define TB_LONG_BURST 0
`endif

// TB_TERMINATE_FAILURE rule: Icarus uses its non-standard exit-status task;
// VCS and other SystemVerilog simulators use the standard $fatal system task.
`ifdef __ICARUS__
`define TB_TERMINATE_FAILURE $finish_and_return(1)
`else
`define TB_TERMINATE_FAILURE $fatal(1, "tb_ospi_slave regression failed")
`endif

module tb_ospi_slave;
`ifdef FSDB
    initial begin
        $fsdbDumpfile(`FSDB_FILE);
        $fsdbDumpvars(0, tb_ospi_slave);
        $fsdbDumpMDA();
    end
`endif

    localparam integer AXI_ADDR_WIDTH = 32;
    localparam integer AXI_DATA_WIDTH = `TB_AXI_DATA_WIDTH;
    localparam integer AXI_ID_WIDTH = 6;
    localparam integer AXI_STRB_WIDTH = AXI_DATA_WIDTH / 8;
    localparam integer AXI_BEAT_BYTES = (AXI_DATA_WIDTH < 32) ?
                                        AXI_STRB_WIDTH : 4;
    localparam [2:0] EXPECTED_AXSIZE = (AXI_DATA_WIDTH == 8) ? 3'd0 :
                                       (AXI_DATA_WIDTH == 16) ? 3'd1 : 3'd2;
    localparam integer SUBBEATS_PER_WORD = (AXI_DATA_WIDTH == 8) ? 4 :
                                           (AXI_DATA_WIDTH == 16) ? 2 : 1;
    localparam [31:0] TEST_ADDRESS = `TB_TEST_ADDRESS;
    localparam integer EXPECTED_SINGLE_COUNT = 2 * SUBBEATS_PER_WORD;
    localparam integer EXPECTED_BURST_COUNT =
                       (`TB_4K_BOUNDARY != 0) ? 2 : 1;

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
    wire [AXI_DATA_WIDTH-1:0] WDATA;
    wire [AXI_STRB_WIDTH-1:0] WSTRB;
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
    reg [2:0] write_size;
    reg [8:0] write_beats_left;
    reg read_active;
    reg [31:0] read_address;
    reg [2:0] read_size;
    reg [8:0] read_beats_left;
    integer aw_count;
    integer ar_count;
    integer errors;
    integer read_count;
    integer read_lane;
    integer write_lane;
    integer long_index;
    integer long_read_count;
    reg [7:0] read_bytes [0:7];

    assign AWREADY = !write_active && !BVALID;
    assign WREADY = write_active;
    assign BID = {AXI_ID_WIDTH{1'b0}};
    assign BRESP = 2'b00;
    assign ARREADY = !read_active;
    assign RID = {AXI_ID_WIDTH{1'b0}};
    assign RRESP = 2'b00;
    assign RVALID = read_active;
    assign RLAST = read_active && (read_beats_left == 9'd1);

    always @* begin
        RDATA = {AXI_DATA_WIDTH{1'b0}};
        for (read_lane = 0; read_lane < AXI_STRB_WIDTH;
             read_lane = read_lane + 1)
            RDATA[(read_lane*8) +: 8] =
                memory[(read_address -
                        (read_address % AXI_STRB_WIDTH)) + read_lane];
    end

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

    // Parameterized AXI byte-addressable memory slave. Every accepted address
    // is checked against the AXI 4KB boundary rule.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            write_active <= 1'b0;
            write_address <= 32'b0;
            write_size <= 3'b0;
            write_beats_left <= 9'b0;
            BVALID <= 1'b0;
            read_active <= 1'b0;
            read_address <= 32'b0;
            read_size <= 3'b0;
            read_beats_left <= 9'b0;
            aw_count <= 0;
            ar_count <= 0;
        end else begin
            if (AWVALID && AWREADY) begin
                write_active <= 1'b1;
                write_address <= AWADDR;
                write_size <= AWSIZE;
                write_beats_left <= {1'b0, AWLEN} + 9'd1;
                aw_count <= aw_count + 1;
                if ((AWPROT !== 3'b001) ||
                    (AWSIZE !== EXPECTED_AXSIZE) ||
                    (AWBURST !== 2'b01)) begin
                    $display("ERROR: invalid AXI Write attributes width=%0d",
                             AXI_DATA_WIDTH);
                    errors = errors + 1;
                end
                if (({1'b0, AWADDR[11:0]} +
                    (({5'b0, AWLEN} + 13'd1) << AWSIZE)) > 13'd4096) begin
                    $display("ERROR: AXI Write burst crosses 4KB boundary");
                    errors = errors + 1;
                end
                if ((`TB_LONG_BURST != 0) && (AWLEN !== 8'd255)) begin
                    $display("ERROR: long Write burst must contain 256 beats");
                    errors = errors + 1;
                end
            end
            if (WVALID && WREADY) begin
                for (write_lane = 0; write_lane < AXI_STRB_WIDTH;
                     write_lane = write_lane + 1) begin
                    if (WSTRB[write_lane])
                        memory[(write_address -
                               (write_address % AXI_STRB_WIDTH)) +
                               write_lane] <=
                               WDATA[(write_lane*8) +: 8];
                end
                if (WLAST !== (write_beats_left == 9'd1)) begin
                    $display("ERROR: AXI WLAST mismatch");
                    errors = errors + 1;
                end
                write_address <= write_address + (1 << write_size);
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
                read_size <= ARSIZE;
                read_beats_left <= {1'b0, ARLEN} + 9'd1;
                ar_count <= ar_count + 1;
                if ((ARPROT !== 3'b001) ||
                    (ARSIZE !== EXPECTED_AXSIZE) ||
                    (ARBURST !== 2'b01)) begin
                    $display("ERROR: invalid AXI Read attributes width=%0d",
                             AXI_DATA_WIDTH);
                    errors = errors + 1;
                end
                if (({1'b0, ARADDR[11:0]} +
                    (({5'b0, ARLEN} + 13'd1) << ARSIZE)) > 13'd4096) begin
                    $display("ERROR: AXI Read burst crosses 4KB boundary");
                    errors = errors + 1;
                end
                if ((`TB_LONG_BURST != 0) && (ARLEN !== 8'd255)) begin
                    $display("ERROR: long Read burst must contain 256 beats");
                    errors = errors + 1;
                end
            end
            if (RVALID && RREADY) begin
                read_address <= read_address + (1 << read_size);
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

    task ospi_send_address;
        input release_a0;
        begin
            ospi_write_byte(TEST_ADDRESS[31:24]);
            ospi_write_byte(TEST_ADDRESS[23:16]);
            ospi_write_byte(TEST_ADDRESS[15:8]);
            if (release_a0)
                ospi_write_a0_release(TEST_ADDRESS[7:0]);
            else
                ospi_write_byte(TEST_ADDRESS[7:0]);
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
        memory[TEST_ADDRESS + 0] = 8'b0;
        memory[TEST_ADDRESS + 1] = 8'b0;
        memory[TEST_ADDRESS + 2] = 8'b0;
        memory[TEST_ADDRESS + 3] = 8'b0;
        memory[TEST_ADDRESS + 4] = 8'b0;
        memory[TEST_ADDRESS + 5] = 8'b0;
        memory[TEST_ADDRESS + 6] = 8'b0;
        memory[TEST_ADDRESS + 7] = 8'b0;
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

        if (`TB_LONG_BURST != 0) begin
            // Maximum-length OSPI request on an 8-bit AXI bus: 512 AXI beats
            // must be split into two legal 256-beat INCR bursts.
            CSN = 1'b0;
            #10;
            ospi_write_byte(8'h1f);
            ospi_send_address(1'b0);
            for (long_index = 0; long_index < 512;
                 long_index = long_index + 1)
                ospi_write_byte(long_index[7:0]);
            CSN = 1'b1;
            master_d_oe = 1'b0;
            wait (!write_active && !BVALID && !AWVALID && (aw_count == 2));
            for (long_index = 0; long_index < 512;
                 long_index = long_index + 1) begin
                if (memory[TEST_ADDRESS + long_index] !==
                    ((((long_index / 4) * 4) +
                      (3 - (long_index % 4))) & 8'hff)) begin
                    $display("ERROR: long Write data mismatch byte=%0d",
                             long_index);
                    errors = errors + 1;
                end
            end

            #20;
            CSN = 1'b0;
            #10;
            ospi_write_byte(8'h17);
            ospi_send_address(1'b1);
            long_read_count = 0;
            while (long_read_count < 512) begin
                #5 SCLK = 1'b1;
                #1;
                if (SRDY === 1'b1) begin
                    if (D !== long_read_count[7:0]) begin
                        $display("ERROR: long Read data mismatch byte=%0d",
                                 long_read_count);
                        errors = errors + 1;
                    end
                    long_read_count = long_read_count + 1;
                end
                #4 SCLK = 1'b0;
            end
            CSN = 1'b1;
            if ((aw_count != 2) || (ar_count != 2)) begin
                $display("ERROR: long request expected two AW and two AR");
                errors = errors + 1;
            end

            if (errors == 0)
                $display("PASS: 512-byte request split into 256-beat bursts");
            else begin
                $display("FAIL: long burst errors=%0d", errors);
                `TB_TERMINATE_FAILURE;
            end
            #20 $finish;
        end else begin
            // OSPI Write: two 32-bit words through the configured AXI width.
            CSN = 1'b0;
            #10;
            ospi_write_byte(8'h19);
            ospi_send_address(1'b0);
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
            wait ((memory[TEST_ADDRESS + 0] == 8'hef) &&
              (memory[TEST_ADDRESS + 1] == 8'hbe) &&
              (memory[TEST_ADDRESS + 2] == 8'had) &&
              (memory[TEST_ADDRESS + 3] == 8'hde) &&
              (memory[TEST_ADDRESS + 4] == 8'h67) &&
              (memory[TEST_ADDRESS + 5] == 8'h45) &&
              (memory[TEST_ADDRESS + 6] == 8'h23) &&
              (memory[TEST_ADDRESS + 7] == 8'h01));
        wait (!write_active && !BVALID && !AWVALID);

        if (`SINGLE_TRAN != 0) begin
            if (aw_count != EXPECTED_SINGLE_COUNT) begin
                $display("ERROR: SINGLE_TRAN expected %0d AW, got %0d",
                         EXPECTED_SINGLE_COUNT, aw_count);
                errors = errors + 1;
            end
        end else if (aw_count != EXPECTED_BURST_COUNT) begin
            $display("ERROR: Burst expected %0d AW, got %0d",
                     EXPECTED_BURST_COUNT, aw_count);
            errors = errors + 1;
        end

        // OSPI Read back through the same parameterized AXI memory.
        #20;
        CSN = 1'b0;
        #10;
        ospi_write_byte(8'h11);
        ospi_send_address(1'b1);
        read_count = 0;
        while (read_count < 8)
            ospi_read_tick();
        CSN = 1'b1;

        if ({read_bytes[0], read_bytes[1], read_bytes[2], read_bytes[3]} !==
            32'hdeadbeef) begin
            $display("ERROR: first AXI Read word mismatch width=%0d",
                     AXI_DATA_WIDTH);
            errors = errors + 1;
        end
        if ({read_bytes[4], read_bytes[5], read_bytes[6], read_bytes[7]} !==
            32'h01234567) begin
            $display("ERROR: second AXI Read word mismatch width=%0d",
                     AXI_DATA_WIDTH);
            errors = errors + 1;
        end
        if (`SINGLE_TRAN != 0) begin
            if (ar_count != EXPECTED_SINGLE_COUNT) begin
                $display("ERROR: SINGLE_TRAN expected %0d AR, got %0d",
                         EXPECTED_SINGLE_COUNT, ar_count);
                errors = errors + 1;
            end
        end else if (ar_count != EXPECTED_BURST_COUNT) begin
            $display("ERROR: Burst expected %0d AR, got %0d",
                     EXPECTED_BURST_COUNT, ar_count);
            errors = errors + 1;
        end

        if (errors == 0) begin
            $display("PASS: ospi_slave AXI width=%0d single=%0d boundary=%0d",
                     AXI_DATA_WIDTH, `SINGLE_TRAN, `TB_4K_BOUNDARY);
        end else begin
            $display("FAIL: ospi_slave errors=%0d width=%0d",
                     errors, AXI_DATA_WIDTH);
            `TB_TERMINATE_FAILURE;
        end
            #20 $finish;
        end
    end
endmodule
