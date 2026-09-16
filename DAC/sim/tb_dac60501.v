`timescale 1ns/1ps

module tb_dac60501;

    // ============================================================
    // ★★★ 在这里修改目标输出电压 ★★★
    //
    // 单位：mV
    //
    // 例如：
    //
    // 100 = 100 mV
    // 200 = 200 mV
    // 300 = 300 mV
    // 399 = 399 mV
    //
    // ============================================================

    parameter integer TARGET_MV = 100;
    integer expected_code;
    integer observed_code;
    // ============================================================
    // 50 MHz FPGA clock
    // ============================================================

    reg clk;

    initial begin

        clk = 0;

        forever #10 clk = ~clk;

    end


    // ============================================================
    // reset
    // ============================================================

    reg reset_n;

    initial begin

        reset_n = 0;

        #1000;

        reset_n = 1;

    end


    // ============================================================
    // UART
    // ============================================================

    reg uart_rx;

    wire dac_sdin;
    wire dac_sclk;
    wire dac_sync_n;


    // ============================================================
    // FPGA
    // ============================================================

    dac60501_uart_top uut (

        .clk       (clk),
        .reset_n   (reset_n),

        .uart_rx   (uart_rx),

        .dac_sdin  (dac_sdin),
        .dac_sclk  (dac_sclk),
        .dac_sync_n(dac_sync_n)

    );


    // ============================================================
    // DAC60501 MODEL
    // ============================================================

    dac60501_model dac (

        .sdin  (dac_sdin),
        .sclk  (dac_sclk),
        .sync_n(dac_sync_n)

    );


    // ============================================================
    // UART 115200 8N1
    //
    // 1 bit = 8.6805 us
    //
    // ============================================================

    task uart_send_byte;

        input [7:0] data;

        integer i;

        begin

            // START
            uart_rx = 1'b0;

            #8680;

            // DATA
            for (i = 0; i < 8; i = i + 1) begin

                uart_rx = data[i];

                #8680;

            end

            // STOP
            uart_rx = 1'b1;

            #8680;

        end

    endtask


    // ============================================================
    // Send decimal number
    //
    // 例如 TARGET_MV = 100
    //
    // 实际发送：
    //
    // '1'
    // '0'
    // '0'
    // CR
    // LF
    //
    // ============================================================

    task uart_send_number;

        input integer value;

        integer h;
        integer t;
        integer o;

        begin

            h = value / 100;
            t = (value % 100) / 10;
            o = value % 10;


            // hundreds

            if (h > 0) begin

                uart_send_byte(8'h30 + h);

            end


            // tens

            if ((h > 0) || (t > 0)) begin

                uart_send_byte(8'h30 + t);

            end


            // ones

            uart_send_byte(8'h30 + o);


            // CR

            uart_send_byte(8'h0D);


            // LF

            uart_send_byte(8'h0A);

        end

    endtask


    // ============================================================
    // Simulation
    // ============================================================

    initial begin

        uart_rx = 1'b1;


        // 等 FPGA reset
        // Wait for reset and every initialization SPI frame to finish.
        // This avoids relying on a fixed delay when SPI parameters change.
        wait (uut.init_done == 1'b1);


        $display("");
        $display("==============================================");
        $display("START UART TEST");
        $display("TARGET = %0d mV", TARGET_MV);
        $display("==============================================");
        $display("");


        // --------------------------------------------------------
        // 发送目标电压
        // --------------------------------------------------------

        uart_send_number(TARGET_MV);


        // --------------------------------------------------------
        // 等待 SPI 完成
        // --------------------------------------------------------

       #100000;

        expected_code = TARGET_MV * 10 + TARGET_MV / 4 + TARGET_MV / 128;
        if (expected_code > 4095)
            expected_code = 4095;
        observed_code = dac.dac_reg[15:4];

        if (observed_code !== expected_code) begin
            $display("ERROR: DAC code mismatch: expected %0d, got %0d",
                     expected_code, observed_code);
            $stop;
        end
        else begin
            $display("PASS: DAC code = %0d", observed_code);
        end

        $display("");
        $display("==============================================");
        $display("SIMULATION FINISHED");
        $display("TARGET = %0d mV", TARGET_MV);
        $display("==============================================");


        #10000;

        $stop;

    end

endmodule
